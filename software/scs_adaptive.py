#!/usr/bin/env python3
"""
Faithful numpy port of SCS v3 with ADAPTIVE SCALING (the missing piece).
Adds to scs_faithful.py the three things that make SCS actually converge
to 1e-3 (validated earlier as the decisive difference):
  1. diag_r = [rho_x*I_n, -r_y] built from scale + cone coupling (set_r_y),
     with KKT = [[rho_x*I, A^T],[A, -diag(r_y)]] refactorized on scale change.
  2. update_scale: global scalar `scale` updated from the log-ratio of
     relative primal/dual residuals; triggers refactor + g recompute +
     AA reset + v remap (v = rsk/diag_r + 2*u_t - u).
  3. R_y-weighted cone projection (Moreau) and R-weighted root_plus/rsk.

This is the SOFTWARE GOLD REFERENCE for the hardware adaptive-scaling block
and the banded refactorizer. It is the prerequisite for the Phase-2
fixed-point model (a fixed-point model built on a non-converging core is
meaningless, as fixed_drs.py showed).

Defaults match SCS: alpha=1.5, rho_x=1e-6, scale=1.0, normalize off,
adaptive_scale on, TAU_FACTOR=10, RESCALING_MIN_ITERS=100, CONVERGED_INTERVAL=25,
FEASIBLE_ITERS=1, Anderson type-I lookback=10/interval=10.
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from scs_faithful import Anderson, proj_soc, blocks, A, b, c, x_ref, obj_ref, NV, NR

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
Z = next(cc["dim"] for cc in D["cones"] if cc["kind"] == "zero")  # zero cone size

# SCS constants (glbopts.h)
RHO_X = 1e-6
ALPHA = 1.5
TAU_FACTOR = 10.0
MIN_SCALE = 1e-6
MAX_SCALE = 1e6
RESCALING_MIN_ITERS = 100
CONVERGED_INTERVAL = 25
FEAS = 1
DIV_EPS = 1e-18
EPS_ABS = EPS_REL = 1e-4


def safediv(x, y):
    return x / y if y > DIV_EPS else x / DIV_EPS


def build_diag_r(scale):
    n, m = NV, NR
    diag_r = np.empty(n + m + 1)
    diag_r[:n] = RHO_X
    # set_r_y cone coupling
    diag_r[n:n + Z] = 1.0 / (1000.0 * scale)   # zero cone: free dual
    diag_r[n + Z:n + m] = 1.0 / scale          # nonneg + SOC
    diag_r[n + m] = TAU_FACTOR
    return diag_r


def build_kkt(diag_r):
    n, m = NV, NR
    r_y = diag_r[n:n + m]
    KKT = sp.bmat([[sp.diags(diag_r[:n], format="csc"), A.T],
                   [A, sp.diags(-r_y, format="csc")]], format="csc")
    return splu(KKT)


def root_plus(diag_r, g, p, mu, eta):
    """R-weighted 1D root for tau (from C root_plus)."""
    r = diag_r
    gg = mug = pg = pp = pmu = 0.0
    for i in range(len(p)):
        gi, pi, mui, ri = g[i], p[i], mu[i], r[i]
        gg  += gi * gi * ri
        mug += mui * gi * ri
        pg  += pi * gi * ri
        pp  += pi * pi * ri
        pmu += pi * mui * ri
    a = r[-1] + gg
    b = mug - 2 * pg - eta * r[-1]
    cc = pp - pmu
    rad = b * b - 4 * a * cc
    if rad < 0:
        return -b / (2 * a)
    sq = np.sqrt(rad)
    if b <= 0:
        return (-b + sq) / (2 * a)
    q = -0.5 * (b + sq)
    return cc / q if q != 0 else 0.0


def proj_cone(y):
    """Projection onto the PRIMAL cone K (from C proj_cone).
    zero->0, nonneg->max, soc->proj_soc. Used inside the Moreau
    proj_dual_cone_r. NOTE: zero cone maps to 0 (primal {0}), unlike
    scs_faithful's direct-dual-cone projection which leaves it unchanged."""
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":
            out[s:s + dim] = 0.0
        elif kind == "nonneg":
            out[s:s + dim] = np.maximum(y[s:s + dim], 0.0)
        elif kind == "soc":
            out[s:s + dim] = proj_soc(y[s:s + dim])
    return out


def proj_dual_cone_r(x, r_y):
    """R_y-weighted Moreau projection (from C proj_dual_cone)."""
    s = x.copy()
    x = -r_y * x
    x = proj_cone(x)
    return x / r_y + s


def solve_adaptive(max_iter, rho_x=RHO_X, adaptive_scale=True, interval=10,
                   track=None, eps=EPS_REL):
    n, m = NV, NR
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    nm_b = np.max(np.abs(b)); nm_c = np.max(np.abs(c))  # norm_inf

    scale = 1.0
    last_scale_update_iter = 0
    sum_log_scale = 0.0
    n_log_scale = 0

    diag_r = build_diag_r(scale)
    lu = build_kkt(diag_r)
    g = lu.solve(np.concatenate([c, -b]))

    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    u = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    aa = Anderson(l)
    hist = []; t0 = time.time()
    n_scale_updates = 0
    scale_hist = []

    def populate(u, rsk):
        """Returns dict of residuals (normalize off -> orig == normalized)."""
        tau = abs(u[l - 1]); kap = abs(rsk[l - 1])
        x_ = u[:n]; y_ = u[n:l - 1]; s_ = rsk[n:l - 1]
        ax = A @ x_
        ax_s = ax + s_
        ax_s_btau = ax_s - tau * b
        aty = A.T @ y_
        px_aty_ctau = aty + tau * c          # P=0 -> px=0
        bty_tau = np.dot(y_, b); ctx_tau = np.dot(x_, c)
        bty = safediv(bty_tau, tau); ctx = safediv(ctx_tau, tau)
        gap = abs(ctx + bty)                 # xt_p_x=0
        res_pri = safediv(np.max(np.abs(ax_s_btau)), tau)
        res_dual = safediv(np.max(np.abs(px_aty_ctau)), tau)
        return dict(tau=tau, kap=kap, ax=ax, s=s_, aty=aty,
                    ax_s=ax_s, ax_s_btau=ax_s_btau, px_aty_ctau=px_aty_ctau,
                    bty_tau=bty_tau, ctx_tau=ctx_tau, bty=bty, ctx=ctx,
                    gap=gap, res_pri=res_pri, res_dual=res_dual)

    def has_converged(r, iter_):
        if r["tau"] > 0:
            grl = max(abs(r["ctx"]), abs(r["bty"]))
            prl = max(nm_b * r["tau"], np.max(np.abs(r["s"])), np.max(np.abs(r["ax"]))) / r["tau"]
            drl = max(nm_c * r["tau"], np.max(np.abs(r["aty"]))) / r["tau"]
            if (r["res_pri"] < eps + eps * prl and
                    r["res_dual"] < eps + eps * drl and
                    r["gap"] < eps + eps * grl):
                return True
        return False

    def do_update_scale(i, r):
        nonlocal scale, sum_log_scale, n_log_scale, last_scale_update_iter, diag_r, lu, g, n_scale_updates
        nm_ax = np.max(np.abs(r["ax"]))
        nm_s = np.max(np.abs(r["s"]))
        nm_px_aty_ctau = np.max(np.abs(r["px_aty_ctau"]))
        nm_aty = np.max(np.abs(r["aty"]))
        nm_ax_s_btau = np.max(np.abs(r["ax_s_btau"]))
        tau = r["tau"]
        denom_pri = max(nm_ax, nm_s, nm_b * tau)
        rel_pri = max(safediv(nm_ax_s_btau, denom_pri), DIV_EPS)
        denom_dual = max(nm_aty, nm_c * tau)          # nm_px=0
        rel_dual = max(safediv(nm_px_aty_ctau, denom_dual), DIV_EPS)
        sum_log_scale += np.log(rel_pri) - np.log(rel_dual)
        n_log_scale += 1
        factor = np.sqrt(np.exp(sum_log_scale / n_log_scale))
        if i - last_scale_update_iter < RESCALING_MIN_ITERS:
            return False
        new_scale = min(max(scale * factor, MIN_SCALE), MAX_SCALE)
        if new_scale == scale:
            return False
        if factor > np.sqrt(10.) or factor < 1. / np.sqrt(10.):
            scale = new_scale
            sum_log_scale = 0.0; n_log_scale = 0
            last_scale_update_iter = i
            n_scale_updates += 1
            diag_r = build_diag_r(scale)
            lu = build_kkt(diag_r)              # refactorize
            g = lu.solve(np.concatenate([c, -b]))  # update_work_cache
            aa.reset()                          # reset acceleration
            v[:] = rsk / diag_r + 2 * u_t - u   # remap v
            return True
        return False

    for i in range(max_iter):
        if i > 0 and i % interval == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0:
                v *= sqrt_l / vn
        v_prev[:] = v

        # --- project_lin_sys ---
        u_t[:n] = v[:n] * diag_r[:n]
        u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
        u_t[l - 1] = v[l - 1]
        u_t[:l - 1] = lu.solve(u_t[:l - 1])
        if i < FEAS:
            u_t[l - 1] = 1.0
        else:
            u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
        u_t[:l - 1] -= u_t[l - 1] * g

        # --- project_cones ---
        u[:] = 2 * u_t - v
        u[n:l - 1] = proj_dual_cone_r(u[n:l - 1], diag_r[n:l - 1])
        if i < FEAS:
            u[l - 1] = 1.0
        else:
            u[l - 1] = max(u[l - 1], 0.0)

        # --- compute_rsk ---
        rsk[:] = (v + u - 2 * u_t) * diag_r

        # --- convergence check + update_scale (every CONVERGED_INTERVAL) ---
        if i % CONVERGED_INTERVAL == 0:
            r = populate(u, rsk)
            if has_converged(r, i):
                break
            if adaptive_scale:
                if do_update_scale(i, r):
                    scale_hist.append((i, scale))

        # --- update_dual_vars ---
        v += ALPHA * (u - u_t)

        # --- AA safeguard ---
        if i % interval == 0:
            aa.safeguard(v, v_prev)

        if track and i % track == 0:
            tau = u[l - 1]
            xx = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
            hist.append((i, np.linalg.norm(xx - x_ref) / np.linalg.norm(x_ref), float(c @ xx)))

    tau = u[l - 1]
    x = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
    return dict(x=x, tau=tau, hist=hist, aa=aa, n_scale_updates=n_scale_updates,
                scale=scale, scale_hist=scale_hist, converged_iter=i)


if __name__ == "__main__":
    import sys
    eps = float(sys.argv[1]) if len(sys.argv) > 1 else 1e-6
    max_iter = int(sys.argv[2]) if len(sys.argv) > 2 else 200000
    print(f"nvars={NV} nrows={NR}  zero_cone={Z}  clarabel obj={obj_ref:.6f}  eps={eps:.0e}")
    res = solve_adaptive(max_iter, eps=eps, track=2500)
    x = res["x"]
    relx = np.linalg.norm(x - x_ref) / np.linalg.norm(x_ref)
    print(f"\nADAPTIVE SCS: tau={res['tau']:.5f} obj={c @ x:.6f} rel_x={relx:.3e}")
    print(f"  scale_updates={res['n_scale_updates']} final_scale={res['scale']:.3e} "
          f"converged_iter={res['converged_iter']}")
    print(f"  AA: accepts={res['aa'].n_accept} rejects={res['aa'].n_reject}")
    print(f"  scale_hist={res['scale_hist']}")
    for it, rx, o in res["hist"]:
        print(f"   {it:6d}  rel_x={rx:.3e}  obj={o:.6f}")
