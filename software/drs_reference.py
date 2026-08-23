#!/usr/bin/env python3
"""DRS reference on the REORDERED (node-major) problem — the frame the HW works in.

Faithful copy of scs_adaptive.solve_adaptive's core loop, parameterized by
A/b/c/blocks. Two purposes:
  1. Validate that the reordered problem (Ab = A[rp][:,cp], br = b[rp],
     cr = q[cp], cones reordered by rp) converges to the SAME optimum as the
     original (final_mass ~= 1799.156 kg).
  2. Export per-iteration state (v, u_t, u, rsk) as hex for drs_iter RTL
     validation (Phase 4b), plus b_r/c_r for the hardware problem data.

All arithmetic is float64 round-nearest (numpy); the RTL is truncating FP64,
so per-iteration comparisons use a tolerance (~1e-8), and the end-to-end
acceptance is convergence of final_mass, not bit-exactness.
"""
import json, os, sys, struct
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from scs_faithful import Anderson, proj_soc, NV, NR, D, x_ref, obj_ref
from banded_reference import row_perm, col_perm, RHO_X, Z, SCALE

HERE = os.path.dirname(os.path.abspath(__file__))
RTLD = os.path.join(HERE, "..", "rtl", "data", "kkt")

# SCS constants (same as scs_adaptive)
ALPHA = 1.5
TAU_FACTOR = 10.0
MIN_SCALE = 1e-6
MAX_SCALE = 1e6
RESCALING_MIN_ITERS = 100
CONVERGED_INTERVAL = 25
FEAS = 1
DIV_EPS = 1e-18


def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]
def write_hex(f, vals):
    with open(f, "w") as fh:
        for v in vals:
            fh.write("%016X\n" % f64(v))


def reorder_problem():
    """Ab/br/cr + cone blocks in the node-major frame (matches rtl/data/kkt/full)."""
    A = sp.csc_matrix((np.array(D["nzval"]), np.array(D["rowval"]),
                       np.array(D["colptr"])), shape=(NR, NV)).tocsr()
    b = np.array(D["b"]); q = np.array(D["q"])
    rp, cp = row_perm(), col_perm()
    Ab = A[rp][:, cp].tocsr()
    br = b[rp]; cr = q[cp]
    # reordered cone blocks: merge consecutive rows with the SAME ORIGINAL
    # cone-block id. Critical: each SOC cone (dim 4 or 3) keeps its own id —
    # merging the 11 consecutive SOC rows of a node into one dim-11 "soc"
    # block would apply proj_soc to the WRONG cone (vel4+slack4+lower3 are
    # three separate cones). zero/nonneg rows can merge freely (their
    # projections are element-wise / uniform).
    bid = np.empty(NR, dtype=int)
    r0 = 0; b = 0
    for cc in D["cones"]:
        if cc["kind"] == "zero":
            bid[r0:r0 + cc["dim"]] = 0
        elif cc["kind"] == "nonneg":
            bid[r0:r0 + cc["dim"]] = 1
        else:
            b += 1
            bid[r0:r0 + cc["dim"]] = b + 1
        r0 += cc["dim"]
    kind_of = {0: "zero", 1: "nonneg"}
    blocks = []
    cur = -1; start = 0
    for i, r in enumerate(rp):
        b = bid[r]
        if b != cur:
            if cur != -1:
                blocks.append((kind_of.get(cur, "soc"), start, i - start))
            cur = b; start = i
    blocks.append((kind_of.get(cur, "soc"), start, NR - start))
    return Ab.tocsr(), br, cr, rp, cp, blocks


def proj_dual_cone_r(x, r_y, blocks):
    """R_y-weighted Moreau (reordered blocks)."""
    s = x.copy()
    x = -r_y * x
    for kind, st, dim in blocks:
        if kind == "zero":
            x[st:st + dim] = 0.0
        elif kind == "nonneg":
            x[st:st + dim] = np.maximum(x[st:st + dim], 0.0)
        elif kind == "soc":
            x[st:st + dim] = proj_soc(x[st:st + dim])
    return x / r_y + s


def root_plus(diag_r, g, p, mu, eta):
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


def build_diag_r(scale, n, m, rp):
    """diag_r in the REORDERED frame: r_y must be permuted by rp (the zero-cone
    rows are scattered node-major, NOT the first Z rows anymore)."""
    r_y_orig = np.empty(m)
    r_y_orig[:Z] = 1.0 / (1000.0 * scale)
    r_y_orig[Z:] = 1.0 / scale
    diag_r = np.empty(n + m + 1)
    diag_r[:n] = RHO_X
    diag_r[n:n + m] = r_y_orig[rp]
    diag_r[n + m] = TAU_FACTOR
    return diag_r


def one_iteration(A, br, cr, blocks, scale, diag_r, lu, g, v, v_prev, i, aa, safeguard=True):
    """One full DRS iteration on the reordered problem. Returns (v, u_t, u, rsk).
    Mirrors scs_adaptive.solve_adaptive's loop body EXACTLY (AA/normalize
    included for i>0). This is the RTL oracle for drs_iter."""
    n, m = A.shape[1], A.shape[0]
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    if i > 0 and i % 10 == 0:
        aa.apply(v, v_prev)
    if i >= FEAS:
        vn = np.linalg.norm(v)
        if vn > 0:
            v *= sqrt_l / vn
    v_prev[:] = v
    # project_lin_sys
    u_t = np.empty(l)
    u_t[:n] = v[:n] * diag_r[:n]
    u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
    u_t[l - 1] = v[l - 1]
    u_t[:l - 1] = lu.solve(u_t[:l - 1])
    if i < FEAS:
        u_t[l - 1] = 1.0
    else:
        u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
    u_t[:l - 1] -= u_t[l - 1] * g
    # project_cones
    u = 2 * u_t - v
    u[n:l - 1] = proj_dual_cone_r(u[n:l - 1], diag_r[n:l - 1], blocks)
    if i < FEAS:
        u[l - 1] = 1.0
    else:
        u[l - 1] = max(u[l - 1], 0.0)
    # compute_rsk
    rsk = (v + u - 2 * u_t) * diag_r
    # update_dual_vars
    v += ALPHA * (u - u_t)
    # AA safeguard
    if safeguard and i % 10 == 0:
        aa.safeguard(v, v_prev)
    return v, u_t, u, rsk


def solve(max_iter, eps=1e-4, adaptive_scale=True, interval=10, track=None, safeguard=True):
    """Full run on the reordered problem; returns result dict (scs_adaptive shape)."""
    Ab, br, cr, rp, cp, blocks = reorder_problem()
    n, m = Ab.shape[1], Ab.shape[0]
    l = n + m + 1
    nm_b = np.max(np.abs(br)); nm_c = np.max(np.abs(cr))
    scale = 1.0
    last_scale_update_iter = 0
    sum_log_scale = 0.0
    n_log_scale = 0
    diag_r = build_diag_r(scale, n, m, rp)
    lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                       [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
    g = lu.solve(np.concatenate([cr, -br]))
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    u = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    aa = Anderson(l)
    n_scale_updates = 0
    scale_hist = []
    converged_iter = max_iter

    def safediv(x, y): return x / y if y > DIV_EPS else x / DIV_EPS
    for i in range(max_iter):
        v, u_t, u, rsk = one_iteration(Ab, br, cr, blocks, scale, diag_r, lu, g,
                                       v, v_prev, i, aa, safeguard=safeguard)
        if i % CONVERGED_INTERVAL == 0:
            tau = abs(u[l - 1]); kap = abs(rsk[l - 1])
            x_ = u[:n]; y_ = u[n:l - 1]; s_ = rsk[n:l - 1]
            ax = Ab @ x_
            ax_s_btau = ax + s_ - tau * br
            aty = Ab.T @ y_
            px_aty_ctau = aty + tau * cr
            bty_tau = np.dot(y_, br); ctx_tau = np.dot(x_, cr)
            gap = abs(safediv(ctx_tau, tau) + safediv(bty_tau, tau))
            res_pri = safediv(np.max(np.abs(ax_s_btau)), tau)
            res_dual = safediv(np.max(np.abs(px_aty_ctau)), tau)
            if tau > 0:
                grl = max(abs(safediv(ctx_tau, tau)), abs(safediv(bty_tau, tau)))
                prl = max(nm_b * tau, np.max(np.abs(s_)), np.max(np.abs(ax))) / tau
                drl = max(nm_c * tau, np.max(np.abs(aty))) / tau
                if (res_pri < eps + eps * prl and res_dual < eps + eps * drl
                        and gap < eps + eps * grl):
                    converged_iter = i
                    break
            if adaptive_scale:
                denom_pri = max(np.max(np.abs(ax)), np.max(np.abs(s_)), nm_b * tau)
                rel_pri = max(safediv(np.max(np.abs(ax_s_btau)), denom_pri), DIV_EPS)
                denom_dual = max(np.max(np.abs(aty)), nm_c * tau)
                rel_dual = max(safediv(np.max(np.abs(px_aty_ctau)), denom_dual), DIV_EPS)
                sum_log_scale += np.log(rel_pri) - np.log(rel_dual)
                n_log_scale += 1
                factor = np.sqrt(np.exp(sum_log_scale / n_log_scale))
                if i - last_scale_update_iter >= RESCALING_MIN_ITERS:
                    new_scale = min(max(scale * factor, MIN_SCALE), MAX_SCALE)
                    if new_scale != scale and (factor > np.sqrt(10.) or factor < 1. / np.sqrt(10.)):
                        scale = new_scale
                        sum_log_scale = 0.0; n_log_scale = 0
                        last_scale_update_iter = i
                        n_scale_updates += 1
                        diag_r = build_diag_r(scale, n, m, rp)
                        lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                                           [Ab, sp.diags(-diag_r[n:n + m], format="csc")]],
                                          format="csc"))
                        g = lu.solve(np.concatenate([cr, -br]))
                        aa.reset()
                        v[:] = rsk / diag_r + 2 * u_t - u
                        scale_hist.append((i, scale))
        if track and i % track == 0:
            tau = u[l - 1]
            xx = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
            print(f"  {i:6d}  rel_x={np.linalg.norm(xx - x_ref) / np.linalg.norm(x_ref):.3e}  "
                  f"obj={cr @ xx:.6f}  tau={tau:.5f}", flush=True)
    tau = u[l - 1]
    x = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
    return dict(x=x, tau=tau, v=v, u_t=u_t, u=u, rsk=rsk, n_scale_updates=n_scale_updates,
                scale=scale, scale_hist=scale_hist, converged_iter=converged_iter,
                aa=aa, Ab=Ab, br=br, cr=cr, blocks=blocks)


def gen_hex(case="full", seed=7, n_iter=2):
    """Write b_r/c_r + the first n_iter iteration states as hex for drs_iter RTL."""
    Ab, br, cr, rp, cp, blocks = reorder_problem()
    n, m = Ab.shape[1], Ab.shape[0]
    l = n + m + 1
    out = os.path.join(RTLD, case)
    os.makedirs(out, exist_ok=True)
    write_hex(os.path.join(out, "b_r.hex"), br)
    write_hex(os.path.join(out, "c_r.hex"), cr)
    diag_r = build_diag_r(1.0, n, m, rp)
    lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                       [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
    g = lu.solve(np.concatenate([cr, -br]))
    write_hex(os.path.join(out, "g.hex"), g)
    write_hex(os.path.join(out, "diag_r.hex"), diag_r)
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    aa = Anderson(l)
    write_hex(os.path.join(out, "v0.hex"), v)
    for i in range(n_iter):
        v, u_t, u, rsk = one_iteration(Ab, br, cr, blocks, 1.0, diag_r, lu, g,
                                       v.copy(), v_prev, i, aa)
        write_hex(os.path.join(out, f"v{i + 1}.hex"), v)
        write_hex(os.path.join(out, f"ut{i + 1}.hex"), u_t)
        write_hex(os.path.join(out, f"u{i + 1}.hex"), u)
        write_hex(os.path.join(out, f"rsk{i + 1}.hex"), rsk)
    print(f"[{case}] n={n} m={m} l={l}  wrote b_r/c_r/g/diag_r/v0 + {n_iter} iters to {out}")
    # sanity: x after iter 2
    tau = u[l - 1]
    xx = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
    print(f"  after {n_iter} iters: tau={tau:.5f} obj={cr @ xx:.6f} (ref {obj_ref:.6f})")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "converge"
    if mode == "converge":
        res = solve(60000, eps=1e-4, track=5000)
        x = res["x"]
        final_mass = float(np.exp(x[1099])) if NV > 1099 else float("nan")
        relx = np.linalg.norm(x - x_ref) / np.linalg.norm(x_ref)
        print(f"\nREORDERED DRS: tau={res['tau']:.5f} obj={res['cr'] @ x:.6f} "
              f"(ref {obj_ref:.6f})  final_mass={final_mass:.3f} kg (ref 1799.156)")
        print(f"  rel_x={relx:.3e}  scale_updates={res['n_scale_updates']} "
              f"final_scale={res['scale']:.3e}  converged_iter={res['converged_iter']}")
        print(f"  AA: accepts={res['aa'].n_accept} rejects={res['aa'].n_reject}")
    elif mode == "gen":
        case = sys.argv[2] if len(sys.argv) > 2 else "full"
        gen_hex(case, n_iter=int(sys.argv[3]) if len(sys.argv) > 3 else 2)
    else:
        print("usage: drs_reference.py [converge|gen [case [n_iter]]]")
