#!/usr/bin/env python3
"""
Phase 2b: fixed-point model on the CONVERGING adaptive SCS solver.

Builds on scs_adaptive.py (which now converges — the fixed_drs.py experiment
failed only because it sat on a non-converging core). Quantizes the DATA-PATH
state (v/u/u_t/rsk + cone projection) to Q(m,n) each iteration, mirroring the
hardware memory/state registers. Keeps in FLOAT (separate phases later):
  - the KKT solve (HW: banded refactorizer)  -> u_t output quantized (register)
  - root_plus / tau scalar
  - g, diag_r, the scale-update control loop, normalize_v's norm

Measures final rel_x vs the float reference at a fixed iteration budget.
The float-vs-fixed delta = the quantization contribution (target <1e-4).
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from scs_adaptive import (Anderson, proj_soc, blocks, A, b, c, x_ref, obj_ref,
                          NV, NR, Z, build_diag_r, build_kkt, root_plus,
                          RHO_X, ALPHA, CONVERGED_INTERVAL, FEAS, DIV_EPS)
from fixed_point import Q

TAU_FACTOR = 10.0
MIN_SCALE = 1e-6; MAX_SCALE = 1e6
RESCALING_MIN_ITERS = 100


def _q(x, Qf, on=True):
    return x if (Qf is None or not on) else Qf.q(x)


def solve_fixed_adaptive(max_iter, Qf=None, adaptive_scale=True, interval=10,
                         track=None, eps=1e-4, q_g=False, which=None):
    # which: set of quantization points to enable {'v','ut','cone','rsk','u'}.
    # Default: all (when Qf given). Empty set == pure float.
    if which is None:
        which = {"v", "ut", "cone", "rsk", "u"} if Qf is not None else set()
    qv = ("v" in which) and Qf is not None
    qut = ("ut" in which) and Qf is not None
    qcone = ("cone" in which) and Qf is not None
    qrsk = ("rsk" in which) and Qf is not None
    qu = ("u" in which) and Qf is not None
    Q0 = Qf if Qf is not None else None
    n, m = NV, NR
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    nm_b = np.max(np.abs(b)); nm_c = np.max(np.abs(c))

    scale = 1.0
    last_scale_update_iter = 0
    sum_log_scale = 0.0; n_log_scale = 0
    diag_r = build_diag_r(scale)
    lu = build_kkt(diag_r)
    g = lu.solve(np.concatenate([c, -b]))
    if q_g:
        g = _q(g, Qf, q_g)

    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    u = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    aa = Anderson(l)
    hist = []; t0 = time.time(); n_scale_updates = 0

    def do_update_scale(i):
        nonlocal scale, sum_log_scale, n_log_scale, last_scale_update_iter, diag_r, lu, g, n_scale_updates
        tau = abs(u[l - 1])
        x_ = u[:n]; y_ = u[n:l - 1]; s_ = rsk[n:l - 1]
        ax = A @ x_
        ax_s_btau = ax + s_ - tau * b
        aty = A.T @ y_
        px_aty_ctau = aty + tau * c
        nm_ax = np.max(np.abs(ax)); nm_s = np.max(np.abs(s_))
        nm_pac = np.max(np.abs(px_aty_ctau)); nm_aty = np.max(np.abs(aty))
        nm_asb = np.max(np.abs(ax_s_btau))
        denom_pri = max(nm_ax, nm_s, nm_b * tau)
        rel_pri = max((nm_asb / denom_pri) if denom_pri > DIV_EPS else nm_asb / DIV_EPS, DIV_EPS)
        denom_dual = max(nm_aty, nm_c * tau)
        rel_dual = max((nm_pac / denom_dual) if denom_dual > DIV_EPS else nm_pac / DIV_EPS, DIV_EPS)
        sum_log_scale += np.log(rel_pri) - np.log(rel_dual)
        n_log_scale += 1
        factor = np.sqrt(np.exp(sum_log_scale / n_log_scale))
        if i - last_scale_update_iter < RESCALING_MIN_ITERS:
            return
        new_scale = min(max(scale * factor, MIN_SCALE), MAX_SCALE)
        if new_scale == scale:
            return
        if factor > np.sqrt(10.) or factor < 1. / np.sqrt(10.):
            scale = new_scale
            sum_log_scale = 0.0; n_log_scale = 0
            last_scale_update_iter = i
            n_scale_updates += 1
            diag_r = build_diag_r(scale)
            lu = build_kkt(diag_r)
            g = lu.solve(np.concatenate([c, -b]))
            if q_g:
                g = _q(g, Qf, q_g)
            aa.reset()
            v[:] = _q(rsk / diag_r + 2 * u_t - u, Qf, qv)

    for i in range(max_iter):
        if i > 0 and i % interval == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0:
                v *= sqrt_l / vn
            v = _q(v, Qf, qv)
        v_prev[:] = v

        # project_lin_sys (KKT solve float; quantize u_t registers)
        u_t[:n] = v[:n] * diag_r[:n]
        u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
        u_t[l - 1] = v[l - 1]
        u_t[:l - 1] = _q(lu.solve(u_t[:l - 1]), Qf, qut)
        if i < FEAS:
            u_t[l - 1] = 1.0
        else:
            u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
        u_t[:l - 1] -= u_t[l - 1] * g
        u_t[:l - 1] = _q(u_t[:l - 1], Qf, qut)

        # project_cones
        u[:] = 2 * u_t - v
        u[n:l - 1] = _q(SA_proj_dual_cone_r(u[n:l - 1], diag_r[n:l - 1]), Qf, qcone)
        if i < FEAS:
            u[l - 1] = 1.0
        else:
            u[l - 1] = max(u[l - 1], 0.0)
        u = _q(u, Qf, qu)

        # compute_rsk
        rsk[:] = (v + u - 2 * u_t) * diag_r
        rsk = _q(rsk, Qf, qrsk)

        if i % CONVERGED_INTERVAL == 0:
            if adaptive_scale:
                do_update_scale(i)

        # update_dual_vars
        v += ALPHA * (u - u_t)
        v = _q(v, Qf, qv)

        if i % interval == 0:
            aa.safeguard(v, v_prev)

        if track and i % track == 0:
            tau = u[l - 1]
            xx = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
            hist.append((i, np.linalg.norm(xx - x_ref) / np.linalg.norm(x_ref), float(c @ xx)))

    tau = u[l - 1]
    x = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
    relx = np.linalg.norm(x - x_ref) / np.linalg.norm(x_ref)
    return dict(x=x, tau=tau, relx=relx, obj=float(c @ x), hist=hist,
                n_scale_updates=n_scale_updates, scale=scale, converged_iter=i)


def SA_proj_dual_cone_r(x, r_y):
    """R_y-weighted Moreau (same as scs_adaptive, kept local to avoid import cycle)."""
    s = x.copy()
    x = -r_y * x
    x = proj_cone_local(x)
    return x / r_y + s


def proj_cone_local(y):
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":
            out[s:s + dim] = 0.0
        elif kind == "nonneg":
            out[s:s + dim] = np.maximum(y[s:s + dim], 0.0)
        elif kind == "soc":
            out[s:s + dim] = proj_soc(y[s:s + dim])
    return out


if __name__ == "__main__":
    import sys
    formats = [(18, 5), (24, 8), (24, 11), (32, 16), (32, 19), (24, 16)]
    max_iter = int(sys.argv[1]) if len(sys.argv) > 1 else 30000
    for (m, n_) in formats:
        Qf = Q(m, n_)
        t0 = time.time()
        r = solve_fixed_adaptive(max_iter, Qf, track=max_iter // 2)
        dt = time.time() - t0
        print(f"Q({m},{n_}): rel_x={r['relx']:.3e} obj={r['obj']:.6f} "
              f"tau={r['tau']:.5f} scale_upd={r['n_scale_updates']} "
              f"scale={r['scale']:.2e} iter={r['converged_iter']} [{dt:.0f}s]")
