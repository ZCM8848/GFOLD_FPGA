#!/usr/bin/env python3
"""
Phase 2b/2c: Block-Floating-Point model of the converging adaptive SCS solver.

Addresses the finding that single-format fixed-point needs ~40-48 bits for
rel_x~1e-3 (tau collapses ~1e-3 and mixes with pos ~2400). BFP gives each
physical block a shared power-of-2 exponent, so the mantissa needs only
moderate width. tau gets its own block (relative precision = 2^-(mant-2)).

Mirrors fixed_adaptive.py: KKT solve / root_plus / scale update stay float
(separate later phases); only the data-path state is quantized, here under BFP.
"""
import json, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from scs_adaptive import (Anderson, proj_soc, blocks, A, b, c, x_ref, obj_ref,
                          NV, NR, Z, build_diag_r, build_kkt, root_plus,
                          RHO_X, ALPHA, CONVERGED_INTERVAL, FEAS, DIV_EPS)
from bfp import make_state_blocks, bfp_q

TAU_FACTOR = 10.0
MIN_SCALE = 1e-6; MAX_SCALE = 1e6
RESCALING_MIN_ITERS = 100


def _full_blocks(n, m):
    bl = make_state_blocks(n)
    bl.append(np.arange(n, n + m))   # dual
    bl.append(np.array([n + m]))     # tau
    return bl


def _blocks_ntau(n, m):
    bl = make_state_blocks(n)
    bl.append(np.arange(n, n + m))   # dual only (no tau; for l-1 length slices)
    return bl


def solve_bfp(max_iter, mant, interval=10, track=None, eps=1e-4):
    n, m = NV, NR
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    nm_b = np.max(np.abs(b)); nm_c = np.max(np.abs(c))
    blocks_full = _full_blocks(n, m)
    blocks_ntau = _blocks_ntau(n, m)

    scale = 1.0
    last_scale_update_iter = 0
    sum_log_scale = 0.0; n_log_scale = 0
    diag_r = build_diag_r(scale)
    lu = build_kkt(diag_r)
    g = lu.solve(np.concatenate([c, -b]))

    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    u = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    aa = Anderson(l)
    hist = []; t0 = __import__("time").time(); n_scale_updates = 0

    def do_update_scale(i):
        nonlocal scale, sum_log_scale, n_log_scale, last_scale_update_iter, diag_r, lu, g, n_scale_updates
        tau = abs(u[l - 1])
        x_ = u[:n]; y_ = u[n:l - 1]; s_ = rsk[n:l - 1]
        ax = A @ x_
        asb = ax + s_ - tau * b
        aty = A.T @ y_
        pac = aty + tau * c
        rel_pri = max((np.max(np.abs(asb)) / max(np.max(np.abs(ax)), np.max(np.abs(s_)), nm_b * tau) or DIV_EPS), DIV_EPS)
        rel_dual = max((np.max(np.abs(pac)) / max(np.max(np.abs(aty)), nm_c * tau) or DIV_EPS), DIV_EPS)
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
            aa.reset()
            v[:] = bfp_q(rsk / diag_r + 2 * u_t - u, mant, blocks_full)

    for i in range(max_iter):
        if i > 0 and i % interval == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0:
                v *= sqrt_l / vn
            v = bfp_q(v, mant, blocks_full)
        v_prev[:] = v

        u_t[:n] = v[:n] * diag_r[:n]
        u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
        u_t[l - 1] = v[l - 1]
        u_t[:l - 1] = bfp_q(lu.solve(u_t[:l - 1]), mant, blocks_ntau)
        if i < FEAS:
            u_t[l - 1] = 1.0
        else:
            u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
        u_t[:l - 1] -= u_t[l - 1] * g
        u_t[:l - 1] = bfp_q(u_t[:l - 1], mant, blocks_ntau)

        u[:] = 2 * u_t - v
        u[n:l - 1] = bfp_q(_proj_dual_cone_r(u[n:l - 1], diag_r[n:l - 1]), mant,
                           [np.arange(0, m)])
        if i < FEAS:
            u[l - 1] = 1.0
        else:
            u[l - 1] = max(u[l - 1], 0.0)
        u = bfp_q(u, mant, blocks_full)

        rsk[:] = (v + u - 2 * u_t) * diag_r
        rsk = bfp_q(rsk, mant, blocks_full)

        if i % CONVERGED_INTERVAL == 0:
            if do_update_scale(i):
                pass

        v += ALPHA * (u - u_t)
        v = bfp_q(v, mant, blocks_full)

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


def _proj_dual_cone_r(x, r_y):
    s = x.copy()
    x = -r_y * x
    out = x.copy()
    for kind, st, dim in blocks:
        if kind == "zero":
            out[st:st + dim] = 0.0
        elif kind == "nonneg":
            out[st:st + dim] = np.maximum(x[st:st + dim], 0.0)
        elif kind == "soc":
            out[st:st + dim] = proj_soc(x[st:st + dim])
    return out / r_y + s


if __name__ == "__main__":
    import sys
    mant = int(sys.argv[1]) if len(sys.argv) > 1 else 24
    max_iter = int(sys.argv[2]) if len(sys.argv) > 2 else 20000
    r = solve_bfp(max_iter, mant, track=max_iter // 4)
    print(f"BFP mant={mant} @{max_iter}: rel_x={r['relx']:.4e} obj={r['obj']:.6f} "
          f"tau={r['tau']:.5f} scale={r['scale']:.2e} upd={r['n_scale_updates']}")
    for it, rx, o in r["hist"]:
        print(f"   {it:6d} rel_x={rx:.3e} obj={o:.6f}")
