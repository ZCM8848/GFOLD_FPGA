#!/usr/bin/env python3
"""Sensitivity: does the BFP solver tolerate KKT-solve noise at the wide-float-32
level? The Stage-B wide-float-32 banded LDL introduces rel(zx)~1e-3 (vs float64).
Inject multiplicative noise of that magnitude (and the float32-level 2.8e-2) into
u_t[:l-1] after the KKT solve and compare final rel_x vs the noise-free baseline.
Fast, decisive: if 1e-3 noise barely moves rel_x, wide-float-32 LDL is sufficient.
"""
import numpy as np
from fixed_bfp import solve_bfp

def solve_bfp_noisy(max_iter, mant, noise_level):
    """Same as solve_bfp but adds rel noise to the KKT-solve output each iter."""
    from scs_adaptive import Anderson, blocks, A, b, c, x_ref, obj_ref, NV, NR, Z
    from scs_adaptive import build_diag_r, build_kkt, root_plus, RHO_X, ALPHA
    from scs_adaptive import CONVERGED_INTERVAL, FEAS, DIV_EPS
    from bfp import make_state_blocks, bfp_q
    n, m = NV, NR
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    nm_b = np.max(np.abs(b)); nm_c = np.max(np.abs(c))
    blf = make_state_blocks(n) + [np.arange(n, n + m), np.array([n + m])]
    bln = make_state_blocks(n) + [np.arange(n, n + m)]
    scale = 1.0
    diag_r = build_diag_r(scale)
    lu = build_kkt(diag_r)
    g = lu.solve(np.concatenate([c, -b]))
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l); u = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    aa = Anderson(l)
    from soc_precision import make_proj_dual_cone_q
    proj_dual = make_proj_dual_cone_q(blocks, None, None, True)
    rng = np.random.default_rng(123)
    hist = []
    for i in range(max_iter):
        if i > 0 and i % 10 == 0: aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0: v *= sqrt_l / vn
            v = bfp_q(v, mant, blf)
        v_prev[:] = v
        u_t[:n] = v[:n] * diag_r[:n]
        u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
        u_t[l - 1] = v[l - 1]
        u_t[:l - 1] = bfp_q(lu.solve(u_t[:l - 1]), mant, bln)
        # inject KKT-solve output noise (magnitude of wide-float-32 LDL error)
        u_t[:l - 1] *= (1.0 + noise_level * rng.standard_normal(l - 1))
        u_t[:l - 1] = bfp_q(u_t[:l - 1], mant, bln)
        if i < FEAS: u_t[l - 1] = 1.0
        else: u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
        u_t[:l - 1] -= u_t[l - 1] * g
        u_t[:l - 1] = bfp_q(u_t[:l - 1], mant, bln)
        u[:] = 2 * u_t - v
        u[n:l - 1] = bfp_q(proj_dual(u[n:l - 1], diag_r[n:l - 1]), mant, [np.arange(0, m)])
        if i < FEAS: u[l - 1] = 1.0
        else: u[l - 1] = max(u[l - 1], 0.0)
        u = bfp_q(u, mant, blf)
        rsk[:] = (v + u - 2 * u_t) * diag_r
        rsk = bfp_q(rsk, mant, blf)
        v += ALPHA * (u - u_t)
        v = bfp_q(v, mant, blf)
        aa.safeguard(v, v_prev)
        if i % 2000 == 0:
            tau = u[l - 1]; xx = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
            hist.append((i, np.linalg.norm(xx - x_ref) / np.linalg.norm(x_ref)))
    tau = u[l - 1]; x = u[:n] / tau if abs(tau) > 1e-12 else u[:n]
    return np.linalg.norm(x - x_ref) / np.linalg.norm(x_ref), hist

if __name__ == "__main__":
    max_iter = 8000
    for mant in [32]:
        print(f"== BFP state mant={mant}, {max_iter} iters ==")
        rx0, h0 = solve_bfp_noisy(max_iter, mant, 0.0)
        print(f"  noise=0      (float64 LDL):  rel_x={rx0:.4e}")
        for nl, tag in [(1e-3, "wide-f32-32 (1e-3)"), (3e-3, "3e-3"), (2.8e-2, "float32 (2.8e-2)")]:
            rx, h = solve_bfp_noisy(max_iter, mant, nl)
            print(f"  noise={nl:g} ({tag}): rel_x={rx:.4e}  ratio={rx/rx0:.2f}")
