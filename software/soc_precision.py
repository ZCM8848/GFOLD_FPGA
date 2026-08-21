#!/usr/bin/env python3
"""
Phase 2c: SOC projection sqrt/div precision requirement.

proj_soc needs nx = sqrt(sum(x^2)) and the ratio lam/nx. Cyclone IV has no hard
sqrt/div. This experiment quantizes the sqrt OUTPUT and the DIVISION OUTPUT to a
controllable relative precision (p mantissa bits) inside the BFP solver, and finds
how many bits each needs so the solver still reaches rel_x ~1e-3.

Relates directly to hardware cost: CORDIC gives ~1 bit/iteration, Newton-Raphson
gives ~2x bits/iteration; the required p tells us the iteration budget.
"""
import numpy as np
from fixed_point import Q
from bfp import bfp_q


def _qrel(x, p):
    """Quantize a scalar/array to p-bit mantissa relative precision (like BFP
    single block): exp=floor(log2(max|x|)), mant in Q(p, p-2) range [1,2)."""
    x = np.asarray(x, float)
    out = x.copy()
    mx = np.max(np.abs(out)) if out.size else 0.0
    if mx == 0:
        return out
    exp = np.floor(np.log2(mx))
    qm = Q(p, p - 2)
    return qm.q(out / (2.0 ** exp)) * (2.0 ** exp)


def proj_soc_p(v, sqrt_p, div_p, nx_exact=True):
    """SOC projection with sqrt/div outputs quantized to sqrt_p/div_p bits.
    nx_exact: if True, compute nx exactly (float) then quantize the OUTPUT;
    models a sqrt block of that output precision. v: np.array([t, x...])."""
    t = v[0]; x = v[1:]
    s2 = np.dot(x, x)
    nx = np.sqrt(s2)
    if sqrt_p is not None:
        nx = _qrel(nx, sqrt_p)
    if nx <= t:
        return v.copy()
    if nx <= -t:
        return np.zeros_like(v)
    lam = (nx + t) / 2.0
    r = lam / nx
    if div_p is not None:
        r = _qrel(r, div_p)
    o = np.empty_like(v)
    o[0] = lam
    o[1:] = r * x
    return o


# ---- build the quantized cone projection (dual part) for solve_bfp ----
def make_proj_dual_cone_q(blocks, sqrt_p, div_p, nx_exact=True):
    def proj(x, r_y):
        s = x.copy()
        x = -r_y * x
        out = x.copy()
        for kind, st, dim in blocks:
            if kind == "zero":
                out[st:st + dim] = 0.0
            elif kind == "nonneg":
                out[st:st + dim] = np.maximum(x[st:st + dim], 0.0)
            elif kind == "soc":
                out[st:st + dim] = proj_soc_p(x[st:st + dim], sqrt_p, div_p, nx_exact)
        return out / r_y + s
    return proj
