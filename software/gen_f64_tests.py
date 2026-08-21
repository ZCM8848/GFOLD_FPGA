#!/usr/bin/env python3
"""Generate FP64 banded-LDL stimulus for banded_ldl_fp64.v.
Small: 8x8 hb=2 random SPD (mirrors gen_small_test).
Full : 1100x1100 hb=17 S0 band at scale=1 (mirrors gen_full_test).
Writes band_f64.hex / rhs_f64.hex / exp_zx_f64.hex (IEEE-754 double, 16 hex)."""
import os, struct, numpy as np
import scipy.sparse as sp
from banded_reference import build, banded_ldl_factor_solve, RHO_X
from assembler import assemble

HERE = os.path.dirname(os.path.abspath(__file__))
RTLD = os.path.join(HERE, "..", "rtl", "data")

def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]

def write(f, vals):
    with open(f, "w") as fh:
        for v in vals: fh.write(f"{f64(v):016X}\n")

def small(scale_hb=2, n=8):
    out = os.path.join(RTLD, "small_f64"); os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(7)
    B = np.zeros((scale_hb + 1, n))
    for k in range(n):
        for i in range(1, scale_hb + 1):
            if k + i < n: B[i, k] = rng.uniform(-1, 1)
    S = np.zeros((n, n))
    for k in range(n):
        S[k, k] = 5.0
        for i in range(1, scale_hb + 1):
            if k + i < n:
                S[k + i, k] = B[i, k]; S[k, k + i] = B[i, k]
    S += np.eye(n) * 0.5
    for k in range(n): B[0, k] = S[k, k]
    rhs = rng.uniform(-1, 1, n)
    zx = np.linalg.solve(S, rhs)
    zx2 = banded_ldl_factor_solve(sp.csc_matrix(S), rhs, hb=scale_hb)
    print(f"small: banded vs dense err={np.linalg.norm(zx-zx2)/np.linalg.norm(zx):.3e}")
    band = np.array([B[i, k] for k in range(n) for i in range(scale_hb + 1)])
    write(os.path.join(out, "band_f64.hex"), band)
    write(os.path.join(out, "rhs_f64.hex"), rhs)
    write(os.path.join(out, "exp_zx_f64.hex"), zx2)
    print(f"small_f64: N={n} HB={scale_hb} -> {out}")

def full(tof=46.6093):
    out = os.path.join(RTLD, "full_f64"); os.makedirs(out, exist_ok=True)
    A, b, q, m, n = assemble(tof)
    bl = build(A, b, q); S0 = bl['S'].toarray()
    Ss = 1.0 * S0; Ss[np.diag_indices(n)] += RHO_X
    hb = 17
    rng = np.random.default_rng(42)
    v_xr = rng.standard_normal(n); v_yr = rng.standard_normal(m)
    rhs = RHO_X * v_xr - bl['Ab'].T @ v_yr
    zx = banded_ldl_factor_solve(sp.csc_matrix(Ss), rhs, hb=hb)
    resid = np.linalg.norm(Ss @ zx - rhs) / np.linalg.norm(rhs)
    band = np.array([Ss[k + i, k] if k + i < n else 0.0
                     for k in range(n) for i in range(hb + 1)])
    write(os.path.join(out, "band_f64.hex"), band)
    write(os.path.join(out, "rhs_f64.hex"), rhs)
    write(os.path.join(out, "exp_zx_f64.hex"), zx)
    print(f"full_f64: N={n} HB={hb} resid={resid:.2e} |zx|max={np.abs(zx).max():.3e} -> {out}")

if __name__ == "__main__":
    small(); full()
