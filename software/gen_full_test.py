#!/usr/bin/env python3
"""Generate the FULL-SIZE (1100x1100, hb=17) banded-LDL RTL stimulus.

For each scale: S = rho_x*I + scale*S0 (S0 fixed ROM band from gen_band_data).
Writes band_{s}.hex / rhs_{s}.hex / exp_zx_{s}.hex (float32, B[k*(HB+1)+i]
layout) into rtl/data/full/, plus a manifest of the scales generated.

This exercises the real Schur-complement band structure after node reordering
and the scale collapse (scale=1e-4 -> zx ~ 2.2e4).
"""
import os, json, struct, numpy as np
import scipy.sparse as sp
from banded_reference import build, banded_ldl_factor_solve, RHO_X, bandwidth
from assembler import assemble, N

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "rtl", "data", "full")
TOF = 46.6093
HB = 17
SCALES = [1.0, 0.1, 1e-4]

def f32(v): return struct.unpack("<I", struct.pack("<f", float(v)))[0]

def main():
    os.makedirs(OUT, exist_ok=True)
    A, b, q, m, n = assemble(TOF)
    assert n == 1100, n
    bl = build(A, b, q)
    S0 = bl["S"].toarray()
    Ab = bl["Ab"]
    bw = bandwidth(bl["S"])
    print(f"A: {m}x{n}  S0 bandwidth={bw} (expect 17)  diag "
          f"{S0.diagonal().min():.2f}..{S0.diagonal().max():.2f}")

    rng = np.random.default_rng(42)
    manifest = {}
    for scale in SCALES:
        key = f"s{scale:.0e}".replace("e-", "e") if scale == 1e-4 else f"s{scale:g}"
        Ss = scale * S0
        Ss[np.diag_indices(n)] += RHO_X
        # extract band B[i][k] = Ss[k+i][k], column-major addr=k*(HB+1)+i
        band = np.zeros((HB + 1, n))
        for k in range(n):
            for i in range(HB + 1):
                if k + i < n:
                    band[i, k] = Ss[k + i, k]
        # rhs from a random iterate (match gen_band_data seed/construction)
        v_xr = rng.standard_normal(n)
        v_yr = rng.standard_normal(m)
        rhs = RHO_X * v_xr - Ab.T @ v_yr
        zx = banded_ldl_factor_solve(sp.csc_matrix(Ss), rhs, hb=HB)
        # residual sanity
        resid = np.linalg.norm(Ss @ zx - rhs) / np.linalg.norm(rhs)
        print(f"scale={scale:g}: |zx|max={np.max(np.abs(zx)):.3e} "
              f"band max={np.max(np.abs(band)):.3e} resid={resid:.2e}")
        with open(os.path.join(OUT, f"band_{key}.hex"), "w") as f:
            for k in range(n):
                for i in range(HB + 1):
                    f.write(f"{f32(band[i, k]):08X}\n")
        with open(os.path.join(OUT, f"rhs_{key}.hex"), "w") as f:
            for k in range(n): f.write(f"{f32(rhs[k]):08X}\n")
        with open(os.path.join(OUT, f"exp_zx_{key}.hex"), "w") as f:
            for k in range(n): f.write(f"{f32(zx[k]):08X}\n")
        manifest[key] = dict(scale=scale, n=n, hb=HB,
                             words=(HB + 1) * n, resid=float(resid))
    with open(os.path.join(OUT, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nwrote {OUT}/band/rhs/exp_zx_{'{s1,s0_1,s1e-4}'}.hex (N={n} HB={HB}, "
          f"{(HB+1)*n} words each), manifest.json")

if __name__ == "__main__":
    main()
