#!/usr/bin/env python3
"""
Generate the banded-LDL RTL stimulus: S0 band ROM (.mif/.hex) + test vectors.

S = rho_x*I + scale*S0, S0 = A^T*D_y0*A (D_y0 = D_y at scale=1, reordered frame).
The RTL stores S0's band in ROM (address = k*(hb+1)+i for B[i,k]=S[k+i,k]), and at
a scale change forms S = scale*S0 (+rho_x on diagonal) then LDL-factorizes.

Outputs (canonical float32 IEEE-754):
  rtl/data/S0_band_f32.mif/.hex   S0 band, 18x1100, column-major (k*(hb+1)+i)
  rtl/data/test_vectors.json      [{scale, rhs:[n], zx:[n], zy:[m]}, ...]
Test vectors generated via the VALIDATED banded LDL reference, at several scales.
"""
import json, os, struct, numpy as np
from banded_reference import build, banded_ldl_factor_solve, RHO_X, SCALE, Z
from assembler import assemble, N

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "rtl", "data")
TOF = 46.6093
HB = 17

def f32(v):
    return struct.unpack("<I", struct.pack("<f", float(v)))[0]

def main():
    os.makedirs(OUT, exist_ok=True)
    A, b, q, m, n = assemble(TOF)
    bl = build(A, b, q)
    S0 = bl["S"]                       # Schur complement at scale=1 (reordered frame)
    S0 = S0.toarray()
    # ---- S0 band B[i,k]=S[k+i,k], column-major addr=k*(hb+1)+i ----
    band = np.zeros((HB + 1, n))
    for k in range(n):
        for i in range(HB + 1):
            if k + i < n:
                band[i, k] = S0[k + i, k]
    # write .mif (Quartus) and .hex (readmemh)
    nwords = (HB + 1) * n
    with open(os.path.join(OUT, "S0_band_f32.mif"), "w") as f:
        f.write(f"DEPTH = {nwords};\nWIDTH = 32;\nADDRESS_RADIX = DEC;\nDATA_RADIX = HEX;\nCONTENT\nBEGIN\n")
        addr = 0
        for k in range(n):
            for i in range(HB + 1):
                f.write(f"  {addr} : {f32(band[i, k]):08X};\n")
                addr += 1
        f.write("END;\n")
    with open(os.path.join(OUT, "S0_band_f32.hex"), "w") as f:
        addr = 0
        for k in range(n):
            for i in range(HB + 1):
                f.write(f"{f32(band[i, k]):08X}\n")
                addr += 1
    print(f"S0 band: {HB+1}x{n} = {nwords} words -> {os.path.join(OUT,'S0_band_f32.mif/.hex')}")
    print(f"  band min={band[np.abs(band)>0].min():.4e} max={np.abs(band).max():.4e}")

    # ---- test vectors at several scales ----
    rng = np.random.default_rng(42)
    Dyb = bl["Dyb"]; r_yb = bl["r_yb"]; Ab = bl["Ab"]
    tests = []
    for scale in [1.0, 0.1, 1e-4]:
        # S for this scale: diag=scale*S0_diag + rho_x, offdiag=scale*S0_offdiag
        Ss = scale * S0
        Ss[np.diag_indices(n)] += RHO_X
        for t in range(2):
            v_xr = rng.standard_normal(n)
            v_yr = rng.standard_normal(m)
            rhs = RHO_X * v_xr - Ab.T @ v_yr
            zx = banded_ldl_factor_solve(sp_mk(Ss), rhs, hb=HB)
            zy = Dyb * (Ab @ zx + r_yb * v_yr)
            tests.append(dict(scale=float(scale),
                              rhs=rhs.tolist(), zx=zx.tolist(), zy=zy.tolist()))
    with open(os.path.join(OUT, "test_vectors.json"), "w") as f:
        json.dump(tests, f)
    print(f"test vectors: {len(tests)} cases (scale in {[1.0,0.1,1e-4]} x 2) -> test_vectors.json")
    for tv in tests:
        print(f"  scale={tv['scale']:.0e}: |rhs|max={np.max(np.abs(tv['rhs'])):.3e} "
              f"|zx|max={np.max(np.abs(tv['zx'])):.3e} |zy|max={np.max(np.abs(tv['zy'])):.3e}")

def sp_mk(Sd):
    import scipy.sparse as sp
    return sp.csc_matrix(Sd)

if __name__ == "__main__":
    main()
