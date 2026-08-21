#!/usr/bin/env python3
"""Generate a small SPD banded test for the Stage-A banded-LDL RTL.
Writes band.hex, rhs.hex, exp_zx.hex (float32) into rtl/data/small/."""
import os, struct, numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "data", "small")
N, HB = 8, 2

def f32(v): return struct.unpack("<I", struct.pack("<f", float(v)))[0]

def main():
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(7)
    # random banded symmetric PD: start with random lower band, force diag-dominant
    B = np.zeros((HB + 1, N))
    for k in range(N):
        for i in range(1, HB + 1):
            if k + i < N:
                B[i, k] = rng.uniform(-1, 1)
        # symmetric: B[i][k] = S[k+i][k], and S[k][k+i] = B[i][k] too
    # build full symmetric S from lower band
    S = np.zeros((N, N))
    for k in range(N):
        S[k, k] = 5.0
        for i in range(1, HB + 1):
            if k + i < N:
                S[k + i, k] = B[i, k]
                S[k, k + i] = B[i, k]
    # ensure PD
    S += np.eye(N) * 0.5
    # re-extract band (after the +0.5 diag)
    for k in range(N):
        B[0, k] = S[k, k]
    # verify PD
    e = np.linalg.eigvalsh(S)
    assert e.min() > 0, f"not PD: {e.min()}"
    rhs = rng.uniform(-1, 1, N)
    zx = np.linalg.solve(S, rhs)
    # sanity: banded LDL solve matches
    from banded_reference import banded_ldl_factor_solve
    import scipy.sparse as sp
    zx2 = banded_ldl_factor_solve(sp.csc_matrix(S), rhs, hb=HB)
    print(f"banded vs dense solve err = {np.linalg.norm(zx-zx2)/np.linalg.norm(zx):.3e}")
    print(f"diag range = {B[0,:].min():.4f}..{B[0,:].max():.4f}, eig_min={e.min():.4f}")

    with open(os.path.join(OUT, "band.hex"), "w") as f:
        for k in range(N):
            for i in range(HB + 1):
                f.write(f"{f32(B[i, k]):08X}\n")
    with open(os.path.join(OUT, "rhs.hex"), "w") as f:
        for k in range(N): f.write(f"{f32(rhs[k]):08X}\n")
    with open(os.path.join(OUT, "exp_zx.hex"), "w") as f:
        for k in range(N): f.write(f"{f32(zx[k]):08X}\n")
    print(f"wrote {OUT}/band.hex, rhs.hex, exp_zx.hex (N={N} HB={HB})")

if __name__ == "__main__":
    main()
