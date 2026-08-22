#!/usr/bin/env python3
"""Generate KKT-solve (Phase 1) stimulus for the hardware kkt_solve datapath.

For a given config produces (into rtl/data/kkt/<case>/):
  Arow.hex Acol.hex Aval.hex   sparse Ab COO (NNZ entries; row/col as hex ints, val FP64)
  r_y.hex Dy.hex               (M) scale=1: zero cone 1/1000, else 1; D_y=1/r_y
  vx.hex (N) vy.hex (M)        random iterate (reordered frame)
  band_f64.hex                 Schur S = rho_x*I + A^T D_y A band (for banded_ldl_fp64_rt)
  zx.hex (N) zy.hex (M)        expected KKT-solve output (block-elim, banded LDL)
  manifest.json

Synthetic 'small' A is a node-structured banded matrix so A^T D_y A stays banded
(hb small) and the LDL solves it. 'full' uses the real G-FOLD Ab (hb=17).
Reference for zx/zy: banded_reference.kkt_solve (full numpy splu KKT).
"""
import os, json, struct
import numpy as np
import scipy.sparse as sp
from banded_reference import build, banded_ldl_factor_solve, RHO_X, Z, SCALE
from assembler import assemble

HERE = os.path.dirname(os.path.abspath(__file__))
RTLD = os.path.join(HERE, "..", "rtl", "data", "kkt")

def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]

def write(f, vals, fmt="%016X"):
    with open(f, "w") as fh:
        for v in vals: fh.write((fmt % v) + "\n")

def synth_small(n=10, m=20, hb=2, seed=0):
    """Node-structured banded sparse A so S=A^T D_y A is banded with half-band ~2*hb."""
    rng = np.random.default_rng(seed)
    rows, cols, vals = [], [], []
    for r in range(m):
        # each row touches 2-3 cols clustered around col~r*(n/m)*... ; keep banded
        base = min(n - 1, max(0, int(r * n / m)))
        for c in range(max(0, base - hb), min(n, base + hb + 1)):
            if rng.random() < 0.45:
                rows.append(r); cols.append(c); vals.append(rng.uniform(-1, 1))
    A = sp.coo_matrix((vals, (rows, cols)), shape=(m, n)).tocsr()
    return A

def gen(case, A, hb, n, m, seed=42):
    out = os.path.join(RTLD, case); os.makedirs(out, exist_ok=True)
    # r_y (scale=1): zero cone 1/(1000*scale), others 1/scale ; D_y = 1/r_y
    r_y = np.empty(m); r_y[:Z] = 1.0 / (1000.0 * SCALE); r_y[Z:] = 1.0 / SCALE
    r_y = r_y[:m]
    Dy = 1.0 / r_y
    A = A.tocoo()
    nnz = A.nnz
    # Schur S = rho_x I + A^T D_y A, then band (HB+1)xN
    S = RHO_X * sp.eye(n) + A.T @ sp.diags(Dy) @ A
    S = (S + S.T) * 0.5
    S = S.tocsc()
    bw = int(np.max(np.abs(S.tocoo().row - S.tocoo().col))) if S.nnz else 0
    assert bw <= hb, f"S bandwidth {bw} > hb {hb}"
    band = np.array([S[k + i, k] if k + i < n else 0.0
                     for k in range(n) for i in range(hb + 1)])
    # random iterate
    rng = np.random.default_rng(seed)
    v_x = rng.standard_normal(n); v_y = rng.standard_normal(m)
    # expected via full KKT solve (banded_reference path)
    rhs_x = RHO_X * v_x - A.T @ v_y
    zx = banded_ldl_factor_solve(S, rhs_x, hb=hb)
    zy = Dy * (A @ zx + r_y * v_y)
    resid = np.linalg.norm(S @ zx - rhs_x) / (np.linalg.norm(rhs_x) + 1e-30)
    # write COO (row/col as ints, val FP64)
    write(os.path.join(out, "Arow.hex"), A.row, "%X")
    write(os.path.join(out, "Acol.hex"), A.col, "%X")
    write(os.path.join(out, "Aval.hex"), [f64(v) for v in A.data])
    write(os.path.join(out, "r_y.hex"), [f64(v) for v in r_y])
    write(os.path.join(out, "Dy.hex"),  [f64(v) for v in Dy])
    write(os.path.join(out, "vx.hex"),  [f64(v) for v in v_x])
    write(os.path.join(out, "vy.hex"),  [f64(v) for v in v_y])
    write(os.path.join(out, "band_f64.hex"), [f64(v) for v in band])
    write(os.path.join(out, "zx.hex"),  [f64(v) for v in zx])
    write(os.path.join(out, "zy.hex"),  [f64(v) for v in zy])
    json.dump(dict(n=n, m=m, hb=hb, nnz=nnz, s_bw=bw, resid=float(resid)),
              open(os.path.join(out, "manifest.json"), "w"), indent=2)
    print(f"[{case}] n={n} m={m} nnz={nnz} hb={hb} S_bw={bw} resid={resid:.2e} "
          f"|zy|max={np.abs(zy).max():.3e} |zx|max={np.abs(zx).max():.3e} -> {out}")

if __name__ == "__main__":
    gen("small", synth_small(), 4, 10, 20, seed=0)
    A, b, q, m, n = assemble(46.6093)
    Ab = build(A, b, q)["Ab"]
    gen("full", Ab, 17, n, m, seed=42)
    print("done")
