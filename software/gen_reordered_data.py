#!/usr/bin/env python3
"""Generate REORDERED r_y/Dy/S-band for the DRS (Phase 4b) hardware path.
The Phase 1-3 data (r_y.hex/Dy.hex/band_f64.hex) used the UNREORDERED zero-cone
layout (first Z rows), which is self-consistent but mathematically wrong for
SCS on the node-major problem. DRS needs r_y permuted by rp.
Writes rtl/data/kkt/full/{r_y_r.hex, Dy_r.hex, band_r.hex}.
"""
import sys, os, struct
import numpy as np
import scipy.sparse as sp
sys.path.insert(0, os.path.dirname(__file__))
from drs_reference import reorder_problem, build_diag_r
from banded_reference import RHO_X

def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]
def write(f, vals):
    with open(f, "w") as fh:
        for v in vals: fh.write("%016X\n" % f64(v))

n, m, hb = 1100, 2107, 17
Ab, br, cr, rp, cp, blocks = reorder_problem()
diag_r = build_diag_r(1.0, n, m, rp)
ry = diag_r[n:n + m]
Dy = 1.0 / ry
S = RHO_X * sp.eye(n) + (Ab.T @ sp.diags(Dy) @ Ab)
S = (S + S.T) * 0.5
S = S.tocsc()
bw = int(np.max(np.abs(S.tocoo().row - S.tocoo().col))) if S.nnz else 0
assert bw <= hb, f"bandwidth {bw} > hb"
# same layout as gen_kkt_test: flat[col*(HB+1)+diag] = S[col+diag][col]
band = np.array([S[k + i, k] if k + i < n else 0.0
                 for k in range(n) for i in range(hb + 1)])
out = "rtl/data/kkt/full"
write(os.path.join(out, "r_y_r.hex"), ry)
write(os.path.join(out, "Dy_r.hex"), Dy)
write(os.path.join(out, "band_r.hex"), band)
print("wrote r_y_r/Dy_r/band_r (bw=%d) nnz=%d" % (bw, S.nnz))
