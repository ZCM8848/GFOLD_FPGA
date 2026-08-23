"""Compute SW band + g after a scale update (scale=0.178742) for HW comparison."""
import sys, os, struct
import numpy as np
import scipy.sparse as sp
sys.path.insert(0, os.path.dirname(__file__))
from drs_reference import reorder_problem, build_diag_r
from banded_reference import RHO_X

def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]
def hexs(a, n=8):
    return [format(f64(float(x)), "016X") for x in np.asarray(a).ravel()[:n]]

Ab, br, cr, rp, cp, blocks = reorder_problem()
n, m = Ab.shape[1], Ab.shape[0]
scale = 0.17874205598417373
diag_r = build_diag_r(scale, n, m, rp)
Dy = 1.0 / diag_r[n:n + m]
S = RHO_X * sp.eye(n) + (Ab.T @ sp.diags(Dy) @ Ab)
S = (S + S.T) * 0.5
S = S.tocsc()
hb = 17
# flat[col*(HB+1)+diag] = S[col+diag][col]
band = np.array([S[k + i, k] if k + i < n else 0.0
                 for k in range(n) for i in range(hb + 1)])
print("SW band[0,1,2]", hexs(band[:3]))
print("SW band[1100col diag]", hexs([band[1100*(hb+1)+0]]))
# g = KKT^{-1}[c;-b] with new diag_r
from scipy.sparse.linalg import splu
lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                   [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
g = lu.solve(np.concatenate([cr, -br]))
print("SW g[0,1,2]", hexs(g[:3]))
