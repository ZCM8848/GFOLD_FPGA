#!/usr/bin/env python3
"""Generate root_plus L=3207 test vectors from the REAL iter-1 DRS state
(reordered problem). rp_l.hex: L lines "r g p mu" + 1 line "eta tau".
"""
import sys, struct, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
sys.path.insert(0, os.path.dirname(__file__))
from drs_reference import reorder_problem, build_diag_r, root_plus
from scs_faithful import NV

def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]
def wh(f, v): open(f, "w").write("\n".join("%016X" % f64(x) for x in v) + "\n")

n, m = NV, 2107
Ab, br, cr, rp, cp, blocks = reorder_problem()
diag_r = build_diag_r(1.0, n, m, rp)
lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                   [Ab, sp.diags(-diag_r[n:n+m], format="csc")]], format="csc"))
g = lu.solve(np.concatenate([cr, -br]))
v1 = np.array([dec(l) for l in open("rtl/data/kkt/full/v1.hex")], dtype=float)
l = n + m + 1
v = v1.copy()
v *= np.sqrt(l) / np.linalg.norm(v)
u_t = np.zeros(l)
u_t[:n] = v[:n] * diag_r[:n]
u_t[n:l-1] = -v[n:l-1] * diag_r[n:l-1]
u_t[l-1] = v[l-1]
u_t[:l-1] = lu.solve(u_t[:l-1])
tau = root_plus(diag_r, g, u_t[:l-1], v[:l-1], v[l-1])
print("sw tau =", tau)
lm1 = l - 1
out = "rtl/data/rp_l3207.hex"
with open(out, "w") as f:
    for i in range(lm1):
        f.write("%016X %016X %016X %016X\n" % (f64(diag_r[i]), f64(g[i]), f64(u_t[i]), f64(v[i])))
    f.write("%016X %016X\n" % (f64(v[l-1]), f64(tau)))
print("wrote", out, "lines:", lm1 + 1)
