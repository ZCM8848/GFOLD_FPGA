"""Replay iter 0..10 via drs_reference.one_iteration (with Anderson), dumping
intermediates at iter 10."""
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
import struct

from drs_reference import (build_diag_r, reorder_problem,
                           Anderson, one_iteration)

def hexs(a, n=4):
    return [struct.pack('>d', float(x)).hex() for x in np.asarray(a).ravel()[:n]]

def main():
    Ab, br, cr, rp, cp, blocks = reorder_problem()
    n, m = Ab.shape[1], Ab.shape[0]
    l = n + m + 1
    scale = 1.0
    diag_r = build_diag_r(scale, n, m, rp)
    lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                       [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
    g = lu.solve(np.concatenate([cr, -br]))
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = v.copy()
    aa = Anderson(l)
    for i in range(0, 11):
        if i == 10:
            print(f"iter10 PRE-AA v={hexs(v)} v_prev={hexs(v_prev)}")
        v, u_t, u, rsk = one_iteration(Ab, br, cr, blocks, scale, diag_r, lu,
                                       g, v, v_prev, i, aa)
        if i == 10:
            print(f"iter10 POST v={hexs(v)} n_reject={aa.n_reject} norm_g={aa.norm_g}")
            print(f"iter10 v_prev={hexs(v_prev)}")
    print("done")

main()
