"""Validate HW scale update: replay iter 0..5 (matching tb), then apply the
scale=0.178742 update EXACTLY as drs_iter does, dump the remapped v for
comparison against the tb 'VS:' line."""
import sys, os, struct
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
sys.path.insert(0, os.path.dirname(__file__))
from drs_reference import reorder_problem, build_diag_r, Anderson, one_iteration
from banded_reference import RHO_X

def f64(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]
def hexs(a, n=8):
    return [format(f64(float(x)), "016X") for x in np.asarray(a).ravel()[:n]]

def main():
    Ab, br, cr, rp, cp, blocks = reorder_problem()
    n, m = Ab.shape[1], Ab.shape[0]
    l = n + m + 1
    diag_r = build_diag_r(1.0, n, m, rp)
    lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                       [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
    g = lu.solve(np.concatenate([cr, -br]))
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = v.copy()
    aa = Anderson(l)
    # replay iter 0..5 (tb does 5 iters)
    for i in range(6):
        v, u_t, u, rsk = one_iteration(Ab, br, cr, blocks, 1.0, diag_r, lu,
                                       g, v, v_prev, i, aa)
    print("pre-scale v", hexs(v[:3]))
    print("iter5 rsk[1114,1135]", hexs(rsk[1114]), hexs(rsk[1135]))
    print("iter5 diag_r[1114,1135]", hexs(diag_r[1114]), hexs(diag_r[1135]))
    # ---- scale update (scale = 0.17874205598417373), mirror drs_iter ----
    scale = 0.17874205598417373
    diag_r = build_diag_r(scale, n, m, rp)
    lu = splu(sp.bmat([[sp.diags(diag_r[:n], format="csc"), Ab.T],
                       [Ab, sp.diags(-diag_r[n:n + m], format="csc")]], format="csc"))
    g = lu.solve(np.concatenate([cr, -br]))
    aa.reset()
    v[:] = rsk / diag_r + 2 * u_t - u
    # ---- keep iterating after the scale update (validate band/g path) ----
    # software v_prev must be the REMAPPED v (mirror drs_iter S_AARW->S_VR0)
    v_prev = v.copy()
    for i in range(6, 116):
        v, u_t, u, rsk = one_iteration(Ab, br, cr, blocks, scale, diag_r, lu,
                                       g, v, v_prev, i, aa, safeguard=True)
        with open(f"rtl/data/kkt/full/v_scale{i + 1}.hex", "w") as fh:
            for x in v: fh.write("%016X\n" % f64(float(x)))
    print("post-scale iters 6..115 done; wrote v_scale7..v_scale116 (first real safeguard at iter110)")
    print("post-scale v", hexs(v[:6]))
    # dump full v for tb comparison
    with open("rtl/data/kkt/full/v_scale0.hex", "w") as fh:
        for x in v: fh.write("%016X\n" % f64(float(x)))
    print("wrote v_scale0.hex; new diag_r[0:3]=", hexs(diag_r[:3]), " new g[0:3]=", hexs(g[:3]))

main()
