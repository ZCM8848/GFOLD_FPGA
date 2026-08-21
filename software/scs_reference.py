#!/usr/bin/env python3
"""
numpy golden reference: SCS homogeneous self-dual embedding (DRS), diag_r=1,
alpha=1.5, normalize_v, NO Anderson acceleration, NO adaptive scale.

Mirrors reference/scs/src/scs.c core iteration. Validates against the `scs`
package and the Clarabel reference solution. This is the algorithm the Verilog
implements, so its arithmetic is the golden contract.

Solves  min c'x  s.t.  Ax + s = b,  s in K   (K = zero x nonneg x SOC).
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV)).tocsr()
b = np.array(D["b"]); c = np.array(D["q"]); x_ref = np.array(D["x"]); obj_ref = D["objective"]
blocks = []; r = 0
for cc in D["cones"]:
    blocks.append((cc["kind"], r, cc["dim"])); r += cc["dim"]

# ---- cone projections (dual cone of our K) ----
def proj_soc(v):
    t = v[0]; x = v[1:]; nx = np.linalg.norm(x)
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t)/2.0
    o = np.empty_like(v); o[0]=lam; o[1:]=(lam/nx)*x; return o

def proj_dual_cone(y):
    """y has length NR, ordered by cones. zero->free(id), nonneg->clamp, soc->soc."""
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    pass                      # dual = free
        elif kind == "nonneg": out[s:s+dim] = np.maximum(y[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(y[s:s+dim])
    return out

# ---- KKT matrix M = [[I, A^T],[A, -I]] and precomputed g = M^{-1}[c;-b] ----
M = sp.bmat([[sp.eye(NV), A.T], [A, -sp.eye(NR)]], format="csc")
lu = splu(M)
g = lu.solve(np.concatenate([c, -b]))

def root_plus(p, mu, eta):
    """p = u_t[:l-1] (post-solve), mu = v[:l-1], eta = v[l-1]; diag_r=1."""
    gg = np.dot(g, g)
    mug = np.dot(g, mu); pg = np.dot(p, g)
    pp = np.dot(p, p);   pmu = np.dot(p, mu)
    a = 1.0 + gg
    bv = mug - 2*pg - eta
    ccoef = pp - pmu
    rad = bv*bv - 4*a*ccoef
    if rad < 0: return -bv/(2*a)
    sq = np.sqrt(rad)
    if bv <= 0: return (-bv + sq)/(2*a)
    qq = -0.5*(bv + sq)
    return ccoef/qq if qq != 0 else 0.0

def solve_scs(max_iter, alpha=1.5, track=None):
    l = NV + NR + 1
    u = np.zeros(l); v = np.zeros(l); u_t = np.zeros(l)
    sqrt_l = np.sqrt(l)
    FEAS = 1
    hist = []
    t0 = time.time()
    for i in range(max_iter):
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0: v *= sqrt_l/vn
        # project_lin_sys
        u_t[:NV] = v[:NV]
        u_t[NV:l-1] = -v[NV:l-1]
        u_t[l-1] = v[l-1]
        u_t[:l-1] = lu.solve(u_t[:l-1])
        if i < FEAS: u_t[l-1] = 1.0
        else:        u_t[l-1] = root_plus(u_t[:l-1], v[:l-1], v[l-1])
        u_t[:l-1] += g * (-u_t[l-1])
        # project_cones
        u[:] = 2*u_t - v
        u[NV:l-1] = proj_dual_cone(u[NV:l-1])
        if i < FEAS: u[l-1] = 1.0
        else:        u[l-1] = max(u[l-1], 0.0)
        # update_dual_vars
        v += alpha*(u - u_t)
        if track and i % track == 0:
            tau = u[l-1]
            x = u[:NV]/tau if abs(tau) > 1e-12 else u[:NV]
            relx = np.linalg.norm(x - x_ref)/np.linalg.norm(x_ref)
            hist.append((i, relx, float(c@x)))
    x = u[:NV]/u[l-1] if abs(u[l-1]) > 1e-12 else u[:NV]
    return x, u[l-1], hist

if __name__ == "__main__":
    print(f"nvars={NV} nrows={NR} nnz(A)={D['nnz_a']}  KKT={NV+NR}x{NV+NR}")
    print(f"clarabel ref: obj={obj_ref:.6f}  x_norm={np.linalg.norm(x_ref):.3f}")
    x, tau, hist = solve_scs(50000, track=1000)
    relx = np.linalg.norm(x - x_ref)/np.linalg.norm(x_ref)
    print(f"\nSCS-DRS core: tau={tau:.5f}  obj={c@x:.6f} (ref {obj_ref:.6f})  rel_x={relx:.3e}")
    print("convergence trajectory (iter, rel_x, obj):")
    for it, rx, o in hist: print(f"  {it:6d}  rel_x={rx:.3e}  obj={o:.6f}")
    # final few positions
    print(f"\nx[0:6] = {x[:6].round(4)}")
    print(f"x[-3:] = {x[-3:].round(4)}  (z = log mass, last = obj)")
