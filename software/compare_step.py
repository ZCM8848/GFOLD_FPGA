#!/usr/bin/env python3
"""Compare my scs_adaptive port vs the scs package for the first ~90 iterations
(scale cannot update before RESCALING_MIN_ITERS=100). Isolates base-DRS bugs."""
import json, os, numpy as np
import scs
import scipy.sparse as sp
import scs_adaptive as SA

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV))
b = np.array(D["b"]); c = np.array(D["q"])

# ---- scs package, run to 90 iters (no scale update yet) ----
stgs = dict(max_iters=90, eps_abs=1e-4, eps_rel=1e-4,
            adaptive_scale=True, normalize=False, rho_x=1e-6,
            acceleration_lookback=10, verbose=False)
sol = scs.solve({"A": A, "b": b, "c": c},
                {"z": 706, "l": 301, "q": [4]*200 + [3]*100}, **stgs)
x_ref = np.array(D["x"])
xs = sol["x"] if isinstance(sol["x"], np.ndarray) else np.array(sol["x"])
print("scs pkg @90: iter=", sol["info"]["iter"], " obj=", float(xs@c),
      " rel_x=", np.linalg.norm(xs-x_ref)/np.linalg.norm(x_ref))

# ---- my port, manual stepping to 90 ----
n, m = NV, NR; l = n+m+1
scale = 1.0
diag_r = SA.build_diag_r(scale)
lu = SA.build_kkt(diag_r)
g = lu.solve(np.concatenate([c, -b]))
v = np.zeros(l); v[l-1]=1.0; v_prev=np.zeros(l)
u = np.zeros(l); u_t=np.zeros(l); rsk=np.zeros(l)
aa = SA.Anderson(l); sqrt_l = np.sqrt(l)
for i in range(90):
    if i>0 and i%10==0: aa.apply(v, v_prev)
    if i>=1:
        vn=np.linalg.norm(v)
        if vn>0: v*=sqrt_l/vn
    v_prev[:]=v
    u_t[:n]=v[:n]*diag_r[:n]
    u_t[n:l-1]=-v[n:l-1]*diag_r[n:l-1]
    u_t[l-1]=v[l-1]
    u_t[:l-1]=lu.solve(u_t[:l-1])
    if i<1: u_t[l-1]=1.0
    else: u_t[l-1]=SA.root_plus(diag_r, g, u_t[:l-1], v[:l-1], v[l-1])
    u_t[:l-1]-=u_t[l-1]*g
    u[:]=2*u_t-v
    u[n:l-1]=SA.proj_dual_cone_r(u[n:l-1], diag_r[n:l-1])
    if i<1: u[l-1]=1.0
    else: u[l-1]=max(u[l-1],0.0)
    rsk[:]=(v+u-2*u_t)*diag_r
    v+=SA.ALPHA*(u-u_t)
    if i%10==0: aa.safeguard(v, v_prev)
tau = u[l-1]; x = u[:n]/tau
print("my port @90: obj=", c@x, " rel_x=", np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref), " tau=", tau)
