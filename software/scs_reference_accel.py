#!/usr/bin/env python3
"""
Accelerated golden reference: SCS homogeneous-embedding DRS core +
textbook Anderson Type-II acceleration (small memory, normal-equations LS).

The DRS fixed-point map is one SCS iteration (v -> v + alpha*(u - u_t)).
Anderson extrapolates recent (iterate, residual) pairs to accelerate.

Target: rel_x ~1e-3 vs the Clarabel reference, in reasonable iterations.
This is the algorithm blueprint for the FPGA (banded KKT solve + Anderson LS
+ cone projections, all small/fixed-size).
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

def proj_soc(v):
    t = v[0]; x = v[1:]; nx = np.linalg.norm(x)
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t)/2.0
    o = np.empty_like(v); o[0]=lam; o[1:]=(lam/nx)*x; return o

def proj_dual_cone(y):
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    pass
        elif kind == "nonneg": out[s:s+dim] = np.maximum(y[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(y[s:s+dim])
    return out

M = sp.bmat([[sp.eye(NV), A.T], [A, -sp.eye(NR)]], format="csc")
lu = splu(M)
g = lu.solve(np.concatenate([c, -b]))

def root_plus(p, mu, eta):
    gg = np.dot(g,g); mug=np.dot(g,mu); pg=np.dot(p,g)
    pp=np.dot(p,p); pmu=np.dot(p,mu)
    a=1.0+gg; bv=mug-2*pg-eta; cc=pp-pmu
    rad=bv*bv-4*a*cc
    if rad<0: return -bv/(2*a)
    sq=np.sqrt(rad)
    if bv<=0: return (-bv+sq)/(2*a)
    qq=-0.5*(bv+sq); return cc/qq if qq!=0 else 0.0

def solve(max_iter, alpha=1.5, aa_mem=0, aa_every=1, aa_r=1e-8, track=None):
    l = NV+NR+1
    u = np.zeros(l); v = np.zeros(l); u_t = np.zeros(l)
    sqrt_l = np.sqrt(l); FEAS=1
    # Anderson history
    hist_x=[]; hist_f=[]
    hist=[]; t0=time.time()
    for i in range(max_iter):
        x_in = v.copy()
        if i>=FEAS:
            vn=np.linalg.norm(v)
            if vn>0: v*=sqrt_l/vn
        u_t[:NV]=v[:NV]; u_t[NV:l-1]=-v[NV:l-1]; u_t[l-1]=v[l-1]
        u_t[:l-1]=lu.solve(u_t[:l-1])
        if i<FEAS: u_t[l-1]=1.0
        else: u_t[l-1]=root_plus(u_t[:l-1], v[:l-1], v[l-1])
        u_t[:l-1]+=g*(-u_t[l-1])
        u[:]=2*u_t-v
        u[NV:l-1]=proj_dual_cone(u[NV:l-1])
        if i<FEAS: u[l-1]=1.0
        else: u[l-1]=max(u[l-1],0.0)
        v+=alpha*(u-u_t)          # fixed-point map output = new v
        f_out=v.copy()
        # Anderson Type-II on the map (iterate x_in -> f_out)
        if aa_mem>0 and (i%aa_every==0) and i>=1:
            hist_x.append(x_in); hist_f.append(f_out)
            if len(hist_x)>aa_mem+1: 
                hist_x.pop(0); hist_f.pop(0)
            if len(hist_x)>=2:
                xk=x_in; fk=f_out; gk=xk-fk
                m=min(aa_mem,len(hist_x)-1)
                Y=np.column_stack([gk-(hist_x[-1-j]-hist_f[-1-j]) for j in range(1,m+1)])
                S=np.column_stack([xk-hist_x[-1-j] for j in range(1,m+1)])
                try:
                    gam=np.linalg.solve(Y.T@Y+aa_r*np.eye(m), Y.T@gk)
                    v[:] = fk - S@gam     # f_new = f_k - (S-Y)gamma? use fk - S@gam
                except np.linalg.LinAlgError:
                    pass
        if track and i%track==0:
            tau=u[l-1]; xx=u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
            hist.append((i,np.linalg.norm(xx-x_ref)/np.linalg.norm(x_ref),float(c@xx)))
    tau=u[l-1]; x=u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
    return x,tau,hist

if __name__=="__main__":
    print(f"nvars={NV} nrows={NR}  KKT={NV+NR}x{NV+NR}")
    print(f"clarabel ref: obj={obj_ref:.6f}")
    for mem in [0, 5, 10, 20]:
        x,tau,hist=solve(50000, aa_mem=mem, aa_every=5, track=5000)
        relx=np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
        print(f"\nAA mem={mem}: tau={tau:.4f}  obj={c@x:.6f}  rel_x={relx:.3e}")
        for it,rx,o in hist: print(f"   {it:6d}  rel_x={rx:.3e}  obj={o:.6f}")
