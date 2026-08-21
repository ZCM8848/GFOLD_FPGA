#!/usr/bin/env python3
"""Fast honest warm-start measurement: run the solver a FIXED number of iters
cold vs warm (init from a nearby converged tof), compare achieved residual +
objective. The residual gap at equal iteration count shows the warm-start
benefit, independent of convergence rate."""
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc
import scs_tof as T

def budget_solve(A, b, c, K, init_u=None, init_v=None):
    m, n = A.shape
    l = n+m+1
    M = sp.bmat([[sp.eye(n), A.T],[A, -sp.eye(m)]], format="csc")
    lu = splu(M); g = lu.solve(np.concatenate([c,-b]))
    u=np.zeros(l); v=np.zeros(l); v_prev=np.zeros(l); u_t=np.zeros(l); rsk=np.zeros(l)
    if init_u is not None: u[:]=init_u
    if init_v is not None: v[:]=init_v
    sqrt_l=np.sqrt(l); FEAS=1
    aa=Anderson(l)
    for i in range(K):
        if i>0 and i%10==0: aa.apply(v, v_prev)
        if i>=FEAS:
            vn=np.linalg.norm(v)
            if vn>0: v*=sqrt_l/vn
        v_prev[:]=v
        u_t[:n]=v[:n]; u_t[n:l-1]=-v[n:l-1]; u_t[l-1]=v[l-1]
        u_t[:l-1]=lu.solve(u_t[:l-1])
        if i<FEAS: u_t[l-1]=1.0
        else:
            gg=np.dot(g,g); mug=np.dot(g,v[:l-1]); pg=np.dot(u_t[:l-1],g)
            pp=np.dot(u_t[:l-1],u_t[:l-1]); pmu=np.dot(u_t[:l-1],v[:l-1])
            a_=1+gg; bv=mug-2*pg-v[l-1]; cc=pp-pmu; rad=bv*bv-4*a_*cc
            u_t[l-1] = -bv/(2*a_) if rad<0 else ((-bv+np.sqrt(rad))/(2*a_) if bv<=0 else (cc/(-0.5*(bv+np.sqrt(rad))) if -0.5*(bv+np.sqrt(rad))!=0 else 0.0))
        u_t[:l-1]+=g*(-u_t[l-1])
        u[:]=2*u_t-v
        u[n:l-1]=T.proj_dual_cone(u[n:l-1])
        if i<FEAS: u[l-1]=1.0
        else: u[l-1]=max(u[l-1],0.0)
        rsk[:n]=(v[:n]+u[:n]-2*u_t[:n]); rsk[n:l-1]=(v[n:l-1]+u[n:l-1]-2*u_t[n:l-1]); rsk[l-1]=(v[l-1]+u[l-1]-2*u_t[l-1])
        v+=1.5*(u-u_t)
    tau=u[l-1]; x=u[:n]/tau if abs(tau)>1e-12 else u[:n]
    # residuals
    yc=u[n:l-1]; ss=rsk[n:l-1]
    npri=np.linalg.norm(A@x + ss - tau*b) if abs(tau)>1e-12 else np.inf
    nm_b=np.linalg.norm(b)
    rel_pri=npri/max(np.linalg.norm(A@x),np.linalg.norm(ss),nm_b*abs(tau),1e-12)
    return rel_pri, c@x, u, v

# base converged-ish warm state
base=44.63
A0,b0,c0,_,_=T.assemble(base)
u0,v0 = None, None
# get a good warm state from the base problem via many iters
_, _, u0, v0, _ = T.solve_fixed(A0,b0,c0, max_iter=40000, tol=5e-3)

print(f"warm state from tof={base} (u0,v0 saved)\n")
for dt in [0.5, 2.0, 6.0, 15.0]:
    t2=base+dt
    A,b,c,_,_=T.assemble(t2)
    print(f"--- target tof={t2:.1f} (d={dt:+.1f}) ---")
    for K in [1000, 4000]:
        r_c,o_c,_,_=budget_solve(A,b,c,K,None,None)
        r_w,o_w,_,_=budget_solve(A,b,c,K,u0,v0)
        print(f"  K={K:5d}: cold rel_pri={r_c:.2e} obj={o_c:.4f} | "
              f"warm rel_pri={r_w:.2e} obj={o_w:.4f} | gap={r_c/max(r_w,1e-30):.0f}x")
    print()
