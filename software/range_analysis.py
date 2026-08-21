#!/usr/bin/env python3
"""Phase 2a: range analysis for fixed-point format selection.

Logs the actual magnitudes of the solution and the DRS iterate quantities
(u, v, u_t, rsk, tau) during a solve, per variable group. This determines the
integer/fraction bit budget of the fixed-point format.
"""
import numpy as np, scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc
import scs_tof as T

def range_analysis(tof=46.6093, iters=3000):
    A, b, c, nr_, nv_ = T.assemble(tof)
    m, n = A.shape
    l = n+m+1
    M = sp.bmat([[sp.eye(n), A.T],[A, -sp.eye(m)]], format="csc")
    lu = splu(M); g = lu.solve(np.concatenate([c,-b]))
    u=np.zeros(l); v=np.zeros(l); v_prev=np.zeros(l); u_t=np.zeros(l); rsk=np.zeros(l)
    sqrt_l=np.sqrt(l); FEAS=1
    aa=Anderson(l)
    maxu=0; maxv=0; maxut=0; maxrsk=0; tau_vals=[]
    for i in range(iters):
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
        maxu=max(maxu,np.abs(u).max()); maxv=max(maxv,np.abs(v).max())
        maxut=max(maxut,np.abs(u_t).max()); maxrsk=max(maxrsk,np.abs(rsk).max())
        tau_vals.append(abs(u[l-1]))
    tau=u[l-1]; x=u[:n]/tau if abs(tau)>1e-12 else u[:n]
    print(f"=== Range analysis (tof={tof}, {iters} iters) ===")
    print(f"max|u|={maxu:.3e}  max|v|={maxv:.3e}  max|u_t|={maxut:.3e}  max|rsk|={maxrsk:.3e}")
    print(f"tau: min={min(tau_vals):.3e} max={max(tau_vals):.3e} final={tau:.3e}")
    # solution per-group ranges
    n_nodes = n//11
    pos = x[[6*i+c for i in range(n_nodes) for c in range(3)]]
    vel = x[[6*i+c for i in range(n_nodes) for c in range(3,6)]]
    uu  = x[6*n_nodes:9*n_nodes]
    ss  = x[9*n_nodes:10*n_nodes]
    zz  = x[10*n_nodes:]
    print(f"\nSolution x ranges:")
    for nm, grp in [("pos",pos),("vel",vel),("u(thrust)",uu),("s(slack)",ss),("z(logm)",zz)]:
        print(f"  {nm:10s}: min={grp.min():.4e} max={grp.max():.4e} |max|={np.abs(grp).max():.4e}")
    # A and A^T A entry ranges
    print(f"\n|A| max={np.abs(A.data).max():.3e}  |b| max={np.abs(b).max():.3e}  |q| max={np.abs(c).max():.3e}")
    ATA=(A.T@A).tocsc()
    print(f"|A^T A| max={np.abs(ATA.data).max():.3e}")
    # ratio of smallest meaningful: what fractional resolution do we need?
    return x

if __name__=="__main__":
    x=range_analysis()
