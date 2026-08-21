#!/usr/bin/env python3
"""Debug: why does quantization block the scale update? Print rel_pri/rel_dual/factor
for float vs Q(32,19) at the first ~10 residual evaluations."""
import numpy as np
from scipy.sparse.linalg import splu
from fixed_point import Q
from scs_adaptive import (A, b, c, NV, NR, Z, build_diag_r, build_kkt, root_plus,
                          RHO_X, ALPHA, CONVERGED_INTERVAL, FEAS, DIV_EPS, Anderson)
from fixed_adaptive import SA_proj_dual_cone_r
import fixed_adaptive as FA

n, m, l = NV, NR, NV + NR + 1

def run(Qf, show_resid=True):
    scale = 1.0; sls = 0; ssum = 0.0; nls = 0
    diag_r = build_diag_r(scale); lu = build_kkt(diag_r)
    g = lu.solve(np.concatenate([c, -b]))
    v = np.zeros(l); v[l-1]=1.0; vp=np.zeros(l); u=np.zeros(l); ut=np.zeros(l); rsk=np.zeros(l)
    aa = Anderson(l); sl = np.sqrt(l)
    nupd = 0
    for i in range(3000):
        if i>0 and i%10==0: aa.apply(v, vp)
        if i>=FEAS:
            vn=np.linalg.norm(v)
            if vn>0: v*=sl/vn
            v = v if Qf is None else Qf.q(v)
        vp[:]=v
        ut[:n]=v[:n]*diag_r[:n]
        ut[n:l-1]=-v[n:l-1]*diag_r[n:l-1]
        ut[l-1]=v[l-1]
        ut[:l-1] = lu.solve(ut[:l-1])
        ut[:l-1] = ut[:l-1] if Qf is None else Qf.q(ut[:l-1])
        if i<FEAS: ut[l-1]=1.0
        else: ut[l-1]=root_plus(diag_r, g, ut[:l-1], v[:l-1], v[l-1])
        ut[:l-1]-=ut[l-1]*g
        ut[:l-1] = ut[:l-1] if Qf is None else Qf.q(ut[:l-1])
        u[:]=2*ut-v
        u[n:l-1]=SA_proj_dual_cone_r(u[n:l-1], diag_r[n:l-1])
        u[n:l-1] = u[n:l-1] if Qf is None else Qf.q(u[n:l-1])
        if i<FEAS: u[l-1]=1.0
        else: u[l-1]=max(u[l-1],0.0)
        u = u if Qf is None else Qf.q(u)
        rsk[:]=(v+u-2*ut)*diag_r
        rsk = rsk if Qf is None else Qf.q(rsk)
        if i%CONVERGED_INTERVAL==0:
            tau=abs(u[l-1]); x_=u[:n]; y_=u[n:l-1]; s_=rsk[n:l-1]
            ax=A@x_; asb=ax+s_-tau*b; aty=A.T@y_; pac=aty+tau*c
            rp=max(np.max(np.abs(asb))/(max(np.max(np.abs(ax)),np.max(np.abs(s_)),np.max(np.abs(b))*tau) or DIV_EPS), DIV_EPS)
            rd=max(np.max(np.abs(pac))/(max(np.max(np.abs(aty)),np.max(np.abs(c))*tau) or DIV_EPS), DIV_EPS)
            if i<150 and show_resid and i%25==0:
                print(f"   iter={i:4d} tau={tau:.3e} rel_pri={rp:.3e} rel_dual={rd:.3e} scale={scale:.2e}")
            ssum+=np.log(rp)-np.log(rd); nls+=1
            fac=np.sqrt(np.exp(ssum/nls))
            if i-sls>=100:
                ns=min(max(scale*fac,1e-6),1e6)
                if ns!=scale and (fac>np.sqrt(10.) or fac<1/np.sqrt(10.)):
                    scale=ns; ssum=0.; nls=0; sls=i; nupd+=1
                    diag_r=build_diag_r(scale); lu=build_kkt(diag_r)
                    g=lu.solve(np.concatenate([c,-b])); aa.reset()
                    v[:]=(rsk/diag_r+2*ut-u)
        v+=ALPHA*(u-ut)
        v = v if Qf is None else Qf.q(v)
        if i%10==0: aa.safeguard(v,vp)
    tau=u[l-1]; x=u[:n]/tau if abs(tau)>1e-12 else u[:n]
    return nupd, scale, tau, np.linalg.norm(x-np.array(FA.x_ref))/np.linalg.norm(FA.x_ref)

print("=== float ===")
nupd, sc, tau, rx = run(None)
print(f"float: scale_updates={nupd} final_scale={sc:.2e} tau={tau:.5f} rel_x={rx:.3e}\n")
print("=== Q(32,19) ===")
nupd, sc, tau, rx = run(Q(32,19))
print(f"Q(32,19): scale_updates={nupd} final_scale={sc:.2e} tau={tau:.5f} rel_x={rx:.3e}")
