#!/usr/bin/env python3
"""Fixed-point DRS solver: quantizes the SCS core arithmetic to Q(m,n) and
measures accuracy vs the float version (same core, same iters). The gap is the
quantization error -> drives format selection."""
import numpy as np, scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc
from fixed_point import Q
import scs_tof as T

def proj_dual_cone_q(y, Qf):
    out = y.copy()
    for kind, s, dim in T.blocks:
        if kind == "zero":    pass
        elif kind == "nonneg": out[s:s+dim] = np.maximum(y[s:s+dim], 0.0)
        elif kind == "soc":
            blk = proj_soc(y[s:s+dim])
            out[s:s+dim] = Qf.q(blk)   # quantize the SOC projection (incl. sqrt)
    return out

def run(tof=46.6093, iters=5000, fmt=None, track=False):
    """fmt=None -> float. fmt=(m,n) -> fixed-point Q(m,n)."""
    A, b, c, nr_, nv_ = T.assemble(tof)
    m, n = A.shape
    l = n+m+1
    M = sp.bmat([[sp.eye(n), A.T],[A, -sp.eye(m)]], format="csc")
    lu = splu(M); g = lu.solve(np.concatenate([c,-b]))
    Qf = Q(*fmt) if fmt else None
    if Qf:
        g = Qf.q(g)   # quantize the precomputed g; A kept exact for matvecs
    u=np.zeros(l); v=np.zeros(l); v_prev=np.zeros(l); u_t=np.zeros(l); rsk=np.zeros(l)
    sqrt_l=np.sqrt(l); FEAS=1
    aa=Anderson(l)
    for i in range(iters):
        if i>0 and i%10==0: aa.apply(v, v_prev)
        if i>=FEAS:
            vn=np.linalg.norm(v)
            if vn>0: v*=sqrt_l/vn
        v_prev[:]=v
        u_t[:n]=v[:n]; u_t[n:l-1]=-v[n:l-1]; u_t[l-1]=v[l-1]
        u_t[:l-1]=lu.solve(u_t[:l-1])
        if Qf: u_t[:l-1]=Qf.q(u_t[:l-1])
        if i<FEAS: u_t[l-1]=1.0
        else:
            gg=np.dot(g,g); mug=np.dot(g,v[:l-1]); pg=np.dot(u_t[:l-1],g)
            pp=np.dot(u_t[:l-1],u_t[:l-1]); pmu=np.dot(u_t[:l-1],v[:l-1])
            a_=1+gg; bv=mug-2*pg-v[l-1]; cc=pp-pmu; rad=bv*bv-4*a_*cc
            u_t[l-1] = -bv/(2*a_) if rad<0 else ((-bv+np.sqrt(rad))/(2*a_) if bv<=0 else (cc/(-0.5*(bv+np.sqrt(rad))) if -0.5*(bv+np.sqrt(rad))!=0 else 0.0))
        u_t[:l-1]+=g*(-u_t[l-1])
        if Qf: u_t[:l-1]=Qf.q(u_t[:l-1])
        u[:]=2*u_t-v
        if Qf:
            u[n:l-1]=proj_dual_cone_q(u[n:l-1], Qf)
        else:
            u[n:l-1]=T.proj_dual_cone(u[n:l-1])
        if i<FEAS: u[l-1]=1.0
        else: u[l-1]=max(u[l-1],0.0)
        if Qf: u[:]=Qf.q(u)
        rsk[:n]=(v[:n]+u[:n]-2*u_t[:n]); rsk[n:l-1]=(v[n:l-1]+u[n:l-1]-2*u_t[n:l-1]); rsk[l-1]=(v[l-1]+u[l-1]-2*u_t[l-1])
        if Qf: rsk[:]=Qf.q(rsk)
        v+=1.5*(u-u_t)
        if Qf: v[:]=Qf.q(v)
    tau=u[l-1]; x=u[:n]/tau if abs(tau)>1e-12 else u[:n]
    return x, tau, u, v

def fmt_acc(fmt, iters=6000):
    xf, _,_,_ = run(iters=iters, fmt=None)       # float baseline (same core)
    xq, _,_,_ = run(iters=iters, fmt=fmt)        # fixed-point
    # compare
    relx = np.linalg.norm(xq-xf)/np.linalg.norm(xf)
    fm_f = np.exp(xf[10*100-1]); fm_q = np.exp(xq[10*100-1])
    posf = xf[0:600]; posq = xq[0:600]
    poserr = np.linalg.norm(posq-posf)  # position error
    return relx, (fm_q-fm_f)/fm_f, poserr, fm_f, fm_q

if __name__ == "__main__":
    print("=== fixed-point DRS accuracy vs float (same core, 6000 iters) ===")
    print("float baseline final_mass = %.3f" % np.exp(run(iters=6000)[0][10*100-1]))
    for fmt in [(18,5),(24,8),(24,11),(32,16),(32,19),(24,16)]:
        relx, drelfm, poserr, fm_f, fm_q = fmt_acc(fmt)
        print(f"Q{fmt}: rel_x={relx:.2e}  d(fm)={drelfm*100:+.3f}%  pos_err={poserr:.3e}  "
              f"fm={fm_q:.2f} (float {fm_f:.2f})")
