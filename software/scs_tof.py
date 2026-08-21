#!/usr/bin/env python3
"""
P1 wrap-up: validate adaptive TOF via golden-section search with warm-start.

Each tof candidate rebuilds A,b,q (dt changes -> A changes -> M changes, i.e.
a re-factorization per candidate -- exactly the hardware adaptive-TOF story).
Measures total SCS iterations cold vs warm-started (warm = previous candidate's
converged u,v as initial iterate), and estimates hardware time.

Search mirrors search.rs: auto_tof_bounds + 13-sample coarse sweep +
20 golden-section iterations, maximizing final_mass.
"""
import json, os, time
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc
from assembler import assemble, N

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
x_ref = np.array(D["x"])
blocks = []; r = 0
for cc in D["cones"]:
    blocks.append((cc["kind"], r, cc["dim"])); r += cc["dim"]

def proj_dual_cone(y):
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    pass
        elif kind == "nonneg": out[s:s+dim] = np.maximum(y[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(y[s:s+dim])
    return out

def solve_fixed(A, b, c, init_u=None, init_v=None, max_iter=60000,
                tol=1e-4, alpha=1.5):
    """Faithful SCS core + Anderson, warm-startable, stops on residual tol.
    Returns (x, tau, u, v, iters_used)."""
    m, n = A.shape            # A is m x n (2107 x 1100)
    NV, NR = n, m
    l = n+m+1
    M = sp.bmat([[sp.eye(n), A.T],[A, -sp.eye(m)]], format="csc")
    lu = splu(M); g = lu.solve(np.concatenate([c,-b]))
    u = np.zeros(l); v = np.zeros(l); v_prev = np.zeros(l)
    if init_u is not None: u[:] = init_u
    if init_v is not None: v[:] = init_v
    u_t = np.zeros(l); rsk = np.zeros(l)
    sqrt_l=np.sqrt(l); FEAS=1
    aa = Anderson(l)
    # residual scale
    nm_b=np.linalg.norm(b); nm_c=np.linalg.norm(c)
    iters=0
    for i in range(max_iter):
        if i>0 and i%10==0: aa.apply(v, v_prev)
        if i>=FEAS:
            vn=np.linalg.norm(v)
            if vn>0: v*=sqrt_l/vn
        v_prev[:]=v
        u_t[:NV]=v[:NV]; u_t[NV:l-1]=-v[NV:l-1]; u_t[l-1]=v[l-1]
        u_t[:l-1]=lu.solve(u_t[:l-1])
        if i<FEAS: u_t[l-1]=1.0
        else:
            gg=np.dot(g,g); mug=np.dot(g,v[:l-1]); pg=np.dot(u_t[:l-1],g)
            pp=np.dot(u_t[:l-1],u_t[:l-1]); pmu=np.dot(u_t[:l-1],v[:l-1])
            a_=1.0+gg; bv=mug-2*pg-v[l-1]; cc=pp-pmu
            rad=bv*bv-4*a_*cc
            u_t[l-1] = -bv/(2*a_) if rad<0 else ((-bv+np.sqrt(rad))/(2*a_) if bv<=0 else (cc/(-0.5*(bv+np.sqrt(rad))) if -0.5*(bv+np.sqrt(rad))!=0 else 0.0))
        u_t[:l-1]+=g*(-u_t[l-1])
        u[:]=2*u_t-v
        u[NV:l-1]=proj_dual_cone(u[NV:l-1])
        if i<FEAS: u[l-1]=1.0
        else: u[l-1]=max(u[l-1],0.0)
        rsk[:NV]=(v[:NV]+u[:NV]-2*u_t[:NV])
        rsk[NV:l-1]=(v[NV:l-1]+u[NV:l-1]-2*u_t[NV:l-1])
        rsk[l-1]=(v[l-1]+u[l-1]-2*u_t[l-1])
        v+=alpha*(u-u_t)
        # convergence: relative primal/dual residual
        if i%25==0 and i>=FEAS:
            xc=u[:NV]; yc=u[NV:l-1]; tau=abs(u[l-1])
            ax=A@xc; aty=A.T@yc; ss=rsk[NV:l-1]
            npri=np.linalg.norm(ax+ss-tau*b)
            ndual=np.linalg.norm(aty+tau*c)
            rel_pri=npri/max(np.linalg.norm(ax),np.linalg.norm(ss),nm_b*tau,1e-12)
            rel_dual=ndual/max(ndual,nm_c*tau,1e-12)
            if rel_pri<tol and rel_dual<tol:
                iters=i+1; break
            iters=i+1
    tau=u[l-1]; x=u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
    return x, tau, u, v, iters

def auto_tof_bounds():
    m_dry=max(2000.0-1700.0,1.0); v0=42.43
    a_max=19200.0/m_dry
    t_lo=max(v0/a_max,1.0)
    t_burn=1700.0/(5e-4*4800.0)
    t_hi=max(t_burn, t_lo*4.0)
    return t_lo, t_hi

def final_mass(x):
    return np.exp(x[10*N-1])   # z(n-1) = last variable

def search_tof(warm, max_iter=60000, tol=1e-4):
    t_lo,t_hi=auto_tof_bounds()
    SAMPLES=13
    step=(t_hi-t_lo)/(SAMPLES-1)
    tofs=[]; masses=[]; iters_total=0; per=[]
    init_u=None; init_v=None
    # coarse sweep
    best_idx=None; best_m=-np.inf
    for i in range(SAMPLES):
        t=t_lo+step*i
        A,b,c,nr_,nv=assemble(t)
        x,tau,u,v,it=solve_fixed(A,b,c,init_u if warm else None,
                                 init_v if warm else None,max_iter=max_iter,tol=tol)
        iters_total+=it; per.append((t,it))
        fm=final_mass(x)
        tofs.append(t); masses.append(fm)
        if np.isfinite(fm) and fm>best_m:
            best_m=fm; best_idx=i
        init_u,init_v=u,v   # warm-start next candidate
    bi=best_idx if best_idx is not None else 0
    lo=t_lo+step*max(bi-1,0); hi=t_lo+step*min(bi+1,SAMPLES-1)
    # golden-section maximize final_mass over [lo,hi]
    invphi=0.618033988749895
    c=hi-(hi-lo)*invphi; d=lo+(hi-lo)*invphi
    def eval_t(t):
        A,b,c_,nr_,nv=assemble(t)
        x,tau,u,v,it=solve_fixed(A,b,c_,init_u if warm else None,
                                 init_v if warm else None,max_iter=max_iter,tol=tol)
        return final_mass(x), u, v, it
    fc,uc,vc,itc=eval_t(c); iters_total+=itc; per.append((c,itc))
    fd,ud,vd,itd=eval_t(d); iters_total+=itd; per.append((d,itd))
    init_u,init_v=ud,vd
    for _ in range(20):
        if fc>=fd:
            hi=d; d=c; fd=fc; ud,vd=uc,vc
            c=hi-(hi-lo)*invphi
            fc,uc,vc,itc=eval_t(c); iters_total+=itc; per.append((c,itc))
            init_u,init_v=uc,vc
        else:
            lo=c; c=d; fc=fd; uc,vc=ud,vd
            d=lo+(hi-lo)*invphi
            fd,ud,vd,itd=eval_t(d); iters_total+=itd; per.append((d,itd))
            init_u,init_v=ud,vd
    best_t = c if fc>=fd else d
    A,b,c_,nr_,nv=assemble(best_t)
    x,tau,u,v,it=solve_fixed(A,b,c_,init_u,init_v,max_iter=max_iter,tol=tol)
    iters_total+=it; per.append((best_t,it))
    return best_t, final_mass(x), iters_total, per

if __name__=="__main__":
    # sanity: solve default tof and check vs clarabel
    A,b,c,nr,nv=assemble(44.63)
    x,tau,u,v,it=solve_fixed(A,b,c,tol=1e-4)
    relx=np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
    print(f"sanity tof=44.63: iters={it} obj={c@x:.5f} (ref {D['objective']:.5f}) rel_x={relx:.3e}")
    # warm search
    print("\n=== tof search with WARM-START ===")
    t0=time.time()
    best_t,fm,itot,per=search_tof(warm=True,tol=1e-4)
    dt=time.time()-t0
    print(f"best tof={best_t:.3f} final_mass={fm:.1f} total_iters={itot} wall={dt:.1f}s")
    print("per-candidate iters:", [p[1] for p in per])
    print(f"HW est: {itot} iters x ~1500 cyc @100MHz = {itot*1500/1e8:.2f} s")
    # cold search (for comparison) - run fewer to save time
    print("\n=== tof search COLD (no warm-start, for comparison) ===")
    A,b,c,nr,nv=assemble(44.63)
    t0=time.time()
    best_t2,fm2,itot2,per2=search_tof(warm=False,tol=1e-4)
    dt2=time.time()-t0
    print(f"best tof={best_t2:.3f} final_mass={fm2:.1f} total_iters={itot2} wall={dt2:.1f}s")
    print(f"HW est cold: {itot2} iters x ~1500 cyc @100MHz = {itot2*1500/1e8:.2f} s")
