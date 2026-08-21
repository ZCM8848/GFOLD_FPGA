#!/usr/bin/env python3
"""
Simplified adaptive scaling test for the SCS-DRS G-FOLD solver.

Idea: SCS's real adaptive scaling uses a cone-coupled per-row R_y (set_r_y) +
residual-based geometric-mean factor + refactor/remap. That's complex for HW.

This tests a SIMPLIFIED scheme: ONE uniform scalar `s` scales all constraint
rows (diag_r = [rho_x*1_n; s*1_m; 1]). Uniform scaling is cone-preserving, so
the SOC projection stays clean (proj(s*y)=s*proj(y)). We adapt `s` by simple
residual balancing every K iters, re-factorizing the banded M when s changes.

If this reaches rel_x 1e-3, the hardware adaptive scaling can be far simpler
than SCS's.

Reuses the faithful SCS core + Type-I Anderson from scs_faithful.
"""
import json, os, time
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc, AA_MEM, AA_MINLEN, AA_R, AA_RELAX, AA_SAFEGUARD, AA_MAXW, AA_IR

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV)).tocsr()
b = np.array(D["b"]); c = np.array(D["q"]); x_ref = np.array(D["x"]); obj_ref = D["objective"]
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

RHO_X = 1e-6

def build_system(s):
    """M = [[rho_x I, A^T],[A, -s I]]  and  g = M^{-1}[c; -b]."""
    M = sp.bmat([[sp.eye(NV)*RHO_X, A.T], [A, -s*sp.eye(NR)]], format="csc")
    lu = splu(M)
    g = lu.solve(np.concatenate([c, -b]))
    return lu, g

def solve(max_iter, s0, adapt_every, adapt_rate, track=None):
    l = NV + NR + 1
    u = np.zeros(l); v = np.zeros(l); v_prev = np.zeros(l); u_t = np.zeros(l)
    sqrt_l = np.sqrt(l); FEAS = 1
    s = s0
    lu, g = build_system(s)
    aa = Anderson(l)
    hist = []; t0 = time.time()
    nrefact = 0
    # residual state
    prev_pri = None; prev_dual = None
    for i in range(max_iter):
        if i > 0 and i % 10 == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0: v *= sqrt_l/vn
        v_prev[:] = v
        # DR step with diag_r = [rho_x*1_n; s*1_m; 1]
        u_t[:NV] = v[:NV]*RHO_X
        u_t[NV:l-1] = -v[NV:l-1]*s
        u_t[l-1] = v[l-1]
        u_t[:l-1] = lu.solve(u_t[:l-1])
        # root_plus with tau_scale=1 and weights [rho_x; s; 1]
        if i < FEAS: u_t[l-1] = 1.0
        else:        u_t[l-1] = _root_plus(u_t[:l-1], v[:l-1], v[l-1], g, s)
        u_t[:l-1] += g * (-u_t[l-1])
        # project cones (scaled u): u[n:l-1] in scaled y-space; proj(s*y) on cone
        u[:] = 2*u_t - v
        # cone projection must act on scaled coords: apply proj to s*y then /s
        yscaled = u[NV:l-1].copy()
        zproj = proj_dual_cone(yscaled)
        u[NV:l-1] = zproj
        if i < FEAS: u[l-1] = 1.0
        else:        u[l-1] = max(u[l-1], 0.0)
        # update dual vars (no diag_r)
        v += 1.5*(u - u_t)
        # ---- adaptive s update (residual balancing) ----
        if adapt_every and i > 0 and i % adapt_every == 0:
            # residuals from current iterate
            xc = u[:NV]; yc = u[NV:l-1]; svec = None  # s from rsk
            # use ax_s_btau and px_aty_ctau style
            ax = A @ xc
            aty = A.T @ yc
            tau = u[l-1]
            # primal: ax + s_slack - tau*b ; dual: A'y + tau*c (P=0)
            # s_slack = rsk[n:l-1]... approximate: use rsk
            rsk = (v + u - 2*u_t) * np.concatenate([np.full(NV,RHO_X), np.full(NR,s), [1.0]])
            s_slack = rsk[NV:l-1]
            pri = ax + s_slack - tau*b
            dual = aty + tau*c
            npri = np.linalg.norm(pri); ndual = np.linalg.norm(dual)
            if npri > 1e-12 and ndual > 1e-12:
                ratio = npri/ndual
                # increase s if primal dominates, decrease if dual dominates
                new_s = s * np.clip(ratio**adapt_rate, 0.5, 2.0)
                if abs(new_s - s)/max(s,1e-12) > 0.05:
                    s = new_s
                    lu, g = build_system(s); nrefact += 1
        if track and i % track == 0:
            tau = u[l-1]; xx = u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
            hist.append((i, np.linalg.norm(xx-x_ref)/np.linalg.norm(x_ref), float(c@xx), s))
    tau = u[l-1]; x = u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
    return x, tau, hist, nrefact

def _root_plus(p, mu, eta, g, s):
    # weights diag_r = [rho_x*1_n; s*1_m]; tau_scale=1
    w = np.concatenate([np.full(NV,RHO_X), np.full(NR,s)])
    gg=np.dot(g,g*w); mug=np.dot(mu,g*w); pg=np.dot(p,g*w)
    pp=np.dot(p,p*w); pmu=np.dot(p,mu*w)
    a=1.0+gg; bv=mug-2*pg-eta; cc=pp-pmu
    rad=bv*bv-4*a*cc
    if rad<0: return -bv/(2*a)
    sq=np.sqrt(rad)
    if bv<=0: return (-bv+sq)/(2*a)
    qq=-0.5*(bv+sq); return cc/qq if qq!=0 else 0.0

if __name__ == "__main__":
    print(f"nvars={NV} nrows={NR} clarabel obj={obj_ref:.6f}")
    for s0 in [1.0, 0.1, 10.0]:
        x, tau, hist, nr = solve(50000, s0, adapt_every=100, adapt_rate=0.3, track=5000)
        relx = np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
        print(f"\n[s0={s0}] refactors={nr} tau={tau:.4f} obj={c@x:.6f} rel_x={relx:.3e}")
        for it,rx,o,ss in hist:
            print(f"   {it:6d} rel_x={rx:.3e} obj={o:.6f} s={ss:.2e}")
