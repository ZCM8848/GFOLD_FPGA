#!/usr/bin/env python3
"""
Corrected simplified adaptive scaling for SCS-DRS G-FOLD.

Fixes that made the naive version diverge:
1. Relative residuals (normalized by problem scale), geometric-mean smoothed
   factor (like SCS update_scale), bounded update.
2. State REMAP on scale change: v = rsk/diag_r_new + 2*u_t - u (SCS line 1236).
3. Uniform scalar s on all constraint rows (cone-preserving).

If this reaches rel_x 1e-3, uniform-s adaptive scaling is a real HW simplification
over SCS's cone-coupled R_y.
"""
import json, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from scs_faithful import Anderson, proj_soc

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
    M = sp.bmat([[sp.eye(NV)*RHO_X, A.T], [A, -s*sp.eye(NR)]], format="csc")
    lu = splu(M)
    g = lu.solve(np.concatenate([c, -b]))
    return lu, g

def root_plus(p, mu, eta, g, s):
    w = np.concatenate([np.full(NV,RHO_X), np.full(NR,s)])
    gg=np.dot(g,g*w); mug=np.dot(mu,g*w); pg=np.dot(p,g*w)
    pp=np.dot(p,p*w); pmu=np.dot(p,mu*w)
    a=1.0+gg; bv=mug-2*pg-eta; cc=pp-pmu
    rad=bv*bv-4*a*cc
    if rad<0: return -bv/(2*a)
    sq=np.sqrt(rad)
    if bv<=0: return (-bv+sq)/(2*a)
    qq=-0.5*(bv+sq); return cc/qq if qq!=0 else 0.0

def solve(max_iter, s0, adapt_every=50, track=None, s_min=1e-4, s_max=1e4):
    l = NV + NR + 1
    u = np.zeros(l); v = np.zeros(l); v_prev = np.zeros(l); u_t = np.zeros(l); rsk = np.zeros(l)
    sqrt_l = np.sqrt(l); FEAS = 1
    s = s0
    lu, g = build_system(s)
    aa = Anderson(l)
    hist = []; nrefact = 0
    sum_log_factor = 0.0; n_log = 0
    nm_b = np.linalg.norm(b); nm_c = np.linalg.norm(c)
    for i in range(max_iter):
        if i > 0 and i % 10 == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0: v *= sqrt_l/vn
        v_prev[:] = v
        u_t[:NV] = v[:NV]*RHO_X
        u_t[NV:l-1] = -v[NV:l-1]*s
        u_t[l-1] = v[l-1]
        u_t[:l-1] = lu.solve(u_t[:l-1])
        if i < FEAS: u_t[l-1] = 1.0
        else:        u_t[l-1] = root_plus(u_t[:l-1], v[:l-1], v[l-1], g, s)
        u_t[:l-1] += g * (-u_t[l-1])
        u[:] = 2*u_t - v
        u[NV:l-1] = proj_dual_cone(u[NV:l-1])
        if i < FEAS: u[l-1] = 1.0
        else:        u[l-1] = max(u[l-1], 0.0)
        # rsk computed BEFORE v update (matches SCS compute_rsk order)
        rsk[:NV] = (v[:NV]+u[:NV]-2*u_t[:NV])*RHO_X
        rsk[NV:l-1] = (v[NV:l-1]+u[NV:l-1]-2*u_t[NV:l-1])*s
        rsk[l-1] = (v[l-1]+u[l-1]-2*u_t[l-1])
        v += 1.5*(u - u_t)
        # adaptive scale (residual balancing)
        if adapt_every and i > 0 and i % adapt_every == 0:
            xc = u[:NV]; yc = u[NV:l-1]; tau = abs(u[l-1])
            ax = A @ xc; aty = A.T @ yc
            s_slack = rsk[NV:l-1]
            pri = ax + s_slack - tau*b
            dual = aty + tau*c   # P=0
            npri = np.linalg.norm(pri); ndual = np.linalg.norm(aty)
            denom_pri = max(np.linalg.norm(ax), np.linalg.norm(s_slack), nm_b*tau, 1e-12)
            denom_dual = max(ndual, nm_c*tau, 1e-12)
            rel_pri = max(npri/denom_pri, 1e-12)
            rel_dual = max(ndual/denom_dual, 1e-12)
            sum_log_factor += np.log(rel_pri) - np.log(rel_dual)
            n_log += 1
            factor = np.sqrt(np.exp(sum_log_factor/n_log))
            if abs(np.log(factor)) > 0.5*np.log(10.0):   # outside [1/sqrt10, sqrt10]
                new_s = float(np.clip(s*factor, s_min, s_max))
                if abs(np.log(new_s/s)) > 1e-3:
                    # state remap: v = rsk/diag_r_new + 2*u_t - u
                    v[:] = np.concatenate([rsk[:NV]/RHO_X, rsk[NV:l-1]/new_s, rsk[l-1:]]) + 2*u_t - u
                    s = new_s
                    lu, g = build_system(s); nrefact += 1
                    sum_log_factor = 0.0; n_log = 0
                    aa.reset()
        if track and i % track == 0:
            tau = u[l-1]; xx = u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
            hist.append((i, np.linalg.norm(xx-x_ref)/np.linalg.norm(x_ref), float(c@xx), s))
    tau = u[l-1]; x = u[:NV]/tau if abs(tau)>1e-12 else u[:NV]
    return x, tau, hist, nrefact

if __name__ == "__main__":
    print(f"nvars={NV} nrows={NR} clarabel obj={obj_ref:.6f}")
    for s0 in [1.0, 0.1, 10.0]:
        x, tau, hist, nr = solve(50000, s0, adapt_every=50, track=5000)
        relx = np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
        print(f"\n[s0={s0}] refactors={nr} obj={c@x:.6f} rel_x={relx:.3e}")
        for it,rx,o,ss in hist:
            print(f"   {it:6d} rel_x={rx:.3e} obj={o:.6f} s={ss:.2e}")
