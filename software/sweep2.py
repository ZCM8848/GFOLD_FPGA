#!/usr/bin/env python3
"""
Focused sweep: does small Tikhonov regularization (P=eps*I) + robust Ruiz
equilibration make first-order solvers converge on the G-FOLD SOCP?
Reference optimum obj=-7.494177 (Clarabel). Test with SCS (supports P).
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(__file__)
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A0 = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV)).tocsr()
b0 = np.array(D["b"]); q0 = np.array(D["q"]); x_ref = np.array(D["x"])
blocks = []; r = 0
for c in D["cones"]:
    blocks.append((c["kind"], r, c["dim"])); r += c["dim"]

def ruiz(A, b, q, iters=5):
    """Ruiz equilibration (robust, min-scaling) -> (A',b',q', E_c_inv)."""
    m, n = A.shape
    Dr = np.ones(m); Dc = np.ones(n)
    for _ in range(iters):
        rn = np.asarray(A.power(2).sum(axis=1)).ravel()**0.5
        rn = np.maximum(rn, 1e-8)
        s = 1.0/np.maximum(rn, 1e-8)
        A = sp.diags(s) @ A; Dr = Dr * s
        cn = np.asarray(A.power(2).sum(axis=0)).ravel()**0.5
        cn = np.maximum(cn, 1e-8)
        s = 1.0/np.maximum(cn, 1e-8)
        A = A @ sp.diags(s); Dc = Dc * s
    return A, Dr*b, Dc*q, 1.0/Dc

def run(cfg):
    eps, ru, maxit = cfg
    A, b, q = A0, b0, q0
    Einv = np.ones(NV)
    if ru:
        A, b, q, Einv = ruiz(A, b, q)
    P = eps * sp.eye(NV, format="csc")
    import scs
    z_dim = sum(c[2] for c in blocks if c[0]=="zero")
    l_dim = sum(c[2] for c in blocks if c[0]=="nonneg")
    qd = [c[2] for c in blocks if c[0]=="soc"]
    data = {"A": sp.csc_matrix(A), "b": b, "c": q, "P": P}
    cone = {"z": z_dim, "l": l_dim, "q": qd}
    t0 = time.time()
    sol = scs.solve(data, cone, max_iters=maxit, eps_abs=1e-5, eps_rel=1e-5,
                    normalize=False, rho_x=1e-6, acceleration_lookback=10, verbose=False)
    dt = time.time()-t0
    x = sol["x"] * Einv
    it = sol["info"]["iter"]; st = sol["info"]["status"]
    obj = q0 @ x; relx = np.linalg.norm(x - x_ref)/np.linalg.norm(x_ref)
    # conic residual
    def proj_soc(v):
        t=v[0]; xx=v[1:]; nx=np.linalg.norm(xx)
        if nx<=t: return v.copy()
        if nx<=-t: return np.zeros_like(v)
        lam=(nx+t)/2; o=np.empty_like(v); o[0]=lam; o[1:]=(lam/nx)*xx; return o
    def proj(v):
        o=v.copy()
        for k,s,dim in blocks:
            if k=="zero": o[s:s+dim]=0
            elif k=="nonneg": o[s:s+dim]=np.maximum(v[s:s+dim],0)
            elif k=="soc": o[s:s+dim]=proj_soc(v[s:s+dim])
        return o
    res = np.linalg.norm((A0@x - b0) - proj(A0@x - b0))
    return {"cfg":(eps,ru,maxit),"it":it,"t":round(dt,2),"status":st,
            "obj":float(obj),"rel_x":float(relx),"cone_res":float(res)}

if __name__ == "__main__":
    cfgs = []
    for eps in [0.0, 1e-2, 1e-3, 1e-4, 1e-5]:
        for ru in [False, True]:
            cfgs.append((eps, ru, 4000))
    print(f"{len(cfgs)} configs, 40 cores", flush=True)
    t0=time.time()
    with ProcessPoolExecutor(max_workers=min(40, os.cpu_count() or 4)) as ex:
        res = list(ex.map(run, cfgs))
    print(f"done {time.time()-t0:.1f}s\n")
    res.sort(key=lambda r:(r["rel_x"], r["cone_res"]))
    print(f"{'eps':>8s} {'ruiz':>5s} {'it':>5s} {'t(s)':>6s} {'cone_res':>10s} {'obj':>10s} {'rel_x':>9s}  status")
    for r in res:
        print(f"{r['cfg'][0]:8.0e} {str(r['cfg'][1]):>5s} {r['it']:5d} {r['t']:6.2f} "
              f"{r['cone_res']:10.2e} {r['obj']:10.4f} {r['rel_x']:9.2e}  {r['status']}")
