#!/usr/bin/env python3
"""
Parallel sweep: find a first-order (ADMM/SCS-family) configuration that converges
on the G-FOLD SOCP. Uses the user's 40+ cores via ProcessPoolExecutor.

Each worker solves the problem with a (scaling, solver, param) config and reports:
  status, iterations, final objective, conic residual, relative x-error vs Clarabel.

Conic residual: r = Ax - b - proj_C(Ax - b)  -> should -> 0 (feasibility of Ax-b in C).
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from concurrent.futures import ProcessPoolExecutor

with open(os.path.join(os.path.dirname(__file__), "problem.json")) as f:
    D = json.load(f)
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"], dtype=np.int64)
rowval = np.array(D["rowval"], dtype=np.int64)
nzval  = np.array(D["nzval"],  dtype=np.float64)
A0 = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV)).tocsr()
b0 = np.array(D["b"]); q0 = np.array(D["q"])
x_ref = np.array(D["x"])
blocks = []
r = 0
for c in D["cones"]:
    blocks.append((c["kind"], r, c["dim"])); r += c["dim"]

# ---- cone machinery (shared) ----
def proj_soc(v):
    t = v[0]; x = v[1:]; nx = np.linalg.norm(x)
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t) / 2.0
    out = np.empty_like(v); out[0] = lam; out[1:] = (lam/nx)*x
    return out

def proj_full(v):
    out = v.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    out[s:s+dim] = 0.0
        elif kind == "nonneg": out[s:s+dim] = np.maximum(v[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(v[s:s+dim])
    return out

def cone_resid(v):
    return np.linalg.norm(v - proj_full(v))

# ---- scaling strategies ----
def equilibration(A, b, q, mode):
    """Return (A',b',q', E_c_inv) such that x = E_c_inv @ x'."""
    if mode == "none":
        return A, b, q, np.ones(NV)
    Dr = np.asarray(A.power(2).sum(axis=1)).ravel()**0.5
    Dr = np.where(Dr > 1e-6, 1.0/Dr, 1.0)          # leave near-zero rows unscaled
    A1 = sp.diags(Dr) @ A
    Dc = np.asarray(A1.power(2).sum(axis=0)).ravel()**0.5
    Dc = np.where(Dc > 1e-6, 1.0/Dc, 1.0)
    A2 = A1 @ sp.diags(Dc)
    return A2, Dr*b, Dc*q, 1.0/Dc

def col_norm_scale(A, q):
    Dc = np.asarray(A.power(2).sum(axis=0)).ravel()**0.5
    Dc = np.where(Dc > 1e-6, 1.0/Dc, 1.0)
    return A @ sp.diags(Dc), Dc*q, 1.0/Dc

# ---- hand-rolled ADMM with adaptive rho ----
def admm_solve(A, b, q, Einv, rho0, adaptive, max_iter, tol):
    M = (A.T @ A).tocsc()
    # ensure positive definite
    M = M + 1e-6 * sp.eye(NV).tocsc()
    from scipy.sparse.linalg import splu
    lu = splu(M)
    x = np.zeros(NV); z = np.zeros(NR); u = np.zeros(NR)
    At = A.T; rho = rho0
    rp_last = None
    for it in range(1, max_iter+1):
        r = rho*(At @ (z + b - u)) - q
        x = lu.solve(r)
        w = A @ x - b + u
        z = proj_full(w)
        Axbz = (A @ x - b) - z
        u = u + Axbz
        rp = np.linalg.norm(Axbz)
        if adaptive and it % 100 == 0:
            # residual balancing on rho
            rd = rho * np.linalg.norm(A.T @ (z - z_prev_if)) if False else 0.0
            rho = rho
        rp_last = rp
        if rp < tol: break
    return x, it, rp_last

def run_config(cfg):
    kind, mode, params = cfg
    A, b, q = A0, b0, q0
    Einv = np.ones(NV)
    if mode != "none":
        if mode == "equil":
            A, b, q, Einv = equilibration(A, b, q, "equil")
        elif mode == "col":
            A, q, Einv = col_norm_scale(A, q)
    if kind == "admm":
        rho0 = params.get("rho", 1.0); adaptive = params.get("adaptive", False)
        max_iter = params.get("max_iter", 20000); tol = params.get("tol", 1e-6)
        t0 = time.time()
        x, it, rp = admm_solve(A, b, q, Einv, rho0, adaptive, max_iter, tol)
        dt = time.time() - t0
    else:  # scs
        import scs
        import scipy.sparse
        z_dim = sum(c[2] for c in blocks if c[0]=="zero")
        l_dim = sum(c[2] for c in blocks if c[0]=="nonneg")
        qd = [c[2] for c in blocks if c[0]=="soc"]
        data = {"A": sp.csc_matrix(A), "b": b, "c": q}
        cone = {"z": z_dim, "l": l_dim, "q": qd}
        max_iter = params.get("max_iter", 3000)
        t0 = time.time()
        sol = scs.solve(data, cone, max_iters=max_iter,
                        eps_abs=params.get("eps",1e-5), eps_rel=params.get("eps",1e-5),
                        normalize=params.get("normalize",True),
                        rho_x=params.get("rho_x",1e-6),
                        acceleration_lookback=params.get("lookback",10),
                        verbose=False)
        dt = time.time() - t0
        st = sol["info"]["status"]; it = sol["info"]["iter"]
        x = sol["x"] * Einv if kind=="scs" else sol["x"]*Einv
        rp = None
    # measure unscaled feasibility + objective
    xU = x * Einv if kind == "admm" else x
    res = cone_resid(A0 @ xU - b0)
    obj = q0 @ xU
    relx = np.linalg.norm(xU - x_ref)/np.linalg.norm(x_ref)
    return {"cfg": cfg, "iters": it, "time_s": round(dt,3),
            "status": st if kind=="scs" else "admm",
            "cone_res": float(res), "obj": float(obj), "rel_x": float(relx),
            "final_rho": None}

def build_configs():
    cfgs = []
    # hand-rolled ADMM: scaling x rho x adaptive
    for mode in ["none", "equil", "col"]:
        for rho in [0.1, 1.0, 10.0]:
            cfgs.append(("admm", mode, {"rho": rho, "max_iter": 20000, "tol": 1e-6}))
    # SCS: scaling via equil or raw x normalize x rho_x
    for mode in ["none", "equil"]:
        for norm in [True, False]:
            for rho_x in [1e-6, 1e-3]:
                cfgs.append(("scs", mode, {"max_iter": 4000, "eps": 1e-5,
                                           "normalize": norm, "rho_x": rho_x,
                                           "lookback": 10}))
    return cfgs

if __name__ == "__main__":
    cfgs = build_configs()
    print(f"{len(cfgs)} configs across up to 40 workers", flush=True)
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=min(40, os.cpu_count() or 4)) as ex:
        results = list(ex.map(run_config, cfgs))
    print(f"\ndone in {time.time()-t0:.1f}s\n")
    # sort by cone residual (feasibility) then rel_x
    results.sort(key=lambda r: (r["cone_res"], r["rel_x"]))
    print(f"{'solver':6s} {'scale':6s} {'params':32s} {'it':>6s} {'t(s)':>6s} "
          f"{'cone_res':>10s} {'obj':>10s} {'rel_x':>9s}  {'status'}")
    for r in results:
        p = r["cfg"][2]
        ps = ",".join(f"{k}={v}" for k,v in p.items())
        print(f"{r['cfg'][0]:6s} {r['cfg'][1]:6s} {ps:32s} {r['iters']:6d} "
              f"{r['time_s']:6.2f} {r['cone_res']:10.2e} {r['obj']:10.4f} "
              f"{r['rel_x']:9.2e}  {r['status']}")
