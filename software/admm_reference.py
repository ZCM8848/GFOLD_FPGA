#!/usr/bin/env python3
"""
numpy ADMM golden reference for the G-FOLD SOCP (v2: sparse solves + rescaling).

Solves  min q'x  s.t.  Ax - b in C  (C = zero x nonneg x SOC cones).

Method: SCS/OSQP-style ADMM with one-time sparse factorization of (rho*A'A + eps*I)
and diagonal rescaling for conditioning (also what the fixed-point hardware needs).
    x-step : x = M^{-1} (rho*A'(z+b-u) - q)          M factored once (sparse LU)
    z-step : z = proj_C(Ax - b + u)
    u-step : u += (Ax - b) - z
"""
import json, time
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

with open("problem.json") as f:
    d = json.load(f)
nvars, nrows = d["nvars"], d["nrows"]
colptr = np.array(d["colptr"], dtype=np.int64)
rowval = np.array(d["rowval"], dtype=np.int64)
nzval  = np.array(d["nzval"],  dtype=np.float64)
A = sp.csc_matrix((nzval, rowval, colptr), shape=(nrows, nvars)).tocsr()
b = np.array(d["b"], dtype=np.float64)
q = np.array(d["q"], dtype=np.float64)
x_ref = np.array(d["x"], dtype=np.float64)
obj_ref = d["objective"]

cones = d["cones"]
blocks = []
row = 0
for c in cones:
    blocks.append((c["kind"], row, c["dim"])); row += c["dim"]

def proj_soc(v, dim):
    t = v[0]; x = v[1:]; nx = np.linalg.norm(x)
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t) / 2.0
    out = np.empty_like(v); out[0] = lam; out[1:] = (lam/nx)*x
    return out

def project_full(v):
    out = v.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    out[s:s+dim] = 0.0
        elif kind == "nonneg": out[s:s+dim] = np.maximum(v[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(v[s:s+dim], dim)
    return out

def run_admm(rho=1.0, eps=1e-10, scale=True, max_iter=60000, tol=1e-7, report=True):
    Ac, bc, qc = A, b, q
    Dr, Dc, E, Einv = None, None, None, None
    # ---- diagonal rescaling (OSQP/SCS-style) ----
    if scale:
        anorm = np.asarray(Ac.power(2).sum(axis=1)).ravel()**0.5
        Dr = 1.0/np.maximum(anorm, 1e-6)
        A1 = sp.diags(Dr) @ Ac
        cnorm = np.asarray(A1.power(2).sum(axis=0)).ravel()**0.5
        Dc = 1.0/np.maximum(cnorm, 1e-6)
        Ac = sp.diags(Dr) @ Ac @ sp.diags(Dc)
        bc = Dr * bc
        E = Dc; Einv = 1.0/Dc
        qc = Dc * qc
    M = (rho * (Ac.T @ Ac) + eps * sp.eye(nvars)).tocsc()
    lu = splu(M)

    x = np.zeros(nvars); z = np.zeros(nrows); u = np.zeros(nrows)
    At = Ac.T
    hist = []
    t0 = time.time()
    for it in range(1, max_iter+1):
        r = rho * (At @ (z + bc - u)) - qc
        x = lu.solve(r)
        w = Ac @ x - bc + u
        z = project_full(w)
        u = u + (Ac @ x - bc) - z
        if report and (it % 1000 == 0 or it == 1):
            rp = np.linalg.norm(Ac@x - bc - z)
            hist.append((it, rp, qc@x))
            print(f"  iter {it:6d}  ||Ax-b-z||={rp:.3e}  obj={qc@x:.6f}", flush=True)
        if it % 1000 == 0:
            rp = np.linalg.norm(Ac@x - bc - z)
            if rp < tol: break
    # unscale
    if scale: x = Einv * x
    return x, it, hist, (lu, Ac, bc, qc)

if __name__ == "__main__":
    print(f"nvars={nvars} nrows={nrows} nnz(A)={d['nnz_a']}")
    print(f"clarabel ref: obj={obj_ref:.6f} iters={d['iterations']}\n")
    for rho in [1.0, 10.0, 100.0]:
        t0=time.time(); x,it,hist,_=run_admm(rho=rho,report=True); dt=time.time()-t0
        obj=q@x; errx=np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
        print(f"\nrho={rho}: {it} iters in {dt:.1f}s, obj={obj:.6f} (ref {obj_ref:.6f}), "
              f"rel||x-x_ref||={errx:.3e}")
