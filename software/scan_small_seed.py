#!/usr/bin/env python3
"""Scan random seeds for the small instance and pick one where the iter-1
root_plus discriminant is comfortably positive (so truncating FP64 and RNE
agree on the branch)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

import gen_small as G

def build(seed):
    rng = np.random.default_rng(seed)
    rows, cols, vals = [], [], []
    for i in range(G.M):
        base = (i * (G.N - 3)) // max(G.M - 1, 1)
        for off in (0, 1, 2):
            rows.append(i); cols.append(base + off); vals.append(rng.uniform(-1, 1))
    A = sp.coo_matrix((vals, (rows, cols)), shape=(G.M, G.N)).tocsr()
    br = rng.uniform(-1, 1, G.M)
    cr = rng.uniform(-1, 1, G.N)
    zmask = G.build_zmask(); blocks = G.build_blocks()
    r_y = np.where(zmask, 1.0 / 1000.0, 1.0)
    diag_r = np.concatenate([np.full(G.N, G.RHO_X), r_y, [G.TAU_FACTOR]])
    K = sp.bmat([[sp.diags(diag_r[:G.N], format="csc"), A.T],
                 [A, sp.diags(-diag_r[G.N:G.N + G.M], format="csc")]], format="csc")
    lu = splu(K)
    g = lu.solve(np.concatenate([cr, -br]))
    return A, br, cr, blocks, diag_r, lu, g, r_y

def rad_after_iter0(A, br, cr, blocks, diag_r, lu, g):
    n, m = A.shape[1], A.shape[0]
    l = n + m + 1
    v = np.zeros(l); v[l - 1] = 1.0
    v_prev = np.zeros(l)
    v, u_t, u, rsk = G.one_iteration(A, br, cr, blocks, diag_r, lu, g, v, v_prev, 0)
    # iter-1 normalize
    vn = np.linalg.norm(v)
    v = v * (np.sqrt(l) / vn)
    u_t = np.empty(l)
    u_t[:n] = v[:n] * diag_r[:n]
    u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
    u_t[l - 1] = v[l - 1]
    u_t[:l - 1] = lu.solve(u_t[:l - 1])
    p = u_t[:l - 1]; mu = v[:l - 1]; eta = v[l - 1]; r = diag_r
    gg = np.sum(p * p * r[:l - 1])  # placeholder, recomputed below
    gg = mug = pg = pp = pmu = 0.0
    for i in range(len(p)):
        gg += g[i] * g[i] * r[i]
        mug += mu[i] * g[i] * r[i]
        pg += p[i] * g[i] * r[i]
        pp += p[i] * p[i] * r[i]
        pmu += p[i] * mu[i] * r[i]
    a = r[-1] + gg
    b = mug - 2 * pg - eta * r[-1]
    cc = pp - pmu
    rad = b * b - 4 * a * cc
    tau = G.root_plus(diag_r, g, p, mu, eta)
    return rad, tau, a, b, cc

if __name__ == '__main__':
    ok = []
    for seed in range(1, 61):
        A, br, cr, blocks, diag_r, lu, g, r_y = build(seed)
        try:
            rad, tau, a, b, cc = rad_after_iter0(A, br, cr, blocks, diag_r, lu, g)
        except Exception as e:
            print(seed, "ERR", e); continue
        ratio = rad / (b * b) if b*b else 0
        tag = "OK" if rad > 0.02 and ratio > 1e-6 else ""
        print(f"seed={seed:2d} rad={rad:+.6e} rad/b2={ratio:+.3e} tau={tau:+.6e} {tag}")
        if tag:
            ok.append((seed, rad, tau))
    print("good seeds:", [s for s, _, _ in ok][:10])
