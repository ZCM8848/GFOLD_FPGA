#!/usr/bin/env python3
"""Decisive: is one-time normalization enough, or is adaptive scaling required?
Tests scs with {normalize} x {adaptive_scale} -> is fixed-scaling (hardware B) viable?"""
import json, time, os
import numpy as np, scipy.sparse as sp
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(__file__)
D = json.load(open(os.path.join(HERE, "problem.json")))
A0 = sp.csc_matrix((np.array(D["nzval"]), np.array(D["rowval"]), np.array(D["colptr"])),
                   shape=(D["nrows"], D["nvars"]))
b0 = np.array(D["b"]); q0 = np.array(D["q"]); x_ref = np.array(D["x"])
blocks = []; r = 0
for c in D["cones"]:
    blocks.append((c["kind"], r, c["dim"])); r += c["dim"]

def run(cfg):
    norm, adap = cfg
    import scs
    z_dim = sum(c[2] for c in blocks if c[0]=="zero")
    l_dim = sum(c[2] for c in blocks if c[0]=="nonneg")
    qd = [c[2] for c in blocks if c[0]=="soc"]
    t0 = time.time()
    settings = {"max_iters":50000, "eps_abs":1e-6, "eps_rel":1e-6,
                "normalize":bool(norm), "adaptive_scale":bool(adap),
                "rho_x":1e-6, "acceleration_lookback":10, "verbose":False}
    sol = scs.solve({"A":A0,"b":b0,"c":q0}, {"z":z_dim,"l":l_dim,"q":qd}, **settings)
    dt = time.time()-t0
    x = sol["x"]; obj = q0@x
    relx = np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
    return {"norm":norm,"adap":adap,"it":sol["info"]["iter"],"status":sol["info"]["status"],
            "obj":float(obj),"rel_x":float(relx),"t":round(dt,1)}

if __name__ == "__main__":
    cfgs = [(n,a) for n in [0,1] for a in [0,1]]
    print(f"{len(cfgs)} configs, 40 cores\n")
    with ProcessPoolExecutor(max_workers=min(40, os.cpu_count() or 4)) as ex:
        res = list(ex.map(run, cfgs))
    res.sort(key=lambda r: r["rel_x"])
    print(f"{'normalize':>9s} {'adaptive':>8s} {'it':>6s} {'t(s)':>6s} {'obj':>10s} {'rel_x':>9s}  status")
    for r in res:
        print(f"{str(bool(r['norm'])):>9s} {str(bool(r['adap'])):>8s} {r['it']:6d} {r['t']:6.1f} "
              f"{r['obj']:10.4f} {r['rel_x']:9.2e}  {r['status']}")
    print("\nKEY: if a config with adaptive=False converges to rel_x<1e-3, fixed-scaling (HW path B) works.")
