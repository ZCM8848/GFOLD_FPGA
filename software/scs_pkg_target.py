#!/usr/bin/env python3
"""Run the scs package to full convergence with adaptive_scale on.
Reports final scale, rel_x, iters, and whether scale collapses like my port."""
import json, os, numpy as np, scs, scipy.sparse as sp

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV))
b = np.array(D["b"]); c = np.array(D["q"]); x_ref = np.array(D["x"])

for eps in [1e-4, 1e-5, 1e-6]:
    sol = scs.solve({"A": A, "b": b, "c": c},
                    {"z": 706, "l": 301, "q": [4]*200 + [3]*100},
                    max_iters=200000, eps_abs=eps, eps_rel=eps,
                    adaptive_scale=True, normalize=False, rho_x=1e-6,
                    acceleration_lookback=10, verbose=False)
    x = np.array(sol["x"]); info = sol["info"]
    relx = np.linalg.norm(x - x_ref)/np.linalg.norm(x_ref)
    print(f"eps={eps:.0e}: status={info['status']} iter={info['iter']} "
          f"obj={x@c:.6f} rel_x={relx:.3e} res_pri={info['res_pri']:.2e} "
          f"res_dual={info['res_dual']:.2e} scale_updates={info.get('scale_updates','?')}")
