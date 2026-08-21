#!/usr/bin/env python3
"""Verify the noise-perturbed solutions are FEASIBLE and fuel-OPTIMAL.
For solve_bfp at kkt_noise=0 (baseline) and 5e-6/3e-5 (wide-float-40 levels),
compute: obj=c@x, final_mass=exp(x[z(n-1)]), and cone feasibility r=Ax-b in C.
If the noisy solutions are feasible + obj-optimal (final_mass ~1799), then
rel_x~3e-2 is just a different optimal point and obj-optimal is the right target.
"""
import numpy as np
import scipy.sparse as sp
from fixed_bfp import solve_bfp
from scs_adaptive import A, b, c, x_ref, obj_ref, blocks, NV, NR, Z

def check(x, label):
    n = len(x)
    obj = float(c @ x)
    final_mass = float(np.exp(x[n - 1]))   # z(n-1) = x[10*N + N-1] = x[n-1]
    # feasibility: r = A x - b, in cones
    r = (A @ x) - b
    maxv = np.max(np.abs(r))
    maxviol = 0.0
    off = 0
    for kind, st, dim in blocks:
        seg = r[off:off + dim]
        if kind == "zero":
            v = np.max(np.abs(seg))
        elif kind == "nonneg":
            v = -np.min(seg) if seg.min() < 0 else 0.0
        else:  # soc: first is t, rest is v; violate if ||v|| > t
            vv = seg[1:]
            v = max(0.0, np.linalg.norm(vv) - seg[0]) if vv.size else 0.0
        maxviol = max(maxviol, v)
        off += dim
    print(f"  {label}: obj={obj:.6f} (ref {obj_ref:.6f}, d={obj-obj_ref:+.4f}) "
          f"final_mass={final_mass:.3f} (ref 1799.156) "
          f"|Ax-b|max={maxv:.2e} cone_viol={maxviol:.2e}")

def main():
    print("reference:")
    check(np.array(x_ref, float), "x_ref(IPM)")
    for nl, tag in [(0.0, "baseline (float64 LDL)"), (5e-6, "wide-f40-rnd"),
                    (3e-5, "wide-f40-trunc"), (1e-4, "1e-4")]:
        r = solve_bfp(50000, 32, kkt_noise=nl)
        check(r["x"], f"noise={nl:g} ({tag})  [rel_x={r['relx']:.2e}]")

if __name__ == "__main__":
    main()
