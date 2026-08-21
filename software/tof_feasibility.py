#!/usr/bin/env python3
"""Diagnose which tof are feasible vs infeasible, and how fast infeasible ones
terminate. In the real golden-section search, infeasible tof (short flights)
should be detected quickly and skipped -- that dominates the timing budget."""
import numpy as np, time
import scs
from assembler import assemble, N

def solve_one(tof, eps=1e-4, max_iters=300000, prev=None):
    A, b, c, _, _ = assemble(tof)
    data = {"A": A.tocsc(), "b": b, "c": c}
    if prev is not None: data["x"], data["y"], data["s"] = prev
    settings = {"max_iters": max_iters, "eps_abs": eps, "eps_rel": eps,
                "normalize": False, "adaptive_scale": True, "rho_x": 1e-6,
                "acceleration_lookback": 10, "verbose": False}
    sol = scs.solve(data, {"z":706,"l":301,"q":[4]*200+[3]*100}, **settings)
    it = sol["info"]["iter"]; st = sol["info"]["status"]
    fm = np.exp(sol["x"][10*N-1])
    return it, st, fm

if __name__ == "__main__":
    print(f"{'tof':>6} {'iters':>8} {'status':>22} {'final_mass':>10}")
    for tof in [30, 34, 36, 38, 40, 41, 42, 43, 44, 45, 46, 48, 50, 55, 60, 70]:
        t0=time.time()
        it, st, fm = solve_one(tof, eps=1e-4)
        print(f"{tof:6.0f} {it:8d} {st:>22} {fm:10.1f}   ({time.time()-t0:.1f}s)")
