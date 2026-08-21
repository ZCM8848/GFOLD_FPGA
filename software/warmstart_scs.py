#!/usr/bin/env python3
"""
Validate warm-start across tof using the scs package (adaptive-scaling solver).

For each tof, build a fresh scs.SCS (new A) and solve, warm-starting x,y,s from
the previous tof's solution. Compares total iterations + estimated hardware time
vs an all-cold run. This is the honest adaptive-TOF timing validation (the
non-adaptive numpy core cannot converge, so we use scs).
"""
import numpy as np, time
import scs
from assembler import assemble, N

def solve_tof_scs(tof, prev=None, eps=1e-5, max_iters=300000):
    A, b, c, _, _ = assemble(tof)
    data = {"A": A.tocsc(), "b": b, "c": c}
    if prev is not None:
        data["x"], data["y"], data["s"] = prev
    settings = {"max_iters": max_iters, "eps_abs": eps, "eps_rel": eps,
                "normalize": False, "adaptive_scale": True,
                "rho_x": 1e-6, "acceleration_lookback": 10, "verbose": False}
    t0 = time.time()
    sol = scs.solve(data, {"z": 706, "l": 301, "q": [4]*200 + [3]*100}, **settings)
    dt = time.time() - t0
    x = sol["x"]; it = sol["info"]["iter"]
    fm = np.exp(x[10*N - 1])  # final_mass
    return sol, it, dt, fm

def run_sequence(tofs, warm):
    total_it = 0; total_t = 0; prev = None; its = []
    for t in tofs:
        sol, it, dt, fm = solve_tof_scs(t, prev=prev if warm else None)
        total_it += it; total_t += dt; its.append(it)
        prev = (sol["x"], sol["y"], sol["s"]) if warm else None
    return total_it, total_t, its

if __name__ == "__main__":
    # representative tof sequence (golden-section-style path through the feasible window)
    tofs = [38.0, 40.0, 42.0, 44.0, 45.0, 46.0, 47.0]
    print("tof sequence:", tofs, "\n")
    for eps in [1e-4, 1e-5]:
        print(f"=== eps={eps:.0e} ===")
        it_c, t_c, its_c = run_sequence(tofs, warm=False)
        it_w, t_w, its_w = run_sequence(tofs, warm=True)
        print(f"COLD: total iters={it_c}  per-tof={its_c}  wall={t_c:.1f}s")
        print(f"WARM: total iters={it_w}  per-tof={its_w}  wall={t_w:.1f}s")
        ratio = it_c/max(it_w,1)
        print(f"warm-start speedup: {ratio:.1f}x   (iters saved {it_c-it_w})")
        print(f"HW est: cold {it_c} x ~1500cyc @100MHz = {it_c*1500/1e8:.1f}s | "
              f"warm {it_w} -> {it_w*1500/1e8:.1f}s")
        print()
