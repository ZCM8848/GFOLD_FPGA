#!/usr/bin/env python3
"""Sensitivity of the BFP solver (mant=32, WITH adaptive scaling) to KKT-solve
output noise at the wide-float-32 level (rel~1e-3) vs the float32 level (2.8e-2).
kkt_noise is injected inside solve_bfp after the KKT solve; noise=0 is the
validated baseline (must reach rel_x~1e-3 with adaptive scaling)."""
import sys, time
from fixed_bfp import solve_bfp

def main():
    max_iter = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    print(f"== BFP state mant=32, {max_iter} iters (adaptive scaling ON) ==")
    for nl, tag in [(0.0, "float64 LDL (baseline)"),
                    (1e-3, "wide-float-32 (1e-3)"),
                    (3e-3, "3e-3"),
                    (2.8e-2, "float32 (2.8e-2)")]:
        t0 = time.time()
        r = solve_bfp(max_iter, 32, track=max_iter // 10, kkt_noise=nl)
        dt = time.time() - t0
        print(f"  noise={nl:g} ({tag}): rel_x={r['relx']:.4e} obj={r['obj']:.6f} "
              f"scale={r['scale']:.2e} upd={r['n_scale_updates']} ({dt:.0f}s)")

if __name__ == "__main__":
    main()
