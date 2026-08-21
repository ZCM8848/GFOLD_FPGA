#!/usr/bin/env python3
"""Phase 2c: sweep sqrt/div precision in the SOC projection (BFP mant=32)."""
import sys, time
from concurrent.futures import ProcessPoolExecutor
from fixed_bfp import solve_bfp

MANT = 32


def one(args):
    sqrt_p, div_p, max_iter = args
    t0 = time.time()
    try:
        r = solve_bfp(max_iter, MANT, soc_sqrt_p=sqrt_p, soc_div_p=div_p)
        return (sqrt_p, div_p, r["relx"], r["obj"], time.time() - t0)
    except Exception as e:
        return (sqrt_p, div_p, float("nan"), float("nan"), time.time() - t0, type(e).__name__)


if __name__ == "__main__":
    MAX_ITER = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    mode = sys.argv[2] if len(sys.argv) > 2 else "sqrt"
    if mode == "sqrt":
        cases = [(p, None, MAX_ITER) for p in [None, 8, 12, 16, 20, 24, 28, 32]]
        label = "sqrt output precision (div exact)"
    else:
        cases = [(None, p, MAX_ITER) for p in [None, 8, 12, 16, 20, 24, 28, 32]]
        label = "div output precision (sqrt exact)"
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=len(cases)) as ex:
        results = list(ex.map(one, cases))
    print(f"BFP mant={MANT} @{MAX_ITER}  {label}  wall={time.time()-t0:.0f}s  (float ref 2.16e-3)")
    for r in results:
        if len(r) == 6:
            sp, dp, relx, obj, dt, err = r
            val = sp if mode == "sqrt" else dp
            print(f"  {mode}={str(val):>5}: rel_x={relx:.3e} obj={obj:.6f} [{dt:.0f}s] FAIL:{err}")
        else:
            sp, dp, relx, obj, dt = r
            val = sp if mode == "sqrt" else dp
            print(f"  {mode}={str(val):>5}: rel_x={relx:.3e} obj={obj:.6f} [{dt:.0f}s]")
