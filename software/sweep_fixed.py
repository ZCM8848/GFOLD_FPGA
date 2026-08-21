#!/usr/bin/env python3
"""Phase 2b format sweep, parallel across cores. Runs each Q(m,n) to a fixed
iteration budget and reports final rel_x vs the float golden reference.
The float-vs-fixed delta = the quantization contribution (target <1e-4)."""
import sys, time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from fixed_point import Q
from fixed_adaptive import solve_fixed_adaptive

formats = [(18, 5), (24, 8), (24, 11), (32, 16), (32, 19), (24, 16)]


def one(args):
    fmt, max_iter = args
    m, n = fmt
    Qf = Q(m, n)
    t0 = time.time()
    try:
        r = solve_fixed_adaptive(max_iter, Qf)
        return (fmt, r["relx"], r["obj"], r["tau"], r["n_scale_updates"], r["scale"],
                r["converged_iter"], time.time() - t0)
    except Exception as e:
        return (fmt, float("nan"), float("nan"), float("nan"), -1, float("nan"),
                -1, time.time() - t0, f"FAIL: {type(e).__name__}")


if __name__ == "__main__":
    MAX_ITER = int(sys.argv[1]) if len(sys.argv) > 1 else 30000
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=len(formats)) as ex:
        results = list(ex.map(one, [(fmt, MAX_ITER) for fmt in formats]))
    print(f"budget={MAX_ITER} iters  wall={time.time()-t0:.0f}s")
    print(f"{'format':>10} {'rel_x':>10} {'obj':>10} {'tau':>9} {'upd':>4} {'scale':>9} {'iters':>7} {'t[s]':>6}")
    for res in results:
        if len(res) == 9:
            fmt, relx, obj, tau, upd, sc, it, dt, note = res
            print(f"{str(Q(*fmt)):>10} {'DIVERGED':>10} {'':>10} {'':>9} "
                  f"{'':>4} {'':>9} {'':>7} {dt:>6.0f}  {note}")
            continue
        (fmt, relx, obj, tau, upd, sc, it, dt) = res
        print(f"{str(Q(*fmt)):>10} {relx:>10.3e} {obj:>10.6f} {tau:>9.5f} "
              f"{upd:>4} {sc:>9.2e} {it:>7} {dt:>6.0f}")
