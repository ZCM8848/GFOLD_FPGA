#!/usr/bin/env python3
"""BFP mantissa-width sweep."""
import sys, time
from concurrent.futures import ProcessPoolExecutor
from fixed_bfp import solve_bfp

mants = [16, 20, 24, 28, 32]


def one(args):
    mant, max_iter = args
    t0 = time.time()
    try:
        r = solve_bfp(max_iter, mant)
        return mant, r["relx"], r["obj"], r["n_scale_updates"], time.time() - t0
    except Exception as e:
        return mant, float("nan"), float("nan"), -1, time.time() - t0, type(e).__name__


if __name__ == "__main__":
    MAX_ITER = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=len(mants)) as ex:
        results = list(ex.map(one, [(m, MAX_ITER) for m in mants]))
    print(f"BFP budget={MAX_ITER}  wall={time.time()-t0:.0f}s  (float ref 2.16e-3 @20k)")
    for r in results:
        if len(r) == 6:
            m, relx, obj, upd, dt, err = r
            print(f"mant={m:>3}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s] FAIL:{err}")
        else:
            m, relx, obj, upd, dt = r
            print(f"mant={m:>3}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s]")
