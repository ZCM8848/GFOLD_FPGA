#!/usr/bin/env python3
"""Wide-format sweep: find the bit width where all-on fixed-point reaches ~1e-3."""
import sys, time
from concurrent.futures import ProcessPoolExecutor
from fixed_point import Q
from fixed_adaptive import solve_fixed_adaptive

formats = [(32, 24), (40, 28), (40, 32), (48, 36), (48, 40), (64, 48)]


def one(args):
    fmt, max_iter = args
    m, n = fmt
    t0 = time.time()
    try:
        r = solve_fixed_adaptive(max_iter, Q(m, n))
        return fmt, r["relx"], r["obj"], r["n_scale_updates"], time.time() - t0
    except Exception as e:
        return fmt, float("nan"), float("nan"), -1, time.time() - t0, type(e).__name__


if __name__ == "__main__":
    MAX_ITER = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=len(formats)) as ex:
        results = list(ex.map(one, [(fmt, MAX_ITER) for fmt in formats]))
    print(f"budget={MAX_ITER}  wall={time.time()-t0:.0f}s  target ~1e-3")
    for r in results:
        if len(r) == 6:
            fmt, relx, obj, upd, dt, err = r
            print(f"{str(Q(*fmt)):>9}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s] FAIL:{err}")
        else:
            fmt, relx, obj, upd, dt = r
            print(f"{str(Q(*fmt)):>9}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s]")
