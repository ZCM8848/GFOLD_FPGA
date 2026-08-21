#!/usr/bin/env python3
"""Diagnose dominant fixed-point error: isolate each quantization point with
Q(32,16) at a fixed iteration budget."""
import sys, time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from fixed_point import Q
from fixed_adaptive import solve_fixed_adaptive

Qf = Q(32, 16)
cases = [
    ("all-off(float)", set()),
    ("v only",        {"v"}),
    ("ut only",       {"ut"}),
    ("cone only",     {"cone"}),
    ("rsk only",      {"rsk"}),
    ("u only",        {"u"}),
    ("ALL",           {"v", "ut", "cone", "rsk", "u"}),
]


def one(args):
    name, which, max_iter = args
    t0 = time.time()
    try:
        r = solve_fixed_adaptive(max_iter, Qf, which=which)
        return name, r["relx"], r["obj"], r["n_scale_updates"], time.time() - t0
    except Exception as e:
        return name, float("nan"), float("nan"), -1, time.time() - t0, type(e).__name__


if __name__ == "__main__":
    MAX_ITER = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=len(cases)) as ex:
        results = list(ex.map(one, [(nm, wh, MAX_ITER) for nm, wh in cases]))
    print(f"Q(32,16) budget={MAX_ITER}  wall={time.time()-t0:.0f}s")
    for r in results:
        if len(r) == 6:
            nm, relx, obj, upd, dt, err = r
            print(f"{nm:>14}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s] FAIL:{err}")
        else:
            nm, relx, obj, upd, dt = r
            print(f"{nm:>14}: rel_x={relx:.3e} obj={obj:.6f} upd={upd} [{dt:.0f}s]")
