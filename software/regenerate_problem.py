#!/usr/bin/env python3
"""Reassemble the G-FOLD SOCP and write it back to problem.json.

After editing the initial state / physical parameters in assembler.py
(INIT_POS, INIT_VEL, WET, FUEL, TGT_*, GRAV, MAX_VEL, REAL_MAX_T, ...), run:

    python regenerate_problem.py [tof]

to rebuild A/b/q from assemble(tof) and update problem.json. The cone structure
is fixed (706 zero + 301 nonneg + 200 SOC dim4 + 100 SOC dim3); only the numeric
values change, so the problem SIZE (nvars/nrows/nnz) is unchanged as long as
N=100 and tof are kept.

The reference solution fields (x/objective/iterations/solve_ms) are NOT
recomputed here — they come from an external IPM (Clarabel). They are carried
over unchanged and flagged stale; re-solve separately if you need a fresh
golden reference for check_*.py.
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TOF = 46.6093


def main():
    tof = float(sys.argv[1]) if len(sys.argv) > 1 else TOF

    from assembler import assemble  # imports numpy + scipy
    A, b, q, nrows, nvars = assemble(tof)
    A = A.tocsc()

    # fixed cone structure (matches assembler row order)
    cones = [{"kind": "zero", "dim": 706},
             {"kind": "nonneg", "dim": 301}]
    cones += [{"kind": "soc", "dim": 4} for _ in range(200)]  # velocity + thrust_slack
    cones += [{"kind": "soc", "dim": 3} for _ in range(100)]  # thrust_lower

    nnz = int(A.nnz)
    d = {
        "horizon": 100,
        "nvars": int(nvars),
        "nrows": int(nrows),
        "nnz_a": nnz,
        "colptr": [int(x) for x in A.indptr],
        "rowval": [int(x) for x in A.indices],
        "nzval": [float(x) for x in A.data],
        "q": [float(x) for x in q],
        "b": [float(x) for x in b],
        "cones": cones,
        "time_of_flight": tof,
    }

    p = os.path.join(HERE, "problem.json")
    old = {}
    if os.path.exists(p):
        old = json.load(open(p))
    for k in ("x", "objective", "iterations", "solve_ms"):
        if k in old:
            d[k] = old[k]

    json.dump(d, open(p, "w"), indent=2)
    print(f"wrote problem.json: nvars={nvars} nrows={nrows} nnz={nnz} tof={tof}")
    print("NOTE: x/objective/iterations/solve_ms are carried over (stale) — "
          "re-solve with an external IPM if a fresh golden reference is needed.")


if __name__ == "__main__":
    main()
