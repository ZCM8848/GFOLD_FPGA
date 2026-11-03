#!/usr/bin/env python3
"""check_small.py — compare solver_core VS1/VS2 dump lines with the small2 golden."""
import sys, os, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KKT = os.path.join(HERE, "..", "rtl", "data", "kkt", "small2")
L = 60
TOL = 5e-10


def rd(name):
    return np.array([struct.unpack(">d", bytes.fromhex(x.strip()))[0]
                     for x in open(os.path.join(KKT, name)) if x.strip()])


def line(tag, path):
    for ln in open(path):
        if ln.startswith(tag + ":"):
            return ln.split(":", 1)[1].split()
    return []


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "rtl", "sim", "core_sim_output.txt")
    ok = True
    for tag, name in (("VS1", "v1.hex"), ("VS2", "v2.hex")):
        ref = rd(name)[:L]
        hs = line(tag, path)
        got = []
        for h in hs[:L]:
            try:
                got.append(struct.unpack(">d", bytes.fromhex(h))[0])
            except ValueError:
                got.append(np.nan)
        got = np.array(got)
        if len(got) < L:
            print(f"{tag}: only {len(got)} words")
            ok = False
            continue
        rel = np.abs(got - ref) / np.maximum(np.abs(ref), 1e-12)
        bad = int((rel > TOL).sum())
        nn = int(np.isnan(got).sum())
        print(f"{tag}: nan={nn} bad(>{TOL:.0e})={bad}/{L} maxrel={np.nanmax(rel):.3e}")
        ok = ok and (bad == 0) and (nn == 0)
    print("SMALL CORE CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
