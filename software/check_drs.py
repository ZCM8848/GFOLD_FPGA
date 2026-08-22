"""Check drs_iter one-iteration output vs drs_reference oracle (full problem).
V1 = iter0 v (FEAS), V2 = iter1 v (normalize+root_plus). Tolerances: truncating
FP64 through a cond-1e6 KKT -> expect ~1e-8 relative; V2 (root_plus + normalize)
a bit looser.
Usage: python check_drs.py <dump_file>
"""
import re, struct, sys
import numpy as np

L = 3208
def dec(h):
    h = h.strip()
    if 'x' in h or 'X' in h:
        return np.nan
    return struct.unpack('>d', bytes.fromhex(h))[0]

def rd(p):
    return np.array([dec(l) for l in open(p)], dtype=float)

def compare(tag, got, exp, tol, atol=0.0):
    ok = ~(np.isnan(got) | np.isnan(exp))
    rel = np.abs(got - exp) / np.maximum(np.abs(exp), 1e-12)
    bad = rel > tol
    if atol > 0:
        bad = bad & (np.abs(got - exp) > atol)
    nbad = int(bad[ok].sum()) if ok.any() else -1
    nn = int((~ok).sum())
    mx = np.nanmax(rel[ok]) if ok.any() else float('nan')
    print(f"{tag}: nan={nn} bad(>{tol:.0e})={nbad}/{len(exp)} maxrel={mx:.2e}")
    return nbad + nn

txt = open(sys.argv[1]).read()
tot = 0
for tag, ref, tol in [("V1", "v1", 1e-7), ("V2", "v2", 1e-6), ("UT2", "ut2", 1e-7), ("U2", "u2", 1e-7), ("RSK2", "rsk2", 1e-7),
                      ("UT1", "ut1", 1e-7), ("U1", "u1", 1e-7),
                      ("RSK1", "rsk1", 1e-7)]:
    m = re.search(tag + r":(.*)", txt)
    if not m:
        print(f"{tag}: NOT FOUND"); continue
    got = np.array([dec(h) for h in m.group(1).split()[:L]], dtype=float)
    exp = rd(f"rtl/data/kkt/full/{ref}.hex")
    tot += compare(tag, got, exp, tol, atol=1e-15 if "RSK" in tag else 0.0)
print("PASS" if tot == 0 else f"FAIL ({tot} bad)")
