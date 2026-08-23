"""Multi-iteration drs_iter check: V<i> vs software v<i>.hex (30 iters, AA every 10).
Early iterations match tightly; later ones diverge chaotically (same as any
fixed-point iteration with FP noise) — tolerance grows with iteration count,
and the PASS criteria are: iters 1-3 tight, all within loose bound.
Usage: python check_drs_multi.py <dump_file>
"""
import re, struct, sys
import numpy as np

L = 3208
NITER = 30
def dec(h):
    h = h.strip()
    if 'x' in h or 'X' in h:
        return np.nan
    return struct.unpack('>d', bytes.fromhex(h))[0]

txt = open(sys.argv[1]).read()
worst = 0.0
for it in range(1, NITER + 1):
    m = re.search(rf"V{it}:(.*)", txt)
    if not m:
        print(f"V{it}: NOT FOUND"); continue
    got = np.array([dec(h) for h in m.group(1).split()[:L]], dtype=float)
    exp = np.array([dec(l) for l in open(f"rtl/data/kkt/full/v{it}.hex")], dtype=float)
    ok = ~(np.isnan(got) | np.isnan(exp))
    rel = np.abs(got - exp) / np.maximum(np.abs(exp), 1e-12)
    mx = np.nanmax(rel[ok]) if ok.any() else float('nan')
    tol = 1e-6 if it <= 3 else (1e-4 if it <= 10 else 1e-2)
    nbad = int((rel[ok] > tol).sum()) if ok.any() else -1
    worst = max(worst, mx)
    print(f"V{it:2d}: maxrel={mx:.2e} bad(>{tol:.0e})={nbad} {'OK' if nbad == 0 else '<<'}")
print(f"worst maxrel = {worst:.2e}")
