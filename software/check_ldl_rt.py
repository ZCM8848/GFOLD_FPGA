"""Check banded_ldl_fp64_rt zx vs exp_zx (FP64)."""
import sys, struct, re
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def rd(f): return np.array([dec(l) for l in open(f)])
N = int(sys.argv[2]) if len(sys.argv) > 2 else 8
WHICH = int(sys.argv[3]) if len(sys.argv) > 3 else 0
exp = rd(f'rtl/data/{"full_f64" if WHICH else "small_f64"}/exp_zx_f64.hex')
got = None
for line in open(sys.argv[1] if len(sys.argv) > 1 else 'rtl/sim/ldlrt_dump.txt'):
    m = re.search(r'CASE 0:(.*)', line)
    if m: got = np.array([dec(h) for h in m.group(1).split()[:N]])
if got is None: print("no CASE found"); sys.exit(1)
tol = 1e-12 if WHICH == 0 else 1e-8   # full-size S is cond~1e6, documented max_rel ~3.2e-9
rel = np.abs(got - exp) / (np.abs(exp) + 1e-30)
print(f"N={N}: max_rel={rel.max():.3e}  #bad(>{tol})={np.sum(rel>tol)}")
if rel.max() > tol:
    for i in np.where(rel > tol)[0][:8]:
        print(f"  [{i}] got={got[i]:.16e} exp={exp[i]:.16e}")
print("PASS" if rel.max() <= tol else "FAIL")
