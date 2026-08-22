"""Check s_build band vs gen_kkt_test oracle band_f64.hex, AND end-to-end:
feed the BUILT band into the banded LDL solve of rhs_x and compare zx vs the
oracle zx.hex (proves the built band is not just close but usable).
Usage: python check_sbuild.py [dump] [small|full]
Tolerances: band 1e-12 (sum-order rounding differs from numpy); zx 1e-9 full /
1e-13 small (LDL conditioning).
"""
import sys, struct, re, json
import numpy as np
import scipy.sparse as sp
sys.path.insert(0, 'software')
from banded_reference import banded_ldl_factor_solve, RHO_X

def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
case = sys.argv[2] if len(sys.argv) > 2 else 'small'
man = json.load(open(f'rtl/data/kkt/{case}/manifest.json'))
N, M, HB = man['n'], man['m'], man['hb']

# oracle band (k-major: B[i][k] = S[k+i][k])
oracle = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/band_f64.hex')])
oracle = oracle.reshape((HB+1, N), order='F')  # k-major -> [i][k]

# got band from dump
got = np.zeros((HB+1, N))
for line in open(sys.argv[1] if len(sys.argv) > 1 else 'rtl/sim/sbuild_dump.txt'):
    m = re.search(r'COL (\d+):(.*)', line)   # search, not match: ModelSim prefixes "# "
    if m:
        k = int(m.group(1))
        got[:, k] = [dec(h) for h in m.group(2).split()]
rel = np.abs(got - oracle) / (np.abs(oracle) + 1e-30)
print(f'[{case}] band: max_rel={rel.max():.3e}  #bad(>1e-12)={np.sum(rel>1e-12)}')

# ---- end-to-end: solve with the BUILT band ----
rp = np.array([int(l.strip(), 16) for l in open(f'rtl/data/kkt/{case}/Arow.hex')])
cp = np.array([int(l.strip(), 16) for l in open(f'rtl/data/kkt/{case}/Acol.hex')])
Av = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/Aval.hex')])
Ab = sp.coo_matrix((Av, (rp, cp)), shape=(M, N)).tocsr()
vx = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/vx.hex')])
vy = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/vy.hex')])
rhs_x = RHO_X * vx - Ab.T @ vy
# rebuild sparse S from the built band
rows, cols, vals = [], [], []
for k in range(N):
    for i in range(HB+1):
        if k + i < N and got[i, k] != 0.0:
            rows.append(k + i); cols.append(k); vals.append(got[i, k])
Sb = sp.coo_matrix((vals, (rows, cols)), shape=(N, N)).tocsr()
zx_built = banded_ldl_factor_solve(Sb, rhs_x, hb=HB)
zx_exp = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/zx.hex')])
relz = np.abs(zx_built - zx_exp) / (np.abs(zx_exp) + 1e-30)
tolz = 1e-13 if case == 'small' else 1e-9
print(f'[{case}] e2e zx (built band -> LDL): max_rel={relz.max():.3e}  #bad(>{tolz})={np.sum(relz>tolz)}')
assert rel.max() <= 1e-12, f'{case} band FAIL'
assert relz.max() <= tolz, f'{case} e2e FAIL'
print('PASS')
