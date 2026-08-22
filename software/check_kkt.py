"""Check kkt_solve output (rtl/sim/kkt_dump.txt) vs generated zx/zy expected.
Usage: python check_kkt.py [dump] [small|full]
Tolerances (truncating FP64 + ill-conditioned S): small 1e-12, full 1e-8.
"""
import sys, struct, re, json
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
case = sys.argv[2] if len(sys.argv) > 2 else 'small'
man = json.load(open(f'rtl/data/kkt/{case}/manifest.json'))
N, M = man['n'], man['m']
zx = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/zx.hex')])
zy = np.array([dec(l) for l in open(f'rtl/data/kkt/{case}/zy.hex')])
exp = np.concatenate([zx, zy])
got = None
for line in open(sys.argv[1] if len(sys.argv) > 1 else 'rtl/sim/kkt_dump.txt'):
    m = re.search(r'CASE 0:(.*)', line)
    if m: got = np.array([dec(h) for h in m.group(1).split()])
assert got is not None, "no CASE 0 line in dump"
assert len(got) == N + M, f"len {len(got)} != {N+M}"
rel = np.abs(got - exp) / (np.abs(exp) + 1e-30)
tol = 1e-12 if case == 'small' else 1e-8
relx = np.abs(got[:N] - zx) / (np.abs(zx) + 1e-30)
rely = np.abs(got[N:] - zy) / (np.abs(zy) + 1e-30)
print(f'[{case}] n={N} m={M}  got_len={len(got)}')
print(f'  zx max_rel={relx.max():.3e}   zy max_rel={rely.max():.3e}')
print(f'  overall max_rel={rel.max():.3e}  n_bad(>{tol})={np.sum(rel>tol)}')
assert rel.max() <= tol, f'{case} FAIL'
print('PASS')
