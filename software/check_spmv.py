"""Check spmv_fp64 (small): CASE 0 = A@vx, CASE 1 = A^T@vy."""
import sys, struct, re
import numpy as np
import scipy.sparse as sp
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
N, M, NNZ = 10, 20, 41
row = np.array([int(l.strip(),16) for l in open('rtl/data/kkt/small/Arow.hex')])
col = np.array([int(l.strip(),16) for l in open('rtl/data/kkt/small/Acol.hex')])
val = np.array([dec(l) for l in open('rtl/data/kkt/small/Aval.hex')])
A = sp.coo_matrix((val,(row,col)), shape=(M,N)).tocsr()
vx = np.array([dec(l) for l in open('rtl/data/kkt/small/vx.hex')])
vy = np.array([dec(l) for l in open('rtl/data/kkt/small/vy.hex')])
exp = {0: A@vx, 1: A.T@vy}
got = {}
for line in open(sys.argv[1] if len(sys.argv)>1 else 'rtl/sim/spmv_dump.txt'):
    m = re.search(r'CASE (\d+):(.*)', line)
    if m: got[int(m.group(1))] = np.array([dec(h) for h in m.group(2).split()])
for mode, L in [(0,M),(1,N)]:
    rel = np.abs(got[mode]-exp[mode])/(np.abs(exp[mode])+1e-30)
    print(f"transpose={mode}: len={len(got[mode])} max_rel={rel.max():.3e} "
          f"#bad(>1e-13)={np.sum(rel>1e-13)}")
    assert len(got[mode])==L, f"len {len(got[mode])} != {L}"
    assert rel.max() <= 1e-13, f"transpose={mode} FAIL"
print("PASS")
