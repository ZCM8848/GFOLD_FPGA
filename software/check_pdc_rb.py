"""Check proj_dual_cone_rb output vs software oracle (reordered frame)."""
import sys, struct
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def rd(f): return np.array([dec(l) for l in open(f)])
got = rd('rtl/sim/pdc_rb_out.hex')
exp = rd('rtl/data/kkt/full/pdc_out.hex')
rel = np.abs(got - exp) / (np.abs(exp) + 1e-30)
print(f'proj_dual_cone_rb: max_rel={rel.max():.3e}  #bad(>1e-10)={np.sum(rel>1e-10)}')
assert rel.max() <= 1e-10, 'FAIL'
print('PASS')
