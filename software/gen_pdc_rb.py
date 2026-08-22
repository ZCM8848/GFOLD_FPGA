#!/usr/bin/env python3
"""Generate proj_dual_cone_rb test vectors (reordered frame, Phase 4b).
Input: u1[n:l-1] (real DRS iteration dual block). Expected: R_y-weighted
Moreau with the REORDERED cone blocks (drs_reference.proj_dual_cone_r)."""
import sys, os, struct
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drs_reference import reorder_problem, proj_dual_cone_r, build_diag_r
from scs_faithful import NV

def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def f64(v): return struct.unpack('<Q', struct.pack('<d', float(v)))[0]
def write_hex(f, vals):
    with open(f, 'w') as fh:
        for v in vals: fh.write('%016X\n' % f64(v))

HERE = os.path.dirname(os.path.abspath(__file__))
K = os.path.join(HERE, '..', 'rtl', 'data', 'kkt', 'full')
n, m = NV, 2107
Ab, br, cr, rp, cp, blocks = reorder_problem()
diag_r = build_diag_r(1.0, n, m, rp)
r_y = diag_r[n:n + m]
u1 = np.array([dec(l) for l in open(os.path.join(K, 'u1.hex'))])
x = u1[n:n + m].copy()
out = proj_dual_cone_r(x, r_y, blocks)
write_hex(os.path.join(K, 'pdc_in.hex'), x)
write_hex(os.path.join(K, 'pdc_out.hex'), out)
print(f'wrote pdc_in/pdc_out ({m} rows); max|x|={np.abs(x).max():.3e}')
