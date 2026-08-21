"""Check proj_dual_cone RTL out hex vs python expected."""
import sys, struct
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
name = sys.argv[1] if len(sys.argv)>1 else 'small'
x  = np.array([dec(l) for l in open(f'rtl/data/pdc_{name}_init.hex')])
got= np.array([dec(l) for l in open(f'rtl/data/pdc_{name}_out.hex')])
exp= np.array([dec(l) for l in open(f'rtl/data/pdc_{name}_exp.hex')])
assert len(got)==len(x)==len(exp), (len(got),len(x),len(exp))
rel = np.abs(got-exp)/(np.abs(exp)+1e-30)
bad = np.where(rel>1e-9)[0]
print(f"{name}: M={len(x)} max_rel={rel.max():.3e}  #bad(>1e-9)={len(bad)}")
if len(bad):
    print("  bad rows:", bad[:20])
    for i in bad[:5]:
        print(f"   row {i}: got={got[i]:.6e} exp={exp[i]:.6e} x={x[i]:.6e}")
