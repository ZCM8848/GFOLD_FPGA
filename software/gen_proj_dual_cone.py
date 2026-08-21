"""Generate test vectors for proj_dual_cone RTL: initial dual vector x (M rows) and
expected proj_dual_cone_r(x). Config via args: 'small' (structural) or 'full' (real)."""
import sys, os, json, struct
import numpy as np

def f2h(x): return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')

def proj_soc_py(v):
    t = v[0]; x = v[1:]; nx = float(np.linalg.norm(x))
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t)/2.0
    o = np.empty_like(v); o[0]=lam; o[1:]=(lam/nx)*x; return o

def proj_dual_cone(x, cfg):
    out = x.copy()
    Z, NONNEG, N, SOC_START, N4, DIM4, N3, DIM3 = cfg
    # zero rows pass (0..Z-1)
    # nonneg
    for i in range(NONNEG, NONNEG+N): out[i] = max(x[i], 0.0)
    # soc
    s = SOC_START; NBLK = N4+N3
    for b in range(NBLK):
        dim = DIM4 if b < N4 else DIM3
        blk = out[s:s+dim].copy()
        blk = proj_soc_py(-blk) + x[s:s+dim]
        out[s:s+dim] = blk
        s += dim
    return out

def make_cfg(name):
    if name == 'small':
        return dict(M=18, Z=2, NONNEG=2, N=2, SOC_START=4, N4=2, DIM4=4, N3=2, DIM3=3, ntrials=1)
    else:  # full (real G-FOLD)
        return dict(M=2107, Z=706, NONNEG=706, N=301, SOC_START=1007, N4=200, DIM4=4, N3=100, DIM3=3, ntrials=1)

def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'full'
    cfg = make_cfg(name)
    M = cfg['M']
    # dual-vector values: mix of positive/negative, and inside/outside SOC cones
    rng = np.random.default_rng(7)
    x = rng.normal(0, 1, M)
    # force some nonneg negatives and some inside/outside SOC for coverage
    # (random normal covers all; add a few boundary-ish cases)
    exp = proj_dual_cone(x, (cfg['Z'],cfg['NONNEG'],cfg['N'],cfg['SOC_START'],
                             cfg['N4'],cfg['DIM4'],cfg['N3'],cfg['DIM3']))
    os.makedirs('rtl/data', exist_ok=True)
    with open(f'rtl/data/pdc_{name}_init.hex','w') as f:
        for i in range(M): f.write(f2h(x[i])+'\n')
    with open(f'rtl/data/pdc_{name}_exp.hex','w') as f:
        for i in range(M): f.write(f2h(exp[i])+'\n')
    with open(f'rtl/data/pdc_{name}_manifest.json','w') as f:
        json.dump(dict(name=name, **cfg), f)
    # coverage report
    print(f"{name}: M={M} wrote init/exp. params Z={cfg['Z']} N={cfg['N']} "
          f"N4={cfg['N4']} N3={cfg['N3']}")

if __name__=='__main__':
    main()
