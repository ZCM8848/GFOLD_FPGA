"""Decode root_plus RTL results (hex got vs python expected), report max rel error."""
import sys, os, re, struct
import numpy as np
sys.path.insert(0, 'software')
from gen_root_plus import root_plus_py, f2h

def decode(hx):
    return struct.unpack('>d', bytes.fromhex(hx))[0]

def main():
    dumpfile = sys.argv[1] if len(sys.argv) > 1 else 'rtl/sim/root_plus_dump.txt'
    if not os.path.exists(dumpfile):
        print(f"no dump at {dumpfile}"); return
    # rebuild expected from the same RNG
    L = 8
    rng = np.random.default_rng(1234)
    maxrel = 0.0; nmatch=0; worst=None; n=0
    for line in open(dumpfile):
        m = re.search(r'CASE (\d+): exp=(\w+) got=(\w+)', line)
        if not m: continue
        ci = int(m.group(1))
        # recompute inputs from generator (must reproduce exact sequence)
        r  = rng.uniform(0.5, 2.0, L); g  = rng.normal(0,1,L)
        p  = rng.normal(0,1,L); mu = rng.normal(0,1,L); eta = rng.uniform(-2,2)
        exp = root_plus_py(r,g,p,mu,eta)
        got = decode(m.group(3))
        rel = abs(got-exp)/(abs(exp)+1e-30)
        n+=1
        if rel>maxrel: maxrel=rel; worst=ci
        if m.group(2)==m.group(3): nmatch+=1
    print(f"cases={n} bit_exact={nmatch} max_rel_err={maxrel:.3e} worst_case={worst}")

if __name__=='__main__':
    main()
