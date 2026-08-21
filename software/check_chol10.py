"""Check chol10 RTL gamma vs numpy for each case."""
import sys, struct, re
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
M = 10

# expected gammas from gen files
def load_exp(c):
    return [dec(l) for l in open(f'rtl/data/chol_{c}_gamma.hex')]

# parse got from dump CASE lines
def load_got(dump):
    out = {}
    for line in open(dump):
        m = re.search(r'CASE (\d+):(.*)', line)
        if m:
            c = int(m.group(1)); hx = m.group(2).split()
            out[c] = [dec(x) for x in hx[:M]]
    return out

got = load_got(sys.argv[1] if len(sys.argv)>1 else 'rtl/sim/chol_dump.txt')
for c in sorted(got):
    exp = np.array(load_exp(c)); g = np.array(got[c])
    rel = np.abs(g-exp)/(np.abs(exp)+1e-30)
    print(f"case {c}: max_rel={rel.max():.3e}  bad={np.where(rel>1e-6)[0].tolist()}")
    if rel.max()>1e-6:
        for i in range(M):
            print(f"   [{i}] got={g[i]:+.6e} exp={exp[i]:+.6e}")
