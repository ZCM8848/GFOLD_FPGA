"""Check aa_gram RTL out vs expected G, rhs."""
import sys, struct
import numpy as np
def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def rd(f): return [dec(l) for l in open(f)]
MEM = int(sys.argv[2]) if len(sys.argv)>2 else 10
N = int(sys.argv[3]) if len(sys.argv)>3 else 6
for c in range(N):
    exp = rd(f'rtl/data/ag_{c}_exp.hex')
    got = rd(f'rtl/data/ag_{c}_out.hex')
    if len(exp)!=len(got): print(f"case {c}: LEN MISMATCH exp={len(exp)} got={len(got)}"); continue
    exp = np.array(exp); got = np.array(got)
    rel = np.abs(got-exp)/(np.abs(exp)+1e-30)
    G = rel[:MEM*MEM]; R = rel[MEM*MEM:]
    print(f"case {c}: G max_rel={G.max():.3e}  rhs max_rel={R.max():.3e}  "
          f"#Gbad(>1e-6)={np.sum(G>1e-6)} #Rbad={np.sum(R>1e-6)}")
    if G.max()>1e-6:
        bad = np.where(G>1e-6)[0]
        for b in bad[:6]:
            print(f"   G[{b//MEM}][{b%MEM}] got={got[b]:.6e} exp={exp[b]:.6e}")
