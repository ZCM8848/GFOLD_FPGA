"""Generate aa_gram test vectors: random S (MEM x DIM), g (DIM), rreg -> G=S^T S + rreg I, rhs=S^T g."""
import sys, os, struct
import numpy as np

def f2h(x): return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')

def main():
    DIM = int(sys.argv[1]) if len(sys.argv)>1 else 32
    MEM = int(sys.argv[2]) if len(sys.argv)>2 else 10
    N = int(sys.argv[3]) if len(sys.argv)>3 else 6
    rng = np.random.default_rng(555)
    os.makedirs('rtl/data', exist_ok=True)
    for c in range(N):
        S = rng.normal(0, 1, (MEM, DIM))
        g = rng.normal(0, 1, DIM)
        rreg = float(rng.uniform(1e-9, 1e-5))
        G = S @ S.T + rreg*np.eye(MEM)
        rhs = S @ g
        with open(f'rtl/data/ag_{c}.hex','w') as f:
            for j in range(MEM):
                for p in range(DIM):
                    f.write(f2h(S[j][p])+'\n')
            for p in range(DIM):
                f.write(f2h(g[p])+'\n')
            f.write(f2h(rreg)+'\n')
        with open(f'rtl/data/ag_{c}_exp.hex','w') as f:
            for j in range(MEM):
                for k in range(MEM):
                    f.write(f2h(G[j][k])+'\n')
            for j in range(MEM):
                f.write(f2h(rhs[j])+'\n')
    print(f"wrote {N} aa_gram cases (DIM={DIM} MEM={MEM}) to rtl/data/ag_*.hex")

if __name__=='__main__':
    main()
