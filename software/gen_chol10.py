"""Generate chol10 test vectors: random SPD G (10x10) + rhs -> gamma = solve(G,rhs)."""
import sys, os, json, struct
import numpy as np

def f2h(x): return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')
def gen_case(M, rng):
    B = rng.normal(0, 1, (M, M))
    G = B @ B.T + np.eye(M)                      # SPD, well-conditioned-ish
    # add a bit of spread so it's not trivial
    D = np.diag(rng.uniform(0.5, 2.0, M))
    G = D @ G @ D
    rhs = rng.normal(0, 1, M)
    gamma = np.linalg.solve(G, rhs)              # via Cholesky internally
    return G, rhs, gamma

def main():
    M = int(sys.argv[1]) if len(sys.argv)>1 else 10
    N = int(sys.argv[2]) if len(sys.argv)>2 else 12   # number of cases
    rng = np.random.default_rng(99)
    os.makedirs('rtl/data', exist_ok=True)
    for c in range(N):
        G, rhs, gamma = gen_case(M, rng)
        with open(f'rtl/data/chol_{c}.hex','w') as f:
            for j in range(M):
                for i in range(M):
                    f.write(f2h(G[j][i])+'\n')
            for i in range(M):
                f.write(f2h(rhs[i])+'\n')
        with open(f'rtl/data/chol_{c}_gamma.hex','w') as f:
            for i in range(M): f.write(f2h(gamma[i])+'\n')
    print(f"wrote {N} chol cases (M={M}) to rtl/data/chol_*.hex + chol_*_gamma.hex")

if __name__=='__main__':
    main()
