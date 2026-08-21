"""Naive Type-I Anderson reference (regularized normal equations + Cholesky),
mirroring the RTL state machine, to generate RTL test vectors.
Matches anderson_naive_test.py's monkeypatched solve (same math).
"""
import sys, os, json, struct
import numpy as np

AA_MEM=10; AA_MINLEN=10; AA_R=1e-8; AA_MAXW=1e10

def f2h(x): return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')

def aa_frob(nrm_col):
    nrm_col = np.asarray(nrm_col, float)
    m = nrm_col.max() if len(nrm_col) else 0.0
    if m == 0: return 0.0
    return m*np.sqrt(np.sum((nrm_col/m)**2))

class AndersonNaive:
    def __init__(self, dim):
        self.dim=dim; self.mem=AA_MEM
        self.x=np.zeros(dim); self.f=np.zeros(dim); self.g=np.zeros(dim); self.g_prev=np.zeros(dim)
        self.S=np.zeros((dim,AA_MEM)); self.D=np.zeros((dim,AA_MEM)); self.Y=np.zeros((dim,AA_MEM))
        self.nrm_s=np.zeros(AA_MEM); self.nrm_y=np.zeros(AA_MEM)
        self.iter=0; self.success=0; self.norm_g=0.0
    def reset(self):
        self.iter=0; self.success=0; self.norm_g=0.0; self.nrm_s[:]=0; self.nrm_y[:]=0
    def apply(self, f, x):
        """f=current DR output, x=previous DR input. Returns accelerated f (in place)."""
        length=min(self.iter,self.mem)
        self.success=0
        if self.iter==0:
            self.x[:]=x; self.f[:]=f; self.g_prev[:]=x-f; self.iter+=1; return f
        # update_params
        idx=(self.iter-1)%self.mem
        self.S[:,idx]=x-self.x; self.D[:,idx]=f-self.f
        self.g[:]=x-f; self.Y[:,idx]=self.g-self.g_prev
        self.nrm_s[idx]=np.linalg.norm(self.S[:,idx]); self.nrm_y[idx]=np.linalg.norm(self.Y[:,idx])
        self.x[:]=x; self.f[:]=f; self.g_prev[:]=self.g; self.norm_g=np.linalg.norm(self.g)
        if self.iter>=AA_MINLEN:
            self.solve(f, length)
        self.iter+=1
        return f
    def solve(self, f, length):
        dim=self.dim; mem=self.mem
        nrm_a=aa_frob(self.nrm_s); nrm_yf=aa_frob(self.nrm_y); rreg=AA_R*nrm_a*nrm_yf
        Scol=self.S[:,:length]
        G=Scol.T@Scol + rreg*np.eye(length)
        rhs=Scol.T@self.g
        try:
            L=np.linalg.cholesky(G)
            gamma=np.linalg.solve(L.T, np.linalg.solve(L, rhs))
        except np.linalg.LinAlgError:
            self.success=0; return
        an=np.linalg.norm(gamma); self.last_aa=an
        if not np.isfinite(an) or an>=AA_MAXW:
            self.success=0; self.reset(); return
        f[:]=f - self.D[:,:length]@gamma
        self.success=1

def main():
    DIM = int(sys.argv[1]) if len(sys.argv)>1 else 32
    NCALL = int(sys.argv[2]) if len(sys.argv)>2 else 25   # apply calls (>=MEM+2 to hit solve)
    rng = np.random.default_rng(2026)
    aa = AndersonNaive(DIM)
    # build a sequence of DR outputs that looks like a converging fixed-point iterate
    # (x = previous f; f = alpha*x + noise*decay so it drifts then settles)
    x = rng.normal(0,1,DIM)
    calls=[]
    for c in range(NCALL):
        f = x*0.5 + rng.normal(0, 1.0/(1+c), DIM)   # DR output
        f_in = f.copy()
        f_out = aa.apply(f_in, x)                    # accelerated (in place)
        calls.append(dict(x=x.copy(), f=f.copy(), fout=f_out.copy()))
        x = f_out.copy()                             # accelerated becomes next input
    os.makedirs('rtl/data', exist_ok=True)
    # files: per-call hex: DIM rows "x f fout"
    for c in range(NCALL):
        with open(f'rtl/data/aa_{c}.hex','w') as fh:
            for i in range(DIM):
                fh.write(f2h(calls[c]['x'][i])+' '+f2h(calls[c]['f'][i])+' '+f2h(calls[c]['fout'][i])+'\n')
    with open('rtl/data/aa_manifest.json','w') as fh:
        json.dump(dict(DIM=DIM, NCALL=NCALL), fh)
    print(f"wrote {NCALL} apply-calls (DIM={DIM}); solves hit from call {AA_MINLEN}")

if __name__=='__main__':
    main()
