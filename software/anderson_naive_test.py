"""Quick de-risk: does a SIMPLIFIED Type-I Anderson (regularized normal equations +
dense Cholesky, NO pivoted QR / rank truncation) converge the SCS DRS to the same
optimum (obj/final_mass) as the faithful pivoted-QR Anderson?

We patch scs_faithful.Anderson.solve to use normal equations:
  gamma = (S^T S + rreg*I)^-1 S^T g   (Tikhonov regularized LS, min ||S gamma - g||^2 + r||gamma||^2)
and run the adaptive DRS. Report obj, rel_x, convergence vs the faithful path.
"""
import sys, os
import numpy as np
sys.path.insert(0, 'software')
import scs_faithful as F
import scs_adaptive as A

def solve_naive(self, f, length):
    """Type-I Anderson via regularized normal equations + dense Cholesky."""
    dim = self.dim
    # regularization (same as faithful)
    nrm_a = F.aa_frob(self.nrm_s); nrm_yf = F.aa_frob(self.nrm_y)
    rreg = F.AA_R * nrm_a * nrm_yf
    # A = S[:, :length] ; solve (S^T S + rreg I) gamma = S^T g
    Scol = self.S[:, :length]
    G = Scol.T @ Scol + rreg * np.eye(length)
    rhs = Scol.T @ self.g
    try:
        L = np.linalg.cholesky(G)
        gamma = np.linalg.solve(L.T, np.linalg.solve(L, rhs))
    except np.linalg.LinAlgError:
        self.success = 0; self.n_reject += 1; self.reset(); return -1.0
    aa_norm = np.linalg.norm(gamma)
    self.last_aa = aa_norm
    if not np.isfinite(aa_norm) or aa_norm >= F.AA_MAXW:
        self.success = 0; self.n_reject += 1; self.reset(); return -1.0
    # f = f - D[:, :length] @ gamma
    f[:] = f - self.D[:, :length] @ gamma
    self.success = 1; self.n_accept += 1
    return aa_norm

# monkeypatch the QR solve with the naive normal-equations version
F.Anderson.solve = solve_naive

def main():
    max_iter = int(sys.argv[1]) if len(sys.argv)>1 else 60000
    eps = float(sys.argv[2]) if len(sys.argv)>2 else 1e-4
    res = A.solve_adaptive(max_iter, eps=eps, track=2500)
    x = res['x']; tau = res['tau']
    c = F.c; b = F.b
    obj = float(c @ x)
    relx = np.linalg.norm(x - F.x_ref)/np.linalg.norm(F.x_ref)
    # feasibility + final_mass via reusing verify_feasibility logic inline
    import importlib.util
    spec = importlib.util.spec_from_file_location('vf','software/verify_feasibility.py')
    # instead of importing (runs main), inline a minimal feasibility check:
    A_ = F.A
    def soc_viol(u):
        # cone: zero(0..705), nonneg(706..1006), SOC 200x4 then 100x3 from 1007
        viol = 0.0; 
        for i in range(706, 1007): viol = max(viol, -u[i])
        s = 1007
        for blk in range(300):
            dim = 4 if blk < 200 else 3
            bl = u[s:s+dim]; t=bl[0]; vv=np.linalg.norm(bl[1:]); viol=max(viol, max(0,vv-t)); s+=dim
        return viol
    # recover primal z: we only have u; final_mass needs full reconstruction --
    # approximate by using scs_faithful's own recovered x (it's the same DRS result).
    final_mass = None
    print(f"NAIVE-ANDERSON: iter={res['converged_iter']} tau={tau:.5f} obj={obj:.6f} "
          f"rel_x={relx:.3e} scale_updates={res['n_scale_updates']} final_scale={res['scale']:.3e}")
    print(f"  AA accepts={res['aa'].n_accept} rejects={res['aa'].n_reject} "
          f"norm_g={res['aa'].norm_g:.3e}")
    for it, rx, o in res['hist']:
        print(f"   {it:6d}  rel_x={rx:.3e}  obj={o:.6f}")

if __name__=='__main__':
    main()
