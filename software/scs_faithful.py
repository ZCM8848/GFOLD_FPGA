#!/usr/bin/env python3
"""
Faithful numpy port of the complete SCS v3 algorithm:
  - homogeneous self-dual embedding DRS core
  - Type-I Anderson acceleration (pivoted QR, rank truncation, iterative
    refinement, safeguard, reset) applied every `interval` SCS iterations.
Defaults match the `scs` package (type1=1, lookback=10, interval=10,
regularization=1e-8, relaxation=1.0, alpha=1.5).

Validated against the `scs` package and the Clarabel reference solution.
"""
import json, time, os
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "problem.json")))
NV, NR = D["nvars"], D["nrows"]
colptr = np.array(D["colptr"]); rowval = np.array(D["rowval"]); nzval = np.array(D["nzval"])
A = sp.csc_matrix((nzval, rowval, colptr), shape=(NR, NV)).tocsr()
b = np.array(D["b"]); c = np.array(D["q"]); x_ref = np.array(D["x"]); obj_ref = D["objective"]
blocks = []; r = 0
for cc in D["cones"]:
    blocks.append((cc["kind"], r, cc["dim"])); r += cc["dim"]

# ================= cone machinery =================
def proj_soc(v):
    t = v[0]; x = v[1:]; nx = np.linalg.norm(x)
    if nx <= t: return v.copy()
    if nx <= -t: return np.zeros_like(v)
    lam = (nx + t)/2.0
    o = np.empty_like(v); o[0]=lam; o[1:]=(lam/nx)*x; return o

def proj_dual_cone(y):
    out = y.copy()
    for kind, s, dim in blocks:
        if kind == "zero":    pass
        elif kind == "nonneg": out[s:s+dim] = np.maximum(y[s:s+dim], 0.0)
        elif kind == "soc":   out[s:s+dim] = proj_soc(y[s:s+dim])
    return out

# ================= KKT + g =================
M = sp.bmat([[sp.eye(NV), A.T], [A, -sp.eye(NR)]], format="csc")
lu = splu(M)
g = lu.solve(np.concatenate([c, -b]))

# ================= Anderson (Type-I) =================
# AA defaults
AA_MEM = 10
AA_MINLEN = 10
AA_R = 1e-8
AA_RELAX = 1.0
AA_SAFEGUARD = 1.0
AA_MAXW = 1e10
AA_IR = 5
AA_EPS = np.finfo(np.float64).eps

def aa_frob(nrm_col):
    m = nrm_col.max() if len(nrm_col) else 0.0
    if m == 0: return 0.0
    return m*np.sqrt(np.sum((nrm_col/m)**2))

class Anderson:
    def __init__(self, dim):
        self.dim = dim; self.mem = AA_MEM
        self.x = np.zeros(dim); self.f = np.zeros(dim); self.g = np.zeros(dim)
        self.g_prev = np.zeros(dim)
        self.S = np.zeros((dim, AA_MEM)); self.D = np.zeros((dim, AA_MEM)); self.Y = np.zeros((dim, AA_MEM))
        self.nrm_s = np.zeros(AA_MEM); self.nrm_y = np.zeros(AA_MEM)
        self.iter = 0; self.success = 0; self.norm_g = 0.0
        self.n_accept = 0; self.n_reject = 0; self.last_rank = 0; self.last_aa = 0.0
        self.gamma = np.zeros(dim)  # work buffer (len<=mem used)
    def reset(self):
        self.iter = 0; self.success = 0; self.norm_g = 0.0
        self.nrm_s[:] = 0; self.nrm_y[:] = 0
    def update_params(self, x, f):
        idx = (self.iter - 1) % self.mem
        self.S[:,idx] = x - self.x
        self.D[:,idx] = f - self.f
        self.g[:] = x - f
        self.Y[:,idx] = self.g - self.g_prev
        self.nrm_s[idx] = np.linalg.norm(self.S[:,idx])
        self.nrm_y[idx] = np.linalg.norm(self.Y[:,idx])
        self.x[:] = x; self.f[:] = f; self.g_prev[:] = self.g
        self.norm_g = np.linalg.norm(self.g)
    def solve(self, f, length):
        """Type-I Anderson LS with pivoted QR; overwrite f = f - D*gamma."""
        dim = self.dim; mem = self.mem
        A_src = self.S  # type1: A = S
        # regularization
        nrm_a = aa_frob(self.nrm_s); nrm_yf = aa_frob(self.nrm_y)
        rreg = AA_R * nrm_a * nrm_yf
        sqrt_r = np.sqrt(rreg) if rreg > 0 else 0.0
        aug_rows = dim + mem
        # build A_aug = [S[:, :length]; sqrt_r*I]
        A_aug = np.zeros((aug_rows, length))
        A_aug[:dim, :] = A_src[:, :length]
        for i in range(length):
            A_aug[dim + i, i] = sqrt_r
        # pivoted QR via LAPACK dgeqp3 (returns qr-array, jpvt 0-indexed, tau)
        from scipy.linalg.lapack import dgeqp3, dormqr
        qr_arr, jpvt, tauq, _work, info_q = dgeqp3(A_aug.copy())
        # rank truncation from R diagonal (upper triangle of qr_arr)
        r11 = abs(qr_arr[0, 0]) if length > 0 else 0.0
        rank = 0
        if r11 > 0:
            tol = r11 * length * AA_EPS
            while rank < length and abs(qr_arr[rank, rank]) >= tol:
                rank += 1
        info = 1 if rank == 0 else 0
        self.last_rank = rank
        if info != 0:
            self.success = 0; self.n_reject += 1; self.reset(); return -1.0
        # c_aug = Q' [g; 0]  (dormqr applies reflectors, no full Q)
        c_aug = np.zeros(aug_rows); c_aug[:dim] = self.g
        c_aug, _, _ = dormqr('L', 'T', qr_arr, tauq, c_aug, lwork=max(64, 4*length))
        c_top = c_aug[:rank].copy()
        # Type-I: B_aug = [Y_piv; sqrt_r diag], then W = top-left rank block of Q'B_aug
        B_aug = np.zeros((aug_rows, rank))
        for i in range(rank):
            piv = jpvt[i] - 1          # LAPACK jpvt is 1-indexed
            B_aug[:dim, i] = self.Y[:, piv]
            B_aug[dim + piv, i] = sqrt_r
        B_aug, _, _ = dormqr('L', 'T', qr_arr, tauq, B_aug, lwork=max(64, 4*length))
        W = B_aug[:rank, :rank].copy(); W_orig = W.copy()
        gamma_red, ipiv = _lu_solve(W, c_top.copy())
        # iterative refinement
        prev_d = 0.0
        for k in range(AA_IR):
            res = c_top - W_orig @ gamma_red
            delta, _ = _lu_solve(W, res)
            dn = np.linalg.norm(delta)
            gamma_red = gamma_red + delta
            if k > 0 and dn >= 0.5*prev_d: break
            prev_d = dn
        # unpermute
        self.gamma[:length] = 0.0
        for i in range(rank):
            self.gamma[jpvt[i] - 1] = gamma_red[i]
        aa_norm = np.linalg.norm(self.gamma[:length])
        self.last_aa = aa_norm
        if not np.isfinite(aa_norm) or aa_norm >= AA_MAXW:
            self.success = 0; self.n_reject += 1; self.reset(); return -aa_norm if aa_norm<0 else -1.0
        # f = f - D*gamma
        f[:] = f - self.D[:, :length] @ self.gamma[:length]
        if AA_RELAX != 1.0:
            f[:] = AA_RELAX*f + (1-AA_RELAX)*(self.x - self.S[:, :length]@self.gamma[:length])
        self.success = 1
        self.n_accept += 1
        return aa_norm
    def apply(self, f, x):
        """f = current DR output, x = previous DR input. Accelerate f in place."""
        length = min(self.iter, self.mem)
        self.success = 0
        if self.iter == 0:
            self.x[:] = x; self.f[:] = f; self.g_prev[:] = x - f
            self.iter += 1; return 0.0
        self.update_params(x, f)
        aa_norm = 0.0
        if self.iter >= AA_MINLEN:
            aa_norm = self.solve(f, length)
        self.iter += 1
        return aa_norm
    def safeguard(self, f_new, x_new):
        if not self.success: return 0
        self.success = 0
        diff = x_new - f_new
        nd = np.linalg.norm(diff)
        if nd > AA_SAFEGUARD * self.norm_g:
            f_new[:] = self.f; x_new[:] = self.x
            self.reset(); self.n_reject += 1; return -1
        return 0

# helpers
def _lu_solve(W, rhs):
    import scipy.linalg as sla
    lu, piv = sla.lu_factor(W)
    x = sla.lu_solve((lu, piv), rhs)
    return x, piv

# ================= full SCS loop =================
def solve_scs(max_iter, alpha=1.5, interval=10, track=None):
    l = NV + NR + 1
    u = np.zeros(l); v = np.zeros(l); v_prev = np.zeros(l); u_t = np.zeros(l)
    sqrt_l = np.sqrt(l); FEAS = 1
    aa = Anderson(l)
    hist = []; t0 = time.time()
    for i in range(max_iter):
        # AA every `interval` iters (on v, using v_prev as the DR input)
        if i > 0 and i % interval == 0:
            aa.apply(v, v_prev)
        if i >= FEAS:
            vn = np.linalg.norm(v)
            if vn > 0: v *= sqrt_l/vn
        v_prev[:] = v
        # DR step
        u_t[:NV] = v[:NV]; u_t[NV:l-1] = -v[NV:l-1]; u_t[l-1] = v[l-1]
        u_t[:l-1] = lu.solve(u_t[:l-1])
        if i < FEAS: u_t[l-1] = 1.0
        else:        u_t[l-1] = _root_plus(u_t[:l-1], v[:l-1], v[l-1])
        u_t[:l-1] += g * (-u_t[l-1])
        u[:] = 2*u_t - v
        u[NV:l-1] = proj_dual_cone(u[NV:l-1])
        if i < FEAS: u[l-1] = 1.0
        else:        u[l-1] = max(u[l-1], 0.0)
        v += alpha*(u - u_t)
        # safeguard
        if i % interval == 0:
            aa.safeguard(v, v_prev)
        if track and i % track == 0:
            tau = u[l-1]; xx = u[:NV]/tau if abs(tau) > 1e-12 else u[:NV]
            hist.append((i, np.linalg.norm(xx-x_ref)/np.linalg.norm(x_ref), float(c@xx)))
    tau = u[l-1]; x = u[:NV]/tau if abs(tau) > 1e-12 else u[:NV]
    return x, tau, hist, aa

def _root_plus(p, mu, eta):
    gg=np.dot(g,g); mug=np.dot(g,mu); pg=np.dot(p,g)
    pp=np.dot(p,p); pmu=np.dot(p,mu)
    a=1.0+gg; bv=mug-2*pg-eta; cc=pp-pmu
    rad=bv*bv-4*a*cc
    if rad<0: return -bv/(2*a)
    sq=np.sqrt(rad)
    if bv<=0: return (-bv+sq)/(2*a)
    qq=-0.5*(bv+sq); return cc/qq if qq!=0 else 0.0

if __name__ == "__main__":
    print(f"nvars={NV} nrows={NR} KKT={NV+NR}  clarabel obj={obj_ref:.6f}")
    x, tau, hist, aa = solve_scs(50000, track=5000)
    relx = np.linalg.norm(x-x_ref)/np.linalg.norm(x_ref)
    print(f"\nFAITHFUL SCS: tau={tau:.5f} obj={c@x:.6f} rel_x={relx:.3e}")
    print(f"AA stats: accepts={aa.n_accept} rejects={aa.n_reject} last_rank={aa.last_rank} last_aa_norm={aa.last_aa:.3e}")
    for it, rx, o in hist:
        print(f"   {it:6d}  rel_x={rx:.3e}  obj={o:.6f}")
