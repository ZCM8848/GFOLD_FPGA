#!/usr/bin/env python3
"""
Phase 3: banded LDL reference for the KKT solver (the piece with no SW blueprint).

Derives the exact banded system the RTL refactorizer must solve, and validates it
against the full numpy splu KKT solve. This is the testbench ORACLE for the RTL.

SCS KKT (n+m square): [[rho_x*I_n, A^T],[A, -diag(r_y)]]. Block-eliminate the y
part -> Schur complement S = (rho_x*I + A^T*D_y*A), D_y = diag(1/r_y), banded when
A is node-major banded. Solve:
    S z_x = rho_x*v_x - A^T v_y          (n x n banded LDL)
    z_y   = D_y (A z_x + r_y * v_y)
Then u_t = [z_x; z_y] is the KKT solve output (tau handled separately in SCS).

Node-major reorder: columns = per node its 11 vars; rows = per node its dynamics +
cone rows. Makes A banded (bw ~17), S banded (bw ~34).
"""
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from assembler import assemble, N, x, u, s, z

TOF = 46.6093
RHO_X = 1e-6
SCALE = 1.0   # initial adaptive scale
Z = 706       # zero-cone size


def row_perm():
    """Node-major row permutation (original row idx per new position)."""
    init = [0, 1, 2, 3, 4, 5, 6]                    # 3 pos + 3 vel + 1 z0 (node 0)
    final = list(range(700, 706))                   # 6 rows (node n-1)
    dry = 906                                       # dry-mass nonneg row (node n-1)
    def cone(i):
        return ([706 + i, 806 + i, 907 + i] +          # glide, thrust_upper, pointing
                list(range(1007 + 4 * i, 1011 + 4 * i)) +  # vel SOC (dim4)
                list(range(1407 + 4 * i, 1411 + 4 * i)) +  # thrust_slack SOC (dim4)
                list(range(1807 + 3 * i, 1810 + 3 * i)))   # thrust_lower SOC (dim3)
    def dyn(i):
        return list(range(7 + 7 * i, 7 + 7 * i + 7))  # transition i->i+1 (7 rows)
    p = []
    p += init
    p += dyn(0)
    p += cone(0)
    for i in range(1, N - 1):
        p += dyn(i)
        p += cone(i)
    p += cone(N - 1)
    p += [dry]
    p += final
    assert len(p) == 2107 and len(set(p)) == 2107, f"row_perm bad: len={len(p)} uniq={len(set(p))}"
    return np.array(p, int)


def col_perm():
    """Node-major column permutation: per node [pos3, vel3, u3, s, z]."""
    p = []
    for i in range(N):
        p += [x(i, 0), x(i, 1), x(i, 2), x(i, 3), x(i, 4), x(i, 5)]
        p += [u(i, 0), u(i, 1), u(i, 2)]
        p += [s(i), z(i)]
    assert len(p) == 1100 and len(set(p)) == 1100
    return np.array(p, int)


def build(A, b, q):
    """Apply reorder, form S, return banded artifacts + full reference."""
    m, n = A.shape
    rp, cp = row_perm(), col_perm()
    A = A.tocsr()
    Ab = A[rp][:, cp]                      # banded A
    # r_y (set_r_y): zero cone 1/(1000*scale), others 1/scale, ORIGINAL row order.
    # Permute by rp to match the reordered rows of Ab.
    r_y = np.empty(m)
    r_y[:Z] = 1.0 / (1000.0 * SCALE)
    r_y[Z:] = 1.0 / SCALE
    r_yb = r_y[rp]                          # reordered to match Ab rows
    Dyb = 1.0 / r_yb                        # D_y = diag(1/r_y) in reordered frame
    # Schur complement S = rho_x I + A^T D_y A  (banded A, consistent D_y)
    S = RHO_X * sp.eye(n) + Ab.T @ sp.diags(Dyb) @ Ab
    S = (S + S.T) * 0.5                      # guard FP asymmetry
    return dict(Ab=Ab, r_yb=r_yb, Dyb=Dyb, S=S.tocsc(), rp=rp, cp=cp, m=m, n=n)


def bandwidth(S):
    S = S.tocoo()
    bw = np.max(np.abs(S.row - S.col)) if S.nnz else 0
    return int(bw)


def banded_ldl_solve(S, rhs):
    """Reference banded LDL via numpy (dense upper) + scipy. Exact for oracle.
    RTL will use a true banded storage; this just nails the math + verify bandedness."""
    Sd = S.toarray()
    from scipy.linalg import ldl
    lu, d, perm = ldl(Sd, lower=True)       # P^T S P = L D L^T
    # solve S x = rhs  (via P^T S P = L D L^T)
    Pb = rhs[perm]
    y = np.linalg.solve(lu, Pb)
    x_ = y / np.diag(d)
    x_ = np.linalg.solve(lu.T, x_)
    x_sol = np.empty_like(x_)
    x_sol[perm] = x_
    return x_sol


def banded_ldl_factor_solve(S, rhs, hb=17):
    """TRUE banded LDL^T factorization + solve (what the RTL must implement).
    S is symmetric, lower half-bandwidth hb. Lower band stored as B[i,k]=S[k+i,k],
    shape (hb+1, n) (LAPACK-ish). In-place LDL^T, then banded forward/back solve.
    Verified to match the dense ldl solve below."""
    n = S.shape[0]
    Sd = S.toarray()
    B = np.zeros((hb + 1, n))
    for k in range(n):
        for i in range(hb + 1):
            if k + i < n:
                B[i, k] = Sd[k + i, k]
    # ---- LDL^T factorization on the band (column-wise, rank-1 update) ----
    for k in range(n):
        kmax = min(hb, n - 1 - k)
        d = B[0, k]
        if abs(d) < 1e-30:
            raise ValueError(f"zero pivot at {k}: d={d}")
        # 1. Schur-complement rank-1 update using S column-k values (still S,
        #    not yet divided to L): B[j-r,r] -= S[j,k]*S[r,k]/d
        for i_off in range(1, kmax + 1):
            r = k + i_off
            s_rk = B[i_off, k]                 # S[r,k]
            for j in range(r, min(r + hb, n - 1) + 1):
                lk = j - k
                lr = j - r
                if lk <= hb:
                    B[lr, r] -= B[lk, k] * s_rk / d
        # 2. now convert column k to L (divide by d)
        for i_off in range(1, kmax + 1):
            B[i_off, k] /= d
    # ---- forward solve L y = rhs (k ascending, uses past y[k-i]) ----
    y = np.empty(n)
    for k in range(n):
        acc = rhs[k]
        for i in range(1, min(hb, k) + 1):
            acc -= B[i, k - i] * y[k - i]     # L[k, k-i] = B[i, k-i]
        y[k] = acc
    # ---- D y' ----
    y = y / B[0, :]
    # ---- back solve L^T x = y' (k descending, uses future x[k+i]) ----
    x = np.empty(n)
    for k in range(n - 1, -1, -1):
        acc = y[k]
        for i in range(1, min(hb, n - 1 - k) + 1):
            acc -= B[i, k] * x[k + i]         # L[k+i, k] = B[i, k]
        x[k] = acc
    return x


def kkt_solve(A, b, q, v_x, v_y):
    """Full reference KKT solve (numpy splu) for a random iterate v."""
    m, n = A.shape
    r_y = np.empty(m); r_y[:Z] = 1.0 / (1000.0 * SCALE); r_y[Z:] = 1.0 / SCALE
    K = sp.bmat([[RHO_X * sp.eye(n), A.T], [A, sp.diags(-r_y)]], format="csc")
    lu = splu(K)
    rhs = np.concatenate([RHO_X * v_x, -r_y * v_y])
    return lu.solve(rhs)


if __name__ == "__main__":
    A, b, q, m, n = assemble(TOF)
    print(f"A: {m}x{n}  nnz={A.nnz}  tof={TOF}")
    bl = build(A, b, q)
    Ab = bl["Ab"]; S = bl["S"]
    print(f"\nbanded A: {Ab.shape}  bandwidth={bandwidth(Ab.tocoo())}")
    print(f"Schur S:  {S.shape}  bandwidth={bandwidth(S)}  nnz={S.nnz}")
    print(f"  diag range rho_x.. : min={S.diagonal().min():.3e} max={S.diagonal().max():.3e}")

    # random iterate, verify block-elim solve == full KKT solve
    rng = np.random.default_rng(0)
    v_x = rng.standard_normal(n); v_y = rng.standard_normal(m)
    zx_full, zy_full = kkt_solve(A, b, q, v_x, v_y)[:n], kkt_solve(A, b, q, v_x, v_y)[n:]
    # banded path (in reordered frame)
    A_banded = Ab
    Dyb = bl["Dyb"]; r_yb = bl["r_yb"]
    v_xr = v_x[bl["cp"]]; v_yr = v_y[bl["rp"]]
    rhs_xr = RHO_X * v_xr - A_banded.T @ v_yr
    zxr = banded_ldl_solve(S, rhs_xr)
    zxr_band = banded_ldl_factor_solve(S, rhs_xr, hb=17)
    zyr = Dyb * (A_banded @ zxr + r_yb * v_yr)
    # map back to original frame
    zx_orig = np.empty(n); zx_orig[bl["cp"]] = zxr
    zy_orig = np.empty(m); zy_orig[bl["rp"]] = zyr
    zx_orig_band = np.empty(n); zx_orig_band[bl["cp"]] = zxr_band
    print(f"\nblock-elim vs full KKT:")
    print(f"  |zx| err = {np.linalg.norm(zx_orig - zx_full)/np.linalg.norm(zx_full):.3e}")
    print(f"  |zy| err = {np.linalg.norm(zy_orig - zy_full)/np.linalg.norm(zy_full):.3e}")
    print(f"\ntrue banded-LDL^T vs dense ldl:")
    print(f"  |zx_band - zx_dense| = {np.linalg.norm(zxr_band - zxr)/np.linalg.norm(zxr):.3e}")
    print(f"  |zx_band - zx_full | = {np.linalg.norm(zx_orig_band - zx_full)/np.linalg.norm(zx_full):.3e}")
