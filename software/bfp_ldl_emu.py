#!/usr/bin/env python3
"""Stage-B: BFP banded-LDL emulation (per-column offdiag block + per-value
diagonal), 32-bit truncating mantissas. Tests whether BFP can represent the
cond-1e6 / near-singular-pivot band and solve S zx = rhs accurately.

Design (per column k):
  - diagonal B[0][k]: per-value exponent (pivot magnitude tracked individually).
  - off-diagonal B[i][k] (i=1..HB): shared column exponent eoff[k], 32-bit
    signed mantissas, DYNAMICALLY re-normalized whenever the column's values are
    rewritten (updates by earlier cols, or L-conversion producing ~1e30 values
    at the near-singular pivot).
Metric: residual ||S zx - rhs||/||rhs|| (element-wise meaningless for cond-1e6).
"""
import numpy as np
from banded_reference import build, RHO_X, banded_ldl_factor_solve
from assembler import assemble
import scipy.sparse as sp

def q_mant(v, mant, rnd=False):
    """Quantize a float to a signed mant-bit integer. rnd=False truncates toward
    zero; rnd=True rounds-to-nearest."""
    if v == 0: return 0
    neg = v < 0
    a = -v if neg else v
    hi = (1 << (mant - 1)) - 1
    if a >= (1 << mant): r = hi
    elif rnd: r = int(a + 0.5)
    else: r = int(a)
    return -r if neg else r

def bfp_val(mant_int, exp):
    return float(mant_int) * (2.0 ** exp)

class BfpLDL:
    def __init__(self, Sd, hb, mant=32):
        self.n, self.hb, self.mant = Sd.shape[0], hb, mant
        self.md = np.zeros(self.n, np.int64)      # diag mantissa
        self.ed = np.zeros(self.n, np.int64)      # diag exponent (per-value)
        self.M  = np.zeros((hb, self.n), np.int64)  # offdiag mantissas
        self.eoff = np.zeros(self.n, np.int64)      # offdiag col exponent
        for k in range(self.n):
            self.md[k], self.ed[k] = self._qval(Sd[k, k])
            self.eoff[k] = self._block_exp([Sd[k + i, k] for i in range(1, min(hb, self.n - 1 - k) + 1)])
            for i in range(1, min(hb, self.n - 1 - k) + 1):
                self.M[i - 1, k] = q_mant(Sd[k + i, k] / (2.0 ** self.eoff[k]), mant)
    def _qval(self, v):
        if v == 0: return 0, 0
        e = int(np.floor(np.log2(abs(v)))) - (self.mant - 1)
        return q_mant(v / (2.0 ** e), self.mant), e
    def _block_exp(self, vals):
        mx = max(abs(v) for v in vals) if vals else 0.0
        return int(np.floor(np.log2(mx))) - (self.mant - 1) if mx > 0 else 0
    def get(self, k, i):
        if i == 0: return bfp_val(self.md[k], self.ed[k])
        return bfp_val(self.M[i - 1, k], self.eoff[k])
    def set_diag(self, k, v): self.md[k], self.ed[k] = self._qval(v)
    def set_off(self, k, i, v): self.M[i - 1, k] = q_mant(v / (2.0 ** self.eoff[k]), self.mant)
    def renorm_off(self, k):
        kmax = min(self.hb, self.n - 1 - k)
        cur = [self.get(k, i) for i in range(1, kmax + 1)]
        ne = self._block_exp(cur)
        if ne == self.eoff[k]: return
        self.eoff[k] = ne
        for i in range(1, kmax + 1):
            self.M[i - 1, k] = q_mant(cur[i - 1] / (2.0 ** ne), self.mant)

def run_bfp_ldl(Sd, rhs, hb, mant=32, wide=False):
    """wide=True -> per-VALUE exponent for offdiag too (a wide-float with 32-bit
    truncating mantissa), NOT block BFP. Tests whether per-value exponents fix
    the near-singular-pivot dynamic-range problem."""
    n = Sd.shape[0]
    ldl = BfpLDL(Sd, hb, mant)
    if wide:
        # per-value exponent for offdiag: eoff[k] tracks nothing shared; instead
        # each entry gets its own exponent -> set eoff[k]=0 and store each as
        # mantissa*2^own_exp via a per-entry q. We re-derive on the fly.
        pass
    # ---- factorization ----
    for k in range(n):
        kmax = min(hb, n - 1 - k)
        d = ldl.get(k, 0)
        if abs(d) < 1e-40: raise ZeroDivisionError(f"pivot {k} = {d}")
        for i_off in range(1, kmax + 1):
            r = k + i_off
            s_rk = ldl.get(k, i_off)
            t = s_rk / d
            for j in range(r, min(r + hb, n - 1) + 1):
                lk = j - k; lr = j - r
                if lk <= hb:
                    nv = ldl.get(r, lr) - ldl.get(k, lk) * t
                    ldl.set_off(r, lr, nv)
            ldl.renorm_off(r)
        for i_off in range(1, kmax + 1):
            ldl.set_off(k, i_off, ldl.get(k, i_off) / d)
        ldl.renorm_off(k)
    # ---- forward solve ----
    y = np.zeros(n)
    for k in range(n):
        acc = rhs[k]
        for i in range(1, min(hb, k) + 1):
            acc -= ldl.get(k - i, i) * y[k - i]
        y[k] = acc
    y = y / np.array([ldl.get(k, 0) for k in range(n)])
    x = np.zeros(n)
    for k in range(n - 1, -1, -1):
        acc = y[k]
        for i in range(1, min(hb, n - 1 - k) + 1):
            acc -= ldl.get(k, i) * x[k + i]
        x[k] = acc
    return x

def run_wide_float(Sd, rhs, hb, mant=32, rnd=False):
    """Per-VALUE exponent, mant-bit arithmetic. Each band value stored as its own
    (mantissa, exponent); every op quantizes its result to mant bits (rnd selects
    round-to-nearest vs truncation). Baseline: does enough mantissa + rounding
    make the LDL accurate regardless of block sharing?"""
    n = Sd.shape[0]
    E = np.zeros((hb + 1, n), np.int64)
    M = np.zeros((hb + 1, n), np.int64)
    def store(k, i, v):
        if v == 0: M[i, k] = 0; E[i, k] = 0; return
        e = int(np.floor(np.log2(abs(v)))) - (mant - 1)
        M[i, k] = q_mant(v / (2.0 ** e), mant, rnd); E[i, k] = e
    def get(k, i):
        return float(M[i, k]) * (2.0 ** E[i, k])
    for k in range(n):
        for i in range(hb + 1):
            if k + i < n: store(k, i, Sd[k + i, k])
    for k in range(n):
        kmax = min(hb, n - 1 - k)
        d = get(k, 0)
        if abs(d) < 1e-40: raise ZeroDivisionError(f"pivot {k} = {d}")
        for i_off in range(1, kmax + 1):
            r = k + i_off
            s_rk = get(k, i_off)
            t = s_rk / d
            for j in range(r, min(r + hb, n - 1) + 1):
                lk = j - k; lr = j - r
                if lk <= hb:
                    nv = get(r, lr) - get(k, lk) * t
                    store(r, lr, nv)
        for i_off in range(1, kmax + 1):
            store(k, i_off, get(k, i_off) / d)
    y = np.zeros(n)
    for k in range(n):
        acc = rhs[k]
        for i in range(1, min(hb, k) + 1):
            acc -= get(k - i, i) * y[k - i]
        y[k] = acc
    y = y / np.array([get(k, 0) for k in range(n)])
    x = np.zeros(n)
    for k in range(n - 1, -1, -1):
        acc = y[k]
        for i in range(1, min(hb, n - 1 - k) + 1):
            acc -= get(k, i) * x[k + i]
        x[k] = acc
    return x

def main():
    A, b, q, m, n = assemble(46.6093)
    bl = build(A, b, q); S0 = bl['S'].toarray()
    Ss = 1.0 * S0; Ss[np.diag_indices(n)] += RHO_X
    rng = np.random.default_rng(42)
    v_xr = rng.standard_normal(n); v_yr = rng.standard_normal(m)
    rhs = RHO_X * v_xr - bl['Ab'].T @ v_yr
    zx64 = banded_ldl_factor_solve(sp.csc_matrix(Ss), rhs, hb=17)
    print("== per-column block BFP ==")
    for mant in [24, 32]:
        try:
            x = run_bfp_ldl(Ss, rhs, 17, mant)
            print(f"  BFP mant={mant}: resid={np.linalg.norm(Ss@x-rhs)/np.linalg.norm(rhs):.3e}")
        except ZeroDivisionError as e:
            print(f"  BFP mant={mant}: DIV-ZERO {e}")
    print("== per-value wide-float ==")
    for mant in [24, 32, 53]:
        for rnd in [False, True]:
            tag = "rnd" if rnd else "trunc"
            try:
                x = run_wide_float(Ss, rhs, 17, mant, rnd)
                resid = np.linalg.norm(Ss @ x - rhs) / np.linalg.norm(rhs)
                rel = np.max(np.abs(x - zx64) / (np.abs(zx64) + 1e-30))
                print(f"  wide mant={mant} {tag}: resid={resid:.3e}  rel(zx)max={rel:.3e}")
            except ZeroDivisionError as e:
                print(f"  wide mant={mant} {tag}: DIV-ZERO {e}")
    print("refs: float64 resid ~1e-13, round-nearest f32 resid 2.5e-5")

if __name__ == "__main__":
    main()
