#!/usr/bin/env python3
"""Isolate RTL-vs-float32 for the collapsed-scale banded LDL.

Emulates the RTL datapath exactly (float32 rounding of every B update, L
conversion, forward/back solve). Returns the solution x plus the factored band
B and forward-solved y so we can localize where the RTL first diverges from a
faithful float32 computation (factorization vs forward vs D-divide vs back).
"""
import os, struct, numpy as np
import scipy.sparse as sp
from banded_reference import build, banded_ldl_factor_solve, RHO_X
from assembler import assemble, N

def f32(x): return np.float32(x)

def banded_ldl_f32(S, rhs, hb=17):
    Sd = np.asarray(S).astype(np.float32)
    n = Sd.shape[0]
    B = np.zeros((hb + 1, n), np.float32)
    for k in range(n):
        for i in range(hb + 1):
            if k + i < n:
                B[i, k] = f32(Sd[k + i, k])
    for k in range(n):
        kmax = min(hb, n - 1 - k)
        d = f32(B[0, k])
        if abs(d) < 1e-38:
            return None, f"zero pivot at {k} d={d}", None, None, None
        for i_off in range(1, kmax + 1):
            r = k + i_off
            s_rk = B[i_off, k]
            t = f32(f32(s_rk) / d)
            for j in range(r, min(r + hb, n - 1) + 1):
                lk = j - k; lr = j - r
                if lk <= hb:
                    B[lr, r] = f32(B[lr, r] - f32(f32(B[lk, k]) * t))
        for i_off in range(1, kmax + 1):
            B[i_off, k] = f32(B[i_off, k] / d)
    # forward solve L y = rhs
    y = np.empty(n, np.float32)
    for k in range(n):
        acc = f32(rhs[k])
        for i in range(1, min(hb, k) + 1):
            acc = f32(acc - f32(B[i, k - i] * y[k - i]))
        y[k] = acc
    # D-divide
    y = f32(y / B[0, :])
    # back solve L^T x = y
    x = np.empty(n, np.float32)
    for k in range(n - 1, -1, -1):
        acc = y[k]
        for i in range(1, min(hb, n - 1 - k) + 1):
            acc = f32(acc - f32(B[i, k] * x[k + i]))
        x[k] = acc
    # flatten factored B in RTL layout k*(hb+1)+i
    Bf = np.zeros((hb + 1) * n, np.float32)
    for k in range(n):
        for i in range(hb + 1):
            Bf[k * (hb + 1) + i] = B[i, k]
    return x, None, Bf, y

def main():
    A, b, q, m, n = assemble(46.6093)
    bl = build(A, b, q)
    S0 = bl["S"].toarray()
    Ab = bl["Ab"]
    rng = np.random.default_rng(42)
    # only dump the scale=1 case (matches the RTL run we're debugging)
    for scale in [1.0]:
        Ss = scale * S0
        Ss[np.diag_indices(n)] += RHO_X
        v_xr = rng.standard_normal(n); v_yr = rng.standard_normal(m)
        rhs = RHO_X * v_xr - Ab.T @ v_yr
        zx64 = banded_ldl_factor_solve(sp.csc_matrix(Ss), rhs, hb=17)
        res = banded_ldl_f32(Ss, rhs, hb=17)
        x32, err, Bf, y = res
        if err:
            print("EMU FAILED:", err); return
        rel = np.max(np.abs(x32 - zx64) / (np.abs(zx64) + 1e-30))
        resid = np.linalg.norm(Ss @ x32 - rhs) / np.linalg.norm(rhs)
        print(f"scale={scale:g}: float32 rel_max={rel:.3e} resid={resid:.3e}")
        with open("zx_emu.mem", "w") as f:
            for v in x32: f.write(f"{struct.unpack('<I',struct.pack('<f',float(v)))[0]:08X}\n")
        with open("B_emu.mem", "w") as f:
            for v in Bf: f.write(f"{struct.unpack('<I',struct.pack('<f',float(v)))[0]:08X}\n")
        with open("y_emu.mem", "w") as f:
            for v in y: f.write(f"{struct.unpack('<I',struct.pack('<f',float(v)))[0]:08X}\n")
        print("dumped zx_emu/B_emu/y_emu.mem")

if __name__ == "__main__":
    main()
