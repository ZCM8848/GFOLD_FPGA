#!/usr/bin/env python3
"""Bit-exact emulation of the RTL float32 units (truncating, NOT round-to-nearest)
and the banded-LDL datapath. This is the definitive test for whether the RTL's
full-size explosion is a LOGIC BUG or pure TRUNCATING-float32 precision.

If this truncating emulation reproduces the RTL's B_fact.mem (bit-for-bit), the
RTL logic is correct and the blowup is inherent to truncating float32 on the
cond-1e6 S0 (pivot ~1e-30). We also get a faithful SW model of the RTL datapath
usable to validate Stage-B BFP arithmetic.
"""
import struct, numpy as np
import scipy.sparse as sp
from banded_reference import build, banded_ldl_factor_solve, RHO_X
from assembler import assemble, N

def bits(f): return struct.unpack("<I", struct.pack("<f", f))[0]
def f2b(f):  return bits(f)
def b2f(b):  return struct.unpack("<f", struct.pack("<I", b))[0]

# ---------------- truncating float32 units (mirror float32.v) ----------------
def mul_t(a, b):
    sa, ea, fa = a >> 31, (a >> 23) & 0xff, a & 0x7fffff
    sb, eb, fb = b >> 31, (b >> 23) & 0xff, b & 0x7fffff
    za = (a & 0x7fffffff) == 0; zb = (b & 0x7fffffff) == 0
    ma = (1 << 23) | fa; mb = (1 << 23) | fb
    m = ma * mb                       # 48-bit
    shift = (m >> 47) & 1
    e = ea + eb + shift - 127
    if za or zb: return 0
    mant = (m >> 24) & 0x7fffff if shift else (m >> 23) & 0x7fffff
    return ((sa ^ sb) << 31) | ((e & 0xff) << 23) | mant

def add_t(a, b, sub):
    bo = b ^ (sub << 31)
    sa, sb = a >> 31, bo >> 31
    ea, eb = (a >> 23) & 0xff, (bo >> 23) & 0xff
    fa, fb = a & 0x7fffff, bo & 0x7fffff
    za = (a & 0x7fffffff) == 0; zb = (bo & 0x7fffffff) == 0
    same = (sa == sb)
    diff = ea - eb
    swap = (diff < 0) or za or (diff == 0 and fa < fb)
    el = eb if swap else ea; es = ea if swap else eb
    fl = fb if swap else fa; fs = fa if swap else fb
    sl = sb if swap else sa
    sh = el - es
    ml = (1 << 23) | fl              # {3'b0,1'b1,fl} -> 1.fl
    if sh > 27: ms = 0
    else: ms = ((1 << 23) | fs) >> sh
    msum = (ml + ms) if same else (ml - ms)
    msign = (msum >> 26) & 1
    mabs = ((~msum + 1) & 0x7ffffff) if msign else msum
    # clz on mabs[25:0]
    lzc = 26
    for i in range(25, -1, -1):
        if (mabs >> i) & 1: lzc = 25 - i; break
    en = el - lzc + 2
    mn = mabs << lzc
    if za: return bo
    if zb: return a
    if mabs == 0: return 0
    # sign: same? sl : (msign ? (sb if swap else sa) : sl)
    if same: s = sl
    else: s = (sb if swap else sa) if msign else sl
    return (s << 31) | ((en & 0xff) << 23) | ((mn >> 2) & 0x7fffff)

def div_t(a, b):
    sa, ea, fa = a >> 31, (a >> 23) & 0xff, a & 0x7fffff
    sb, eb, fb = b >> 31, (b >> 23) & 0xff, b & 0x7fffff
    az = (a & 0x7fffffff) == 0; bz = (b & 0x7fffffff) == 0
    if az or bz:
        if bz and not az: return ((sa ^ sb) << 31) | 0x7f800000   # inf
        return (sa ^ sb) << 31                                     # ±0
    divs = (1 << 23) | fb
    if ((1 << 23) | fa) >= divs:
        divd = (1 << 23) | fa; expo = ea - eb + 127
    else:
        divd = ((1 << 23) | fa) << 1; expo = ea - eb + 126
    R = divd - divs                        # it=0: extract leading 1
    Q = 0
    for it in range(1, 24):
        Rsh = (R << 1) & 0x1ffffff
        if Rsh >= divs: R = Rsh - divs; Q |= 1 << (23 - it)
        else: R = Rsh
    return ((sa ^ sb) << 31) | ((expo & 0xff) << 23) | Q

# ---------------- banded LDL using truncating units (mirror RTL FSM) ---------
def banded_ldl_trunc(Sd, rhs, hb=17):
    n = Sd.shape[0]
    Sd = np.asarray(Sd).astype(np.float32)
    B = np.zeros((hb + 1, n), np.int64)
    for k in range(n):
        for i in range(hb + 1):
            if k + i < n: B[i, k] = bits(Sd[k + i, k])
    for k in range(n):
        kmax = min(hb, n - 1 - k)
        d = B[0, k]
        for i_off in range(1, kmax + 1):
            r = k + i_off
            s_rk = B[i_off, k]
            t = div_t(s_rk, d)
            for j in range(r, min(r + hb, n - 1) + 1):
                lk = j - k; lr = j - r
                if lk <= hb:
                    prod = mul_t(B[lk, k], t)
                    B[lr, r] = add_t(B[lr, r], prod, 1)   # a - b
        for i_off in range(1, kmax + 1):
            B[i_off, k] = div_t(B[i_off, k], d)
    # forward solve
    y = np.empty(n, np.int64)
    for k in range(n):
        acc = bits(rhs[k])
        for i in range(1, min(hb, k) + 1):
            acc = add_t(acc, mul_t(B[i, k - i], y[k - i]), 1)
        y[k] = acc
    # D-divide
    yd = np.empty(n, np.int64)
    for k in range(n): yd[k] = div_t(y[k], B[0, k])
    # back solve
    x = np.empty(n, np.int64)
    for k in range(n - 1, -1, -1):
        acc = yd[k]
        for i in range(1, min(hb, n - 1 - k) + 1):
            acc = add_t(acc, mul_t(B[i, k], x[k + i]), 1)
        x[k] = acc
    Bf = np.zeros((hb + 1) * n, np.int64)
    for k in range(n):
        for i in range(hb + 1): Bf[k * (hb + 1) + i] = B[i, k]
    return x, Bf, y

def main():
    # validate units against known values
    print("unit checks:")
    print("  1.0/2000  RTL=0x3a03126e  mine=", f"{div_t(0x3f800000,0x44fa0000):08X}")
    print("  0/2000    RTL=0x00000000  mine=", f"{div_t(0,0x44fa0000):08X}")
    print("  2000/0    RTL=0x7f800000  mine=", f"{div_t(0x44fa0000,0):08X}")
    print("  5.5*2000  mul=", f"{mul_t(0x40b00000,0x44fa0000):08X}")
    A, b, q, m, n = assemble(46.6093)
    bl = build(A, b, q); S0 = bl["S"].toarray()
    Ss = 1.0 * S0; Ss[np.diag_indices(n)] += RHO_X
    rng = np.random.default_rng(42)
    v_xr = rng.standard_normal(n); v_yr = rng.standard_normal(m)
    Ab = bl["Ab"]
    rhs = RHO_X * v_xr - Ab.T @ v_yr
    x, Bf, y = banded_ldl_trunc(Ss, rhs, hb=17)
    # compare Bf against RTL B_fact.mem
    rtlB = np.array([int(l, 16) for l in open("../rtl/sim/B_fact.mem").read().split()])
    nB = len(Bf)
    same = (Bf[:nB] == rtlB[:nB])
    ndiff = int((~same).sum())
    print(f"\nB_fact trunc-emu vs RTL: {nB} entries, {ndiff} differ")
    if ndiff:
        first = int(np.argmax(~same))
        print(f"  first differ idx={first} (k={first//18} i={first%18}) "
              f"emu={Bf[first]:08X} rtl={rtlB[first]:08X}")
    # does truncating emu explode like RTL? report a few tail zx
    print("  zx trunc-emu tail:", [f"{x[k]:08X}({b2f(x[k]):.4e})" for k in [1097,1098,1099]])

if __name__ == "__main__":
    main()
