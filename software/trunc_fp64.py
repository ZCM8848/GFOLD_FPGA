#!/usr/bin/env python3
"""trunc_fp64.py — 截断 FP64 算术（匹配 RTL fp64_mul/fp64_add 的 truncation 行为）。
RTL 用截断（丢弃低位，不四舍五入），numpy 用 round-to-nearest。
本模块用 Python int 精确模拟 RTL 的截断行为。"""
import struct, math

def f2u(f):
    return struct.unpack('<Q', struct.pack('<d', f))[0]

def u2f(u):
    return struct.unpack('<d', struct.pack('<Q', u))[0]

def tmul(a, b):
    """截断 FP64 乘法，匹配 fp64_mul.v"""
    if a == 0.0 or b == 0.0:
        return 0.0
    ua = f2u(a); ub = f2u(b)
    sa = (ua >> 63) & 1; sb = (ub >> 63) & 1
    ea = (ua >> 52) & 0x7FF; eb = (ub >> 52) & 0x7FF
    fa = ua & 0xFFFFFFFFFFFFF; fb = ub & 0xFFFFFFFFFFFFF
    ma = (1 << 52) | fa
    mb = (1 << 52) | fb
    m = ma * mb  # 106-bit exact
    shift = (m >> 105) & 1
    if shift:
        mant = (m >> 53) & ((1 << 52) - 1)
    else:
        mant = (m >> 52) & ((1 << 52) - 1)
    e = ea + eb + shift - 1023
    if e <= 0: return 0.0
    if e >= 2047: e = 2046  # cap (no inf handling)
    return u2f((sa ^ sb) << 63 | e << 52 | mant)

def tadd(a, b, sub=False):
    """截断 FP64 加/减，匹配 fp64_add.v"""
    if a == 0.0: return -b if sub else b
    if b == 0.0: return a
    ua = f2u(a); ub = f2u(b)
    bo = ub ^ (sub << 63)  # flip sign for sub (match RTL: b[63]^sub)
    sa = (ua >> 63) & 1; sb = (bo >> 63) & 1
    ea = (ua >> 52) & 0x7FF; eb = (bo >> 52) & 0x7FF
    fa = ua & 0xFFFFFFFFFFFFF; fb = bo & 0xFFFFFFFFFFFFF
    za = (ea == 0 and fa == 0); zb = (eb == 0 and fb == 0)
    if za: return u2f(bo)
    if zb: return u2f(ua)
    same = (sa == sb)
    # Ensure a has larger exponent
    if ea < eb or (ea == eb and fa < fb):
        ua, bo = bo, ua; sa, sb = sb, sa; ea, eb = eb, ea; fa, fb = fb, fa
    ml = (1 << 52) | fa
    sh = ea - eb
    if sh > 53:
        ms = 0
    else:
        ms = ((1 << 52) | fb) >> sh
    msum = ml + ms if same else ml - ms
    if msum == 0:
        return 0.0
    if same:
        if msum >> 53:  # carry
            mant = (msum >> 1) & ((1 << 52) - 1)
            e = ea + 1
        else:
            mant = msum & ((1 << 52) - 1)
            e = ea
    else:
        # Leading zero count
        lzc = 0
        for i in range(53):
            if (msum >> (52 - i)) & 1:
                break
            lzc += 1
        if lzc == 0:
            mant = msum & ((1 << 52) - 1)
            e = ea
        else:
            mant = (msum << lzc) & ((1 << 52) - 1)
            e = ea - lzc
    if e <= 0: return 0.0
    if e >= 2047: e = 2046
    return u2f(sa << 63 | e << 52 | mant)

def tsub(a, b):
    return tadd(a, b, sub=True)

def tdiv(a, b):
    """FP64 除法 — RTL 用 restoring division (52 iter)。
    Python 用 round-to-nearest（误差 ≤ 0.5 ULP，远小于 5e-10 阈值）。"""
    if b == 0.0: return float('inf')
    return a / b  # Python float division (RNE, but error negligible)

def trsqrt(x):
    """FP64 rsqrt — RTL 用 Newton-Raphson (5 iter)。
    Python 用 math.sqrt，误差可忽略。"""
    if x <= 0.0: return 0.0
    return 1.0 / math.sqrt(x)
