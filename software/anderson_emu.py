"""Bit-exact truncating-FP64 emulation of the anderson.v orchestrator pipeline.

Replicates the RTL's truncating fp64_mul/add/div/rsqrt and the exact operation
order of aa_gram, chol10, and the LOAD/FROB/RREG/APPLY states. If the RTL f_out
matches this emulator to ~1e-13, the RTL datapath is faithful and any deviation
from the round-nearest numpy oracle (gen_anderson.py) is purely the conditioning
amplification of the ill-conditioned normal-equations matrix, not a logic bug.

NOTE: this intentionally uses TRUNCATING (no round-to-nearest) FP64, exactly like
the RTL fp64.v units.
"""
import struct
import numpy as np

MASK52 = 0xFFFFFFFFFFFFF
MASK63 = 0x7FFFFFFFFFFFFFFF
SIGN   = 0x8000000000000000

def d2i(x): return struct.unpack('>Q', struct.pack('>d', float(x)))[0]
def i2d(i): return struct.unpack('>d', struct.pack('>Q', i & 0xFFFFFFFFFFFFFFFF))[0]

def mul(a, b):
    ia = d2i(a); ib = d2i(b)
    sa = ia >> 63; sb = ib >> 63
    ea = (ia >> 52) & 0x7FF; eb = (ib >> 52) & 0x7FF
    fa = ia & MASK52; fb = ib & MASK52
    if (ia & MASK63) == 0 or (ib & MASK63) == 0:
        return 0.0
    ma = (1 << 52) | fa; mb = (1 << 52) | fb
    m = ma * mb
    shift = (m >> 105) & 1
    e = ea + eb + shift - 1023
    mant = (m >> 53) if shift else (m >> 52)
    return i2d(((sa ^ sb) << 63) | ((e & 0x7FF) << 52) | (mant & MASK52))

def add(a, b, sub=False):
    ia = d2i(a); ib = d2i(b)
    bo = ib ^ SIGN if sub else ib
    sa = ia >> 63; sb = bo >> 63
    ea = (ia >> 52) & 0x7FF; eb = (bo >> 52) & 0x7FF
    fa = ia & MASK52; fb = bo & MASK52
    za = (ia & MASK63) == 0; zb = (bo & MASK63) == 0
    if za: return i2d(bo)
    if zb: return i2d(ia)
    same = (sa == sb)
    diff = ea - eb
    swap = (diff < 0) or (diff == 0 and fa < fb)
    if swap:
        el, es = eb, ea; fl, fs = fb, fa; sl = sb
    else:
        el, es = ea, eb; fl, fs = fa, fb; sl = sa
    sh = el - es
    ml = (1 << 52) | fl
    ms = 0 if sh > 53 else ((1 << 52 | fs) >> sh)
    msum = (ml + ms) if same else (ml - ms)
    if not same:
        if msum == 0: return 0.0
        lzc = 53 - msum.bit_length()
        return i2d((sl << 63) | (((el - lzc) & 0x7FF) << 52) | ((msum << lzc) & MASK52))
    else:
        if msum >> 53:
            return i2d((sl << 63) | (((el + 1) & 0x7FF) << 52) | ((msum >> 1) & MASK52))
        else:
            return i2d((sl << 63) | ((el & 0x7FF) << 52) | (msum & MASK52))

def rsqrt(x):
    ix = d2i(x)
    e_true = ((ix >> 52) & 0x7FF) - 1023
    etr2 = e_true >> 1                      # floor(e/2)
    se_s = 1022 - etr2
    se = 1 if se_s < 1 else (2046 if se_s > 2046 else se_s)
    y = i2d((se & 0x7FF) << 52)
    c15 = i2d(0x3FF8000000000000); c05 = i2d(0x3FE0000000000000)
    for _ in range(6):
        t1 = mul(y, y); t2 = mul(x, t1); t3 = mul(c05, t2)
        t4 = add(c15, t3, sub=True)
        y = mul(y, t4)
    return y

def sqrtf(a): return mul(a, rsqrt(a))

def div(a, b):
    ia = d2i(a); ib = d2i(b)
    sa = ia >> 63; sb = ib >> 63
    ea = (ia >> 52) & 0x7FF; eb = (ib >> 52) & 0x7FF
    fa = ia & MASK52; fb = ib & MASK52
    if (ia & MASK63) == 0 or (ib & MASK63) == 0:
        if (ib & MASK63) == 0 and (ia & MASK63) != 0:
            return i2d(((sa ^ sb) << 63) | (0x7FF << 52))   # ±inf
        return 0.0
    divs = (1 << 52) | fb
    if ((1 << 52) | fa) >= divs:
        divd = (1 << 52) | fa; expo = ea - eb + 1023   # {1'b0,1'b1,a} = 2^52+fa
    else:
        divd = ((1 << 52) | fa) << 1; expo = ea - eb + 1022
    R = divd - divs
    Q = 0
    for it in range(1, 53):
        Rsh = R << 1
        if Rsh >= divs:
            R = Rsh - divs; Q |= (1 << (52 - it))
        else:
            R = Rsh
    return i2d(((sa ^ sb) << 63) | ((expo & 0x7FF) << 52) | (Q & MASK52))

# ---------------------------------------------------------------------------
# Pipeline emulation (mirrors anderson.v states exactly)
# ---------------------------------------------------------------------------
AA_R = 1e-8
DIM, MEM = 8, 10

def aa_gram_emu(S, garr, rreg):
    # S: row-major [p*MEM+col]; returns G (row-major MEM*MEM), rhs
    G = [0.0]*(MEM*MEM); rhs = [0.0]*MEM
    for dj in range(MEM):
        for dk in range(dj, MEM):
            gacc = 0.0
            for p in range(DIM):
                gacc = add(gacc, mul(S[p*MEM+dj], S[p*MEM+dk]))
            if dj == dk:
                G[dj*MEM+dk] = add(gacc, rreg)
            else:
                G[dj*MEM+dk] = gacc; G[dk*MEM+dj] = gacc
    for dj in range(MEM):
        gacc = 0.0
        for p in range(DIM):
            gacc = add(gacc, mul(S[p*MEM+dj], garr[p]))
        rhs[dj] = gacc
    return G, rhs

def chol10_emu(G, rhs):
    # returns gamma
    L = [0.0]*(MEM*MEM); z = list(rhs); gamma = [0.0]*MEM
    for cj in range(MEM):
        lacc = G[cj*MEM+cj]
        for q in range(cj):
            lacc = add(lacc, mul(L[cj*MEM+q], L[cj*MEM+q]), sub=True)
        L[cj*MEM+cj] = mul(lacc, rsqrt(lacc))
        for ci in range(cj+1, MEM):
            lacc = G[ci*MEM+cj]
            for q in range(cj):
                lacc = add(lacc, mul(L[ci*MEM+q], L[cj*MEM+q]), sub=True)
            L[ci*MEM+cj] = div(lacc, L[cj*MEM+cj])
    for i in range(MEM):                       # forward L z = rhs
        lacc = z[i]
        for q in range(i):
            lacc = add(lacc, mul(L[i*MEM+q], z[q]), sub=True)
        z[i] = div(lacc, L[i*MEM+i])
    for i in range(MEM-1, -1, -1):             # back L^T gamma = z
        lacc = z[i]
        for q in range(MEM-1, i, -1):
            lacc = add(lacc, mul(L[q*MEM+i], gamma[q]), sub=True)
        gamma[i] = div(lacc, L[i*MEM+i])
    return gamma

def apply_emu(farr, D, gamma):
    out = [0.0]*DIM
    for i in range(DIM):
        appacc = farr[i]
        for j in range(MEM):
            appacc = add(appacc, mul(D[i*MEM+j], gamma[j]), sub=True)
        out[i] = appacc
    return out

def frob_emu(acc):
    fsum = 0.0
    for i in range(MEM):
        fsum = add(fsum, acc[i])
    return mul(fsum, rsqrt(fsum))          # sqrt(fsum)

# state
xarr = [0.0]*DIM; farr = [0.0]*DIM; garr = [0.0]*DIM; gprev = [0.0]*DIM
S = [0.0]*(DIM*MEM); D = [0.0]*(DIM*MEM); Y = [0.0]*(DIM*MEM)
acc_s = [0.0]*MEM; acc_y = [0.0]*MEM
it = 0
last_rreg = 0.0; last_gamma = [0.0]*MEM

def reset_global():
    global xarr, farr, garr, gprev, S, D, Y, acc_s, acc_y, it, last_rreg, last_gamma
    xarr = [0.0]*DIM; farr = [0.0]*DIM; garr = [0.0]*DIM; gprev = [0.0]*DIM
    S = [0.0]*(DIM*MEM); D = [0.0]*(DIM*MEM); Y = [0.0]*(DIM*MEM)
    acc_s = [0.0]*MEM; acc_y = [0.0]*MEM
    it = 0; last_rreg = 0.0; last_gamma = [0.0]*MEM

def apply_emu_call(x, f):
    global xarr, farr, garr, gprev, S, D, Y, acc_s, acc_y, it, last_rreg, last_gamma
    if it == 0:
        for p in range(DIM):
            xarr[p] = x[p]; farr[p] = f[p]
            gprev[p] = add(x[p], f[p], sub=True)
        it += 1
        return list(f)
    idx = (it - 1) % MEM
    acc_s[idx] = 0.0; acc_y[idx] = 0.0
    for p in range(DIM):
        Sreg = add(x[p], xarr[p], sub=True)
        Dreg = add(f[p], farr[p], sub=True)
        Greg = add(x[p], f[p], sub=True)
        Yreg = add(Greg, gprev[p], sub=True)
        S[p*MEM+idx] = Sreg; D[p*MEM+idx] = Dreg
        garr[p] = Greg; gprev[p] = Greg
        acc_s[idx] = add(acc_s[idx], mul(Sreg, Sreg))
        acc_y[idx] = add(acc_y[idx], mul(Yreg, Yreg))
        xarr[p] = x[p]; farr[p] = f[p]
    if it >= MEM:
        frob_s = frob_emu(acc_s); frob_y = frob_emu(acc_y)
        rreg = mul(mul(frob_s, frob_y), AA_R)
        G, rhs = aa_gram_emu(S, garr, rreg)
        gamma = chol10_emu(G, rhs)
        last_rreg = rreg; last_gamma = gamma
        out = apply_emu(farr, D, gamma)
    else:
        out = list(f)
    it += 1
    return out

def main():
    global last_rreg, last_gamma
    # Drive the emulator with the SAME x/f inputs the RTL testbench reads from the
    # hex files (aa_<c>.hex: "x f fout"), so both see identical inputs. The RTL tb
    # feeds x/f from these files; the emulator must do the same (not chain its own
    # truncating fout, which would differ from gen_anderson's round-nearest x by the
    # conditioning-amplified ~1e-5 and make a false mismatch).
    for c in range(25):
        rows = [l.split() for l in open(f'rtl/data/aa_{c}.hex')]
        x = np.array([struct.unpack('>d', bytes.fromhex(rows[i][0]))[0] for i in range(DIM)])
        f = np.array([struct.unpack('>d', bytes.fromhex(rows[i][1]))[0] for i in range(DIM)])
        fout = apply_emu_call(x, f)
        line = f"call {c}: " + ' '.join('%.16e'%v for v in fout)
        print(line)
        if c >= MEM:
            idx = (c-1) % MEM
            scol = [S[p*MEM+idx] for p in range(DIM)]
            print(f"SCOL {c}: " + ' '.join('%.16e'%v for v in scol))
            print(f"XARR {c}: " + ' '.join('%.16e'%v for v in xarr))
            print(f"ACC_S {c}: " + ' '.join('%.16e'%v for v in acc_s))
            print(f"ACC_Y {c}: " + ' '.join('%.16e'%v for v in acc_y))
            print(f"FROB {c}: {frob_emu(acc_s):.16e} {frob_emu(acc_y):.16e}")
            print(f"RREG {c}: {last_rreg:.16e}")
            print(f"GAMMA {c}: " + ' '.join('%.16e'%v for v in last_gamma))

if __name__ == '__main__':
    main()
