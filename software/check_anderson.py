"""Bit-exact verification of anderson.v against the truncating-FP64 emulator.

The RTL's FP64 units (fp64.v) are TRUNCATING (no round-to-nearest). The round-nearest
numpy oracle (gen_anderson.py / aa_*.hex) therefore differs from the RTL in the solve
calls by ~1e-5 — the conditioning-limited amplification (cond(G)~3.5e7 for the
normal-equations matrix) of a truncate-vs-round 1-ULP difference. That is inherent,
NOT a bug.

The correct oracle for a truncating implementation is anderson_emu.py, which replays
the RTL's exact truncating mul/add/div/rsqrt and operation order. Driven by the SAME
x/f inputs the testbench reads from aa_<c>.hex, the RTL must match it BIT-EXACT for
every apply call (and every internal quantity on the solve path).

Usage: python check_anderson.py <rtl_dump.txt> [DIM] [NCALL]
"""
import sys, struct, re
import numpy as np
import anderson_emu as E

def dec(h): return struct.unpack('>d', bytes.fromhex(h.strip()))[0]
def i2h(x): return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')

DIM = int(sys.argv[2]) if len(sys.argv) > 2 else 8
NCALL = int(sys.argv[3]) if len(sys.argv) > 3 else 25
E.DIM = DIM; E.MEM = 10

# ---- run the emulator, driven by the same hex x/f the tb feeds the RTL ----
E.reset_global()
emu_f = {}
for c in range(NCALL):
    rows = [l.split() for l in open(f'rtl/data/aa_{c}.hex')]
    x = np.array([dec(rows[i][0]) for i in range(DIM)])
    f = np.array([dec(rows[i][1]) for i in range(DIM)])
    emu_f[c] = np.array(E.apply_emu_call(x, f))

# ---- parse RTL dump ----
rtl_f = {}
for line in open(sys.argv[1] if len(sys.argv) > 1 else 'rtl/sim/an_dump.txt'):
    m = re.search(r'CASE (\d+):(.*)', line)
    if m: rtl_f[int(m.group(1))] = [dec(h) for h in m.group(2).split()[:DIM]]

worst = 0; worstc = None
for c in range(NCALL):
    e = emu_f[c]; r = rtl_f[c]
    diff = sum(i2h(a) != i2h(b) for a, b in zip(e, r))
    rel = float(np.max(np.abs(np.array(r) - e) / (np.abs(e) + 1e-30)))
    if diff > worst: worst = diff; worstc = c
    flag = "  <-- BIT MISMATCH" if diff else ""
    print(f"case {c:2d}: bit_diff={diff:2d}  max_rel={rel:.2e}{flag}")

print(f"\nWORST bit_diff = {worst} at case {worstc}")
print("PASS (RTL bit-exact vs truncating-FP64 emulator)" if worst == 0 else "FAIL")
