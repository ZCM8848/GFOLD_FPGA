#!/usr/bin/env python3
"""Fixed-point arithmetic core for Phase 2.

Simulates Qm.n fixed-point (m total bits incl sign, n fractional bits) with
configurable rounding and saturation. Used to build a fixed-point version of
the SCS DRS solver and measure its accuracy vs the float golden reference.
"""
import numpy as np

class Q:
    """Fixed-point quantizer: Q(m, n) = signed, m total bits, n fractional bits.
    Range: [-(2^(m-1)), 2^(m-1) - 2^-n]. LSB = 2^-n."""
    __slots__ = ("m", "n", "max_int", "lsb", "rng", "intbits", "round_mode", "max_lsb")

    def __init__(self, m, n, round_mode="rnd"):
        self.m = m
        self.n = n
        self.lsb = 2.0 ** (-n)
        # integer bits (excl sign) = m-1-n ; representable magnitude < 2^(m-1-n)
        self.intbits = m - 1 - n
        # max value magnitude
        self.max_int = 2.0 ** self.intbits
        self.rng = self.max_int
        # clip bound in LSB units: value max_int needs max_int/lsb = 2^(m-1) LSBs
        self.max_lsb = 2.0 ** (m - 1)
        self.round_mode = round_mode

    def q(self, x):
        """Quantize a float (or array) to this format."""
        x = np.asarray(x, dtype=np.float64)
        # scale to fixed units, round
        s = x / self.lsb
        if self.round_mode == "rnd":
            q = np.floor(s + 0.5)
        elif self.round_mode == "trunc":
            q = np.trunc(s)
        elif self.round_mode == "rnd_away":
            q = np.sign(s) * np.floor(np.abs(s) + 0.5)
        else:
            raise ValueError(self.round_mode)
        # saturation in LSB units: max value 2^intbits == 2^(m-1) LSBs
        q = np.clip(q, -self.max_lsb, self.max_lsb - 1)
        return q * self.lsb

    def __repr__(self):
        return f"Q({self.m},{self.n})"

def qmul(Qa, Qb, Qout, a, b):
    """Fixed-point multiply: a,b in Qa/Qb (already quantized floats); result in Qout.
    Assumes a,b are Python floats that are exact multiples of their LSBs (i.e.,
    already quantized), so the product is exact in double, then we quantize."""
    return Qout.q(a * b)

def qadd(Qout, a, b):
    """Fixed-point add with saturation."""
    return Qout.q(a + b)

# ---- quantization error helpers ----
def qerr(Q, x):
    """Max absolute quantization error = half LSB."""
    return Q.lsb / 2.0

if __name__ == "__main__":
    q16 = Q(16, 12)
    print("Q(16,12): lsb=%.5f range=±%.0f" % (q16.lsb, q16.rng))
    for v in [1.0, 0.0005, 3500.0, 4096.0, -3500.0]:
        print(f"  {v:9.1f} -> {q16.q(v):9.4f}  (err {q16.q(v)-v:.2e})")
