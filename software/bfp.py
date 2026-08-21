#!/usr/bin/env python3
"""Block Floating Point (BFP) quantizer for Phase 2b/2c exploration.

Each block gets a SHARED power-of-2 exponent (from the block max); values are
stored as fixed-point mantissas normalized to ~[1,2). This hands the dynamic
range to the exponent so the mantissa needs only moderate precision, addressing
the finding that single-format fixed-point needs ~40-48 bit to reach rel_x 1e-3
(because tau collapses to 1e-3 and mixes with pos ~2400 in one format).

Variable layout (per node, 100 nodes, n=1100):
  per-node: pos(3)@6i+0..2, vel(3)@6i+3..5 ; then thrust(3)@[600,900),
  slack(1)@[900,1000), z(1)@[1000,1100). dual = [n, n+m), tau = index n+m.
"""
import numpy as np
from fixed_point import Q


def make_state_blocks(n):
    """Return list of index arrays grouping the x-state by physical variable.
    Each block has small dynamic range so the shared exponent does the work."""
    nn = n // 11
    blocks = []
    blocks.append(np.array([6 * i + c for i in range(nn) for c in range(3)], int))   # pos
    blocks.append(np.array([6 * i + c for i in range(nn) for c in range(3, 6)], int))  # vel
    blocks.append(np.arange(6 * nn, 9 * nn))        # thrust u
    blocks.append(np.arange(9 * nn, 10 * nn))       # slack s
    blocks.append(np.arange(10 * nn, n))            # z (log mass)
    return blocks


def bfp_q(x, mant, blocks):
    """Quantize array x under BFP: each block gets power-of-2 exp from its max,
    mantissa kept in signed Q(mant, mant-2) (range ~[1,2)), rescaled by 2^exp.
    Returns quantized x. Blocks not listed are left untouched (float)."""
    x = x.copy()
    for idx in blocks:
        blk = x[idx]
        mx = np.max(np.abs(blk)) if blk.size else 0.0
        if mx == 0.0:
            continue
        exp = np.floor(np.log2(mx))
        qm = Q(mant, mant - 2)          # range ±2, LSB 2^-(mant-2)
        x[idx] = qm.q(blk / (2.0 ** exp)) * (2.0 ** exp)
    return x


# BFP block layouts (each a list of index arrays)
STATE_BLOCKS = make_state_blocks(1100)          # 5 physical state blocks
STATE_DUAL = make_state_blocks(1100) + [None]    # state + dual as one list
# a layout that puts state, dual, tau each in one block
COARSE = [np.arange(0, 1100), np.arange(1100, 1100 + 2107)]
