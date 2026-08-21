#!/usr/bin/env python3
"""Smoke: Qf=None (must reproduce float solver) then Q(32,19)."""
import numpy as np
from fixed_point import Q
from fixed_adaptive import solve_fixed_adaptive
from scs_adaptive import c

for name, Qf in [("float(None)", None), ("Q(32,19)", Q(32, 19)), ("Q(24,11)", Q(24, 11))]:
    r = solve_fixed_adaptive(5000, Qf, track=2500)
    print(f"{name} @5000: rel_x={r['relx']:.4e} obj={r['obj']:.6f} "
          f"tau={r['tau']:.5f} scale={r['scale']:.2e} upd={r['n_scale_updates']}")
    for it, rx, o in r['hist']:
        print(f"      {it:5d} rel_x={rx:.3e} obj={o:.6f}")
