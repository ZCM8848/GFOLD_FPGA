#!/usr/bin/env python3
"""Analyze S0 band magnitudes to design the Stage-B BFP block format for the
banded LDL. Question: what shared-exponent grouping keeps block dynamic range
small enough for a 32-bit mantissa (rel prec 2^-30)?"""
import numpy as np
from banded_reference import build, RHO_X
from assembler import assemble

A, b, q, m, n = assemble(46.6093)
bl = build(A, b, q); S0 = bl['S'].toarray()
Ss = 1.0 * S0; Ss[np.diag_indices(n)] += RHO_X
hb = 17

diag = np.abs(np.diag(Ss))
offmax = np.array([np.max(np.abs(Ss[k+1:min(k+hb, n), k])) if k + 1 < n else 0 for k in range(n)])
# col 9 (near-singular pivot) examine full column
for k in [9]:
    print(f"column {k}: diag={diag[k]:.3e}")
    for i in range(hb+1):
        if k+i<n and Ss[k+i,k]!=0: print(f"  off i={i}: S[{k+i}][{k}]={Ss[k+i,k]:.3e}")

print("\nper-column dynamic range (max|col| / min nonzero|col|, log2):")
dr = np.array([np.log2(np.abs(Ss[k:min(k+hb+1,n),k]).max() /
                       np.abs(Ss[k:min(k+hb+1,n),k])[np.abs(Ss[k:min(k+hb+1,n),k])>0].min())
               for k in range(n)])
print(f"  col dynamic-range log2: min={dr.min():.1f} med={np.median(dr):.1f} max={dr.max():.1f}")

# BFP options: per-column shared exponent
print("\nPer-column BFP: exp=floor(log2(max|col|)); smallest col value needs",
      int(dr.max()), "bits below max -> lost entirely if >30.")
# how many columns have a value >2^30 below the col max (i.e. would zero out at mant=32)?
colmax = np.array([np.abs(Ss[k:min(k+hb+1,n),k]).max() for k in range(n)])
colmin = np.array([np.abs(Ss[k:min(k+hb+1,n),k])[np.abs(Ss[k:min(k+hb+1,n),k])>0].min() for k in range(n)])
lost = int((np.log2(colmax/colmin) > 30).sum())
print(f"  cols with intra-col range >2^30 (value lost at mant=32): {lost}/{n}")

# Off-diagonal vs diagonal: if diagonal row separate block from off-diag?
print("\ndiag row (i=0) vs off-diag rows: diag values ~1e3, off-diag:")
offall = np.abs(Ss[np.tril_indices(n,-1)])
print(f"  all off-diag nonzero: min={offall[offall>0].min():.3e} max={offall.max():.3e}")
