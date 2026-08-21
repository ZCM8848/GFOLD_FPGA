"""Reproduce gen_chol10 case-0 exactly and print numpy G, L, gamma for comparison."""
import sys, struct
import numpy as np
sys.path.insert(0,'software')
from gen_chol10 import gen_case
rng = np.random.default_rng(99)
G, rhs, gamma = gen_case(10, rng)
np.set_printoptions(precision=4, suppress=True)
print("numpy G r0:", G[0][:3])
print("numpy G r1:", G[1][:3])
print("numpy rhs0,1:", rhs[0], rhs[1])
L = np.linalg.cholesky(G)
print("numpy L r0:", L[0][:3])
print("numpy L r1:", L[1][:3])
print("numpy L r2:", L[2][:3])
print("numpy gamma:", gamma)
