#!/usr/bin/env python3
"""gen_small.py — generate a small on-chip-testable SOCP instance that uses the
SAME node-major cone layout as the real G-FOLD problem, but with NODE=2 and
N=10 so the whole solver state fits in PH1P35 ERAM (no DDR2).

Layout (must match proj_dual_cone_rb parameters):
    Z0=14 zero, then per node:
        NP=3 nonneg, 2 x SOC(dim4), 1 x SOC(dim3), ZP=7 zero,
    tail: TAIL_NN=1 nonneg, TAIL_Z=6 zero.
    NODE=2 -> M = 14 + 2*(3+8+3+7) + 7 = 63, N=10, L=74.

Outputs (rtl/data/kkt/small2/):
    v0.hex v1.hex v2.hex, u1/u2, ut1/ut2, rsk1/rsk2
    Arow.hex Acol.hex Aval.hex, b_r.hex, c_r.hex, zmask.hex,
    band_r.hex, g.hex, r_y.hex, Dy.hex, coo_sram.hex
    flash_image.txt  (COO | c | nb | zmask | band | g)
    manifest.json
"""
import os, json, struct
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "rtl", "data", "kkt", "small2")
os.makedirs(OUT, exist_ok=True)

# ---- problem size / cone layout ----
N = 10
NODE = 2
Z0, ZP, NP = 14, 7, 3
NS4, DIM4, NS3, DIM3 = 2, 4, 1, 3
TAIL_NN, TAIL_Z = 1, 6
# The RTL's proj_dual_cone_rb inserts ZP zero rows only between nodes
# 0 .. NODE-3 (NODE-2 transitions). This is why the real problem has
# M = 14 + 100*14 + 98*7 + 7 = 2107.
M = Z0 + NODE * (NP + NS4 * DIM4 + NS3 * DIM3) + max(NODE - 2, 0) * ZP + TAIL_NN + TAIL_Z
HB = 4
L = N + M + 1

RHO_X = 1e-6
TAU_FACTOR = 10.0
ALPHA = 1.5
FEAS = 1


def f64(v):
    return struct.unpack("<Q", struct.pack("<d", float(v)))[0]


def write_hex(name, vals):
    with open(os.path.join(OUT, name), "w") as f:
        for v in vals:
            f.write("%016X\n" % f64(v))


def build_zmask():
    zm = np.zeros(M, dtype=bool)
    zm[:Z0] = True
    r = Z0
    for node in range(NODE):
        r += NP + NS4 * DIM4 + NS3 * DIM3
        if node < NODE - 2:
            zm[r:r + ZP] = True
            r += ZP
    r += TAIL_NN
    zm[r:r + TAIL_Z] = True
    return zm


def build_blocks():
    blocks = [("zero", 0, Z0)]
    r = Z0
    for node in range(NODE):
        blocks.append(("nonneg", r, NP)); r += NP
        blocks.append(("soc", r, DIM4)); r += DIM4
        blocks.append(("soc", r, DIM4)); r += DIM4
        blocks.append(("soc", r, DIM3)); r += DIM3
        if node < NODE - 2:
            blocks.append(("zero", r, ZP)); r += ZP
    blocks.append(("nonneg", r, TAIL_NN)); r += TAIL_NN
    blocks.append(("zero", r, TAIL_Z)); r += TAIL_Z
    assert r == M, (r, M)
    return blocks


def proj_soc(x):
    """SCS SOC projection (dim from len)."""
    x = np.asarray(x, dtype=float)
    t = x[0]; v = x[1:]
    nv = np.linalg.norm(v)
    if nv <= t:
        return x.copy()
    if nv <= -t:
        return np.zeros_like(x)
    lam = 0.5 * (nv + t)
    return np.concatenate([[lam], (lam / nv) * v])


def proj_dual_cone_r(x, r_y, blocks):
    s = x.copy()
    x = -r_y * x
    for kind, st, dim in blocks:
        if kind == "zero":
            x[st:st + dim] = 0.0
        elif kind == "nonneg":
            x[st:st + dim] = np.maximum(x[st:st + dim], 0.0)
        else:
            x[st:st + dim] = proj_soc(x[st:st + dim])
    return x / r_y + s


def root_plus(diag_r, g, p, mu, eta):
    r = diag_r
    gg = mug = pg = pp = pmu = 0.0
    for i in range(len(p)):
        gi, pi, mui, ri = g[i], p[i], mu[i], r[i]
        gg += gi * gi * ri
        mug += mui * gi * ri
        pg += pi * gi * ri
        pp += pi * pi * ri
        pmu += pi * mui * ri
    a = r[-1] + gg
    b = mug - 2 * pg - eta * r[-1]
    cc = pp - pmu
    rad = b * b - 4 * a * cc
    if rad < 0:
        return -b / (2 * a)
    sq = np.sqrt(rad)
    if b <= 0:
        return (-b + sq) / (2 * a)
    q = -0.5 * (b + sq)
    return cc / q if q != 0 else 0.0


def one_iteration(A, br, cr, blocks, diag_r, lu, g, v, v_prev, i):
    n, m = A.shape[1], A.shape[0]
    l = n + m + 1
    sqrt_l = np.sqrt(l)
    if i >= FEAS:
        vn = np.linalg.norm(v)
        if vn > 0:
            v *= sqrt_l / vn
    v_prev[:] = v
    u_t = np.empty(l)
    u_t[:n] = v[:n] * diag_r[:n]
    u_t[n:l - 1] = -v[n:l - 1] * diag_r[n:l - 1]
    u_t[l - 1] = v[l - 1]
    u_t[:l - 1] = lu.solve(u_t[:l - 1])
    if i < FEAS:
        u_t[l - 1] = 1.0
    else:
        u_t[l - 1] = root_plus(diag_r, g, u_t[:l - 1], v[:l - 1], v[l - 1])
    u_t[:l - 1] -= u_t[l - 1] * g
    u = 2 * u_t - v
    u[n:l - 1] = proj_dual_cone_r(u[n:l - 1], diag_r[n:l - 1], blocks)
    if i < FEAS:
        u[l - 1] = 1.0
    else:
        u[l - 1] = max(u[l - 1], 0.0)
    rsk = (v + u - 2 * u_t) * diag_r
    v += ALPHA * (u - u_t)
    return v, u_t, u, rsk


def main():
    rng = np.random.default_rng(7)
    # A: M x N sparse; each row connects to 3 adjacent columns. The window
    # slides slowly across the column range (no modulo wrap), so the Schur
    # complement S = rho_x I + A^T D A stays within HB=4.
    rows, cols, vals = [], [], []
    for i in range(M):
        base = (i * (N - 3)) // max(M - 1, 1)
        for off in (0, 1, 2):
            rows.append(i); cols.append(base + off); vals.append(rng.uniform(-1, 1))
    A = sp.coo_matrix((vals, (rows, cols)), shape=(M, N)).tocsr()

    br = rng.uniform(-1, 1, M)
    cr = rng.uniform(-1, 1, N)
    zmask = build_zmask()
    blocks = build_blocks()

    r_y = np.where(zmask, 1.0 / 1000.0, 1.0)
    diag_r = np.concatenate([np.full(N, RHO_X), r_y, [TAU_FACTOR]])

    K = sp.bmat([[sp.diags(diag_r[:N], format="csc"), A.T],
                 [A, sp.diags(-diag_r[N:N + M], format="csc")]], format="csc")
    lu = splu(K)
    g = lu.solve(np.concatenate([cr, -br]))

    v = np.zeros(L); v[L - 1] = 1.0
    v_prev = np.zeros(L)
    states = []
    for i in range(2):
        v, u_t, u, rsk = one_iteration(A, br, cr, blocks, diag_r, lu, g, v, v_prev, i)
        states.append((v.copy(), u_t.copy(), u.copy(), rsk.copy()))

    # ---- band S = rho_x I + A^T D_y A,  D_y = 1/r_y ----
    Dy = 1.0 / r_y
    S = RHO_X * sp.eye(N) + A.T @ sp.diags(Dy) @ A
    S = ((S + S.T) * 0.5).tocsc()
    band = np.array([S[k + d, k] if k + d < N else 0.0
                     for k in range(N) for d in range(HB + 1)])
    bw = int(np.max(np.abs(S.tocoo().row - S.tocoo().col))) if S.nnz else 0
    assert bw <= HB, f"S bandwidth {bw} > HB {HB}"

    # ---- hex dumps ----
    write_hex("v0.hex", np.concatenate([[0.0] * (L - 1), [1.0]]))
    for i, (v, u_t, u, rsk) in enumerate(states, start=1):
        write_hex(f"v{i}.hex", v)
        write_hex(f"u{i}.hex", u)
        write_hex(f"ut{i}.hex", u_t)
        write_hex(f"rsk{i}.hex", rsk)
    write_hex("b_r.hex", br)
    write_hex("c_r.hex", cr)
    write_hex("r_y.hex", r_y)
    write_hex("Dy.hex", 1.0 / r_y)
    write_hex("zmask.hex", zmask.astype(float))
    write_hex("band_r.hex", band)
    write_hex("g.hex", g)
    A = A.tocoo()
    write_hex("Arow.hex", A.row.astype(float))
    write_hex("Acol.hex", A.col.astype(float))
    write_hex("Aval.hex", A.data)

    # COO SRAM format: word[2k] = (Acol<<16)|Arow, word[2k+1] = Aval
    coo = []
    for r, c, a in zip(A.row, A.col, A.data):
        coo.append((int(c) << 16) | int(r))
        coo.append(f64(a))
    with open(os.path.join(OUT, "coo_sram.hex"), "w") as f:
        for w in coo:
            f.write("%016X\n" % w)

    # ---- boot image: COO | c | nb(-b) | zmask | band | g ----
    nb = -br
    words = list(coo)
    words += [f64(x) for x in cr]
    words += [f64(x) for x in nb]
    words += [f64(x) for x in zmask.astype(float)]
    words += [f64(x) for x in band]
    words += [f64(x) for x in g]
    with open(os.path.join(OUT, "flash_image.txt"), "w") as f:
        for w in words:
            f.write("%016X\n" % (w & 0xFFFFFFFFFFFFFFFF))
    with open(os.path.join(OUT, "flash_image.bin"), "wb") as f:
        for w in words:
            f.write(struct.pack("<Q", w & 0xFFFFFFFFFFFFFFFF))

    manifest = {
        "N": N, "M": M, "NNZ": int(A.nnz), "HB": HB, "L": L,
        "NODE": NODE, "Z0": Z0, "ZP": ZP, "NP": NP,
        "NS4": NS4, "DIM4": DIM4, "NS3": NS3, "DIM3": DIM3,
        "TAIL_NN": TAIL_NN, "TAIL_Z": TAIL_Z,
        "segments": {
            "coo": 0, "c": 2 * int(A.nnz),
            "nb": 2 * int(A.nnz) + N,
            "zmask": 2 * int(A.nnz) + N + M,
            "band": 2 * int(A.nnz) + N + 2 * M,
            "g": 2 * int(A.nnz) + N + 2 * M + (HB + 1) * N,
        },
        "total_words": len(words),
    }
    with open(os.path.join(OUT, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"N={N} M={M} NNZ={A.nnz} HB={HB} L={L} image_words={len(words)}")
    print("v1 last (tau0) =", states[0][0][-1], " v2 last (tau1) =", states[1][0][-1])


if __name__ == "__main__":
    main()
