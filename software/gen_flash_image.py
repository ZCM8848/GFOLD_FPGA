#!/usr/bin/env python3
"""Pack the solver input into a contiguous Flash image for the DE2-115 8Mx8
CFI Flash, and emit a manifest of segment base addresses.

Flash layout (64-bit word addresses; byte address = word<<3):
    [0, 9566)                    COO  (2 words/nnz: {Acol,Arow}, Aval)
    [9566, 10666)                c    (cr, N=1100)
    [10666, 12773)               nb   (-br, M=2107)
    [12773, 14880)               zmask (M=2107, 1.0 = zero cone)
    [14880, 34680)               band (S band, (HB+1)*N = 19800)
    [34680, 37887)               g    (KKT^-1[c;-b], LM1 = 3207)

band and g are burned in as round-nearest references: computing them at boot
with the truncating FP64 datapath introduces a few-ULP error that the
ill-conditioned LDL factorization (S cond >> 1e6) then amplifies into a
sign flip in root_plus' discriminant, making iter1 diverge. They are precomputed
to round-nearest in software (band_r.hex / g.hex) and baked in verbatim.

Outputs:
    rtl/data/flash/flash_image.txt   64-bit words (16 hex), for tb $readmemh
    rtl/data/flash/flash_image.hex   bytes (2 hex), for Quartus .jic conversion
    rtl/data/flash/flash_manifest.json  segment base/len (word + byte)
"""
import os, json

HERE = os.path.dirname(os.path.abspath(__file__))
KKT = os.path.join(HERE, "..", "rtl", "data", "kkt", "full")
OUT = os.path.join(HERE, "..", "rtl", "data", "flash")
os.makedirs(OUT, exist_ok=True)

N, M, HB = 1100, 2107, 17


def rd(fn):
    return [int(x, 16) for x in open(os.path.join(KKT, fn)).read().split()]


def main():
    coo  = rd("coo_sram.hex")      # 2*NNZ = 9566
    c    = rd("c_r.hex")           # N = 1100
    nb   = rd("nb_r.hex")          # M = 2107
    zm   = rd("zmask.hex")         # M = 2107
    band = rd("band_r.hex")        # (HB+1)*N = 19800
    g    = rd("g.hex")             # LM1 = 3207

    assert len(coo) == 9566
    assert len(c) == N and len(nb) == M and len(zm) == M
    assert len(band) == (HB + 1) * N and len(g) == N + M

    words = coo + c + nb + zm + band + g
    total = len(words)

    COO_BASE_W  = 0
    C_BASE_W    = len(coo)
    NB_BASE_W   = C_BASE_W + len(c)
    ZM_BASE_W   = NB_BASE_W + len(nb)
    BAND_BASE_W = ZM_BASE_W + len(zm)
    G_BASE_W    = BAND_BASE_W + len(band)

    with open(os.path.join(OUT, "flash_image.txt"), "w") as f:
        for w in words:
            f.write("%016X\n" % w)

    with open(os.path.join(OUT, "flash_image.hex"), "w") as f:
        for w in words:
            for b in range(8):
                f.write("%02X\n" % ((w >> (8 * b)) & 0xFF))  # little-endian bytes

    manifest = {
        "total_words": total,
        "total_bytes": total * 8,
        "segments": {
            "coo":   {"word_base": COO_BASE_W,  "byte_base": COO_BASE_W * 8,  "len": len(coo)},
            "c":     {"word_base": C_BASE_W,    "byte_base": C_BASE_W * 8,    "len": len(c)},
            "nb":    {"word_base": NB_BASE_W,   "byte_base": NB_BASE_W * 8,   "len": len(nb)},
            "zmask": {"word_base": ZM_BASE_W,   "byte_base": ZM_BASE_W * 8,   "len": len(zm)},
            "band":  {"word_base": BAND_BASE_W, "byte_base": BAND_BASE_W * 8, "len": len(band)},
            "g":     {"word_base": G_BASE_W,    "byte_base": G_BASE_W * 8,    "len": len(g)},
        },
    }
    with open(os.path.join(OUT, "flash_manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    for nm, base, ln in [("COO", COO_BASE_W, len(coo)), ("c", C_BASE_W, len(c)),
                         ("nb", NB_BASE_W, len(nb)), ("zmask", ZM_BASE_W, len(zm)),
                         ("band", BAND_BASE_W, len(band)), ("g", G_BASE_W, len(g))]:
        print(f"{nm:6s} words {ln:6d}  @ 0x{base:05X}")
    print(f"total {total} words = {total*8} bytes -> {OUT}")


if __name__ == "__main__":
    main()

