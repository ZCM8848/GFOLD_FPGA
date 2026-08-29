#!/usr/bin/env python3
"""Pack the sparse COO (Arow/Acol/Aval) into the external-SRAM word layout.

External SRAM is a 64-bit word space (sram64_ctrl). The COO is stored
sequentially, two words per entry:
    word[2k]   = { Acol[k][15:0], Arow[k][15:0] }   (Arow in bits [15:0], Acol in [31:16])
    word[2k+1] = Aval[k]
COO_BASE = 0, so COO occupies SRAM word addresses [0, 2*NNZ).

This mirrors the banded_ldl_fp64_rb / sram64_ctrl design: the sparse-matrix
coefficients live in external SRAM (loaded by the host at boot), freeing the
~262 KB of on-chip M9K that 5 duplicated internal COO ROMs would otherwise burn.

Outputs: rtl/data/kkt/{full,small}/coo_sram.hex
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
KKT = os.path.join(HERE, "..", "rtl", "data", "kkt")


def pack(case):
    d = os.path.join(KKT, case)
    with open(os.path.join(d, "Arow.hex")) as f:
        arow = [int(x, 16) for x in f.read().split()]
    with open(os.path.join(d, "Acol.hex")) as f:
        acol = [int(x, 16) for x in f.read().split()]
    with open(os.path.join(d, "Aval.hex")) as f:
        aval = [int(x, 16) for x in f.read().split()]
    nnz = len(arow)
    assert len(acol) == nnz and len(aval) == nnz, f"{case}: length mismatch"
    words = []
    for k in range(nnz):
        words.append((acol[k] << 16) | arow[k])   # {Acol, Arow}
        words.append(aval[k])
    with open(os.path.join(d, "coo_sram.hex"), "w") as f:
        for w in words:
            f.write("%016X\n" % w)
    print(f"[{case}] nnz={nnz} -> coo_sram.hex {len(words)} words "
          f"(SRAM addr [0, {2*nnz}))")


if __name__ == "__main__":
    pack("full")
    pack("small")
    print("done")
