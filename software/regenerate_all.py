#!/usr/bin/env python3
"""One-shot regeneration of ALL solver data after editing problem/params.

Run from software/ (or anywhere — paths resolve relative to this file):

    python regenerate_all.py [tof]

Pipeline (in order):
    1. regenerate_problem.py   assemble(tof) -> problem.json (A/b/q/cones)
    2. gen_kkt_test.py         Arow/Acol/Aval + r_y/Dy + zx/zy  (KKS tests)
    3. gen_reordered_data.py   band_r/Dy_r/zmask/c_r/nb_r       (DRS frame)
    4. drs_reference.py gen    b_r/c_r/g.hex/diag_r/v0 + iters  (golden)
    5. gen_coo_sram.py         coo_sram.hex                     (external SRAM COO)
    6. gen_flash_image.py      flash_image.{txt,hex,bin} + manifest

Then validates every output length and the flash image size against the RTL
localparams (N/M/NNZ/HB). After this, re-burn the CFI Flash with the new
`flash_image.bin`; no recompile is needed unless N/M/NNZ/HB changed.

Requires numpy + scipy (same env as the other gen_*.py scripts).

NOTE: gen_kkt_test.py hard-codes tof=46.6093 internally. If you change tof,
update it there too (and regenerate_problem.py's default).
"""
import os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
KKS = os.path.join(HERE, "..", "rtl", "data", "kkt", "full")
FLASH = os.path.join(HERE, "..", "rtl", "data", "flash")

N, M, HB, NNZ = 1100, 2107, 17, 4783
LM1 = N + M                       # 3207


def run(args):
    print("==> " + " ".join(args), flush=True)
    r = subprocess.run([sys.executable] + args, cwd=HERE)
    if r.returncode != 0:
        sys.exit("FAILED: " + " ".join(args))


def wc(path):
    with open(path) as f:
        return len(f.read().split())


def main():
    tof = sys.argv[1] if len(sys.argv) > 1 else "46.6093"

    run(["regenerate_problem.py", tof])
    run(["gen_kkt_test.py"])
    run(["gen_reordered_data.py"])
    run(["drs_reference.py", "gen", "full"])
    run(["gen_coo_sram.py"])
    run(["gen_flash_image.py"])

    print("==> validate", flush=True)
    checks = [
        ("Arow.hex", NNZ), ("Acol.hex", NNZ), ("Aval.hex", NNZ),
        ("coo_sram.hex", 2 * NNZ),
        ("c_r.hex", N), ("nb_r.hex", M), ("zmask.hex", M),
        ("band_r.hex", (HB + 1) * N), ("g.hex", LM1),
    ]
    for name, expect in checks:
        got = wc(os.path.join(KKS, name))
        assert got == expect, f"{name}: got {got} expect {expect}"
        print(f"  {name}: {got} ok")

    flash_bin = os.path.join(FLASH, "flash_image.bin")
    words = 2 * NNZ + N + M + M + (HB + 1) * N + LM1
    size = os.path.getsize(flash_bin)
    assert size == words * 8, f"flash_image.bin size {size} != {words * 8}"
    print(f"  flash_image.bin: {size} bytes ({words} words) ok")

    print(f"\nDONE: regenerated all solver data (tof={tof}). "
          f"Re-burn CFI Flash with {os.path.relpath(flash_bin, os.path.join(HERE, '..'))}.")


if __name__ == "__main__":
    main()
