# GFOLD_FPGA

Fuel-optimal powered-descent guidance (G-FOLD) implemented as a hardware
accelerator on an **Intel Cyclone IV FPGA (EP4CE115F29C7, DE2-115)**.

G-FOLD computes the fuel-optimal trajectory for a soft-landing spacecraft by
solving a **second-order cone program (SOCP)**. The convex solver is implemented
in pure RTL (Verilog) and runs on the DE2-115 board — a genuine hardware
acceleration of an onboard trajectory-optimization workload.

## Status

The full flow is implemented and running on the DE2-115:

| Stage | Status |
|-------|--------|
| Software golden reference (SCS / Clarabel) | ✅ Done |
| RTL implementation (DRS + banded LDL + cone projection, FP64) | ✅ Done |
| Testbench vs golden reference | ✅ iter0/iter1 bit-exact (rel ≈ 5e-10) |
| Quartus synthesis / on-board | ✅ Running @30 MHz |

- Problem: **1100 vars / 2107 constraints / 4783 nonzeros**, node-major
  reordering → bandwidth 17.
- Solver: **SCS homogeneous self-dual embedding + Douglas-Rachford splitting +
  adaptive scaling** (no Anderson acceleration on the board).
- Numerics: **truncating FP64** (no round-to-nearest). `final_mass ≈ 1795 kg`
  after ~50k iterations (reference optimum 1799.156 kg, −0.22%).

## Hardware

| Resource | Usage |
|----------|-------|
| Clock | `CLOCK_50` → PLL **30 MHz** (fp64_add is ~29 ns combinational) |
| Logic | ~50k LE / 114,480 |
| On-chip RAM | ~2.9 Mbit / 3.98 Mbit (M9K) |
| DSP | 164 × 18×18 (fp64_mul) |
| External SRAM | 2 MB IS61WV25616BLL (COO + LDL band) |
| External Flash | 8 MB CFI S29GL064N (solver input, burned at boot) |

## Data flow

1. Solver input (`COO/c/nb/zmask/band/g`, 37887 × 64-bit words) is burned into
   the **CFI Flash** (`gen_flash_image.py` → `flash_image.bin`).
2. On power-up, `drs_iter`'s boot FSM loads Flash → SRAM / on-chip RAM, then
   computes `diag_r/D_y` and hard-codes `v0`.
3. Each DRS iteration: normalize → KKT solve (banded LDL) → root_plus →
   `u` update → cone projection → adaptive-scale residual.

## On-board controls / display (AGC/DSKY style)

- `KEY0` reset, `KEY1` start.
- `KEY2`/`KEY3`: page up/down (NOUN).
- `SW[15:0]`: iteration limit (binary; all-zero = never runs). Reaching the
  limit shows `VERB 09` and lights the SOLVED LED.
- `HEX7..6` = **VERB** (phase), `HEX5..4` = **NOUN** (page), `HEX3..0` = iteration.
- LCD 16×2: line1 `ITER xxxxx`, line2 page-selectable `MASS / SCALE / TAU / ITER`.
- LEDs: COMP ACTY / PROG / RST / DONE / PLL / SOLVED + phase lamps.

## Programming

1. Write solver input to CFI Flash (DE2-115 Control Panel, Sequential Write of
   `rtl/data/flash/flash_image.bin`).
2. Program the FPGA: JTAG `Quartus/output_files/GFOLD_FPGA.sof`.

## Repository layout

```
software/    Python golden reference + validation scripts
             (gen_flash_image.py, gen_coo_sram.py, check_*.py, ...)
rtl/         Verilog: drs_iter + KKT/LDL + cone/root + FP64 units
             + GFOLD_FPGA top-level + DSKY display (gf_display/lcd_driver)
rtl/data/    generated solver data (kkt/full/*, flash/*)
Quartus/     Quartus Prime 18.1 project (EP4CE115F29C7)
board/       DE2-115 pin map + schematic reference
reference/   (gitignored) SCS, A2R-Lab/ADMM_FPGA, EiCOS source
```

## References

- Blackmore et al., *Minimum-Landing-Error Powered-Descent Guidance for Mars
  Landing Using Convex Optimization*, JGCD 2010.
- Acikmese & Ploen, *Convex Programming Approach to Powered Descent Guidance
  for Mars Landing*, JGCD 2007.
- O'Donoghue et al., *Conic Optimization via Operator Splitting and Homogeneous
  Self-Dual Embedding* (SCS).
- `samutoljamo/g-fold` — the Rust SOCP solver this is based on.
- `A2R-Lab/ADMM_FPGA` — proven banded-LDL ADMM hardware template.

## License

MIT
