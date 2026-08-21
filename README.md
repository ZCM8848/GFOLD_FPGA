# GFOLD_FPGA

Fuel-optimal powered-descent guidance (G-FOLD) implemented as a hardware
accelerator on an **Intel Cyclone IV FPGA (EP4CE115F29C7)**.

G-FOLD computes the fuel-optimal trajectory for a soft-landing spacecraft by
solving a **second-order cone program (SOCP)**. This project explores whether
that convex solver can be implemented in pure RTL (Verilog) — a genuine
hardware acceleration of an onboard trajectory-optimization workload.

## Current status

| Phase | Status |
|-------|--------|
| **1. Software golden reference** | ✅ Done — algorithm + hardware structure validated |
| 2. Fixed-point numeric format | ⏳ Next |
| 3. RTL architecture | Pending |
| 4. Verilog implementation | Pending |
| 5. Testbench vs golden reference | Pending |
| 6. Quartus synthesis / on-board | Pending |

## What was validated (Phase 1)

- The G-FOLD SOCP (1100 vars / 2107 constraints / 4783 nonzeros) is solvable by
  a **first-order method** (SCS homogeneous self-dual embedding + Type-I
  Anderson acceleration + adaptive scaling). Reference: Clarabel IPM solves the
  gfold default example (tof=46.6093 s) to `final_mass = 1799.156 kg` in 23
  iterations / ~35 ms; `scs` reproduces it to `rel_x ≈ 1.6e-3`.
- **Hardware-enabling structure**: re-ordering variables and constraints to
  node-major order drops `A` to bandwidth 17; block-eliminating the KKT gives
  `AᵀA` (1100×1100, banded), which fits **entirely on-chip** (~0.15 MB fp32,
  no external SDRAM). Adaptive scaling and adaptive time-of-flight share the
  same banded re-factorizer.
- **Adaptive TOF** (fuel-optimal flight-time search) is viable: infeasible
  (too-short) flights are detected fast; warm-starting feasible candidates
  gives ~3.4×; realistic budget ≈ **~3 s @100 MHz** with a per-solve iteration
  cap + infeasibility detection.
- Key verified pitfalls: bare ADMM doesn't converge; element-wise equilibration
  breaks SOC cones; a simplified uniform-s adaptive scaling only reaches
  `rel_x ~3e-2` (SCS's cone-coupled `R_y` is required for `1e-3`).

## Architecture (Verilog, program-style)

Follows a C-like program decomposition:

```
top: time-of-flight golden-search controller (FSM)   <-- tof is a parameter
solve(tof, config):                                  <-- subroutine
    A,b,q = assemble(tof, ...)                       (hardware assembler)
    M = I + ρ·AᵀA                                    (banded)
    L = banded_cholesky(M)                           (re-factor per tof / scale)
    return scs_drs_iterate(L, A, b, q, warm_state)   (DRS + Anderson + cone proj)
```

- **Input**: config / initial state via registers + BRAM (host over UART/SPI).
- **Fixed problem data**: baked as ROM (Quartus `.mif`).
- **Output**: trajectory (positions / velocities / thrust / mass) in BRAM.

## Repository layout

```
software/    Python golden reference + validation scripts
             (problem.json = gfold default example; scs/Clarabel dual oracle;
              assembler.py, warmstart_scs.py, tof_feasibility.py, ...)
Quartus/     Quartus Prime 18.1 project (EP4CE115F29C7)
reference/   (gitignored) SCS, A2R-Lab/ADMM_FPGA, EiCOS source as algorithm blueprints
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
