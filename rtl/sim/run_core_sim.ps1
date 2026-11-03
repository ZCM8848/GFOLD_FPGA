# run_core_sim.ps1 — verify the on-chip small-problem solver_core end-to-end
# (image RAM -> boot ROM -> drs_iter -> vs_ram), 2 DRS iterations.
$ErrorActionPreference = "Stop"
$env:PATH = "D:\iverilog\bin;" + $env:PATH
$wt = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$s  = Join-Path $wt "rtl\solver"
Set-Location (Join-Path $wt "rtl\sim")

& iverilog -g2005 -o core_sim.vvp `
  "$s\fp64.v" "$s\fp64_cmp.v" "$s\fp64_log.v" "$s\fp64_rsqrt.v" "$s\fp64_pool.v" `
  "$s\drs_iter.v" "$s\kkt_solve.v" "$s\banded_ldl_fp64_rb.v" "$s\spmv_fp64.v" `
  "$s\root_plus_pool.v" "$s\proj_soc_pool.v" "$s\proj_dual_cone_rb.v" "$s\s_build.v" `
  "$s\solver_core.v" "$wt\rtl\board\sram64_ctrl.v" "$wt\rtl\board\boot_rom_ctrl.v" `
  "tb_solver_core.v"
if ($LASTEXITCODE -ne 0) { throw "iverilog failed" }

& vvp core_sim.vvp 2>&1 | Out-File -FilePath core_sim_output.txt -Encoding ascii
Set-Location $wt
& python (Join-Path $wt "software\check_small.py") (Join-Path $wt "rtl\sim\core_sim_output.txt")
