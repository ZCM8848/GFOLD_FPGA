# 里程碑 5 验证记录（pool 版 RTL）

## 结论

PH1P35 的全局共享运算池版本（`fp64_pool` 4×mul + 5×add + 1×div，
外加 1 个共享 `fp64_rsqrt`）在 `iverilog` 中完整运行了 **2 轮 DRS 迭代**，
`check_drs.py` 判定 **PASS**：

```
V1: nan=0 bad(>1e-07)=0/3208 maxrel=5.16e-10
V2: nan=0 bad(>1e-06)=0/3208 maxrel=5.72e-10
PASS
```

这两个误差值与 master（原始 164-DSP 版本）RTL 仿真的结果 **完全一致**
（master: V1 5.16e-10 / V2 5.72e-10），说明 pool 化没有改变数值路径。

## 复现

```powershell
cd rtl\sim
powershell -ExecutionPolicy Bypass -File .\run_pool_sim.ps1
```

或手动：

```powershell
$env:PATH = "D:\iverilog\bin;" + $env:PATH
cd rtl\sim
iverilog -g2005 -o pool_sim.vvp `
  ..\solver\fp64.v ..\solver\fp64_cmp.v ..\solver\fp64_log.v ..\solver\fp64_rsqrt.v `
  ..\solver\fp64_pool.v ..\solver\drs_iter.v ..\solver\kkt_solve.v `
  ..\solver\banded_ldl_fp64_rb.v ..\solver\spmv_fp64.v ..\solver\root_plus_pool.v `
  ..\solver\proj_soc_pool.v ..\solver\proj_dual_cone_rb.v ..\solver\s_build.v `
  ..\sram64_ctrl.v ..\flash_ctrl.v tb_drs_iter_2.v
vvp pool_sim.vvp > pool_sim_output.txt
# 提取 VS1/VS2 -> V1/V2，从项目根运行：
python software\check_drs.py rtl\data\pool_sim_dump.txt
```

仿真日志关键行：

```
BOOT coo: bad=0/9566
BOOT Dy: bad=0/2107
BOOT c: bad=0/1100
BOOT nb: bad=0/2107
BOOT flash words read: 37887 (expect 37887)
BOOT g: bad=0/3207
BOOT zmask_bits: bad=0/2107
BOOT diag_r: bad=0/3208
ITER 0 done ...
VS1: 408517c373dfb361 c07eefa0c0faa820 40abfcfcbe67aaef ...
ITER 1 done ...
VS2: 3fef9ebff1b43f12 bfe7302d6b310222 4015450efa85fbb0 ...
ALL DONE
```

## 本次调试中发现并修复的 pool 化 bug

pool 化不是简单的端口替换；以下 5 个问题都会导致结果错误（部分为 NaN/X）：

1. **`kkt_solve` own 算术操作数选早了两拍**。
   原设计是组合乘法 + 在后续状态锁存；pool 版在 `S_RHS0/S_RHS1`
   就选操作数，但真正锁存发生在 `S_RHS3/S_RHS3b`，导致 `po_rhs`、
   `so_rhs`、`zy` 全部用了过期值。改为在锁存状态驱动操作数。

2. **`proj_soc_pool` 组合结果晚锁一拍**。
   `S_SQ1` 驱动乘法、`S_SQ1W` 才锁存，但 `S_SQ1W` 时操作数已归零；
   `S_OUTA/S_OUTB` 同样问题。改为在驱动状态同拍锁存。

3. **`proj_dual_cone_rb` 未把内部 `proj_soc_pool` 的乘法端口转发出去**，
   全局 pool 看到未驱动（X）的操作数。补上 `mul0..2` 的 `assign`。

4. **`drs_iter` 的 normalize 上升沿检测被共享 rsqrt 的残留 `done` 误触发**。
   `rs_done_p` 原来只在 `S_NMRW` 内更新，进入 normalize 时共享 rsqrt 的
   `done` 仍是上一阶段（cone）留下的 1，于是第一拍就用了过期的 `rs_o`。
   改为每拍跟踪 `rs_done_p`。

5. **pool mux 用状态区间判断，漏掉了共享等待状态**。
   流式阶段大部分时间在 `S_W29..S_W33` 等等待状态，而
   `rp_active` 只覆盖 `S_RP0..S_RPW`，导致 `root_plus` 在这些周期
   被 pool 断开。改为 `start..done` 的 sticky busy 标志
   （`ks_busy/rp_busy/pdc_busy/sb_busy`）。

另外修复了 `root_plus_pool` 的两个流式协议问题：
- `din_valid` 在 `S_ACC_W` 采样后，原实现下一拍才用 `r_in/g_in/...`，
  此时上游已换元素；改为在 `din_valid` 同拍锁存第一级乘积与输入。
- `b <= 0` 分支直接进 `S_DIV`，但 pool 版只在该分支外的 `S_SQWc`
  设置了除法输入；补上 `S_SQW` 分支的 `div_a/div_b/div_start`。

## 单元回归

`rtl/sim/tb_proj_soc_pool.v`：pool 版 `proj_soc` 与 master 的 5 个用例
逐位一致。

`rtl/sim/tb_pdc_cmp.v` / `tb_rp_cmp.v`：在 master 仓库存在时，
可用相同激励对比 pool 版与 master 版 `proj_dual_cone_rb` / `root_plus`；
本次调试中两者均为 0 mismatch（2107 个字 / tau 逐位一致）。

## 尚未完成（上板）

本记录验证的是 **RTL 数值通路**。要得到可上板的 bit 还需要：

- 存储适配器：`sram64_ctrl`（2 MB 外部 SRAM）与 `flash_ctrl`（CFI Flash）
  在 PH1P35 开发板上不存在，需改为 DDR2 / ETH 下发 + 片上 BRAM；
- `solver_core` 顶层：solver 30 MHz 时钟域与 ETH 125 MHz 域的 CDC，
  START/DUMP 命令；
- TD 综合确认 40 DSP / 38k LUT4 预算；
- 上板烧录（用户手动）与 `gfold_eth.py` 读取 VS1/VS2。
