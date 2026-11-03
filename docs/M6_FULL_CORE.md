# M6 完整问题核心（DDR2 + 全规模 DRS）

日期：2026-11-04 | 分支：`anlogic-ph1p35`

## 当前进展

完整问题 `N=1100 / M=2107 / HB=17 / L=3208` 的求解器数据通路已经能在
PH1P35 上综合：

| 资源 | 占用 | 余量 |
|---|---:|---:|
| Slice | **17,476 / 21,216 (82.4%)** | 3,740 |
| LUT | 24,987 | — |
| ERAM | **104 / 108 (96.3%)** | 4 |
| DSP | **40 / 40 (100%)** | 0 |
| DDR byte | 6 / 12 | 6 |
| PLL | 2 / 6 | 4 |

顶层：`rtl/top/gfold_m6_core_top.v`，工程：
`anlogic/td_project/syn_m6_core/`、`phy_m6_core/`。

## 关键存储决策

### 1. state + BAND + KKT 合并为一个双端口 ERAM

TD 对多个大 RAM 的推断不可靠：单独放 `smem`/`kmem` 时只有一个能进
ERAM，其余会掉到 LUTRAM，导致 slice 超过 29k。最终改成：

```verilog
(* ram_style = "bram" *) reg [63:0] mem [0:32767];
// 端口 A: V/UT/U/RSK/G/DR + BAND，地址 0..23,460
// 端口 B: KKT RAM，地址 24,576..32,767
```

一个 32K×64 的双端口 ERAM 正好用 104 块，两个端口可同时给
`drs_iter` 读状态和 `kkt_solve` 写 KKT，解决了单端口冲突与 TD 单大
RAM 推断限制。

### 2. CB/DY 工作区在 DDR2

`drs_iter` 的 `EXT_WS=1`：

- 片上保留 V/UT/U/RSK/G/DR + BAND；
- CB（c/-b）和 DY（D_y）搬到外部 DDR2，逻辑地址从 `WS_BASE` 开始；
- 访问 CB/DY 时冻结主 FSM，等 DDR2 事务完成；这些访问只发生在 boot
  和每 25 轮的 scale 更新，不在主迭代热路径。

### 3. r_y/D_y 系数在 DDR2

`kkt_solve` 的 `EXT_COEF=1`：

- 去掉 `r_y_arr`/`Dy_arr` 两个 2107×64 的片上数组；
- scale 更新时把 r_y 副本写入外部工作区，D_y 本来就在外部；
- zy 阶段逐行从外部 SRAM 读 r_y/D_y，再算 `Dy*(az + r_y*vy)`；
- 全 2 轮仿真 `EXT_WS=1 / EXT_COEF=1` 仍然 bit-exact：
  `V1 5.16e-10`、`V2 5.72e-10`。

### 4. KKT az 区复用

`A^T vy` 在 RHS 流之后已死，`az/zy` 直接复用它，KKT RAM 从
`4*LMAX=8,428` 降到 `3*LMAX=6,321` 字（合并到上面的 32K ERAM）。

### 5. DDR2 适配器

`rtl/board/ddr2_sram_ctrl.v`：

- 客户端是 `sram64_ctrl` 的 `req/we/waddr/wdata/busy/rdata` 协议；
- DDR2 用户时钟 `dhi_clk` 与 solver clock 异步，内部用 toggle 握手
  做 CDC；
- 写事务等 AW/W 握手后加 32 拍 drain，读事务等 `rvalid`；
- 地址 18 位直接映射到 DDR2 低 2 MB 字地址。

`drs_iter` 新增 `MEM_IF=1`，把内部 SRAM 仲裁器直接暴露给 DDR2 适配器，
不再例化 `sram64_ctrl` 物理引脚。

## P&R 结果

`phy_m6_core` place 阶段：

| 项 | 值 |
|---|---:|
| Slice | **16,488 / 21,216 (77.7%)** |
| LUT6 | 16,469 / 21,216 |
| ERAM | 104 / 108 |
| DSP | 40 / 40 |
| Setup WNS | **+3.763 ns** |
| FMAX | 88.99 MHz |

route 后：

| 项 | 值 |
|---|---:|
| Setup WNS | **+1.216 ns** |
| Hold WNS | **+0.020 ns** |
| Slice | 16,488 |
| ERAM | 104 |
| DSP | 40 |

bitgen 尚未成功：TD 报 `BIT-8123 unconstrained io`。首次 route DB 里
`start` 被放到 H2，而不是 ADC 指定的 A4；已把 `pin_ddr2.adc` 改为
KEY1（D5），下一次需要从 `opt_place` 重新跑一遍 place/route/bitgen，
让新约束真正进入 DB。当前 route DB 的时序结果仍然有效。

## 尚未完成

1. **启动镜像通道**：当前 `gfold_m6_core_top` 的 boot 端口是一个返回 0
   的测试桩；需要把 `flash_image.bin`（303,096 B / 37,887 字）通过
   ETH 或 SPI Flash 写入 DDR2 地址 0，再让 boot FSM 读。
2. **ETH/MCU 控制面**：LOAD/ACK/FINISH/START/STATUS/DUMP。
3. **50k 迭代与 final_mass 验收**：顶层还需要 `iter_cnt`（done 递增、
   到 50,000 停）和结果回传。
4. 用 D5 约束重跑 place/route/bitgen，并做上板 DDR2 适配器冒烟。

## 复现

```powershell
python software\make_m6_core_project.py
cd anlogic\td_project\syn_m6_core
td_commands_prompt.exe .../DefaultFlow.tcl
cd ..\phy_m6_core
td_commands_prompt.exe .../DefaultFlow.tcl
```

## 说明

- `gfold_m6_core_top` 目前用 50 MHz/2 = 25 MHz solver 时钟；
- `start` 为高时 boot FSM 从测试桩读 0，随后进入零数据迭代，仅用于
  综合/布局和后续镜像通道接入；
- 完整问题数值正确性已由 `rtl/sim/tb_drs_iter_2.v`（EXT_WS=1 /
  EXT_COEF=1）验证。
