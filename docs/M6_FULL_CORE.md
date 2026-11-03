# M6 完整问题核心（DDR2 + 全规�?DRS�?
日期�?026-11-04 | 分支：`anlogic-ph1p35`

## 当前进展

完整问题 `N=1100 / M=2107 / HB=17 / L=3208` 的求解器数据通路已经能在
PH1P35 上综合：

| 资源 | 占用 | 余量 |
|---|---:|---:|
| Slice | **17,476 / 21,216 (82.4%)** | 3,740 |
| LUT | 24,987 | �?|
| ERAM | **104 / 108 (96.3%)** | 4 |
| DSP | **40 / 40 (100%)** | 0 |
| DDR byte | 6 / 12 | 6 |
| PLL | 2 / 6 | 4 |

顶层：`rtl/top/gfold_m6_core_top.v`，工程：
`anlogic/td_project/syn_m6_core/`、`phy_m6_core/`�?
## 关键存储决策

### 1. state + BAND + KKT 合并为一个双端口 ERAM

TD 对多个大 RAM 的推断不可靠：单独放 `smem`/`kmem` 时只有一个能�?ERAM�?其余会掉�?LUTRAM，导�?slice 超过 29k。最终改成：

```verilog
(* ram_style = "bram" *) reg [63:0] mem [0:32767];
// 端口 A: V/UT/U/RSK/G/DR + BAND，地址 0..23,460
// 端口 B: KKT RAM，地址 24,576..32,767
```

一�?32K×64 的双端口 ERAM 正好�?104 块，两个端口可同时给
`drs_iter` 读状态和 `kkt_solve` �?KKT，解决了单端口冲突与 TD 单大
RAM 推断限制�?
### 2. CB/DY 工作区在 DDR2

`drs_iter` �?`EXT_WS=1`�?
- 片上保留 V/UT/U/RSK/G/DR + BAND�?- CB（c/-b）和 DY（D_y）搬到外�?DDR2，逻辑地址�?`WS_BASE` 开始；
- 访问 CB/DY 时冻结主 FSM，等 DDR2 事务完成；这些访问只发生�?boot
  和每 25 轮的 scale 更新，不在主迭代热路径�?
### 3. r_y/D_y 系数�?DDR2

`kkt_solve` �?`EXT_COEF=1`�?
- 去掉 `r_y_arr`/`Dy_arr` 两个 2107×64 的片上数组；
- scale 更新时把 r_y 副本写入外部工作区，D_y 本来就在外部�?- zy 阶段逐行从外�?SRAM �?r_y/D_y，再�?`Dy*(az + r_y*vy)`�?- �?2 轮仿�?`EXT_WS=1 / EXT_COEF=1` 仍然 bit-exact�?  `V1 5.16e-10`、`V2 5.72e-10`�?
### 4. KKT az 区复�?
`A^T vy` �?RHS 流之后已死，`az/zy` 直接复用它，KKT RAM �?`4*LMAX=8,428` 降到 `3*LMAX=6,321` 字（合并到上面的 32K ERAM）�?
### 5. DDR2 适配�?
`rtl/board/ddr2_sram_ctrl.v`�?
- 客户端是 `sram64_ctrl` �?`req/we/waddr/wdata/busy/rdata` 协议�?- DDR2 用户时钟 `dhi_clk` �?solver clock 异步，内部用 toggle 握手
  �?CDC�?- 写事务等 AW/W 握手后加 32 �?drain，读事务�?`rvalid`�?- 地址 18 位直接映射到 DDR2 �?2 MB 字地址�?
`drs_iter` 新增 `MEM_IF=1`，把内部 SRAM 仲裁器直接暴露给 DDR2 适配器，
不再例化 `sram64_ctrl` 物理引脚�?
## P&R 放置结果

`phy_m6_core` �?place 阶段�?
| �?| �?|
|---|---:|
| Slice | **16,488 / 21,216 (77.7%)** |
| LUT6 | 16,469 / 21,216 (77.6%) |
| ERAM | 104 / 108 |
| DSP | 40 / 40 |
| Setup WNS | **+3.763 ns** (place) |
| FMAX | 88.99 MHz |

route 后：

| �?| �?|
|---|---:|
| Setup WNS | **+1.216 ns** |
| Hold WNS | **+0.020 ns** |
| Slice | 16,488 |
| ERAM | 104 |
| DSP | 40 |

首次 route �?bitgen �?`start` 未约束被 TD 拒绝；`pin_ddr2.adc` 已把
`start` 指到 SW1（A4），正在重新�?place/route/bitgen�?
## 尚未完成

1. **启动镜像通道**：当�?`gfold_m6_core_top` �?boot 端口是一个返�?0
   的测试桩；需要把 `flash_image.bin`�?03,096 B / 37,887 字）通过
   ETH �?SPI Flash 写入 DDR2 地址 0，再�?boot FSM 读�?2. **ETH/MCU 控制�?*：LOAD/ACK/FINISH/START/STATUS/DUMP�?3. **50k 迭代�?final_mass 验收**�?4. route/bitgen 与上�?DDR2 适配器测试�?
## 复现

```powershell
python software\make_m6_core_project.py
cd anlogic\td_project\syn_m6_core
td_commands_prompt.exe .../DefaultFlow.tcl
cd ..\phy_m6_core
td_commands_prompt.exe .../DefaultFlow.tcl
```

## 说明

- `gfold_m6_core_top` 目前�?50 MHz/2 = 25 MHz solver 时钟�?- `start` 为高�?boot FSM 从测试桩�?0，随后进入零数据迭代，仅用于
  综合/布局和后续镜像通道接入�?- 完整问题数值正确性已�?`rtl/sim/tb_drs_iter_2.v`（EXT_WS=1 /
  EXT_COEF=1）验证�?
