# 里程�?5 上板路径（PH1P35 片上小规模演示）

## 为什么不是完�?G-FOLD 问题

完整问题需�?`sram64_ctrl` �?2 MB 外部 SRAM �?CFI Flash；PH1P35 开发板没有这两者�?完整状�?RAM�? MB�? KKT RAM�?56 KB）也放不�?270 KB ERAM，必须接 DDR2�?为了先得到一�?*可上板、可对拍**的里程碑 5 结果，采用同一�?DRS 流水线的
**小规模实�?*，全部数据放片上 ERAM�?
| 参数 | 完整 G-FOLD | 上板小实�?|
|---|---|---|
| N | 1100 | 10 |
| NODE | 100 | 2 |
| M | 2107 | 49 |
| HB | 17 | 4 |
| L | 3208 | 60 |
| NNZ | 4783 | 147 |
| 状�?RAM | 1 MB (外部) | 1024×64 = 8 KB |
| KKT RAM | 256 KB | 256×64 = 2 KB |
| COO/band/LDL | 2 MB 外部 SRAM | 4096×64 = 32 KB ERAM |
| Boot | CFI Flash | ETH 下发 + image_ram |
| 黄金数据 | `rtl/data/kkt/full/v1,v2.hex` | `rtl/data/kkt/small2/v1,v2.hex` |

小实例的 cone 布局与真实问题完全一致：`Z0=14`，每�?node �?`NP=3` nonneg + `2×SOC(4)` + `1×SOC(3)`，只�?node `0..NODE-3` 之间插入
`ZP=7` �?zero 行（所�?`M = 14 + NODE*14 + (NODE-2)*7 + 7`）�?
## 数据生成

```powershell
python software\gen_small.py
```

输出 `rtl/data/kkt/small2/`：`v0/v1/v2`、`u1/u2`、`ut1/ut2`、`rsk1/rsk2`�?`Arow/Acol/Aval`、`b_r/c_r/zmask`、`band_r/g`、`r_y/Dy`、`coo_sram.hex`�?`flash_image.txt`（boot 镜像）、`flash_image.bin`（ETH LOAD 用，511 words = 4088 B）�?
小实例的 band 必须�?`S = rho_x I + A^T D_y A`（`D_y = 1/r_y`），
这与 `kkt_solve` �?Schur 消元一致；�?`r_y` 会得到错�?band（调试时踩过）�?
## RTL 结构

```
gfold_m5_top
  ├─ PLL: C0 = 125 MHz (ETH), C1 = 23.809524 MHz (solver)
  ├─ ETH RX/TX (�?M1-M4 相同) + rx_proto
  ├─ image_ram 1024×64  (�? RX �? �? clk_sol, boot �?
  ├─ solver_core (clk_sol)
  �?   ├─ drs_iter #(N=10,M=49,HB=4,L=60,NODE=2,BOOT_EXT=1)
  �?   �?   ├─ fp64_pool (3 mul + 3 add + 1 div)
  �?   �?   ├─ fp64_rsqrt (共享)
  �?   �?   ├─ kkt_solve �?spmv1/spmv2/banded_ldl_fp64_rb
  �?   �?   ├─ root_plus / proj_dual_cone_rb / s_build
  �?   �?   └─ sram64_ctrl (board �? 4096×64 ERAM)
  �?   ├─ boot_rom_ctrl (�?image_ram)
  �?   ├─ 状�?RAM 1024×64 / KKT RAM 256×64
  �?   └─ 运行控制: boot �?iter0 �?copy VS1 �?iter1 �?copy VS2
  └─ image_ram 256×64  (�? clk_sol, �? clk_125, DUMP �?
```

`drs_iter` 新增�?- `BOOT_EXT=1`：不实例�?CFI Flash 控制器，改用 `boot_req/boot_waddr/boot_rdata/boot_busy/boot_ready` 端口�?- `SRAM_AW` 参数：小实例�?12�?096 words），完整问题�?18�?- boot 段基址/长度�?`N/M/NNZ/HB/L` 自动推导，不再硬编码�?- `idle` 输出，供 `solver_core` 判断 boot 完成�?- cone 布局参数（`NODE/Z0/ZP/NP/NS4/DIM4/NS3/DIM3/TAIL_*`）可覆盖�?
## 上板流程

1. 烧录 `gfold_m5.bit`（用户手动）�?2. `python host\gfold_eth.py ping` 确认链路�?3. `python host\gfold_eth.py load rtl\data\kkt\small2\flash_image.bin`
   �?LOAD 3 �?+ FINISH，CRC 对拍�?4. `python host\gfold_eth.py start`�?x05）→ 释放 solver 复位，自�?boot + �?2 轮�?5. 等待�?0.5 s�?6. `python host\gfold_eth.py check` �?�?10 字一�?DUMP `vs_ram` �?120 �?   （VS1 60 + VS2 60），�?`small2/v1.hex`、`v2.hex` 对拍，阈�?5e-10�?
DUMP 仍沿�?M4 �?4-bit nwords / 10 字一帧，所�?check �?12 次读取�?
## 仿真验证（已通过�?
`rtl/sim/tb_solver_core.v`（image RAM �?solver_core �?vs_ram）：

```
VS1: worst rel = 3.07e-13
VS2: worst rel = 1.20e-10
```

即小实例在完整片上路径（BRAM sram64_ctrl + image boot + VS 复制）下
两轮 DRS 均满�?5e-10�?
## 资源（TD 综合，ph1_35p�?
```
#dsp   40 / 40   (100%)
#eram  29 / 108  (26.9%)
#lut   34562
#slice* 20574 / 21216 (96.97% at opt_gate; 3-mul/3-add version)
```

40 DSP ����������3-mul/3-add �汾�� slice �� 126.8% ���� 96.97%��
4 ���˷�����3 pool + 1 rsqrt������ 10 DSP����������� LUT��
（root_plus �?pp 挪到下一拍），让 4 个乘法器各拿 10 DSP，减�?LUT 溢出�?
## 待办

- [ ] TD `phy_m5` place & route 实际资源/时序
- [ ] 生成 bit，用户烧�?- [ ] 上板 `check` 对拍
- [ ] 完整问题 DDR2 路径（后续里程碑�?
