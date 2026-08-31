# GFOLD_FPGA 技术手册

> 面向开发者：G-FOLD 求解器的算法、架构、数值、时序与实现细节。

## 1. 算法背景

G-FOLD（Fuel-optimal powered-descent guidance）求解软着陆飞行器的燃料最优轨迹，
其核心是一个 **二阶锥规划（SOCP）**：

```
minimize  -m              (最大化末质量)
s.t.      动力学约束（离散化）
          推力幅值约束（二阶锥）
          非负质量约束
```

本设计采用 **SCS**（Operator Splitting Conic Solver）的 **齐次自对偶嵌入**
（homogeneous self-dual embedding）形式，用 **Douglas-Rachford 分裂（DRS）** 迭代求解，
配合 **自适应缩放（adaptive scaling）** 加速收敛。

- 参考最优：`final_mass = 1799.156 kg`（Clarabel IPM，tof=46.6093 s）
- 本硬件收敛：`final_mass ≈ 1795 kg`（约 5 万轮，−0.22%）

## 2. 问题规模与数据

| 参数 | 值 |
|------|-----|
| 变量数 N | 1100 |
| 约束数 M | 2107 |
| 非零元 NNZ | 4783 |
| 带宽 HB | 17（节点主序重排后） |
| 嵌入维 L | N + M + 1 = 3208 |

节点主序（node-major）重排把矩阵 `A` 压缩为带宽 17；块消去 KKT 得到带状 `AᵀA`
（1100×1100，banded），可完全片上存储。

## 3. 系统架构

```
GFOLD_FPGA (top)
├── pll30                          CLOCK_50 → 30 MHz
├── drs_iter                       DRS 单次迭代（核心）
│   ├── flash_ctrl                 CFI Flash 只读控制器（boot）
│   ├── sram64_ctrl                外部 SRAM 控制器（7 用户仲裁）
│   ├── kkt_solve                  KKT 求解
│   │   ├── banded_ldl_fp64_rb     带状 LDL 分解 + 前向/后向回代
│   │   └── spmv_fp64              稀疏矩阵向量乘（A x / Aᵀ x）
│   ├── root_plus                  R 加权 1D 根（tau）
│   ├── proj_dual_cone_rb          对偶锥投影
│   │   └── proj_soc               SOC 锥投影
│   └── s_build                    带状矩阵 S 组装（boot 用）
├── gf_display + lcd_driver        LCD 显示（ITER + 分页数据）
├── num_7seg                      7 段数码管（VERB/NOUN/iter）
└── key_debounce                   按键消抖（KEY0–3）
```

## 4. 数据流（boot 流程）

1. 上电后 `drs_iter` 进入 boot FSM。
2. 从 **CFI Flash** 顺序读取求解器输入（见第 5 节布局）：
   - `COO` → 外部 SRAM
   - `c/nb/zmask/band/g` → 片上 RAM
3. 硬编码初始点 `v0 = [0…0, 1.0]`。
4. 计算 `diag_r` / `D_y`（`r_y` 由 S_SX 链填充）。
5. 进入 S_IDLE，等待 `start`。

## 5. 数据布局

### 5.1 CFI Flash 镜像（`gen_flash_image.py` 生成，64-bit 字地址）

| 段 | 字地址基址 | 长度 |
|----|-----------|------|
| COO | 0 | 9566 |
| c | 9566 | 1100 |
| nb | 10666 | 2107 |
| zmask | 12773 | 2107 |
| band | 14880 | 19800 |
| g | 34680 | 3207 |

总计 37887 字 = 303,096 字节（little-endian 字节序）。

`band` 和 `g` 用 round-nearest 预计算并烘焙：截断 FP64 数据路径会让病态 LDL 在
root_plus 判别式处产生符号翻转，导致 iter1 发散，故必须在软件端预计算。

### 5.2 片上 RAM 布局（smem，31983 字）

| 区域 | 地址 |
|------|------|
| V | 0 |
| UT | L |
| U | 2L |
| RSK | 3L |
| G | 4L |
| DR | 4L + LM1 |
| VPR | 5L + LM1 |
| CB | 6L + LM1 |
| BAND | CB + LM1 |
| DY | BAND + 2·LMAX |
| ZMASK | DY |

## 6. 数值格式

- 全部浮点运算用 **截断 FP64**（无 round-to-nearest），53-bit 尾数，相对误差 ~2⁻⁵²。
- `fp64_mul` 用 DSP（164 × 18×18）实现 53×53 尾数乘法。
- `fp64_add` 纯 LUT 组合：52-bit 比较 + 对齐移位 + 54-bit 加减 + LZC + 归一化移位，
  组合延迟 ~29 ns（无硬浮点）。
- 超越函数 `fp64_log` / `fp64_exp` / `fp64_rsqrt` 分时复用 ALU 以节省 LE。

## 7. 时序

- 时钟：`CLOCK_50`（50 MHz）→ `pll30`（VCO 600 MHz，C0=20）→ **30 MHz**。
- 因 `fp64_add` 组合延迟 ~29 ns > 20 ns（50 MHz 周期），必须降到 30 MHz（33.3 ns）。
- 为满足 30 MHz，对多条原本被常量折叠掩盖的长组合链做了流水线化：
  - `kkt_solve` rhs 计算（1 拍锁存 `rho_x·vx`）
  - `proj_soc` sum-of-squares（3 拍）
  - `banded_ldl` UPDATE/FWD/BACK（`mul_o_r` 1 拍）
  - `s_build` pair 累加（1 拍）
  - `root_plus` accumulation（2 级 mul）+ tail 链（1 个 fp64 运算/拍）
- 最终：Fmax ≈ 31 MHz，Setup slack +1.1 ns，无违例。

## 8. 资源占用（EP4CE115F29C7）

| 资源 | 占用 |
|------|------|
| 逻辑单元 LE | ~50k / 114,480（44%） |
| 片上 M9K | ~2.9 Mbit / 3.98 Mbit（74%） |
| DSP 18×18 | 164 |
| 引脚 | 178 / 529 |

## 9. 关键设计决策

| 决策 | 原因 |
|------|------|
| COO + band 存外部 SRAM | 5 份内部 COO ROM 会打爆 M9K（Error 276003） |
| 输入固化 CFI Flash | 免 UART，上电自动加载 |
| band/g 预计算烘焙 | 截断 FP64 的病态 LDL 会导致符号翻转发散 |
| 30 MHz（PLL） | fp64_add 组合延迟 ~29 ns，流水线化所有 fp64 代价过大 |
| 自适应缩放每 25 轮判定 | `mod25_cnt` 计数器替代 `iter%25` 组合除法器 |
| SW[15:0] 迭代上限 | 无收敛即停，用拨码开关设固定轮数 |

## 10. 编译与验证

```powershell
# 完整编译（Quartus 18.1，约 45 分钟）
quartus_sh --flow compile GFOLD_FPGA -c GFOLD_FPGA

# 仿真（ModelSim，ITERS=2 对拍 golden）
vlog -work drsi ../fp64.v ... ../tb_drs_iter.v
vsim -c drsi.tb_drs_iter -gITERS=2 -do "run -all; quit -f"

# 生成烧写文件
quartus_cpf -c -d EPCS64 output_files/GFOLD_FPGA.sof GFOLD_FPGA.pof

# 生成 Flash 镜像
python software/gen_flash_image.py
```

验证标准：`VS1`/`VS2` 与软件 golden（`rtl/data/kkt/full/v1.hex` / `v2.hex`）逐元素
对拍，最大相对误差 ≈ 5e-10（bit-exact）。

## 11. 修改求解器参数 / 初始状态

### 参数分布

| 参数 | 位置 |
|------|------|
| 初始位置 `INIT_POS`、初始速度 `INIT_VEL` | `software/assembler.py` |
| 初始质量 `WET`/`FUEL`、目标 `TGT_*`、重力 `GRAV` | `software/assembler.py` |
| 最大推力 `REAL_MAX_T`、推力区间 `MIN_PCT`/`MAX_PCT`、`MAX_VEL`、`FUEL_CONS` | `software/assembler.py` |
| 离散化点数 `N` | `software/assembler.py`（=100） |
| 飞行时间 `tof` | 多个脚本硬编码 46.6093（见下） |
| `RHO_X` | `software/banded_reference.py` + `rtl/drs_iter.v` |
| `TAU_FACTOR`、`SC_MIN`/`SC_MAX` | `rtl/drs_iter.v` |
| 规模 `N/M/NNZ/HB/L` | `rtl/drs_iter.v` + `rtl/GFOLD_FPGA.v` localparam |

### 只改初始位置 / 速度 / 质量（规模不变）

1. 编辑 `software/assembler.py` 的 `INIT_POS` / `INIT_VEL` / `WET` / `FUEL`（以及
   需要的 `TGT_POS`/`TGT_VEL`/`GRAV` 等）。
2. 一键重新生成：
   ```powershell
   python software/regenerate_all.py
   ```
   它按顺序执行 `regenerate_problem → gen_kkt_test → gen_reordered_data →
   drs_reference gen → gen_coo_sram → gen_flash_image`，并校验所有输出长度与
   `flash_image.bin` 大小。
3. **烧 CFI Flash**（DE2-115 Control Panel，写 `rtl/data/flash/flash_image.bin`）。
4. **不需要重新编译 / 重烧 EPCS64**：只要 `N/M/NNZ/HB` 不变，RTL 和 FPGA 配置
   都不受影响，只是 Flash 里的数据变了。

> `problem.json` 由 `regenerate_problem.py` 用 `assemble(tof)` 重新组装并写回；
> 其 `x/objective/iterations/solve_ms` 是外部 IPM（Clarabel）的参考解，脚本只
> 原样保留不重算——若要用 `check_*.py` 重新对拍，需单独重求参考解。

### 改算法参数（RHO_X 等）或规模

- 改算法参数：`banded_reference.py` + `rtl/drs_iter.v` 都要改，然后重新生成
  （`band_r`/`g.hex` 依赖 `RHO_X`）+ 重新编译 + 烧 Flash + 烧 EPCS64。
- 改规模 `N/M/NNZ/HB`：改 `assembler.py`（N）、`gen_kkt_test.py`（`assemble(46.6093)`
  的 tof/规模）、`regenerate_problem.py`、`rtl/drs_iter.v` 和 `rtl/GFOLD_FPGA.v`
  的 localparam + RAM 布局，重新生成 + 重新编译 + 烧 Flash + 烧 EPCS64。

### tof 的硬编码位置（改 tof 时需同步）

| 文件 | 位置 |
|------|------|
| `gen_kkt_test.py` | `assemble(46.6093)` |
| `regenerate_problem.py` | `TOF = 46.6093` |
| `regenerate_all.py` | 默认参数 `"46.6093"` |
| `assembler.py` | 只作比较的 `assemble(44.63)`（历史遗留，非数据路径） |

## 12. 文件索引

```
software/     Python 参考实现与数据生成（gen_flash_image.py, gen_coo_sram.py, check_*.py）
rtl/          Verilog RTL（drs_iter, kkt_solve, banded_ldl, root_plus, proj_*, fp64*, ...）
rtl/data/     生成数据（kkt/full/*, flash/*）
Quartus/      Quartus 工程（GFOLD_FPGA.qsf, clk_50mhz.sdc, output_files/*）
board/        DE2-115 引脚表与原理图参考
docs/         文档（本手册、50k 验收报告等）
```
