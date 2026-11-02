# 里程碑 5 移植计划：solver 核心移植

## 目标
在 PH1P35 上运行 2 轮 DRS 迭代，DUMP 出 VS1/VS2，与 golden `v1.hex`/`v2.hex` 逐元素对拍，worst rel ≤ 5e-10。

## 资源缺口

| 项 | master（EP4CE115） | PH1P35 | 差距 |
|---|---|---|---|
| DSP | 164 × 18×18 | 40 × 27×18 | 4× 不够 |
| LUT | 50k LE | 38k LUT4 | 1.3× 不够 |
| 存储 | 2 MB 外部 SRAM | 108 块 ERAM (270 KB) + DDR2 | 需 DDR2 |
| 时钟 | 30 MHz | 30 MHz（PLL） | OK |

## solver RTL 清单（已复制到 rtl/solver/）

21 个文件。关键模块：

| 模块 | fp64_mul | fp64_add | fp64_div | 说明 |
|---|---|---|---|---|
| root_plus.v | 15 | 13 | 1 | 二次方程求根，流水线最深 |
| drs_iter.v | 3 | 2 | 1 | 顶层 FSM，调度各阶段 |
| kkt_solve.v | 3 | 2 | 0 | KKT 求解 |
| proj_soc.v | 3 | 3 | 0 | SOC 锥投影 |
| fp64_exp.v | 3 | 3 | 0 | 指数（已分时复用） |
| fp64_log.v | 3 | 3 | 0 | 对数（已分时复用） |
| fp64_rsqrt.v | 3 | 2 | 0 | 倒数平方根（已分时复用） |
| s_build.v | 2 | 1 | 0 | 带状矩阵组装 |
| banded_ldl_fp64_rb.v | 1 | 1 | 0 | LDL 分解+回代 |
| spmv_fp64.v | 1 | 1 | 0 | 稀疏矩阵向量乘 |
| normalize_v.v | 1 | 1 | 0 | 向量归一化 |
| 其余 | 2 | 4 | 3 | chol10/proj_dual_cone/banded_ldl_rt 等 |
| **合计** | **40** | **36** | **6** | |

## 共享运算池设计

DRS 迭代是顺序的：normalize → KKT → root_plus → update → cone → residual → scale。任意时刻只有一个模块活跃。

**pool 规格**：4 × fp64_mul + 5 × fp64_add + 1 × fp64_div = 32 DSP + ~10k LUT

**接口**：
```
fp64_pool(
    clk, rst_n,
    // 申请口（多组，由各模块的 FSM 驱动）
    mul_req[3:0], mul_a[3:0][63:0], mul_b[3:0][63:0], mul_grant[3:0], mul_o[3:0][63:0],
    add_req[4:0], add_a[4:0][63:0], add_b[4:0][63:0], add_sub[4:0], add_grant[4:0], add_o[4:0][63:0],
    div_req, div_a[63:0], div_b[63:0], div_grant, div_o[63:0], div_done
)
```

每个模块将其 fp64_mul/add/div 实例替换为 pool 申请，FSM 逐步执行（与 fp64_exp/log/rsqrt 现有模式一致）。

## 存储替换

| master（sram64_ctrl） | PH1P35 替代 |
|---|---|
| COO (9566 字) | DDR2 或 image_ram 扩容 |
| band LDL (109072 字) | DDR2 |
| 状态向量 (31983 字) | ERAM（88 块） |
| Anderson 记忆 | DDR2（里程碑 5 不用） |

里程碑 5 只跑 2 轮，不触发 Anderson 加速，可跳过 Anderson 存储。

boot FSM 改为从 ETH 下发的 image_ram/DDR2 读取数据（替代 flash_ctrl）。

## 时钟域

- 求解器：30 MHz（PLL 第二路输出）
- ETH：125 MHz
- CDC：START/DONE/STATUS 走 toggle 握手；DUMP 走 image_ram 双时钟读口

## 移植步骤

1. **fp64_pool.v** — 写共享运算池模块
2. **root_plus.v** — 拆 15 个并行乘法为 4 路分时（最大改动）
3. **kkt_solve.v / banded_ldl_rb / spmv / proj_soc** — 改用 pool
4. **drs_iter.v** — 替换 sram64_ctrl 为 storage adapter，替换 flash_ctrl 为 ETH boot
5. **storage adapter** — image_ram 扩容或 DDR2 控制器
6. **顶层集成** — solver 时钟域 + ETH 时钟域 + START/DUMP 命令
7. **仿真** — tb_drs_iter.v 对拍 v1.hex/v2.hex
8. **上板** — ETH load → boot → START{2} → DUMP → check_drs.py

## golden 数据（已生成）

- `rtl/data/kkt/full/v1.hex` — 3208 字（iter 1 输出）
- `rtl/data/kkt/full/v2.hex` — 3208 字（iter 2 输出）
- `rtl/data/flash/flash_image.bin` — 303096 字节（solver 输入）
- `rtl/data/kkt/full/band_r.hex` — 19800 字（预计算带状矩阵）
- `rtl/data/kkt/full/g.hex` — 3207 字（预计算 g 向量）
