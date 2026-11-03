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

## 移植状态（2026-11-03）

### 已完成

| 模块 | 状态 | 文件 |
|---|---|---|
| fp64_pool.v | ✅ 4 mul + 5 add + 1 div = 32 DSP | rtl/solver/fp64_pool.v |
| root_plus | ✅ 全重写（15 mul → 4 pool ch, 4 cyc/elem） | rtl/solver/root_plus_pool.v |
| proj_soc | ✅ 全重写（3 mul → 3 pool ch, 顺序 adds） | rtl/solver/proj_soc_pool.v |
| kkt_solve | ✅ 4-way mux（own/spmv1/spmv2/ldl） | rtl/solver/kkt_solve.v |
| spmv_fp64 | ✅ 端口别名（1 mul + 1 add） | rtl/solver/spmv_fp64.v |
| banded_ldl_fp64_rb | ✅ 端口直连（1 mul + 1 add + 1 div） | rtl/solver/banded_ldl_fp64_rb.v |
| banded_ldl_fp64_rt | ✅ 端口直连（1 mul + 1 add） | rtl/solver/banded_ldl_fp64_rt.v |
| normalize_v | ✅ 端口别名（1 mul + 1 add） | rtl/solver/normalize_v.v |
| proj_dual_cone_rb | ✅ 端口别名（1 add） | rtl/solver/proj_dual_cone_rb.v |
| s_build | ✅ 状态 mux（2 mul → 1 ch） | rtl/solver/s_build.v |
| golden 数据 | ✅ v1.hex/v2.hex (3208 字), flash_image.bin | rtl/data/kkt/full/, rtl/data/flash/ |

### 待完成

| 任务 | 说明 | 预估 |
|---|---|---|
| drs_iter 重写 | 核心 FSM（~64 状态），pool mux 到子模块，替换 SRAM/Flash | 3-4h |
| rsqrt 共享 | root_plus + proj_soc 各有 1 rsqrt（8 DSP），需合并为 1 | 30min |
| solver_core | 实例化 pool + rsqrt + drs_iter，顶层包装 | 1h |
| 存储适配器 | sram64_ctrl → on-chip BRAM（仿真）或 DDR2（上板） | 1-2h |
| 仿真 | tb_drs_iter 对拍 v1.hex/v2.hex | 1h |
| 综合 + 上板 | TD 综合，ETH 集成，START/DUMP | 2h |

### drs_iter 重写详细步骤

drs_iter 是 solver 的中枢 FSM（原始 1600 行，~300 状态）。里程碑 5 只需核心迭代（不含 boot/scale/Anderson），约 64 状态。

**保留的 FSM 阶段（从原始 drs_iter.v 复制）：**
1. S_IDLE → S_NM0-S_NM4, S_NMR-S_NMV4（normalize，~12 状态）
2. S_KSSTART-S_KSDONE（KKT solve，~8 状态）
3. S_RP0-S_RPW, S_UTT2（root_plus，~7 状态）
4. S_GT0-S_GT6（u_t -= tau*g，~7 状态）
5. S_U0-S_U5（u = 2*ut - v，~6 状态）
6. S_CONE-S_CONEW（cone projection，~2 状态）
7. S_UM0-S_UMW（u[l-1] = max，~3 状态）
8. S_RSK0-S_RSK9（residual，~10 状态）
9. S_V0-S_V7（v update，~8 状态）
10. S_DONE

**排除的 FSM 阶段（里程碑 5 不用）：**
- S_BOOT_*（Flash boot，用 ETH 替代）
- S_SCL*（scale，iter 25 才触发）
- S_SX*（s_build for scale）
- S_GKS*（g recompute for scale）
- S_VR*（v remap for scale）
- S_W1-S_W85（sync-read wait states for scale）
- Anderson（chol10）

**drs_iter 自有算术（需转 pool）：**
- um1, um2（2 mul）：u_update 和 v_update 阶段的乘法
- ua1, ua2（2 add）：同上的加减
- urs（1 rsqrt）：用于 normalize 或 mass 显示（需确认）
- unm, uexp, ulogp, ulogd（4 实例）：scale 阶段，排除

**pool mux（drs_iter 层）：**
根据 FSM 阶段，将 pool 通道路由到活跃子模块：
- S_NM*: normalize_v
- S_KS*: kkt_solve（内部再 mux 到 spmv/ldl）
- S_RP*: root_plus
- S_GT*, S_U*, S_RSK*, S_V*: drs_iter 自有算术
- S_CONE*: proj_dual_cone_rb

**存储替换：**
- sram64_ctrl → 简单 BRAM 接口（仿真用 $readmemh，上板用 image_ram/DDR2）
- flash_ctrl → testbench 直接加载（仿真）或 ETH download（上板）
- 接口保持不变（req/we/waddr/wdata/busy/rdata），只换实现

**rsqrt 共享：**
- 从 root_plus_pool 和 proj_soc_pool 移除本地 rsqrt
- 添加 rsqrt 端口（output: start, x; input: done, o）
- drs_iter 实例化 1 个 fp64_rsqrt，mux 到 root_plus 或 proj_soc

## golden 数据（已生成）

- `rtl/data/kkt/full/v1.hex` — 3208 字（iter 1 输出）
- `rtl/data/kkt/full/v2.hex` — 3208 字（iter 2 输出）
- `rtl/data/flash/flash_image.bin` — 303096 字节（solver 输入）
- `rtl/data/kkt/full/band_r.hex` — 19800 字（预计算带状矩阵）
- `rtl/data/kkt/full/g.hex` — 3207 字（预计算 g 向量）
