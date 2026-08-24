# 硬件 50k 收敛验收报告（Verilator 仿真）

日期:2026-08-25 | 阶段:Phase 4b 最终验收 | 结论:**PASS**

## 结论

**硬件 DRS 数据通路完整跑完 50k 轮并收敛,final_mass ≈ 1795.14(参考最优 1799.156,误差 0.22%)。**
50k 硬件仿真中发现的 iter450 分叉,根因是对拍 SW 基准(QR Anderson)与硬件实现(Cholesky Anderson)的算法级差异,**硬件无 bug**。

## 运行条件

- 平台:目标机(9950X3D),打包 exe(ITERS=50001 + 3 个 mingw DLL,零配置)
- 输出:`veri50k-2.txt`(109MB,2003 个 VS dump)
- `ALL DONE` ✓,非 TIMEOUT(watchdog 2s→300s 仿真时间修复后)

## 硬件收敛关键数据

| 指标 | 值 |
|---|---|
| 末轮收敛 | iter50000→50001 `maxdiff = 0.0`(连续两轮零变化) |
| scale 触发 | 6 次:iter25/125/225/325 + iter25925/26025 |
| scale 值 | 0.2138/0.0272/0.0035/0.0011(前 4 次与 iter0..400 验证逐值一致) |
| HW 收敛态 final_mass | **1795.14**(从末轮 v 经 root_plus 路径推导,tau=0.003772) |

## 诊断过程:iter450 分叉根因(重要教训)

1. **现象**:iter0..400 逐元素对拍全 PASS(worst 7.7e-4),但 50k 对拍 iter426 PASS → iter451 FAIL。
2. **定位**:分叉在 iter450 轮,它是 50k 中第一个"AA + 残差 + scale 候选"组合轮(距上次 scale 触发 125 轮 ≥ 100)。
3. **缩小**:iter430 是 iter325 scale 触发后第一次真正执行 AA solve 的轮(AA 记忆重新累积满)。
4. **根因确认**:SW 基准 `scs_faithful.Anderson` 用 **pivoted QR**(dgeqp3),而硬件 `anderson.v` 用 **regularized normal equations + dense Cholesky**(用户批准的设计,`gen_anderson.AndersonNaive` 镜像)。两者数学等价但数值路径不同(normal equations 条件数 = cond(S)²),在 iter430(scale=0.0011 下 AA 记忆满)首次数值分歧(worst_rel 263)。
5. **HW 无 bug(最终确认)**:探针状态流证实 iter430 轮走 `S_AA0→S_AA8-10→norm→S_SG0-9→S_SC0-3(回退v)→S_SCP0-3(回退v_prev)→S_DONE`——**safeguard 保护性回退**。触发机制:iter430 是 scale=0.0011 下第一次真正 AA solve(G 矩阵 cond=5.7e3),HW(truncating)与 SW(round-nearest)输入 v 的 ~3e-5 差异被 cond 放大到 gamma ~20%,加速后 v 不同 → `||diff||` 略超 `||x-f||` → HW 保守触发回退(拒绝一次加速),SW 不触发(ratio 0.970)。**这是 Cholesky AA 在 ill-conditioned 时的固有数值敏感性(用户批准的 Cholesky 设计),非功能错误**;回退后 HW 自洽收敛(final_mass 1795.14,三实现中最接近参考)。
6. **50k 验收标准回归 final_mass**(Key Decision:"end-to-end acceptance is convergence of final_mass, not bit-exactness"):长跑中 scale 触发时机对数值细节敏感(HW 25925/26025 vs SW-NC 37250/37350 vs SW-QR 45350/45450),逐元素对拍在长跑中必然分叉。

## 三实现收敛对比(50k)

| 实现 | final_mass | vs 参考 1799.156 |
|---|---|---|
| **HW(truncating fp64 + Cholesky)** | **1795.14** | **−0.22%** |
| NC(Cholesky, round-nearest SW) | 1808.26 | +0.5% |
| QR(scs_faithful, round-nearest SW) | 1783.6 | −0.9% |
| 参考(最优解) | 1799.156 | — |

三个实现都收敛到参考解 ±1% 内,HW 最接近。HW final_mass 的推导含 SW 推导的固有不确定性(~0.2%),真实硬件值可能在 1795 附近 ±0.5%。

## 修改清单(本次)

- `rtl/tb_drs_iter.v`:watchdog `#2000000000`(2s)→ `#300000000000`(300s 仿真时间,够 50k 的 ~195s)
- 诊断用 tb 每轮 dump(临时,已恢复为每 25 轮 + 末 3 轮)

## 遗留事项

- 各实现 50k 最终 mass ±1%:DRS 在 50k 时尚未完全收敛到最优(求解器收敛特性,非硬件问题)。如需更精确,增大 ITERS 或收紧 scale 触发阈值。
- 上板仍受 RAM 容量限制(主 RAM 521KB > EP4CE115 的 486KB),数据固化/压缩另行处理。
