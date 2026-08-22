# Phase 1d/1e 手off — 新会话初始提示词(可直接粘贴)

## 背景
D:\projects\GFOLD_FPGA — 把 SCS 风格 SOCP 求解器搬到 Cyclone IV(DE2-115)纯 Verilog,
全 FP64(IEEE-754)。先加载 skill:skill_view(name='scs-socp-fpga-port'),
严格遵守其中的 ModelSim 10.5b 陷阱(RAM 端口模式、$sformatf 挂起、level-done 需 ARM 等)。

## 当前状态(已完成并提交,勿重做)
- **1a** `rtl/banded_ldl_fp64_rt.v`:banded LDLᵀ,带运行时 rhs 流式输入 + zx 流式输出
  (S 带仍 $readmemh 静态加载,scale=1)。协议:pulse start → 流 N 个 rhs(rhs_valid)→
  流 N 个 zx(zx_valid)→ done。验证:small 1.4e-15;full(cond~1e6)3.16e-9。
- **1b** `software/gen_kkt_test.py`:生成稀疏 Ab COO(Arow/Acol/Aval hex)+ r_y/D_y/vx/vy
  + Schur S 带(band_f64.hex)+ 期望 zx/zy,到 rtl/data/kkt/{small,full}/。
  small(n10/m20/nnz41/hb4)、full(n1100/m2107/nnz4783/hb17)。
- **1c** `rtl/spmv_fp64.v`:稀疏 A/Aᵀ SpMV,transpose 参数选 acc/src 索引。
  **RAM 端口模式**(proj_dual_cone 模式,模块驱动外部 RAM,x@[0,LMAX),out@[LMAX,2*LMAX))。
  协议:pulse start → 流 LENX 个 x → 散射累加 NNZ 次 → 流 LENO 个 out。验证 small 双
  transpose 均 ~5e-16。do:rtl/sim/run_spmv_small.do(从 rtl/sim/ 运行)。
- 提交:`git log --oneline` 可见 1a/1b+1c 的 commit。

## 关键陷阱(务必遵守)
1. **ModelSim 10.5b 对"数据派生的变量索引访问模块内部 unpacked 数组"会 elaboration 卡死**。
   凡需按数据索引随机访问的数据(x/out 等),放**外部 RAM**,模块只驱动 addr/wdata/we/rdata。
   COO 数组(Arow/Acol/Aval)用 $readmemh + kk 顺序索引是安全的(aa_gram 模式)。
2. **RAM 异步读有 1 周期延迟**:先设 addr,下一周期才能读 rdata(输出阶段用 S_OUT0 设地址、
   S_OUT1 读数据两拍)。
3. **流式输入时序**:spmv 必须**先 S_X 流 x,再 S_CLR 清 out**(否则调用方 start 后立即流的
   x 会被 S_CLR 错过,挂起)。
4. do 文件从 **rtl/sim/** 运行,源码用 `../fp64.v` 等相对路径;模块内 $readmemh 用 `../data/...`。
5. 64 位数据扁平总线 + 可变 part-select(如 `out_bus[acc_r*64 +: 64]`),不直接索引 unpacked 数组。
6. fp64_div/rsqrt 的 done 是电平,需 ARM 状态(等 done 拉低)再等下一次。

## 本会话任务:Phase 1d/1e — kkt_solve 胶水
把已验证块串成**完整 KKT 求解**(Schur 块消元),对拍 banded_reference.kkt_solve。

### 算法(Schur 块消元,参考 software/banded_reference.py)
给定 v_x(n)、v_y(m)、r_y、D_y=1/r_y,RHO_X=1e-6,scale=1:
1. rhs_x[i] = RHO_X*v_x[i] − Σ_{A[r,i]≠0} A[r,i]*v_y[r]   (= RHO_X*v_x − Aᵀ v_y)
2. S·z_x = rhs_x     (banded LDLᵀ,带宽 hb)
3. z_y[r] = D_y[r] * (Σ_{A[r,c]≠0} A[r,c]*z_x[c] + r_y[r]*v_y[r])   (= D_y(A·z_x + r_y·v_y))
输出 [z_x; z_y] 即 KKT 解。

### 建议架构(用已验证块)
新模块 `rtl/kkt_solve.v`(参数 N,M,NNZ,HB),实例化:
- 2 个 spmv_fp64(transpose=1 算 Aᵀv_y;transpose=0 算 A z_x)——或复用 1 个带 transpose 输入。
- 1 个 banded_ldl_fp64_rt(S 带用 rtl/data/kkt/{small,full}/band_f64.hex)。
- 顶层 RAM 存放状态向量(vx/vy/rhs_x/zx/zy/az 等)。注意 spmv 用 x@[0,LMAX)/out@[LMAX,2*LMAX),
  需与 kkt_solve 的向量 RAM 分配协调(可给每个向量独立 RAM 区段)。

流程:流 vx+vy → 初始化 rhs_x=RHO_X*vx → spmv(Aᵀ,vy) 得 Aᵀv_y → rhs_x −= Aᵀv_y →
  流 rhs_x 进 LDL → 收 z_x → 初始化 az=0 → spmv(A,zx) 得 A z_x →
  z_y[r]=D_y[r]*(az[r]+r_y[r]*vy[r]) → 输出 z_x(N)+z_y(M)。

### 验证
1. 先 small 再 full。生成器 software/gen_kkt_test.py 已产出期望 zx.hex/zy.hex。
2. 对拍脚本:读 RTL 输出的 zx/zy,与 rtl/data/kkt/<case>/zx.hex/zy.hex 比,容差
   small ~1e-12 / full ~1e-8(截断 FP64 + 病态 S)。
3. 也可用 banded_reference.kkt_solve 直接算期望核对。
4. do 文件从 rtl/sim/ 跑,新文件记得加进 .gitignore(rtl/sim/ 下的 work 库和 *_dump.txt)。

### 完成后
- 提交(风格对齐之前的 commit),更新 skill 记录 kkt_solve 完成与验证结果、新陷阱。
- 下一步是 Phase 2(可运行时重构 LDL)和 Phase 3(S 运行时构建)——先别做,等用户指示。
