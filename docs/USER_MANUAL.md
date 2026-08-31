# GFOLD_FPGA 使用说明书

> 基于 DE2-115（EP4CE115F29C7）的 G-FOLD 动力下降制导求解器 —— 使用手册。

## 1. 简介

本设计在 **DE2-115 开发板**上以纯 Verilog 实现 G-FOLD（燃料最优动力下降制导）的
**二阶锥规划（SOCP）** 求解器。上电后自动加载配置，从板载 CFI Flash 读取求解器
输入数据，运行 DRS 迭代求解，并把结果以 **阿波罗制导计算机（AGC/DSKY）风格**显示
在数码管 / LCD / LED 上。

- 求解问题：1100 变量 / 2107 约束 / 4783 非零元（带宽 17）
- 求解算法：SCS 齐次自对偶嵌入 + Douglas-Rachford 分裂 + 自适应缩放
- 数值格式：截断 FP64（双精度）
- 工作时钟：30 MHz（由 50 MHz 经 PLL 分频）

## 2. 硬件清单

| 器件 | 说明 |
|------|------|
| DE2-115 开发板 | 目标板，EP4CE115F29C7 |
| USB-Blaster | 板载，用于编程 |
| 供电 | 板载电源（12V） |

无需外部 SDRAM；求解器数据固化在板载 **CFI Flash（S29GL064N，8MB）** 中，FPGA 配置
固化在 **EPCS64** 串行配置器件中。

## 3. 快速开始

1. 连接 USB-Blaster，给 DE2-115 上电。
2. 若尚未烧写，先完成 [第 4 节](#4-烧写步骤) 的两次烧写（一次即可，之后免烧）。
3. 上电后 FPGA 自动从 EPCS64 加载配置，LCD 亮起。
4. 拨动 **SW[15:0]** 设定迭代次数上限（二进制）。
5. 按 **KEY1** 启动求解。
6. 观察 LCD / HEX / LED 查看求解进度与结果。

> 断电重上电会自动重新加载配置，无需再次烧写 `.sof`。

## 4. 烧写步骤

需要烧写两处存储（互不影响，各自独立）：

### 4.1 求解器输入数据 → CFI Flash（一次性）

数据文件：`rtl/data/flash/flash_image.bin`（303,096 字节，37887 × 64-bit 字）。

1. 运行 **DE2-115 Control Panel**（经 USB-Blaster 把 Nios 软核配进 FPGA）。
2. Memory 页签 → 选 **Flash**。
3. 先 **Erase**（写前必须擦除）。
4. **Sequential Write**：起始地址 `0`，Length 勾选 File Length（或填 `303096`），
   选 `flash_image.bin`，Write。
5. 关闭 Control Panel。

> 注意：Control Panel 的 Flash 写入只接受 **raw binary（.bin）**，不接受 `.hex`
> （其 `.hex` 解析器会报 "invalid hex file"）。

### 4.2 FPGA 配置 → EPCS64（一次性，AS 模式）

文件：`Quartus/GFOLD_FPGA.pof`。

1. 把 DE2-115 的 **RUN/PROG 滑动开关（SW19）拨到 `PROG`** 位置。
2. Quartus → Tools → Programmer → **Mode 选 Active Serial**。
3. Hardware Setup 选 USB-Blaster。
4. Add File → `GFOLD_FPGA.pof`。
5. 勾 Program/Configure → Start（约几十秒）。
6. 烧完把 SW19 **拨回 `RUN`**，断电重上电，FPGA 自动加载配置。

> AS 模式下 USB-Blaster 报"不支持 Active Serial"，通常是因为 SW19 没有拨到 PROG。

### 4.3 重新生成烧写文件（改代码后）

```powershell
# 重新编译（约 45 分钟）
quartus_sh --flow compile GFOLD_FPGA -c GFOLD_FPGA
# 生成 .pof（AS 模式烧 EPCS64 用）
quartus_cpf -c -d EPCS64 output_files/GFOLD_FPGA.sof GFOLD_FPGA.pof
```

## 5. 按键与开关

| 输入 | 功能 |
|------|------|
| **KEY0** | 复位（`rst_n`，按下为低） |
| **KEY1** | 启动求解（边沿触发） |
| **KEY2** | NOUN 页面上一页 |
| **KEY3** | NOUN 页面下一页 |
| **SW[15:0]** | 迭代次数上限（二进制，0–65535） |
| **SW19** | RUN/PROG 开关（烧 EPCS64 时拨 PROG，运行拨 RUN） |

- `SW[15:0]` 全 0 时**不运行**（上限为 0），必须拨到非 0 才能启动。
- 到达迭代上限后求解停止：`LEDG[5]` 常亮，HEX 的 VERB 显示 `09`。

## 6. 显示说明（DSKY 风格）

### 6.1 数码管 HEX（8 个）

| 位置 | 内容 |
|------|------|
| HEX7–6 | **VERB**（阶段码，2 位） |
| HEX5–4 | **NOUN**（页号，2 位） |
| HEX3–0 | 迭代计数（十六进制，含 A–F） |

**VERB 阶段码**：

| 码 | 含义 |
|----|------|
| 00 | IDLE（空闲） |
| 01 | BOOT（从 Flash 加载） |
| 02 | NORM（归一化） |
| 03 | KKT（KKT 求解，最耗时） |
| 04 | ROOT（root_plus） |
| 05 | UPDT（u/u_t 更新） |
| 06 | CONE（锥投影） |
| 07 | RESID（残差） |
| 08 | SCALE（缩放判定/更新，每 25 轮一次） |
| 09 | DONE（达到迭代上限） |

### 6.2 LCD（16×2）

- **第一行**：`ITER xxxxx`（十进制迭代数，常驻）。
- **第二行**：跟随 NOUN 页切换（KEY2/KEY3 翻页）：

| 页 | NOUN | 第二行显示 |
|----|------|-----------|
| 0（默认） | 01 | `MASS xxxxxxxx`（log final_mass 的 float64 高 32 位，16 进制） |
| 1 | 02 | `SCAL xxxxxxxx`（自适应缩放 scale_cur） |
| 2 | 03 | `TAU  xxxxxxxx`（root_plus tau） |
| 3 | 04 | `ITER xxxxx`（十进制迭代数） |

### 6.3 LED

**LEDG（绿色）**：

| 位 | 名称 | 含义 |
|----|------|------|
| 0 | COMP ACTY | 求解进行中 |
| 1 | PROG | 已启动 |
| 2 | RST | 复位状态（按下 KEY0 亮） |
| 3 | DONE | 单次迭代完成（闪） |
| 4 | PLL | PLL 锁定 |
| 5 | SOLVED | 达到迭代上限（完成） |

**LEDR（红色）**：阶段忙灯 `BOOT/NORM/KKT/ROOT/CONE/SCALE`（对应 VERB 01/02/03/04/06/08）。

## 7. 收敛判断

- 迭代持续推进，VERB 大部分时间显示 `03`（KKT）。
- 每 25 轮 VERB 短暂闪 `08`（缩放判定），属正常现象。
- 收敛判据：NOUN 页 01（MASS）的 8 位 hex 稳定在 `401DF4C2…` 附近
  （即 `log(1795)≈7.49`），对应 `final_mass ≈ 1795 kg`（参考最优 1799.156，−0.22%）。
- 参考迭代量：约 **5 万轮**收敛；用 `SW[15:0]` 设定上限即可自动停在指定轮数。

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 上电无显示 | EPCS64 未烧/未拨回 RUN | 重烧 .pof，SW19 拨回 RUN，断电重上电 |
| boot 卡住 | CFI Flash 数据缺失/字节序错 | 用 Control Panel 重写 `flash_image.bin` |
| USB-Blaster 不支持 AS | SW19 未拨 PROG | 把 SW19 拨到 PROG 再烧 .pof |
| Control Panel 报 invalid hex file | 用了 .hex 格式 | 改用 `flash_image.bin`（raw binary） |
| SW 全 0 不运行 | 迭代上限为 0 | 拨 SW[15:0] 到非 0 |
| LCD ITER 数字跳变快 | 迭代快、刷新跟不上的正常现象 | 无需处理 |
