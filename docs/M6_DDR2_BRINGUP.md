# M6 第一步：DDR2 控制器 bring-up

## 目标

在完整问题（N=1100/M=2107）接入 solver 之前，先单独验证：

- TD 能综合/布局 DDR2 IP（128-bit AXI，x16 DDR2）；
- 板上 DDR2 初始化/校准完成；
- 一个 128-bit 写→读回环正确；
- 地址步进、字节通道、DQS/DQ 映射正常。

## Bit（v2 调试版）

```
anlogic/td_project/phy_ddr2/gfold_ddr2.bit
```

| 项 | 值 |
|---|---|
| 大小 | 1,789,489 bytes |
| SHA256 | `34CB1AFC1C157263F723D4AE6FC9407C9F7CF2D7BA1AEC68944A3D420971A166` |
| 生成时间 | 2026-11-03 21:51:17 |
| 顶层 | `ddr2_smoke_top` |
| 资源 | 2,741 slice (12.92%), 2 PLL, 6 DDR byte |
| 时序 | SWNS +2.838 ns, HWNS +0.021 ns |

## 旧版失败原因

旧版 FSM 有两个 AXI 协议问题：

1. **写数据 ready 用错信号**：wrapper 的 `O_mc_app_wdf_rdy` 接的是
   `paxi_awready`（写地址 ready），不是 `paxi_wready`（写数据 ready）。
   旧 FSM 在 `awready` 后立刻撤销 `WVALID`，如果 `wready` 当拍为低，
   写数据就丢了。
2. **读命令 ready 混用**：`O_mc_app_rdy = paxi_arready || paxi_awready`，
   读 FSM 可能在 `arready` 还没拉高时就前进。

旧版写完 256 个字后立刻读，也没有等待写管道排空，读可能超过未落盘写。

## v2 测试内容

`ph1p35_324_ddr_wrapper.v` 新增三个独立 ready 输出：
`O_mc_app_awready` / `O_mc_app_arready` / `O_mc_app_wready`，旧端口保持不变。
FSM 分别跟踪 `AWVALID` 和 `WVALID` 的握手，只有两者都被接受才算一次写完成。

上电后分三级测试：

| 阶段 | 字数 | 起始地址 | 写间隔 | 内容 |
|---|---|---|---|---|
| 0 | 1 | `0x0000` | 0 | 固定 `0xA5` 重复 16 字节 |
| 1 | 16 | `0x1000` | 16 拍 | 每字按 `idx` 变化 |
| 2 | 256 | `0x2000` | 8 拍 | 每字按 `idx` 变化 |

每级写完都等待 **20,000 个 `ddr_clk`** 让写管道排空，再逐个读回比较。
测试模式 16 个字节完全相同，因此即使 DDR2 字节/字 lane 顺序与预期不同也能通过；
这级先验证地址、握手和读写顺序，lane 顺序留到后续测试。

## 烧录后的 LED 判读

LED 丝印映射已由 `gfold_led.bit` 实测确认：D10→LED1、E10→LED2、
C6→LED3、D6→LED4；LED14 是电源灯。

| LED | 引脚 | 含义 |
|---|---|---|
| LED1 | D10 | `init_calib_complete` |
| LED2 | E10 | 阶段 0（单字）通过 |
| LED3 | C6 | 阶段 1（16 字）通过 |
| LED4 | D6 | 阶段 2（256 字）通过后闪烁 |

| 观察 | 结论 |
|---|---|
| 只有 LED1 亮 | 单字写/读失败或卡在 AW/W/AR 握手 |
| LED1+LED2 亮 | 单字通过，16 字地址步进/节流失败 |
| LED1+LED2+LED3 亮，LED4 不闪 | 256 字吞吐/连续写失败 |
| LED1+LED2+LED3 亮，LED4 闪 | **全部通过** |

## 设计说明

- `ph1p35_324_ddr_wrapper.v` 来自 `FPGA_is_good/work/camera_hdmi_720p/ip/ddr2`（同封装 PH1P35MDG324，相机项目已验证）；
- DDR2 IP 通过 TD 的 `IpADCList`/`IpSDCList` 和 `ddr2.xml` 引入；
- 用户接口直接驱动 AXI（省掉相机项目的 `w155_d512_fifo`），单拍写/读；
- 写地址步进 `+8`，与相机 `video_out/video_in` 的 `O_ddr_user_addr + 'd8` 一致
  （该地址单位是 16-bit word，`+8` 正好是一个 128-bit beat）。

## 下一步（M6 主体）

1. 把 `ddr2_smoke_top` 的 FSM 换成完整问题存储适配器：
   - `sram64_ctrl` 兼容接口（COO/band/LDL）→ DDR2；
   - 状态 RAM/KKT RAM → DDR2（或部分 ERAM + 部分 DDR2）；
2. 完整 `drs_iter`（N=1100/M=2107/HB=17，3-mul pool）；
3. ETH/MCU 控制面：303,096 B `flash_image.bin` 下发到 DDR2、START、STATUS、DUMP；
4. 50k 迭代 + `final_mass ∈ [1790,1800]` 验收。
