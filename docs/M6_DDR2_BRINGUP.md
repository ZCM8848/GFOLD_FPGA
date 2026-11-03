# M6 第一步：DDR2 控制器 bring-up

## 目标

在完整问题（N=1100/M=2107）接入 solver 之前，先单独验证：

- TD 能综合/布局 DDR2 IP（128-bit AXI，x16 DDR2）；
- 板上 DDR2 初始化/校准完成；
- 一个 128-bit 写→读回环正确；
- 地址步进、字节通道、DQS/DQ 映射正常。

## Bit

```
anlogic/td_project/phy_ddr2/gfold_ddr2.bit
```

| 项 | 值 |
|---|---|
| 大小 | 1,789,489 bytes |
| SHA256 | `482389792621F997810C860F20FD04C66A9EEE162869DA08220489155E094C65` |
| 生成时间 | 2026-11-03 21:36:05 |
| 顶层 | `ddr2_smoke_top` |
| 资源 | 3,356 LUT, 2,934 reg, 2 PLL, 6 DDR byte, slice 11.84% |
| 时序 | SWNS +2.776 ns, HWNS +0.020 ns, STA coverage 89.9% |

## 测试内容

上电后 `ddr2_smoke_top` 的 FSM：

1. 等待 `O_init_calib_complete`（DDR2 PHY 校准完成）。
2. 向 DDR2 地址 `idx*8`（`idx=0..255`）写 128-bit 模式：
   `{idx[7:0], ~idx[7:0], 64'h0123456789ABCDEF ^ {8{idx[7:0]}}}`。
3. 从相同地址读回 256 个字，逐字比较。
4. 全部一致 → `pass=1`；任一不一致 → `fail=1`（粘滞）。

## 烧录后的预期 LED

| LED | 引脚 | 含义 | 预期 |
|---|---|---|---|
| LED0 | D10 | `init_calib_complete` | 配置后几 ms 内常亮 |
| LED1 | E10 | PASS | 写/读 256 字全部一致后常亮 |
| LED2 | C6 | FAIL | 任一不一致时常亮 |
| LED3 | D6 | heartbeat | 约 0.3 s 翻转（ddr_clk 域） |

正常情况：**LED0 亮 + LED1 亮 + LED2 灭**。

测试只需要几十微秒到几毫秒（256 写 + 256 读，DDR2 延迟主导）。

## 如果失败

| 现象 | 可能原因 | 下一步 |
|---|---|---|
| LED0 不亮 | DDR2 校准失败 / 时钟/PLL / 引脚 ADC | 检查 `ddr2.adc`、PLL 参考时钟、板上 DDR2 供电 |
| LED0 亮、LED2 亮 | 写/读数据不一致 | 先减小测试字数、改成固定地址、加 ILM 观察；检查 AXI `paxi_wready` 握手 |
| LED0 亮、LED1/LED2 都不亮 | FSM 卡住 | 检查 `mc_rdy`/`wdf_rdy`/`rd_data_valid` 握手；可能写数据未被 AXI 接受 |
| LED3 不闪 | `ddr_clk` 未起振 | 检查 DDR2 IP 的 `dhi_clk`/PLL |

## 设计说明

- `ph1p35_324_ddr_wrapper.v` 来自 `FPGA_is_good/work/camera_hdmi_720p/ip/ddr2`（同封装 PH1P35MDG324，相机项目已验证）；
- DDR2 IP 通过 TD 的 `IpADCList`/`IpSDCList` 和 `ddr2.xml` 引入；
- 用户接口直接驱动 AXI（省掉相机项目的 `w155_d512_fifo`），单拍写/读；
- 写地址步进 `+8`，与相机 `video_out/video_in` 的 `O_ddr_user_addr + 'd8` 一致。

## 下一步（M6 主体）

1. 把 `ddr2_smoke_top` 的 FSM 换成完整问题存储适配器：
   - `sram64_ctrl` 兼容接口（COO/band/LDL）→ DDR2；
   - 状态 RAM/KKT RAM → DDR2（或部分 ERAM + 部分 DDR2）；
2. 完整 `drs_iter`（N=1100/M=2107/HB=17，3-mul pool）；
3. ETH/MCU 控制面：303,096 B `flash_image.bin` 下发到 DDR2、START、STATUS、DUMP；
4. 50k 迭代 + `final_mass ∈ [1790,1800]` 验收。
