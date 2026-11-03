# 里程碑 5 上板 bit（PH1P35 片上小实例）

## Bit 文件

```
anlogic/td_project/phy_m5/gfold_m5.bit
```

| 项 | 值 |
|---|---|
| 大小 | 1,789,533 bytes |
| SHA256 | `3A25AA88A135321BEFC139DCB29C220C4AA588173485FED824A3CC873A9F4A78` |
| 生成时间 | 2026-11-03 12:11:49 |
| 顶层 | `gfold_m5_top` |
| 器件 | PH1P35MDG324 |
| 综合/布局 | `syn_m5` / `phy_m5` |

## 资源（最终 P&R）

```
#lut6   18263 / 21216  (86.08%)
#reg    13361
#dsp       40 / 40     (100%)
#eram      29 / 108    (26.85%)
```

## 时序

```
SWNS -1.249 ns, STNS -34.142 ns
HWNS -0.351 ns, HTNS -0.709 ns
```

最差 setup 路径是 125 MHz `dump_tx` 的
`dump_ip_125_reg[16] -> u_crc/crc_reg[4]`（9.018 ns，8 ns 预算）。
这条路径在 M4 版本已有 -18 ps；本版本因 solver 加入后拥挤度上升。

**注意**：`clk_sol` 是由 `clk_125/4` 经 BUFG 产生的 fabric 时钟，TD 的
STA 把它当成 local clock，`clk_sol` 的 fanout 报 0、`u_solver` 路径
没有被时序分析覆盖（STA coverage 10%）。solver 综合阶段在 50 MHz 下
最差数据路径约 28.3 ns，31.25 MHz（32 ns）理论上还有约 3.7 ns 余量，
但没有工具保证。上板时以实际 `check` 结果为准。

如果 DUMP 因 FCS 错误被 PC 网卡丢弃，下一步是把 `dump_tx` 的 CRC
改成“发送前用 2 拍/字节预计算整个帧的 FCS”，把 CRC 从 125 MHz
发送路径上拿掉。

## 上板测试

1. 用 BW/TD GUI 烧录 `gfold_m5.bit`。
2. PC 网口配置 `192.168.1.100/24`，FPGA 为 `192.168.1.200`。
3. 链路检查：

```powershell
python host\gfold_eth.py ping
```

4. 加载小实例 boot 镜像（511 words，4088 B）：

```powershell
python host\gfold_eth.py load rtl\data\kkt\small2\flash_image.bin
```

5. 启动（释放 solver 复位，自动 boot + 2 轮 DRS）：

```powershell
python host\gfold_eth.py start
```

6. 等待约 0.5 s，然后对拍：

```powershell
python host\gfold_eth.py check
```

期望输出：

```
VS1: worst_rel=... bad=0/60 PASS
VS2: worst_rel=... bad=0/60 PASS
SMALL-PROBLEM CHECK: PASS
```

阈值 5e-10；仿真结果 VS1 3.07e-13、VS2 1.20e-10。

## 已完成的验证链

| 层级 | 结果 |
|---|---|
| 完整问题 pool RTL 2 轮（iverilog） | V1 5.16e-10 / V2 5.72e-10，PASS |
| 小实例 `solver_core` 端到端（image→boot→2 轮→vs_ram） | VS1 3.07e-13 / VS2 1.20e-10，PASS |
| `gfold_m5_top` iverilog  elaboration（vendor 原语 stub） | 通过 |
| TD 综合 | 40 DSP, 29 ERAM, slice 74.9% |
| TD P&R | bit 生成，LUT6 86.08% |
