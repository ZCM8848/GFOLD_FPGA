# 里程碑 5 上板 bit（PH1P35 片上小实例）

## Bit 文件

```
anlogic/td_project/phy_m5/gfold_m5.bit
```

| 项 | 值 |
|---|---|
| 大小 | 1,789,533 bytes |
| SHA256 | `1416299C3105EE8FD66DABE09F3CB454B7E23FA43855C592BAADD036315C3F2D` |
| 生成时间 | 2026-11-03 21:23:48 |
| RX 输入延迟 | `INDEL = 4`（I_rgmii_rxd[3:0] / I_rgmii_rx_ctl） |
| 顶层 | `gfold_m5_top` |
| 器件 | PH1P35MDG324 |
| 综合/布局 | `syn_m5` / `phy_m5` |

## 资源（最终 P&R）

```
#lut6   18237 / 21216  (85.96%)
#reg    13393
#dsp       40 / 40     (100%)
#eram      29 / 108    (26.85%)
```

## 时序

| 时钟 | 频率 | Setup WNS | Hold WNS |
|---|---|---|---|
| `clk_125` | 125 MHz | **+0.150 ns** | +0.049 ns |
| `rgmii_rxc` | 125 MHz | **+0.614 ns** | **-0.357 ns** |
| `clk_in` | 50 MHz | +14.453 ns | +0.107 ns |
| `clk_sol` | 31.25 MHz | 未覆盖（fabric clock） | 未覆盖 |

**setup 违例已全部消除。** 之前的两条关键路径都已修复：

1. `dump_ip_125 -> dump_tx CRC -> crc_reg`（-1.249 ns）
2. `udp_ip_125 -> udp_reply_tx CRC -> crc_reg`（-1.392 ns）

修复方法：所有四个 TX 模块（DUMP/UDP/ICMP/ARP）的 CRC 输入改为寄存器
`crc_data`，FCS 首字节改用 `~crc_next`；深 `b` mux 只到寄存器，不再串 CRC。
仿真验证的 FCS：

- DUMP: `0xBB9A8B4F` PASS
- UDP ACK: `0x2D45D065` PASS
- ARP: `0x98DF9D0A` PASS
- ICMP: `0x70F16EA6` PASS

### 剩余 hold 违例

`I_rgmii_rxd[2] -> u_rx_mac/frame_data_reg[0]`，hold -0.357 ns，
数据路径 1.055 ns、时钟 skew 1.324 ns。这是 RGMII RX 源同步接口的
hold，和 solver 无关。M4 版本的同一条 RX 路径（同一 PHY、同一板）是
能工作的；静态分析按 0 外部 hold 计算，实际 PHY 输出 hold 通常大于
0.4 ns。若上板 `ping`/`load` 正常，说明该 hold 在板上满足；若 RX 出错，
下一步在 `rgmii_rx_phy` 的 IDDR 前加 2 级 LUT 延迟（约 0.4 ns），
此时 RX setup 余量 0.614 ns 仍够。

### `clk_sol` 未覆盖

`clk_sol` 是 `clk_125/4` 经 BUFG 的 fabric 时钟，TD 把它当 local clock
（fanout 0），`u_solver` 路径没有 STA。solver 综合时在 50 MHz 下最差
数据路径约 28.3 ns，31.25 MHz（32 ns）理论余量约 3.7 ns，板级大概率
正常，但严格 signoff 还需让 TD 识别这个时钟。

## 上板后的预期行为

烧录 `gfold_m5.bit`，PC 网口 `192.168.1.100/24`，FPGA `192.168.1.200`。

### 1. 上电 / 烧录后

- `O_led[0]` = PLL lock，常亮。
- `O_led[3]` = 心跳，约 0.75 s 翻转一次（50 MHz / 2^26）。
- `O_led[1]` = solver done，初始灭。
- `O_led[2]` = solver armed，初始灭。

### 2. `python host\gfold_eth.py ping`

预期：

```
ACK seq=... rx_frames=... link=0x00000001
...
5/5 replied
```

说明 ARP/ICMP/UDP 链路和 TX FCS 正常（本次修复的四个 TX 模块之一）。

### 3. `python host\gfold_eth.py load rtl\data\kkt\small2\flash_image.bin`

镜像 511 words / 4088 B，3 个 LOAD 包 + FINISH。预期：

```
Image: 4088B 511w CRC=0x........
LOAD 3 pkts...
  3/3
FINISH...
  FINACK CRC=0x........ status=0
  PASS
```

说明 ETH→image_ram 通路和 CRC 正常。

### 4. `python host\gfold_eth.py start`

发送 START(0x05)，top 置 `solver_armed`，释放 solver 复位。随后：

- `O_led[2]`（armed）点亮。
- solver 从 image_ram boot，跑 iter0 → 复制 VS1 → iter1 → 复制 VS2。
- 约 5–10 ms 后 `O_led[1]`（done）点亮并保持。
- 整个过程 `clk_sol` 31.25 MHz；host 等 0.5 s 足够。

### 5. `python host\gfold_eth.py check`

分 12 次 DUMP（每次 10 words）读 `vs_ram` 的 120 words，与
`small2/v1.hex`、`v2.hex` 对拍。预期：

```
VS1: worst_rel=3.0e-13  bad=0/60 PASS
VS2: worst_rel=1.2e-10  bad=0/60 PASS
SMALL-PROBLEM CHECK: PASS
```

阈值 5e-10。数值应与仿真一致（VS1 3.07e-13、VS2 1.20e-10）。

### 6. 可选：`python host\gfold_eth.py dump 0 10`

手动看 VS1 的前 10 个 64 位字；应与 `small2/v1.hex` 前 10 行一致
（十六进制位模式，截断 FP64 与 golden 的 RNE 有 ~1e-13 相对差）。

## 可能出现的异常与定位

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `ping` 无回复 | RX hold 违例 / PHY 链路 | 先看 LED0、`arp -a`；必要时给 RX 数据加 2 级 LUT 延迟 |
| `load` FINACK CRC 错 | image_ram 写时序 / LOAD 包 | 检查 3 包是否都 ACK；重发 |
| `start` 后 LED1 不亮 | solver boot 未完成 / 时钟 | 检查 `clk_sol`（BUFG 输出）；用 PING ACK 的 `solver_done` 位 |
| `check` DUMP 超时 | 125 MHz TX FCS（已修）或 dump_tx 时序 | 本版已修 TX CRC；若仍超时，看 `final_timing.rpt` |
| `check` 数值错但 DUMP 正常 | solver 路径（clk_sol 未 STA） | 先看 VS1；若 VS1 错，查 boot/image |

## �ϰ�ʵ������2026-11-03��

��¼ `gfold_m5.bit` ���ڱ�����192.168.1.100��ʵ�����У�

### PING

```
ACK seq=1 rx_frames=91 link=0x00000000
...
5/5 replied
```

### LOAD + START + CHECK

```
Image: 4088B 511w CRC=0x9767674C
RESET...
LOAD 3 pkts...
  3/3
FINISH...
  FINACK CRC=0x9767674C status=0
  PASS
START sent; waiting 0.5s for boot + 2 iterations...
VS1: worst_rel=3.071e-13 bad=0/60 PASS
VS2: worst_rel=1.199e-10 bad=0/60 PASS
SMALL-PROBLEM CHECK: PASS
```

**�弶����� RTL ������λ��һ�£�**

| �� | ���� | �ϰ� |
|---|---|---|
| VS1 worst rel | 3.07e-13 | **3.071e-13** |
| VS2 worst rel | 1.20e-10 | **1.199e-10** |
| check_drs | PASS | **PASS** |

��֤����

- pool �� solver��3 mul + 3 add + 1 div + ���� rsqrt���� PH1P35 ����ʵ���У�
- ETH �·�����boot��2 �� DRS��VS1/VS2 ���ơ�DUMP ȫ��·������
- �ض� FP64 ��ֵ·���� golden �Ĳ����� 5e-10 ���ڣ�
- ֮ǰ�޸��� TX CRC ʱ��û��Ӱ�� PING/LOAD/DUMP��

### ʣ������

- ���� G-FOLD ���⣨N=1100/M=2107������ DDR2����ǰ��Ƭ��Сʵ����
- `rgmii_rxc` hold -0.357 ns δӰ��ʵ�⣨ping/load/check ����������
- `clk_sol` ��δ���� TD STA���� 31.25 MHz ʵ��������

## INDEL=4 �汾���⣨2026-11-03 21:30��

��¼ `1416299C...`��`INDEL = 4`����ʵ�⣺

```
ping:  �ڶ��� 5/5����һ�� seq=1 �� ARP/��·��ʼ�����װ���ʧ��
load:  FINACK CRC=0x9767674C status=0 PASS
check: VS1 3.071e-13, VS2 1.199e-10, both bad=0/60, PASS
```

���ۣ�

- `INDEL=4` û���ƻ� RX��ping/load/check ȫ��������
- ������ RX ������� RX_CLK ������Լ 0.4 ns �����ӳ٣�hold ����ʵ�ʸ��ƣ�
- TD ���� STA ����ģ `INDEL`�������� `rgmii_rxc` hold ����ʾ -0.357 ns��
  ���ǹ��߽�ģ���ƣ����ǹ������⣻
- ��ǰ����֤ bit Ϊ `1416299C3105EE8FD66DABE09F3CB454B7E23FA43855C592BAADD036315C3F2D`��
