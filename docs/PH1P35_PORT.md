# PH1P35 移植规格（anlogic-ph1p35 分支）

目标：在安路 PH1P35MDG324（HX1P35A 开发板）上实现与 master 分支相同的 G-FOLD 求解器，全部 IO 走千兆以太网。

## 1 资源对照

| 资源 | EP4CE115（master） | PH1P35MDG324 | 备注 |
|---|---|---|---|
| 逻辑 | 50k LE / 114,480 | 38,189 LUT4 | 需共享运算池压缩 |
| DSP | 164 × 18×18 / 532 | 40 × 27×18 | 53×53 乘法占 8 个，并行乘法器 18→5 |
| 片上 RAM | 2.9 Mbit M9K / 3.98 | 2,160 Kbit ERAM（108 × 20K） | 七个状态向量占 88 块 |
| 外部随机存储 | 2 MB SRAM，6 周期/字 | DDR2 x16，128 位 AXI | 需重写控制器 |
| 启动数据 | 8 MB CFI Flash | N25Q128 QSPI（bit + MCU bin）+ ETH 下发 | |
| IO | LCD + 16 拨码 + UART | RGMII 千兆 + 4 LED + KEY0 | 全部走 ETH |

## 2 网络参数

| 项 | 值 |
|---|---|
| MAC | `02:00:00:00:00:01` |
| FPGA IP | `192.168.1.200/24` |
| PC IP | `192.168.1.100/24` |
| 命令端口 | UDP 4661 双向 |
| 状态端口 | UDP 4662 单向上行 |
| 速率 | 仅 1000BASE-T |
| ARP | fabric 应答 |
| ICMP | fabric echo reply |

## 3 应用层协议

### 3.1 应用头（8 字节，大端）

| 字段 | 位宽 | 说明 |
|---|---|---|
| magic | 16 | `0x4746` |
| type | 8 | 见下表 |
| flags | 8 | 保留 |
| seq | 16 | 包序号 |
| nwords | 16 | 净荷中 64 位字数（0..180） |

### 3.2 报文类型

| type | 名 | 方向 | 净荷 |
|---:|---|---|---|
| 0x01 | LOAD | H→F | 180 字（尾包 87 字） |
| 0x02 | ACK | F→H | 32 字节位图 |
| 0x03 | FINISH | H→F | `{u32 total_words, u32 expected_crc32}` |
| 0x04 | FINACK | F→H | `{u32 actual_crc32, u32 actual_words, u8 status, u8 missing}` |
| 0x05 | START | H→F | `{u32 iter_limit}` |
| 0x06 | STATUS | F→H (4662) | `{u32 iter, u8 verb, u8 phase, u8 done, u8 running, u64 scale, u64 tau, u64 mass_log, u32 crc32}` |
| 0x07 | DUMP_REQ | H→F | `{u32 start_word, u32 nwords}` |
| 0x08 | DUMP_DATA | F→H (4662) | 最多 180 字；末包 `seq=0xFFFF, nwords=0` |
| 0x09 | RESET | H→F | 空 |
| 0x0A | PING | H→F | 空；回 ACK 带 LINK_STAT |

### 3.3 镜像切分

37,887 个 64 位字 = 210 包 × 180 字 + 1 包 × 87 字。`seq × 180` = DDR2 目标字地址，与 `flash_image.bin` 段偏移一致（COO 0 / c 9566 / nb 10666 / zmask 12773 / band 14880 / g 34680）。镜像小端 64 位字，网线按字节传，接收端拼字写 DDR2。

### 3.4 完整性

fabric 累计 CRC32（poly `0xEDB88320`，初值 `0xFFFFFFFF`，终值取反，等价 `zlib.crc32`）。`FINISH` 比对整镜像 CRC 与总字数。每包 ACK 携带 256 位收包位图，主机按位图补发。

## 4 功能分配

| 功能 | 归属 |
|---|---|
| RGMII PHY 收发、HD_ODDR/HD_IDDR、GTX_CLK OUTDEL=2 | fabric |
| RX MAC、ARP 应答、ICMP echo、UDP/IPv4 收发头、变长帧、FCS | fabric |
| 8 字节应用头解析、seq×180 地址、1440 B 包缓冲、DDR2 写事务 | fabric |
| CRC32 流水 | fabric |
| DDR2 IP（128 位 AXI） | fabric |
| PLL（求解时钟、125M、MCU 300M/100M） | fabric |
| 共享运算池、drs_iter/kkt_solve/banded_ldl/root_plus/proj_*/spmv/s_build | fabric |
| AHB 从口寄存器组、DUMP 读通道 | fabric |
| ETH 会话状态机、位图、FINISH 比对、命令解析、STATUS/DUMP 组包 | MCU 固件 |
| golden 数据生成、flash_image.bin | 主机脚本 |
| 主机控制台 gfold_eth.py | 主机脚本 |

净荷不经 MCU：fabric 把 1440 B 直接写 DDR2，MCU 每包只做三次寄存器访问。

## 5 AHB 寄存器映射

基址 `0x8000_0000`，32 位单拍，volatile 不缓存。

| 偏移 | 名 | 方向 | 内容 |
|---:|---|---|---|
| 0x00 | ETH_CMD | W | session_reset / commit_pkt / start_solve / dump_req / ping_reply / set_img_ready / clr_img_ready |
| 0x04 | ETH_HDR | R | {magic, type, flags, seq} |
| 0x08 | PKT_ADDR | W | 目标 DDR2 字地址 |
| 0x0C | PKT_NWORDS | R | 当前包字数 |
| 0x10 | IMG_STATE | R | img_valid / crc_ok / boot_done / last_seq / pkt_count |
| 0x14 | CRC_RX | R | fabric 累计 CRC32 |
| 0x18 | WORD_CNT | R | 累计收到字数 |
| 0x1C | ITER_LIMIT | W | 32 位迭代上限 |
| 0x20 | SOLVER_STAT | R | {iter, verb, phase, running, done, err} |
| 0x24 | MASS_IDX | W | 待读向量索引 |
| 0x28/0x2C | MASS_DAT_LO/H | R | 该索引 64 位值 |
| 0x30/0x34 | DUMP_ADDR/LEN | W | 读回起始字与字数 |
| 0x38/0x3C | DUMP_DAT_LO/H | R | 该地址 64 位值 |
| 0x40 | LINK_STAT | R | link_up / speed / rx_frames |
| 0x44 | ERROR | R/W1C | crc_fail / missing_pkt / link_lost / ddr_err |
| 0x48 | HEARTBEAT | W | MCU 心跳 |
| 0x4C | VER | R | `0x47464F01` |

## 6 板的预期行为

| 阶段 | fabric | MCU | 板上可见 |
|---|---|---|---|
| 上电 | N25Q128 加载 bit + MCU bin；PLL 锁定；PHY 复位 | XIP 启动 | LED1（PLL）亮；2–3 s 后网口绿灯亮 |
| 链路建立 | ARP 应答、ICMP echo | 心跳 | LED2（链路）亮；PC ping 通 |
| 镜像下发 | 每包写 DDR2、累计 CRC、回 ACK | 读 ETH_HDR、写 PKT_ADDR、commit | ACT 橙灯间歇闪 |
| 镜像校验 | 比对 CRC 与字数 | 置 img_ready | FINACK 返回 status=0 |
| boot 构建 | s_build + diag_r | 轮询 boot_done | — |
| 启动 | 写 ITER_LIMIT、start_solve | 转发 | LED3（求解）亮 |
| 求解 | DRS 迭代；每 25 轮写 SOLVER_STAT | 组 STATUS 发 4662 | ACT 灯每 25 轮短闪 |
| 完成 | 置 done | 发末轮 STATUS | LED3 灭、LED4 亮 |
| 结果回传 | DDR2 取字 | 组 DUMP_DATA 发 4662 | ACT 连闪 0.3 s |

KEY0 = fabric 与 MCU 全复位。5 s 无 4661 包 → MCU 清位图、回等镜像态；求解中掉线只停 4662 上报。

## 7 数值内核

共享运算池：4 个 fp64 乘法器 + 5 个 fp64 加法器，全局请求/授权。各模块按微序列执行，与 `fp64_log/exp/rsqrt` 的分时复用方式一致。

片上保留 V/UT/U/RSK/G/DR/VPR 七个 3208 字向量（88 块 ERAM）。CB/BAND 工作区/DY/COO/band/Anderson 记忆放 DDR2。

时钟取 fp64_add 组合路径允许的最高值，目标 30–40 MHz，由第一次 TD 综合定。

## 8 发布物

| 文件 | 烧写位置 | 频率 |
|---|---|---|
| `gfold_ph1p.bit` | N25Q128 配置区 | 一次性 |
| `gfold_mcu.bin` | N25Q128 MCU 区（复位向量 0x20E00000） | 一次性 |
| `flash_image.bin`（303,096 B） | ETH 下发到 DDR2 | 每次运行 |

## 9 验证里程碑

| # | 内容 | 验收 |
|---|---|---|
| 1 | ETH 链路：PLL + RGMII + ARP 应答 + ICMP echo | PC ping 192.168.1.200 通；arp -a 出现 02:00:00:00:00:01 |
| 2 | UDP 环回：PING 命令 → ACK | gfold_eth.py ping 打印 rx_frames |
| 3 | 镜像加载 + CRC：211 包 LOAD + FINISH | FINACK status=0，CRC 匹配 |
| 4 | boot 构建：img_ready → boot_done | DUMP 读回 band/COO 与 golden 对拍 |
| 5 | 两轮 bit-exact：START{2} | check_drs.py worst rel ≤ 5e-10 |
| 6 | 50k 收敛 | final_mass ∈ [1790, 1800] |
| 7 | 掉线恢复 | 拔线 10 s 再插回，求解未中断 |
