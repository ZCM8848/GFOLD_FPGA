# M6 上板流程（ETH LOAD + 全问题 50k）

日期：2026-11-04 | 分支：`anlogic-ph1p35`

## Bit

| Bit | 用途 | SHA256 |
|---|---|---|
| `anlogic/td_project/phy_ddr2_sram/gfold_ddr2_sram.bit` | DDR2 64-bit 字映射自检 | `3A72B1BCF4C108E6ECD6D58196DB9F29701DF891F5CB1FD926E2F11E20EAF3F9` |
| `anlogic/td_project/phy_m6/gfold_m6.bit` | 完整问题 ETH+DDR2 求解 | `81834B786348FCC82977DECA5EAB3C8430D1090CB6724559269C3E592EFB2106` |

完整 bit：1,789,533 B；slice 17,857/21,216（84.2%），ERAM 104/108，
DSP 40/40；setup WNS **+0.118 ns**，hold 仅有 RGMII RX 输入 -0.422 ns
（INDEL=4 未被 STA 建模，M5 同样报告，板级正常）。

## DDR2 字映射自检（先烧）

`gfold_ddr2_sram.bit`：

- LED1（D10）= DDR2 init/校准完成，应常亮；
- LED2（E10）= 16 个 64-bit 字写/读一致，应常亮；
- LED3（C6）= FAIL，应灭；
- LED4（D6）= 心跳闪烁。

正常 = LED1 + LED2 常亮，LED3 灭，LED4 闪。
如果 LED3 亮，说明 64-bit 字 → 128-bit beat 的地址/数据映射不对，
不要烧完整问题 bit。

## 完整问题 bit

`gfold_m6.bit` 顶层 `gfold_m6_top`：

- 25/31.25 MHz solver（125/4），DDR2 用户时钟由 IP 提供；
- ETH LOAD 把 303,096 B `flash_image.bin` 写入 DDR2 地址 0；
- LOAD 包经 `async_fifo_256x64` 缓冲，`img_ddr_writer` 每个 64-bit 字写
  一个 128-bit beat（地址 `word*8`，数据复制到高低 64 位）；
- START 后 `drs_iter` 从 DDR2 读 boot image，写 COO/band 到外部存储，
  跑 50,000 轮；
- 末轮 v[1099]（log final mass）由 type 0x0B 查询返回 64 位值；
  type 0x0C 返回 `{iter, run}`。

## 主机命令

```powershell
cd host
python gfold_eth.py ping 5
python gfold_eth.py load ..\rtl\data\flash\flash_image.bin
python gfold_eth.py start
python gfold_eth.py mass
```

一条命令完成 load + start + 轮询 50k：

```powershell
python gfold_eth.py run50k ..\rtl\data\flash\flash_image.bin 7200
```

`run50k` 每 10 s DUMP 2 个字，打印 `iter`、原始 v[1099]、log 和 exp；
当 `iter >= 49999` 时打印：

```
RUN COMPLETE iter=... final_mass_log=... final_mass=...
```

验收标准：`final_mass ∈ [1790, 1800]`（参考 1795.14 / 最优 1799.156）。

## 注意

- `gfold_eth.py` 在 LOAD 每包后等 2 ms，给 DDR2 写管道排空；不要在
  LOAD 过程中打断。
- RESET 后等 1 s 再 LOAD，给 DDR2 校准留时间。
- 50k 运行时间取决于 DDR2 访问延迟；`run50k` 默认超时 7200 s。
- 如果 `mass` 一直是 0 或 boot 不启动，先检查 FINACK 是否 CRC PASS，
  再检查 `gfold_ddr2_sram.bit` 的映射自检。
