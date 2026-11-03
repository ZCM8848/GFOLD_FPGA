# LED 映射确认 bit

## 目的

实测确认 `O_led[3:0]`（D10/E10/C6/D6）与板载丝印 LED1..LED4 的对应关系。板级文档
`HW_HX1P35A_BOARD.md §9` / `HW_PINMAP.md` 的映射是 D10→LED1、E10→LED2、
C6→LED3、D6→LED4，本 bit 用于独立验证。

## Bit

```
anlogic/td_project/phy_led/gfold_led.bit
```

| 项 | 值 |
|---|---|
| 大小 | 1,789,487 bytes |
| SHA256 | `A429CFB6CFF86116DBCFA4D0BBDD9DB8F0EB3C8AD7D74946BF434A7241E59C7D` |
| 生成时间 | 2026-11-03 21:43:28 |
| 顶层 | `led_walk_top` |
| 时序 | SWNS +14.126 ns, HWNS +0.171 ns |

## 行为

上电后四个用户 LED **依次单独点亮**，每步约 **1.34 s**（50 MHz / 2^26），循环：

| 步 | 点亮的 FPGA 引脚 | 设计信号 | 板级文档称 |
|---|---|---|---|
| 0 | D10 | `O_led[0]` | LED1 |
| 1 | E10 | `O_led[1]` | LED2 |
| 2 | C6  | `O_led[2]` | LED3 |
| 3 | D6  | `O_led[3]` | LED4 |

LED14 是电源指示灯，始终亮，与测试无关。

## 判读

观察“第 1 个亮的是哪个丝印、第 2 个、第 3 个、第 4 个”：

- 若顺序是 **LED1 → LED2 → LED3 → LED4**：板级文档正确，
  M6 DDR2 smoke 的 LED 判读为：LED1=init OK、LED2=pass 灭、LED3=fail 亮、
  LED4=heartbeat 闪 → 回环失败。
- 若顺序不同（例如 LED4 → LED3 → LED2 → LED1，或某个普通 LED 不对应）：
  说明 `pin.adc` 或板级丝印有误，需要按实测顺序重新解释 DDR2 smoke 的 LED。

## 构建

```powershell
cd anlogic\td_project\syn_led
td_commands_prompt.exe .../DefaultFlow.tcl
cd ..\phy_led
td_commands_prompt.exe .../DefaultFlow.tcl
```

`syn_led`/`phy_led` 只包含 `led_walk_top.v` 和 `pin_ddr2.adc`，无 DDR2 IP，构建约 3 秒。
