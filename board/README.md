# DE2-115 Board Reference (GFOLD_FPGA)

Target FPGA: **EP4CE115F29C7** (Cyclone IV E, DE2-115 by Terasic, 780-pin FBGA).

## Resources (this folder)
- `DE2_115.qsf` — authoritative Quartus pin assignments (1205 lines, full board).
- `de2_115_schematic.pdf` + `.txt` — full board schematic (extracted text).

## Verified key pins
| Signal | Pin | I/O std |
|---|---|---|
| CLOCK_50 | PIN_Y2 | 3.3-V LVTTL |
| UART_TXD (FPGA->DB9) | PIN_G9 | 3.3-V LVTTL |
| UART_RXD (FPGA<-DB9) | PIN_G12 | 3.3-V LVTTL |
| LEDG[0] (status LED) | PIN_E21 | 2.5 V |

## I/O path (VERIFIED against schematic — corrected 2026-08-21)
- **NO user-accessible USB virtual COM on the DE2-115.** USB Blaster = JTAG
  programming only; USB OTG (ISP1362) = device/host mode, not a serial bridge.
- **Data path = DB-9 RS-232 (soft UART)**: FPGA implements the UART (Verilog) ->
  MAX3232 transceiver -> DB-9 connector -> **USB-to-RS-232 adapter** -> PC.
- On the PC, pyserial talks to the USB-adapter's virtual COM port (same as any COM).
- PC-side: `pip install pyserial`; `import serial`; `serial.Serial('COMx', 115200)`.
  Binary framing protocol is our design (Phase-3 top-level).

## Clock / resource notes
- 50 MHz onboard oscillator (CLOCK_50, PIN_Y2); use PLL to multiply if needed.
- NO external SDRAM needed: banded KKT (0.15 MB) fits fully in on-chip M9K
  (486 KB). Saves the entire SDRAM interface.
- On-chip budget: 114,480 LE, 3,888 Kbit (486 KB), 266 x 18x18 multipliers.
