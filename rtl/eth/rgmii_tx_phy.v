// rgmii_tx_phy.v — RGMII TX 物理层（HD_ODDR 原语）
//
// 关键：Anlogic ODDR 的 d0 对应 **时钟上升沿**，d1 对应 **下降沿**
//       （依据厂商 RTL：32_Ethernet_test/src/rgmii_interface.v
//         注释“上升沿发送低四位”，代码 .d0(gmii_txd[i]) .d1(gmii_txd_falling[i])）
//
// 因此 RGMII v2.0 映射：
//   TXD : d0 = gmii_txd[3:0]（上升沿，低 nibble）
//         d1 = gmii_txd[7:4]（下降沿，高 nibble）
//   CTL : d0 = TX_EN（上升沿）
//         d1 = TX_ER（下降沿）
//   TXC : d0 = 1, d1 = 0
//         时钟用 **125MHz 0°**（与厂商 udp_transmit_test.v 一致：
//         .gtx_clk(temac_clk) / .gtx_clk_90(temac_clk) 是同一个时钟）。
//         PHY 内部再有 0.75~1.25ns 的 Tx_clk_delay（EXT 0x0C bit[7:4] 默认 5）。
//         之前用 PLL C1 的 90° 输出，但 C1 实际未起振 → GTX_CLK 恒为 0V。
//
// 所有 RGMII 引脚都在 HDIO bank，必须用 PH1P_LOGIC_HD_ODDR
// （用 PH1_LOGIC_ODDR 会报 SYN-8228 can't be mapped into HDIO）
//
`timescale 1ns/1ps

module rgmii_tx_phy (
    input  wire        clk_125,        // 125 MHz —— 驱动数据与 GTX_CLK
    input  wire        clk_125_90,     // 125 MHz 90°（当前不使用，保留接口）
    input  wire        rst,            // 高有效复位（ODDR 同步复位）

    input  wire [7:0]  gmii_txd,
    input  wire        gmii_tx_en,
    input  wire        gmii_tx_er,

    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd
);

    // ------------------------------------------------------------------
    // GTX_CLK：125MHz 0° + ODDR，与数据同沿（厂商方案）
    // ------------------------------------------------------------------
    PH1P_LOGIC_HD_ODDR u_oddr_txc (
        .q   ( rgmii_txc ),
        .clk ( clk_125_90 ),
        .d0  ( 1'b1 ),          // 上升沿 → 1
        .d1  ( 1'b0 ),          // 下降沿 → 0
        .rst ( rst )
    );

    // ------------------------------------------------------------------
    // TXD[3:0]：DDR
    // ------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_txd
            PH1P_LOGIC_HD_ODDR u_oddr_txd (
                .q   ( rgmii_txd[i]     ),
                .clk ( clk_125          ),
                .d0  ( gmii_txd[i]      ),   // 上升沿 = 低 nibble
                .d1  ( gmii_txd[i+4]    ),   // 下降沿 = 高 nibble
                .rst ( rst )
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // TX_CTL：RGMII 规定
    //   上升沿 = TX_EN
    //   下降沿 = TX_EN XOR TX_ER   ← 不是 TX_ER 本身！
    // 厂商 rgmii_interface.v：d0 = tx_en_to_ddr, d1 = gmii_tx_en ^ gmii_tx_er
    // 若下降沿直接送 TX_ER(=0)，PHY 会解出 TX_ER=1，整帧被判为 error frame 丢弃。
    // ------------------------------------------------------------------
    wire tx_ctl_falling = gmii_tx_en ^ gmii_tx_er;

    PH1P_LOGIC_HD_ODDR u_oddr_txctl (
        .q   ( rgmii_tx_ctl    ),
        .clk ( clk_125         ),
        .d0  ( gmii_tx_en      ),
        .d1  ( tx_ctl_falling  ),
        .rst ( rst )
    );

endmodule
