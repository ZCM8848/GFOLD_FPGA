// rgmii_rx_phy.v — RGMII RX 物理层（HD_IDDR 原语）
//
// Anlogic IDDR 语义（TD 原语模板 ph1p_i_o.v）：
//   q0 = 上升沿采样数据  → RGMII 的**低 nibble / RX_DV**
//   q1 = 下降沿采样数据  → RGMII 的**高 nibble / (RX_DV XOR RX_ER)**
//
// 注意：厂商 rgmii_interface.v 把 q0 接低 nibble，与上面一致。
// （旧 rgmii_ddr.v 里把 q1 当低 nibble，是错的，本模块不沿用。）
//
`timescale 1ns/1ps

module rgmii_rx_phy (
    input  wire        rgmii_rxc,      // PHY 送来的 125 MHz
    input  wire        rgmii_rx_ctl,   // RX_DV / RX_ER 复用
    input  wire [3:0]  rgmii_rxd,

    output wire [7:0]  gmii_rxd,
    output wire        gmii_rx_dv,
    output wire        gmii_rx_er
);

    wire [3:0] q0_rxd, q1_rxd;

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_rxd
            PH1P_LOGIC_HD_IDDR #(.MODE("OPPOSITE_EDGE")) u_iddr (
                .q1  ( q1_rxd[i]   ),
                .q0  ( q0_rxd[i]   ),
                .clk ( rgmii_rxc   ),
                .d   ( rgmii_rxd[i]),
                .rst ( 1'b0        )
            );
        end
    endgenerate

    wire q0_ctl, q1_ctl;
    PH1P_LOGIC_HD_IDDR #(.MODE("OPPOSITE_EDGE")) u_iddr_ctl (
        .q1  ( q1_ctl       ),
        .q0  ( q0_ctl       ),
        .clk ( rgmii_rxc    ),
        .d   ( rgmii_rx_ctl ),
        .rst ( 1'b0         )
    );

    // 上升沿那一拍同时拿到低 nibble(q0) 与本字节此前下降沿采到的高 nibble(q1)
    assign gmii_rxd   = { q1_rxd, q0_rxd };
    assign gmii_rx_dv = q0_ctl;
    assign gmii_rx_er = q0_ctl ^ q1_ctl;   // ER = DV xor (DV^ER)

endmodule
