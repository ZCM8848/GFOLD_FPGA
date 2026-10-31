// gfold_m1_top.v — 里程碑 1：ETH 链路（ARP 应答 + ICMP echo reply）
//
// 全部 IO 走千兆以太网。PC ping 192.168.1.200 应通。
//
// LED：
//   LED1 D10 = PLL lock
//   LED2 E10 = RX alive（收到过帧）
//   LED3 C6  = TX fired（发过应答）
//   LED4 D6  = 50MHz 心跳
//
`timescale 1ns/1ps

module gfold_m1_top (
    input  wire        I_sys_clk,        // C4  50MHz

    output wire [3:0]  O_led,            // D10/E10/C6/D6

    // RGMII TX
    output wire        O_rgmii_txc,      // U9
    output wire        O_rgmii_tx_ctl,   // R10
    output wire [3:0]  O_rgmii_txd,      // R11/P11/T10/R12

    // RGMII RX
    input  wire        I_rgmii_rxc,      // V3
    input  wire        I_rgmii_rx_ctl,   // N8
    input  wire [3:0]  I_rgmii_rxd,      // N9/T7/T8/T9

    output wire        O_rgmii_mdc,      // V9
    inout  wire        IO_rgmii_mdio,    // U10
    output wire        O_phy_rst_n       // V8
);
    // ==================================================================
    // 上电复位
    // ==================================================================
    reg [19:0] por_cnt;
    reg        sys_rst_n;
    initial begin
        por_cnt   = 20'd0;
        sys_rst_n = 1'b0;
    end
    always @(posedge I_sys_clk) begin
        if (por_cnt < 20'hF_FFFF) begin
            por_cnt   <= por_cnt + 1'b1;
            sys_rst_n <= 1'b0;
        end else begin
            sys_rst_n <= 1'b1;
        end
    end

    // ==================================================================
    // PLL：50MHz → 125MHz
    // ==================================================================
    wire clk_125;
    wire pll_lock;

    PLL u_pll (
        .refclk   ( I_sys_clk  ),
        .clk0_out ( clk_125    ),
        .clk1_out (            ),
        .clk4_out (            ),
        .clk5_out (            ),
        .lock     ( pll_lock   ),
        .reset    ( ~sys_rst_n )
    );

    wire rst_125_n = sys_rst_n & pll_lock;
    wire rst_rx_n  = sys_rst_n & pll_lock;

    // ==================================================================
    // PHY 硬复位
    // ==================================================================
    wire phy_rst_done;
    phy_rst #(
        .RESET_TIME_MS ( 2          ),
        .POST_RST_MS   ( 5          ),
        .CLK_FREQ_HZ   ( 50_000_000 )
    ) u_phy_rst (
        .clk       ( I_sys_clk   ),
        .rst_n     ( sys_rst_n   ),
        .phy_rst_n ( O_phy_rst_n ),
        .rst_done  ( phy_rst_done)
    );

    // ==================================================================
    // RX path（RX_CLK 域）
    // ==================================================================
    wire [7:0] rxd_gmii;
    wire       rx_dv, rx_er;

    rgmii_rx_phy u_rgmii_rx (
        .rgmii_rxc   ( I_rgmii_rxc   ),
        .rgmii_rx_ctl( I_rgmii_rx_ctl),
        .rgmii_rxd   ( I_rgmii_rxd   ),
        .gmii_rxd    ( rxd_gmii      ),
        .gmii_rx_dv  ( rx_dv         ),
        .gmii_rx_er  ( rx_er         )
    );

    wire        f_start, f_byte_v, f_end, f_err;
    wire [7:0]  f_data;
    wire [10:0] f_idx, f_len;

    eth_rx_mac u_rx_mac (
        .clk         ( I_rgmii_rxc ),
        .rst_n       ( rst_rx_n    ),
        .gmii_rxd    ( rxd_gmii    ),
        .gmii_rx_dv  ( rx_dv       ),
        .gmii_rx_er  ( rx_er       ),
        .frame_start ( f_start     ),
        .frame_data  ( f_data      ),
        .frame_idx   ( f_idx       ),
        .frame_byte_v( f_byte_v    ),
        .frame_end   ( f_end       ),
        .frame_len   ( f_len       ),
        .frame_err   ( f_err       )
    );

    // 协议解析
    wire        arp_req_rx, icmp_req_rx;
    wire [47:0] arp_mac_rx, icmp_mac_rx;
    wire [31:0] arp_ip_rx,  icmp_ip_rx;
    wire [15:0] icmp_id_rx, icmp_seq_rx;
    wire [6:0]  icmp_dlen_rx;
    wire        ram_wen;
    wire [6:0]  ram_waddr;
    wire [7:0]  ram_wdata;

    rx_proto #(.LOCAL_IP(32'hC0A8_01C8)) u_rx_proto (
        .clk            ( I_rgmii_rxc  ),
        .rst_n          ( rst_rx_n     ),
        .frame_start    ( f_start      ),
        .frame_data     ( f_data       ),
        .frame_idx      ( f_idx        ),
        .frame_byte_v   ( f_byte_v     ),
        .frame_end      ( f_end        ),
        .frame_len      ( f_len        ),
        .frame_err      ( f_err        ),
        .arp_req        ( arp_req_rx   ),
        .arp_sender_mac ( arp_mac_rx   ),
        .arp_sender_ip  ( arp_ip_rx    ),
        .icmp_req       ( icmp_req_rx  ),
        .icmp_sender_mac( icmp_mac_rx  ),
        .icmp_sender_ip ( icmp_ip_rx   ),
        .icmp_id        ( icmp_id_rx   ),
        .icmp_seq       ( icmp_seq_rx  ),
        .icmp_data_len  ( icmp_dlen_rx ),
        .ram_wen        ( ram_wen      ),
        .ram_waddr      ( ram_waddr    ),
        .ram_wdata      ( ram_wdata    )
    );

    // 双时钟 RAM（ICMP echo data 跨域）
    wire [6:0] ram_raddr;
    wire [7:0] ram_rdata;

    dc_ram_128x8 u_ram (
        .wclk  ( I_rgmii_rxc ),
        .wen   ( ram_wen     ),
        .waddr ( ram_waddr   ),
        .wdata ( ram_wdata   ),
        .raddr ( ram_raddr   ),
        .rdata ( ram_rdata   )
    );

    // ==================================================================
    // CDC：RX_CLK → clk_125（toggle 握手）
    // ==================================================================
    reg arp_tog_rx, icmp_tog_rx;
    always @(posedge I_rgmii_rxc or negedge rst_rx_n) begin
        if (!rst_rx_n) begin
            arp_tog_rx  <= 1'b0;
            icmp_tog_rx <= 1'b0;
        end else begin
            if (arp_req_rx)  arp_tog_rx  <= ~arp_tog_rx;
            if (icmp_req_rx) icmp_tog_rx <= ~icmp_tog_rx;
        end
    end

    reg [2:0] arp_tog_s, icmp_tog_s;
    always @(posedge clk_125) begin
        arp_tog_s  <= { arp_tog_s[1:0],  arp_tog_rx };
        icmp_tog_s <= { icmp_tog_s[1:0], icmp_tog_rx };
    end

    wire arp_req_tx  = arp_tog_s[2]  ^ arp_tog_s[1];
    wire icmp_req_tx = icmp_tog_s[2] ^ icmp_tog_s[1];

    // 锁存数据拷贝到 clk_125 域（toggle 边沿后数据已稳定 ≥2 拍）
    reg [47:0] arp_mac_125,  icmp_mac_125;
    reg [31:0] arp_ip_125,   icmp_ip_125;
    reg [15:0] icmp_id_125,  icmp_seq_125;
    reg [6:0]  icmp_dlen_125;

    always @(posedge clk_125) begin
        if (arp_req_tx) begin
            arp_mac_125 <= arp_mac_rx;
            arp_ip_125  <= arp_ip_rx;
        end
        if (icmp_req_tx) begin
            icmp_mac_125  <= icmp_mac_rx;
            icmp_ip_125   <= icmp_ip_rx;
            icmp_id_125   <= icmp_id_rx;
            icmp_seq_125  <= icmp_seq_rx;
            icmp_dlen_125 <= icmp_dlen_rx;
        end
    end

    // ==================================================================
    // TX path（clk_125 域）
    // ==================================================================
    wire        arp_start, icmp_start;
    wire        arp_busy,  icmp_busy;
    wire        arp_done,  icmp_done;
    wire [7:0]  arp_txd,   icmp_txd;
    wire        arp_en,    icmp_en;

    tx_arbiter u_arb (
        .clk        ( clk_125    ),
        .rst_n      ( rst_125_n  ),
        .arp_req    ( arp_req_tx ),
        .icmp_req   ( icmp_req_tx),
        .arp_start  ( arp_start  ),
        .icmp_start ( icmp_start ),
        .arp_busy   ( arp_busy   ),
        .icmp_busy  ( icmp_busy  ),
        .tx_busy    (            )
    );

    arp_reply_tx #(
        .LOCAL_MAC ( 48'h02_00_00_00_00_01 ),
        .LOCAL_IP  ( 32'hC0A8_01C8         )
    ) u_arp_tx (
        .clk        ( clk_125     ),
        .rst_n      ( rst_125_n   ),
        .req        ( arp_start   ),
        .dst_mac    ( arp_mac_125 ),
        .dst_ip     ( arp_ip_125  ),
        .busy       ( arp_busy    ),
        .done       ( arp_done    ),
        .gmii_txd   ( arp_txd     ),
        .gmii_tx_en ( arp_en      )
    );

    icmp_reply_tx #(
        .LOCAL_MAC ( 48'h02_00_00_00_00_01 ),
        .LOCAL_IP  ( 32'hC0A8_01C8         )
    ) u_icmp_tx (
        .clk        ( clk_125      ),
        .rst_n      ( rst_125_n    ),
        .req        ( icmp_start   ),
        .dst_mac    ( icmp_mac_125 ),
        .dst_ip     ( icmp_ip_125  ),
        .icmp_id    ( icmp_id_125  ),
        .icmp_seq   ( icmp_seq_125 ),
        .data_len   ( icmp_dlen_125),
        .ram_raddr  ( ram_raddr    ),
        .ram_rdata  ( ram_rdata    ),
        .busy       ( icmp_busy    ),
        .done       ( icmp_done    ),
        .gmii_txd   ( icmp_txd     ),
        .gmii_tx_en ( icmp_en      )
    );

    // TX mux（arp 优先；同一时刻只有一个 busy）
    wire [7:0] gmii_txd   = arp_busy ? arp_txd : icmp_txd;
    wire       gmii_tx_en = arp_busy ? arp_en  : icmp_en;

    rgmii_tx_phy u_rgmii_tx (
        .clk_125    ( clk_125    ),
        .clk_125_90 ( clk_125    ),
        .rst        ( ~rst_125_n ),
        .gmii_txd   ( gmii_txd   ),
        .gmii_tx_en ( gmii_tx_en ),
        .gmii_tx_er ( 1'b0       ),
        .rgmii_txc  ( O_rgmii_txc    ),
        .rgmii_tx_ctl( O_rgmii_tx_ctl),
        .rgmii_txd  ( O_rgmii_txd    )
    );

    // ==================================================================
    // LED
    // ==================================================================
    reg rx_alive;
    always @(posedge I_rgmii_rxc or negedge rst_rx_n) begin
        if (!rst_rx_n)            rx_alive <= 1'b0;
        else if (f_end && !f_err) rx_alive <= 1'b1;
    end

    reg tx_fired;
    always @(posedge clk_125 or negedge rst_125_n) begin
        if (!rst_125_n)                   tx_fired <= 1'b0;
        else if (arp_done | icmp_done)    tx_fired <= 1'b1;
    end

    reg [25:0] hb_cnt;
    always @(posedge I_sys_clk) hb_cnt <= hb_cnt + 1'b1;

    assign O_led[0] = pll_lock;
    assign O_led[1] = rx_alive;
    assign O_led[2] = tx_fired;
    assign O_led[3] = hb_cnt[25];

    // ==================================================================
    // 不用
    // ==================================================================
    assign O_rgmii_mdc   = 1'b0;
    assign IO_rgmii_mdio = 1'bz;

endmodule
