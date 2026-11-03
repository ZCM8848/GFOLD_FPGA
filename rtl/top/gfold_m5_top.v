// gfold_m5_top.v �?PH1P35 milestone 5: ETH + on-chip small-problem DRS solver.
// The host loads the small2 boot image over the existing LOAD protocol, sends
// START (0x05), waits, then DUMPs the 120-word VS1/VS2 buffer in 10-word chunks.
`timescale 1ns/1ps
module gfold_m5_top #(
    parameter RY_FILE = "small2_r_y.hex",
    parameter DY_FILE = "small2_Dy.hex"
)(
    input  wire        I_sys_clk,
    output wire [3:0]  O_led,
    output wire        O_rgmii_txc, O_rgmii_tx_ctl,
    output wire [3:0]  O_rgmii_txd,
    input  wire        I_rgmii_rxc, I_rgmii_rx_ctl,
    input  wire [3:0]  I_rgmii_rxd,
    output wire        O_rgmii_mdc,
    inout  wire        IO_rgmii_mdio,
    output wire        O_phy_rst_n
);
    reg [19:0] por_cnt; reg sys_rst_n;
    initial begin por_cnt=20'd0; sys_rst_n=1'b0; end
    always @(posedge I_sys_clk) begin
        if (por_cnt<20'hF_FFFF) begin por_cnt<=por_cnt+1'b1; sys_rst_n<=1'b0; end
        else sys_rst_n<=1'b1;
    end

    wire clk_125, clk_sol_raw, pll_lock;
    PLL u_pll(.refclk(I_sys_clk),.clk0_out(clk_125),.clk1_out(clk_sol_raw),
              .clk4_out(),.clk5_out(),.lock(pll_lock),.reset(~sys_rst_n));
    wire clk_sol;
    PH1P_LOGIC_BUFG u_bufg_sol(.i(clk_sol_raw), .o(clk_sol));
    wire rst_125_n=sys_rst_n&pll_lock, rst_rx_n=sys_rst_n&pll_lock;

    wire phy_rst_done;
    phy_rst #(.RESET_TIME_MS(2),.POST_RST_MS(5),.CLK_FREQ_HZ(50_000_000)) u_phy_rst(
        .clk(I_sys_clk),.rst_n(sys_rst_n),.phy_rst_n(O_phy_rst_n),.rst_done(phy_rst_done));

    // ---------------- Ethernet RX ----------------
    wire [7:0] rxd_gmii; wire rx_dv,rx_er;
    rgmii_rx_phy u_rgmii_rx(.rgmii_rxc(I_rgmii_rxc),.rgmii_rx_ctl(I_rgmii_rx_ctl),.rgmii_rxd(I_rgmii_rxd),.gmii_rxd(rxd_gmii),.gmii_rx_dv(rx_dv),.gmii_rx_er(rx_er));
    wire f_start,f_byte_v,f_end,f_err; wire [7:0] f_data; wire [10:0] f_idx,f_len;
    eth_rx_mac u_rx_mac(.clk(I_rgmii_rxc),.rst_n(rst_rx_n),.gmii_rxd(rxd_gmii),.gmii_rx_dv(rx_dv),.gmii_rx_er(rx_er),.frame_start(f_start),.frame_data(f_data),.frame_idx(f_idx),.frame_byte_v(f_byte_v),.frame_end(f_end),.frame_len(f_len),.frame_err(f_err));

    wire arp_req_rx,icmp_req_rx,udp_cmd_req_rx,dump_req_rx;
    wire [47:0] arp_mac_rx,icmp_mac_rx,udp_mac_rx;
    wire [31:0] arp_ip_rx,icmp_ip_rx,udp_ip_rx;
    wire [15:0] icmp_id_rx,icmp_seq_rx; wire [6:0] icmp_dlen_rx;
    wire [15:0] udp_src_port_rx,udp_app_seq_rx; wire [7:0] udp_app_type_rx;
    wire [15:0] rx_frame_cnt_rx;
    wire ram_wen; wire [6:0] ram_waddr; wire [7:0] ram_wdata;
    wire [31:0] fin_actual_crc_rx,fin_actual_words_rx; wire [7:0] fin_status_rx; wire img_valid_rx;
    wire img_wen; wire [9:0] img_waddr; wire [63:0] img_wdata;
    wire [9:0] dump_start_rx; wire [3:0] dump_nwords_rx;

    rx_proto #(.LOCAL_IP(32'hC0A8_01C8),.CMD_PORT(16'd4661)) u_rx_proto(
        .clk(I_rgmii_rxc),.rst_n(rst_rx_n),
        .frame_start(f_start),.frame_data(f_data),.frame_idx(f_idx),.frame_byte_v(f_byte_v),.frame_end(f_end),.frame_len(f_len),.frame_err(f_err),
        .arp_req(arp_req_rx),.arp_sender_mac(arp_mac_rx),.arp_sender_ip(arp_ip_rx),
        .icmp_req(icmp_req_rx),.icmp_sender_mac(icmp_mac_rx),.icmp_sender_ip(icmp_ip_rx),
        .icmp_id(icmp_id_rx),.icmp_seq(icmp_seq_rx),.icmp_data_len(icmp_dlen_rx),
        .udp_cmd_req(udp_cmd_req_rx),.udp_sender_mac(udp_mac_rx),.udp_sender_ip(udp_ip_rx),
        .udp_src_port(udp_src_port_rx),.udp_app_type(udp_app_type_rx),.udp_app_seq(udp_app_seq_rx),
        .ram_wen(ram_wen),.ram_waddr(ram_waddr),.ram_wdata(ram_wdata),
        .rx_frame_cnt(rx_frame_cnt_rx),
        .fin_actual_crc(fin_actual_crc_rx),.fin_actual_words(fin_actual_words_rx),.fin_status(fin_status_rx),.img_valid(img_valid_rx),
        .img_wen(img_wen),.img_waddr(img_waddr),.img_wdata(img_wdata),
        .dump_req(dump_req_rx),.dump_start_out(dump_start_rx),.dump_nwords_out(dump_nwords_rx));

    wire [6:0] ram_raddr; wire [7:0] ram_rdata;
    dc_ram_128x8 u_ram(.wclk(I_rgmii_rxc),.wen(ram_wen),.waddr(ram_waddr),.wdata(ram_wdata),.raddr(ram_raddr),.rdata(ram_rdata));

    // ---------------- CDC of command requests ----------------
    reg arp_tog_rx,icmp_tog_rx,udp_tog_rx,dump_tog_rx;
    always @(posedge I_rgmii_rxc or negedge rst_rx_n) begin
        if(!rst_rx_n)begin arp_tog_rx<=0;icmp_tog_rx<=0;udp_tog_rx<=0;dump_tog_rx<=0; end
        else begin
            if(arp_req_rx)arp_tog_rx<=~arp_tog_rx;
            if(icmp_req_rx)icmp_tog_rx<=~icmp_tog_rx;
            if(udp_cmd_req_rx)udp_tog_rx<=~udp_tog_rx;
            if(dump_req_rx)dump_tog_rx<=~dump_tog_rx;
        end
    end
    reg [2:0] arp_tog_s,icmp_tog_s,udp_tog_s,dump_tog_s;
    always @(posedge clk_125) begin
        arp_tog_s<={arp_tog_s[1:0],arp_tog_rx};
        icmp_tog_s<={icmp_tog_s[1:0],icmp_tog_rx};
        udp_tog_s<={udp_tog_s[1:0],udp_tog_rx};
        dump_tog_s<={dump_tog_s[1:0],dump_tog_rx};
    end
    wire arp_req_tx=arp_tog_s[2]^arp_tog_s[1];
    wire icmp_req_tx=icmp_tog_s[2]^icmp_tog_s[1];
    wire udp_req_tx=udp_tog_s[2]^udp_tog_s[1];
    wire dump_req_tx=dump_tog_s[2]^dump_tog_s[1];

    // ---------------- solver arming / image RAM ----------------
    reg solver_armed;
    always @(posedge clk_125 or negedge rst_125_n) begin
        if(!rst_125_n) solver_armed<=0;
        else if(udp_req_tx && udp_app_type_rx==8'h05) solver_armed<=1;   // START
        else if(udp_req_tx && udp_app_type_rx==8'h09) solver_armed<=0;   // RESET re-arms
    end
    wire rst_sol_n = sys_rst_n & pll_lock & solver_armed;
    // synchronize the reset release into clk_sol (2 FF)
    reg [1:0] rst_sol_sync;
    always @(posedge clk_sol or negedge rst_sol_n) begin
        if (!rst_sol_n) rst_sol_sync <= 2'b00;
        else rst_sol_sync <= {rst_sol_sync[0], 1'b1};
    end
    wire rst_sol_sync_n = rst_sol_sync[1];

    wire [9:0] img_raddr; wire [63:0] img_rdata_sol;
    image_ram u_img_ram(.wclk(I_rgmii_rxc),.wen(img_wen),.waddr(img_waddr),.wdata(img_wdata),
                        .rclk(clk_sol),.raddr(img_raddr),.rdata(img_rdata_sol));

    wire vs_we; wire [7:0] vs_waddr; wire [63:0] vs_wdata;
    wire solver_done;
    solver_core #(.RY_FILE(RY_FILE), .DY_FILE(DY_FILE)) u_solver(
        .clk(clk_sol), .rst_n(rst_sol_sync_n), .start(1'b1), .done(solver_done),
        .img_addr(img_raddr), .img_rdata(img_rdata_sol),
        .vs_we(vs_we), .vs_waddr(vs_waddr), .vs_wdata(vs_wdata));

    // VS buffer: solver write / dump read
    wire [9:0] dump_ram_raddr; wire [63:0] dump_ram_rdata;
    image_ram u_vs_ram(.wclk(clk_sol),.wen(vs_we),.waddr({2'b0,vs_waddr}),.wdata(vs_wdata),
                       .rclk(clk_125),.raddr(dump_ram_raddr),.rdata(dump_ram_rdata));

    // ---------------- TX side ----------------
    reg [47:0] arp_mac_125,icmp_mac_125,udp_mac_125,dump_mac_125;
    reg [31:0] arp_ip_125,icmp_ip_125,udp_ip_125,dump_ip_125;
    reg [15:0] icmp_id_125,icmp_seq_125;
    reg [6:0] icmp_dlen_125;
    reg [15:0] udp_port_125,udp_seq_125;
    reg [7:0] udp_ack_type_r;
    reg [31:0] udp_payload_w0_r,udp_payload_w1_r;
    reg [15:0] dump_port_125;
    reg [9:0] dump_start_125;
    reg [3:0] dump_nwords_125;
    reg solver_done_s1,solver_done_s2;
    always @(posedge clk_125) begin solver_done_s1<=solver_done; solver_done_s2<=solver_done_s1; end

    always @(posedge clk_125) begin
        if(arp_req_tx)begin arp_mac_125<=arp_mac_rx; arp_ip_125<=arp_ip_rx; end
        if(icmp_req_tx)begin icmp_mac_125<=icmp_mac_rx; icmp_ip_125<=icmp_ip_rx; icmp_id_125<=icmp_id_rx; icmp_seq_125<=icmp_seq_rx; icmp_dlen_125<=icmp_dlen_rx; end
        if(udp_req_tx) begin
            udp_mac_125<=udp_mac_rx; udp_ip_125<=udp_ip_rx; udp_port_125<=udp_src_port_rx; udp_seq_125<=udp_app_seq_rx;
            case(udp_app_type_rx)
                8'h0A:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,rx_frame_cnt_rx}; udp_payload_w1_r<={31'd0,solver_done_s2}; end
                8'h01:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<={31'd0,img_valid_rx}; end
                8'h03:begin udp_ack_type_r<=8'h04; udp_payload_w0_r<=fin_actual_crc_rx; udp_payload_w1_r<={fin_actual_words_rx[15:8],fin_actual_words_rx[7:0],8'd0,fin_status_rx}; end
                8'h05:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<={31'd0,solver_armed}; end
                8'h09:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<=32'd0; end
                default:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<=32'd0; end
            endcase
        end
        if(dump_req_tx) begin
            dump_mac_125<=udp_mac_rx; dump_ip_125<=udp_ip_rx; dump_port_125<=udp_src_port_rx;
            dump_start_125<=dump_start_rx; dump_nwords_125<=dump_nwords_rx;
        end
    end

    wire arp_start,icmp_start,udp_start,dump_start;
    wire arp_busy,icmp_busy,udp_busy,dump_busy;
    wire arp_done,icmp_done,udp_done,dump_done;
    wire [7:0] arp_txd,icmp_txd,udp_txd,dump_txd;
    wire arp_en,icmp_en,udp_en,dump_en;

    tx_arbiter u_arb(.clk(clk_125),.rst_n(rst_125_n),
        .arp_req(arp_req_tx),.icmp_req(icmp_req_tx),.udp_req(udp_req_tx),.dump_req(dump_req_tx),
        .arp_start(arp_start),.icmp_start(icmp_start),.udp_start(udp_start),.dump_start(dump_start),
        .arp_busy(arp_busy),.icmp_busy(icmp_busy),.udp_busy(udp_busy),.dump_busy(dump_busy),.tx_busy());

    arp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8)) u_arp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(arp_start),.dst_mac(arp_mac_125),.dst_ip(arp_ip_125),.busy(arp_busy),.done(arp_done),.gmii_txd(arp_txd),.gmii_tx_en(arp_en));

    icmp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8)) u_icmp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(icmp_start),.dst_mac(icmp_mac_125),.dst_ip(icmp_ip_125),.icmp_id(icmp_id_125),.icmp_seq(icmp_seq_125),.data_len(icmp_dlen_125),.ram_raddr(ram_raddr),.ram_rdata(ram_rdata),.busy(icmp_busy),.done(icmp_done),.gmii_txd(icmp_txd),.gmii_tx_en(icmp_en));

    udp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8),.CMD_PORT(16'd4661)) u_udp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(udp_start),.dst_mac(udp_mac_125),.dst_ip(udp_ip_125),.dst_port(udp_port_125),.ack_type(udp_ack_type_r),.ack_seq(udp_seq_125),.payload_w0(udp_payload_w0_r),.payload_w1(udp_payload_w1_r),.busy(udp_busy),.done(udp_done),.gmii_txd(udp_txd),.gmii_tx_en(udp_en));

    dump_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8),.CMD_PORT(16'd4661)) u_dump_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(dump_start),.dst_mac(dump_mac_125),.dst_ip(dump_ip_125),.dst_port(dump_port_125),.dump_start(dump_start_125),.dump_nwords(dump_nwords_125),.ram_raddr(dump_ram_raddr),.ram_rdata(dump_ram_rdata),.busy(dump_busy),.done(dump_done),.gmii_txd(dump_txd),.gmii_tx_en(dump_en));

    wire [7:0] gmii_txd = arp_busy?arp_txd:(icmp_busy?icmp_txd:(udp_busy?udp_txd:dump_txd));
    wire gmii_tx_en = arp_busy?arp_en:(icmp_busy?icmp_en:(udp_busy?udp_en:dump_en));

    rgmii_tx_phy u_rgmii_tx(.clk_125(clk_125),.clk_125_90(clk_125),.rst(~rst_125_n),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_tx_er(1'b0),.rgmii_txc(O_rgmii_txc),.rgmii_tx_ctl(O_rgmii_tx_ctl),.rgmii_txd(O_rgmii_txd));

    // LEDs: lock, solver done, armed, heartbeat
    reg [25:0] hb_cnt; always @(posedge I_sys_clk) hb_cnt<=hb_cnt+1'b1;
    assign O_led[0]=pll_lock; assign O_led[1]=solver_done_s2; assign O_led[2]=solver_armed; assign O_led[3]=hb_cnt[25];
    assign O_rgmii_mdc=1'b0; assign IO_rgmii_mdio=1'bz;
endmodule
