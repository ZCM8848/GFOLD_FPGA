// gfold_m6_top.v -- PH1P35 M6: full G-FOLD solver, DDR2 external memory,
// and the M5 Ethernet LOAD/START/DUMP control plane.
`timescale 1ns/1ps
module gfold_m6_top (
    input  wire        I_sys_clk,
    output wire [3:0]  O_led,
    output wire        O_rgmii_txc, O_rgmii_tx_ctl,
    output wire [3:0]  O_rgmii_txd,
    input  wire        I_rgmii_rxc, I_rgmii_rx_ctl,
    input  wire [3:0]  I_rgmii_rxd,
    output wire        O_rgmii_mdc,
    inout  wire        IO_rgmii_mdio,
    output wire        O_phy_rst_n,
    // DDR2 pins
    output wire [12:0] ddr_addr,
    output wire [1:0]  ddr_ba,
    output wire        ddr_cke,
    output wire        ddr_odt,
    output wire        ddr_cs_n,
    output wire        ddr_ras_n,
    output wire        ddr_cas_n,
    output wire        ddr_we_n,
    output wire        ddr_ck_p,
    output wire        ddr_ck_n,
    inout  wire [1:0]  ddr_dm,
    inout  wire [15:0] ddr_dq,
    inout  wire [1:0]  ddr_dqs_p,
    inout  wire [1:0]  ddr_dqs_n
);
    reg [19:0] por_cnt; reg sys_rst_n;
    initial begin por_cnt=20'd0; sys_rst_n=1'b0; end
    always @(posedge I_sys_clk) begin
        if (por_cnt<20'hF_FFFF) begin por_cnt<=por_cnt+1'b1; sys_rst_n<=1'b0; end
        else sys_rst_n<=1'b1;
    end

    wire clk_125, pll_lock;
    PLL u_pll(.refclk(I_sys_clk),.clk0_out(clk_125),.clk1_out(),
              .clk4_out(),.clk5_out(),.lock(pll_lock),.reset(~sys_rst_n));
    wire rst_125_n=sys_rst_n&pll_lock, rst_rx_n=sys_rst_n&pll_lock;

    // Solver clock: 125/4 = 31.25 MHz, generated in fabric and driven through
    // a global clock buffer (the PLL C1 output is not recognised as a clock
    // net by the timing engine on this device/IP combination).
    reg [1:0] sol_div;
    always @(posedge clk_125 or negedge rst_125_n) begin
        if (!rst_125_n) sol_div <= 2'b00;
        else sol_div <= sol_div + 1'b1;
    end
    wire clk_sol_raw = sol_div[1];
    wire clk_sol;
    PH1P_LOGIC_BUFG u_bufg_sol(.i(clk_sol_raw), .o(clk_sol));

    wire phy_rst_done;
    phy_rst #(.RESET_TIME_MS(2),.POST_RST_MS(5),.CLK_FREQ_HZ(50_000_000)) u_phy_rst(
        .clk(I_sys_clk),.rst_n(sys_rst_n),.phy_rst_n(O_phy_rst_n),.rst_done(phy_rst_done));

    // ---------------- Ethernet RX ----------------
    wire [7:0] rxd_gmii; wire rx_dv,rx_er;
    rgmii_rx_phy u_rgmii_rx(.rgmii_rxc(I_rgmii_rxc),.rgmii_rx_ctl(I_rgmii_rx_ctl),.rgmii_rxd(I_rgmii_rxd),.gmii_rxd(rxd_gmii),.gmii_rx_dv(rx_dv),.gmii_rx_er(rx_er));
    wire f_start,f_byte_v,f_end,f_err; wire [7:0] f_data; wire [10:0] f_idx,f_len;
    eth_rx_mac u_rx_mac(.clk(I_rgmii_rxc),.rst_n(rst_rx_n),.gmii_rxd(rxd_gmii),.gmii_rx_dv(rx_dv),.gmii_rx_er(rx_er),.frame_start(f_start),.frame_data(f_data),.frame_idx(f_idx),.frame_byte_v(f_byte_v),.frame_end(f_end),.frame_len(f_len),.frame_err(f_err));

    wire arp_req_rx,icmp_req_rx,udp_cmd_req_rx;
    wire [47:0] arp_mac_rx,icmp_mac_rx,udp_mac_rx;
    wire [31:0] arp_ip_rx,icmp_ip_rx,udp_ip_rx;
    wire [15:0] icmp_id_rx,icmp_seq_rx; wire [6:0] icmp_dlen_rx;
    wire [15:0] udp_src_port_rx,udp_app_seq_rx; wire [7:0] udp_app_type_rx;
    wire [15:0] rx_frame_cnt_rx;
    wire ram_wen; wire [6:0] ram_waddr; wire [7:0] ram_wdata;
    wire [31:0] fin_actual_crc_rx,fin_actual_words_rx; wire [7:0] fin_status_rx; wire img_valid_rx;
    wire img_wen; wire [15:0] img_waddr; wire [63:0] img_wdata;
    wire [15:0] img_base_rx, img_nwords_rx; wire img_start_rx;
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
        .img_base(img_base_rx),.img_nwords(img_nwords_rx),.img_start(img_start_rx)
        );

    wire [6:0] ram_raddr; wire [7:0] ram_rdata;
    dc_ram_128x8 u_ram(.wclk(I_rgmii_rxc),.wen(ram_wen),.waddr(ram_waddr),.wdata(ram_wdata),.raddr(ram_raddr),.rdata(ram_rdata));

    // ---------------- CDC of command requests ----------------
    reg arp_tog_rx,icmp_tog_rx,udp_tog_rx;
    always @(posedge I_rgmii_rxc or negedge rst_rx_n) begin
        if(!rst_rx_n)begin arp_tog_rx<=0;icmp_tog_rx<=0;udp_tog_rx<=0; end
        else begin
            if(arp_req_rx)arp_tog_rx<=~arp_tog_rx;
            if(icmp_req_rx)icmp_tog_rx<=~icmp_tog_rx;
            if(udp_cmd_req_rx)udp_tog_rx<=~udp_tog_rx;
        end
    end
    reg [2:0] arp_tog_s,icmp_tog_s,udp_tog_s;
    always @(posedge clk_125) begin
        arp_tog_s<={arp_tog_s[1:0],arp_tog_rx};
        icmp_tog_s<={icmp_tog_s[1:0],icmp_tog_rx};
        udp_tog_s<={udp_tog_s[1:0],udp_tog_rx};
    end
    wire arp_req_tx=arp_tog_s[2]^arp_tog_s[1];
    wire icmp_req_tx=icmp_tog_s[2]^icmp_tog_s[1];
    wire udp_req_tx=udp_tog_s[2]^udp_tog_s[1];

    // ---------------- solver arming / image RAM ----------------
    reg solver_armed;
    always @(posedge clk_125 or negedge rst_125_n) begin
        if(!rst_125_n) solver_armed<=0;
        else if(udp_req_tx && udp_app_type_rx==8'h05) solver_armed<=1;   // START
        else if(udp_req_tx && udp_app_type_rx==8'h09) solver_armed<=0;   // RESET re-arms
    end
    wire ddr_clk, init_done;
    reg [1:0] init_sync;
    always @(posedge clk_sol or negedge sys_rst_n) begin
        if (!sys_rst_n) init_sync <= 2'd0;
        else init_sync <= {init_sync[0], init_done};
    end
    wire rst_sol_n = sys_rst_n & pll_lock & solver_armed & init_sync[1];
    // synchronize the reset release into clk_sol (2 FF)
    reg [1:0] rst_sol_sync;
    always @(posedge clk_sol or negedge rst_sol_n) begin
        if (!rst_sol_n) rst_sol_sync <= 2'b00;
        else rst_sol_sync <= {rst_sol_sync[0], 1'b1};
    end
    wire rst_sol_sync_n = rst_sol_sync[1];

    // ---------------- DDR2 external memory + image/boot/runtime clients ----------------
    wire mem_req, mem_we;
    wire [17:0] mem_addr;
    wire [63:0] mem_wdata, mem_rdata;
    wire mem_busy;
    wire rst_mem_n = sys_rst_n & pll_lock & init_sync[1];

    ddr2_sram_ctrl u_ddr_mem (
        .clk_sol(clk_sol), .rst_sol_n(rst_mem_n),
        .req(mem_req), .we(mem_we), .waddr(mem_addr), .wdata(mem_wdata),
        .busy(mem_busy), .rdata(mem_rdata),
        .I_sys_clk(I_sys_clk), .I_sys_rst_n(sys_rst_n),
        .O_ddr_clk(ddr_clk), .O_init_calib_complete(init_done),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke), .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n), .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p), .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm), .ddr_dq(ddr_dq), .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n)
    );

    wire imgw_req, imgw_we, imgw_busy, boot_busy, sol_busy;
    wire [17:0] imgw_addr, boot_addr, sol_addr;
    wire [63:0] imgw_wdata, imgw_rdata, boot_rdata, sol_rdata;
    wire sol_req, sol_we;
    wire [63:0] sol_wdata;
    wire boot_req;

    ddr2_mem_arbiter u_mem_arb (
        .clk(clk_sol), .rst_n(rst_mem_n),
        .img_req(imgw_req), .img_we(imgw_we), .img_addr(imgw_addr), .img_wdata(imgw_wdata),
        .img_busy(imgw_busy), .img_rdata(imgw_rdata),
        .boot_req(boot_req), .boot_addr(boot_addr), .boot_busy(boot_busy), .boot_rdata(boot_rdata),
        .sol_req(sol_req), .sol_we(sol_we), .sol_addr(sol_addr), .sol_wdata(sol_wdata),
        .sol_busy(sol_busy), .sol_rdata(sol_rdata),
        .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr), .mem_wdata(mem_wdata),
        .mem_busy(mem_busy), .mem_rdata(mem_rdata)
    );

    // Image packet FIFO: RX clock write, clk_sol read.
    wire fifo_empty, fifo_rd_en, fifo_full;
    wire [63:0] fifo_dout;
    async_fifo_256x64 u_img_fifo (
        .wclk(I_rgmii_rxc), .wrst_n(rst_rx_n),
        .wr_en(img_wen), .din(img_wdata), .full(fifo_full),
        .rclk(clk_sol), .rrst_n(rst_mem_n),
        .rd_en(fifo_rd_en), .dout(fifo_dout), .empty(fifo_empty)
    );
    reg img_start_tog_rx;
    always @(posedge I_rgmii_rxc or negedge rst_rx_n) begin
        if (!rst_rx_n) img_start_tog_rx <= 1'b0;
        else if (img_start_rx) img_start_tog_rx <= ~img_start_tog_rx;
    end
    wire img_done_tog;
    img_ddr_writer u_imgwr (
        .clk(clk_sol), .rst_n(rst_mem_n),
        .pkt_start_tog(img_start_tog_rx), .pkt_base(img_base_rx), .pkt_nwords(img_nwords_rx),
        .fifo_empty(fifo_empty), .fifo_dout(fifo_dout), .fifo_rd_en(fifo_rd_en),
        .req(imgw_req), .we(imgw_we), .addr(imgw_addr), .wdata(imgw_wdata),
        .busy(imgw_busy), .rdata(imgw_rdata), .done_tog(img_done_tog)
    );

    // ---------------- full solver ----------------
    localparam N = 1100, M = 2107, NNZ = 4783, HB = 17;
    localparam L = N + M + 1;
    localparam LM1 = L - 1;
    localparam KKT_BASE = 15'd24576;
    wire [14:0] dmem_addr;
    wire [63:0] dmem_wdata;
    wire        dmem_we;
    reg  [63:0] dmem_rdata;
    wire [13:0] kkt_addr;
    wire [63:0] kkt_wdata;
    wire        kkt_we;
    reg  [63:0] kkt_rdata;
    wire [14:0] kkt_mem_addr = KKT_BASE + {2'b0, kkt_addr[12:0]};
    (* ram_style = "bram" *) reg [63:0] mem [0:32767];
    always @(posedge clk_sol) begin
        if (dmem_we) mem[dmem_addr] <= dmem_wdata;
        dmem_rdata <= mem[dmem_addr];
        if (kkt_we) mem[kkt_mem_addr] <= kkt_wdata;
        kkt_rdata <= mem[kkt_mem_addr];
    end

    wire [19:0] boot_waddr;
    wire solver_done, solver_idle, band_ready;
    wire [3:0] verb;
    wire [63:0] scale_cur_out, tau_out;
    reg [15:0] iter_r;
    reg run;
    wire refactor_sol = (iter_r == 16'd0);

    assign boot_addr = {2'b0, boot_waddr[15:0]};
    // boot_req -> arbiter, boot_busy/rdata -> drs_iter
    wire boot_busy_w = boot_busy;
    wire [63:0] boot_rdata_w = boot_rdata;

    drs_iter #(
        .N(N), .M(M), .NNZ(NNZ), .HB(HB), .L(L),
        .RAM_AW(15), .SRAM_AW(18), .BOOT_EXT(1), .SCALE_EN(1),
        .EXT_WS(1), .EXT_COEF(1), .MEM_IF(1), .NODE(100)
    ) u_solver (
        .clk(clk_sol), .rst_n(rst_sol_sync_n), .start(run),
        .refactor(refactor_sol), .band_in(64'd0), .band_valid(1'b0),
        .iter(iter_r), .scale_valid(1'b0), .scale_r(64'h3FF0000000000000),
        .ram_addr(dmem_addr), .ram_wdata(dmem_wdata), .ram_we(dmem_we), .ram_rdata(dmem_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(solver_done), .idle(solver_idle), .verb(verb),
        .scale_cur_out(scale_cur_out), .tau_out(tau_out), .band_ready(band_ready),
        .SRAM_ADDR(), .SRAM_DQ(), .SRAM_CE_N(), .SRAM_OE_N(), .SRAM_WE_N(),
        .SRAM_UB_N(), .SRAM_LB_N(),
        .mem_req(sol_req), .mem_we(sol_we), .mem_waddr(sol_addr),
        .mem_wdata(sol_wdata), .mem_busy(sol_busy), .mem_rdata(sol_rdata),
        .FL_ADDR(), .FL_DQ(), .FL_CE_N(), .FL_OE_N(), .FL_WE_N(),
        .FL_RESET_N(), .FL_WP_N(), .FL_RY(1'b1),
        .boot_req(boot_req), .boot_waddr(boot_waddr), .boot_rdata(boot_rdata_w),
        .boot_busy(boot_busy_w), .boot_ready(1'b1)
    );

    always @(posedge clk_sol or negedge rst_sol_sync_n) begin
        if (!rst_sol_sync_n) begin iter_r <= 16'd0; run <= 1'b1; end
        else if (solver_done) begin
            iter_r <= iter_r + 1'b1;
            if (iter_r + 1'b1 >= 16'd50000) run <= 1'b0;
        end
    end

    // Capture v[1099] (log final mass) for DUMP at address 0.
    reg [63:0] mass_sol;
    reg        mass_tog;
    always @(posedge clk_sol or negedge rst_sol_sync_n) begin
        if (!rst_sol_sync_n) begin
            mass_sol <= 64'd0; mass_tog <= 1'b0;
        end else if (dmem_we && dmem_addr == 15'd1099) begin
            mass_sol <= dmem_wdata;
            mass_tog <= ~mass_tog;
        end
    end
    reg [2:0] mass_sync;
    reg       mass_seen;
    reg [63:0] mass_125;
    always @(posedge clk_125 or negedge rst_125_n) begin
        if (!rst_125_n) begin
            mass_sync <= 3'd0; mass_seen <= 1'b0; mass_125 <= 64'd0;
        end else begin
            mass_sync <= {mass_sync[1:0], mass_tog};
            if (mass_sync[2] ^ mass_seen) begin
                mass_seen <= mass_sync[2];
                mass_125 <= mass_sol;
            end
        end
    end

    // ---------------- TX side ----------------
    reg [47:0] arp_mac_125,icmp_mac_125,udp_mac_125;
    reg [31:0] arp_ip_125,icmp_ip_125,udp_ip_125;
    reg [15:0] icmp_id_125,icmp_seq_125;
    reg [6:0] icmp_dlen_125;
    reg [15:0] udp_port_125,udp_seq_125;
    reg [7:0] udp_ack_type_r;
    reg [31:0] udp_payload_w0_r,udp_payload_w1_r;
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
                8'h0B:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<=mass_125[31:0]; udp_payload_w1_r<=mass_125[63:32]; end
                8'h0C:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,iter_r}; udp_payload_w1_r<={31'd0,run}; end
                8'h09:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<=32'd0; end
                default:begin udp_ack_type_r<=8'h02; udp_payload_w0_r<={16'd0,udp_app_seq_rx}; udp_payload_w1_r<=32'd0; end
            endcase
        end
    end

    wire arp_start,icmp_start,udp_start,dump_start;
    wire arp_busy,icmp_busy,udp_busy,dump_busy;
    wire arp_done,icmp_done,udp_done,dump_done;
    wire [7:0] arp_txd,icmp_txd,udp_txd;
    wire arp_en,icmp_en,udp_en;

    tx_arbiter u_arb(.clk(clk_125),.rst_n(rst_125_n),
        .arp_req(arp_req_tx),.icmp_req(icmp_req_tx),.udp_req(udp_req_tx),.dump_req(1'b0),
        .arp_start(arp_start),.icmp_start(icmp_start),.udp_start(udp_start),.dump_start(dump_start),
        .arp_busy(arp_busy),.icmp_busy(icmp_busy),.udp_busy(udp_busy),.dump_busy(dump_busy),.tx_busy());

    arp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8)) u_arp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(arp_start),.dst_mac(arp_mac_125),.dst_ip(arp_ip_125),.busy(arp_busy),.done(arp_done),.gmii_txd(arp_txd),.gmii_tx_en(arp_en));

    icmp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8)) u_icmp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(icmp_start),.dst_mac(icmp_mac_125),.dst_ip(icmp_ip_125),.icmp_id(icmp_id_125),.icmp_seq(icmp_seq_125),.data_len(icmp_dlen_125),.ram_raddr(ram_raddr),.ram_rdata(ram_rdata),.busy(icmp_busy),.done(icmp_done),.gmii_txd(icmp_txd),.gmii_tx_en(icmp_en));

    udp_reply_tx #(.LOCAL_MAC(48'h02_00_00_00_00_01),.LOCAL_IP(32'hC0A8_01C8),.CMD_PORT(16'd4661)) u_udp_tx(
        .clk(clk_125),.rst_n(rst_125_n),.req(udp_start),.dst_mac(udp_mac_125),.dst_ip(udp_ip_125),.dst_port(udp_port_125),.ack_type(udp_ack_type_r),.ack_seq(udp_seq_125),.payload_w0(udp_payload_w0_r),.payload_w1(udp_payload_w1_r),.busy(udp_busy),.done(udp_done),.gmii_txd(udp_txd),.gmii_tx_en(udp_en));

    wire [7:0] gmii_txd = arp_busy?arp_txd:(icmp_busy?icmp_txd:udp_txd);
    wire gmii_tx_en = arp_busy?arp_en:(icmp_busy?icmp_en:udp_en);

    rgmii_tx_phy u_rgmii_tx(.clk_125(clk_125),.clk_125_90(clk_125),.rst(~rst_125_n),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_tx_er(1'b0),.rgmii_txc(O_rgmii_txc),.rgmii_tx_ctl(O_rgmii_tx_ctl),.rgmii_txd(O_rgmii_txd));

    // LEDs: lock, solver done, armed, heartbeat
    reg [25:0] hb_cnt; always @(posedge I_sys_clk) hb_cnt<=hb_cnt+1'b1;
    assign O_led[0]=pll_lock; assign O_led[1]=solver_done_s2; assign O_led[2]=solver_armed; assign O_led[3]=hb_cnt[25];
    assign O_rgmii_mdc=1'b0; assign IO_rgmii_mdio=1'bz;
endmodule
