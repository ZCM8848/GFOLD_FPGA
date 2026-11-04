`timescale 1ns/1ps
// ddr2_sram_ctrl -- DDR2-backed 64-bit word memory with the sram64_ctrl
// handshake used by drs_iter.
//
// Client side (clk_sol): req/we/waddr/wdata/busy/rdata. A request is a pulse;
// busy rises and falls when the DDR2 transaction has completed; rdata is valid
// when busy falls (reads).
//
// The client and the DDR2 user interface are asynchronous, so a toggle
// handshake crosses the request/ack; the address/data are held stable by the
// drs_iter sub-FSM while busy is high.
module ddr2_sram_ctrl (
    // client side (solver clock)
    input  wire        clk_sol,
    input  wire        rst_sol_n,
    input  wire        req,
    input  wire        we,
    input  wire [17:0] waddr,
    input  wire [63:0] wdata,
    output reg         busy,
    output reg  [63:0] rdata,
    // board / DDR2
    input  wire        I_sys_clk,
    input  wire        I_sys_rst_n,
    output wire        O_ddr_clk,
    output wire        O_init_calib_complete,
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
    localparam WR_DRAIN = 8'd32;

    wire        ddr_clk;
    wire        init_done;
    wire [24:0] mc_addr;
    wire [2:0]  mc_cmd;
    wire        mc_en;
    wire [127:0] mc_wdf_data;
    wire        mc_wdf_end;
    wire [15:0]  mc_wdf_mask;
    wire        mc_wdf_wren;
    wire [127:0] mc_rd_data;
    wire        mc_rd_data_end;
    wire        mc_rd_data_valid;
    wire        mc_rdy, mc_wdf_rdy;
    wire        mc_awready, mc_arready, mc_wready;

    ph1p35_324_ddr_wrapper u_ddr (
        .I_sys_clk(I_sys_clk), .I_sys_rst_n(I_sys_rst_n),
        .O_ddr_clk(O_ddr_clk), .O_init_calib_complete(O_init_calib_complete),
        .I_mc_app_addr(mc_addr), .I_mc_app_cmd(mc_cmd), .I_mc_app_en(mc_en),
        .I_mc_app_wdf_data(mc_wdf_data), .I_mc_app_wdf_end(mc_wdf_end),
        .I_mc_app_wdf_mask(mc_wdf_mask), .I_mc_app_wdf_wren(mc_wdf_wren),
        .O_mc_app_rd_data(mc_rd_data), .O_mc_app_rd_data_end(mc_rd_data_end),
        .O_mc_app_rd_data_valid(mc_rd_data_valid),
        .O_mc_app_rdy(mc_rdy), .O_mc_app_wdf_rdy(mc_wdf_rdy),
        .O_mc_app_awready(mc_awready), .O_mc_app_arready(mc_arready),
        .O_mc_app_wready(mc_wready),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke), .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n), .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p), .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm), .ddr_dq(ddr_dq), .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n)
    );
    assign ddr_clk = O_ddr_clk;

    // ---------------- request/ack CDC ----------------
    reg        req_tog_sol;
    reg        ack_tog_ddr;
    reg [63:0] rdata_ddr;
    reg [1:0]  req_sync_ddr;
    reg [2:0]  req_sync_ddr_p;
    reg [1:0]  ack_sync_sol, ack_sync_sol_p;
    reg        ack_tog_sol_p;

    always @(posedge clk_sol or negedge rst_sol_n) begin
        if (!rst_sol_n) begin
            req_tog_sol <= 1'b0; busy <= 1'b0; rdata <= 64'd0;
            ack_sync_sol <= 2'b00; ack_sync_sol_p <= 2'b00; ack_tog_sol_p <= 1'b0;
        end else begin
            ack_sync_sol <= {ack_sync_sol[0], ack_tog_ddr};
            ack_sync_sol_p <= ack_sync_sol;
            if (req && !busy) begin
                req_tog_sol <= ~req_tog_sol;
                busy <= 1'b1;
            end
            if (ack_sync_sol[1] ^ ack_tog_sol_p) begin
                ack_tog_sol_p <= ack_sync_sol[1];
                rdata <= rdata_ddr;
                busy <= 1'b0;
            end
        end
    end

    // ---------------- DDR2 transaction FSM (ddr_clk) ----------------
    localparam D_IDLE=0, D_WREQ=1, D_WWAIT=2, D_WDRAIN=3,
               D_RREQ=4, D_RWAIT=5, D_RDATA=6, D_DONE=7;
    reg [2:0]  dst;
    reg [24:0] addr_r;
    reg [63:0] wdata_r;
    reg        we_r;
    reg        aw_en, w_en, ar_en, aw_done, w_done;
    reg [7:0]  drain_cnt;
    reg [2:0]  req_seen_ddr;

    always @(posedge ddr_clk or negedge I_sys_rst_n) begin
        if (!I_sys_rst_n) begin
            dst <= D_IDLE; addr_r <= 0; wdata_r <= 0; we_r <= 0;
            aw_en <= 0; w_en <= 0; ar_en <= 0; aw_done <= 0; w_done <= 0;
            drain_cnt <= 0; ack_tog_ddr <= 0; rdata_ddr <= 0;
            req_sync_ddr <= 0; req_sync_ddr_p <= 0; req_seen_ddr <= 0;
        end else begin
            req_sync_ddr <= {req_sync_ddr[0], req_tog_sol};
            req_sync_ddr_p <= {req_sync_ddr_p[1:0], req_sync_ddr[1]};
            case (dst)
            D_IDLE: begin
                aw_en <= 0; w_en <= 0; ar_en <= 0; aw_done <= 0; w_done <= 0;
                if (init_done && (req_sync_ddr[1] ^ req_sync_ddr_p[2])) begin
                    addr_r <= {7'd0, waddr};
                    wdata_r <= wdata;
                    we_r <= we;
                    dst <= we ? D_WREQ : D_RREQ;
                end
            end
            D_WREQ: begin
                aw_en <= 1'b1; w_en <= 1'b1; aw_done <= 0; w_done <= 0;
                dst <= D_WWAIT;
            end
            D_WWAIT: begin
                if (aw_en && mc_awready) begin aw_en <= 0; aw_done <= 1; end
                if (w_en && mc_wready) begin w_en <= 0; w_done <= 1; end
                if ((aw_done || (aw_en && mc_awready)) &&
                    (w_done  || (w_en  && mc_wready))) begin
                    drain_cnt <= WR_DRAIN;
                    dst <= D_WDRAIN;
                end
            end
            D_WDRAIN: begin
                if (drain_cnt != 0) drain_cnt <= drain_cnt - 1'b1;
                else begin ack_tog_ddr <= ~ack_tog_ddr; dst <= D_IDLE; end
            end
            D_RREQ: begin
                ar_en <= 1'b1;
                dst <= D_RWAIT;
            end
            D_RWAIT: begin
                if (ar_en && mc_arready) begin ar_en <= 0; dst <= D_RDATA; end
            end
            D_RDATA: begin
                if (mc_rd_data_valid) begin
                    rdata_ddr <= mc_rd_data[63:0];
                    ack_tog_ddr <= ~ack_tog_ddr;
                    dst <= D_IDLE;
                end
            end
            default: dst <= D_IDLE;
            endcase
        end
    end

    // Each 64-bit client word occupies one 128-bit AXI beat (stride 8 in the
    // MC's 16-bit address units). Replicate the 64-bit word into both halves
    // so the result is independent of byte-lane mapping; reads use the low half.
    assign mc_addr = {addr_r[21:0], 3'b0};
    assign mc_cmd  = (dst == D_RREQ || dst == D_RWAIT || dst == D_RDATA) ? 3'd1 : 3'd0;
    assign mc_en   = (dst == D_RREQ || dst == D_RWAIT) ? ar_en :
                     (dst == D_WREQ || dst == D_WWAIT) ? aw_en : 1'b0;
    assign mc_wdf_data = {wdata_r, wdata_r};
    assign mc_wdf_end  = (dst == D_WREQ || dst == D_WWAIT) && w_en;
    assign mc_wdf_mask = 16'h0;
    assign mc_wdf_wren = (dst == D_WREQ || dst == D_WWAIT) && w_en;
endmodule
