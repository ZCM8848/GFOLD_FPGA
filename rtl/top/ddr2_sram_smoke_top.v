`timescale 1ns/1ps
// ddr2_sram_smoke_top -- validate ddr2_sram_ctrl (64-bit word -> 128-bit beat)
// with 16 write/read words. LED1 init, LED2 pass, LED3 fail, LED4 heartbeat.
module ddr2_sram_smoke_top (
    input  wire        I_sys_clk,
    output wire [3:0]  O_led,
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
    initial begin por_cnt = 0; sys_rst_n = 0; end
    always @(posedge I_sys_clk) begin
        if (por_cnt < 20'hF_FFFF) begin por_cnt <= por_cnt + 1; sys_rst_n <= 0; end
        else sys_rst_n <= 1;
    end

    wire init_done, ddr_clk;
    reg [1:0] init_sync;
    always @(posedge I_sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) init_sync <= 0;
        else init_sync <= {init_sync[0], init_done};
    end
    wire rst_n = sys_rst_n & init_sync[1];

    wire        req, we, busy;
    wire [17:0] waddr;
    wire [63:0] wdata, rdata;
    ddr2_sram_ctrl u_mem (
        .clk_sol(I_sys_clk), .rst_sol_n(rst_n),
        .req(req), .we(we), .waddr(waddr), .wdata(wdata), .busy(busy), .rdata(rdata),
        .I_sys_clk(I_sys_clk), .I_sys_rst_n(sys_rst_n),
        .O_ddr_clk(ddr_clk), .O_init_calib_complete(init_done),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke), .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n), .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p), .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm), .ddr_dq(ddr_dq), .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n)
    );

    localparam S_IDLE=0, S_WREQ=1, S_WBUSY=2, S_WWAIT=3,
               S_RREQ=4, S_RBUSY=5, S_RWAIT=6, S_DONE=7, S_FAIL=8;
    reg [3:0] st;
    reg [4:0] idx;
    reg [63:0] rdval;
    reg pass, fail;
    reg [25:0] hb;

    wire [63:0] pattern = {8{16'hA5A5 ^ {8{3'b0, idx}}}};

    always @(posedge I_sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
        end else begin
            hb <= hb + 1;
            case (st)
            S_IDLE: if (rst_n) begin idx<=0; st<=S_WREQ; end
            S_WREQ: begin
                st <= S_WBUSY;
            end
            S_WBUSY: begin
                if (busy) st <= S_WWAIT;
            end
            S_WWAIT: begin
                if (!busy) begin
                    if (idx == 5'd15) begin idx<=0; st<=S_RREQ; end
                    else begin idx<=idx+1; st<=S_WREQ; end
                end
            end
            S_RREQ: begin
                st <= S_RBUSY;
            end
            S_RBUSY: begin
                if (busy) st <= S_RWAIT;
            end
            S_RWAIT: begin
                if (!busy) begin
                    rdval <= rdata;
                    if (rdata !== pattern) begin fail<=1; st<=S_FAIL; end
                    else if (idx == 5'd15) begin pass<=1; st<=S_DONE; end
                    else begin idx<=idx+1; st<=S_RREQ; end
                end
            end
            S_DONE, S_FAIL: ;
            default: st<=S_IDLE;
            endcase
        end
    end

    // Request pulses are generated combinationally from the request states.
    assign req   = (st == S_WREQ) || (st == S_RREQ);
    assign we    = (st == S_WREQ);
    assign waddr = {13'd0, idx};
    assign wdata = pattern;

    assign O_led[0] = init_done;
    assign O_led[1] = pass;
    assign O_led[2] = fail;
    assign O_led[3] = hb[25];
endmodule
