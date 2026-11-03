`timescale 1ns/1ps
// ddr2_smoke_top — M6 step 1: DDR2 controller bring-up.
// Writes 256 x 128-bit patterns to DDR2, reads them back, compares.
// LED0 = init_calib_complete, LED1 = PASS, LED2 = FAIL, LED3 = heartbeat.
module ddr2_smoke_top (
    input  wire        I_sys_clk,
    output wire [3:0]  O_led,
    // DDR2 pins (assigned by the DDR2 IP ADC)
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

    wire ddr_clk, init_done;
    wire [24:0] mc_addr;
    wire [2:0]  mc_cmd;
    wire        mc_en;
    wire [127:0] mc_wdf_data;
    wire        mc_wdf_end;
    wire [15:0] mc_wdf_mask;
    wire        mc_wdf_wren;
    wire [127:0] mc_rd_data;
    wire        mc_rd_data_end, mc_rd_data_valid;
    wire        mc_rdy, mc_wdf_rdy;

    ph1p35_324_ddr_wrapper u_ddr (
        .I_sys_clk(I_sys_clk), .I_sys_rst_n(sys_rst_n),
        .O_ddr_clk(ddr_clk), .O_init_calib_complete(init_done),
        .I_mc_app_addr(mc_addr), .I_mc_app_cmd(mc_cmd), .I_mc_app_en(mc_en),
        .I_mc_app_wdf_data(mc_wdf_data), .I_mc_app_wdf_end(mc_wdf_end),
        .I_mc_app_wdf_mask(mc_wdf_mask), .I_mc_app_wdf_wren(mc_wdf_wren),
        .O_mc_app_rd_data(mc_rd_data), .O_mc_app_rd_data_end(mc_rd_data_end),
        .O_mc_app_rd_data_valid(mc_rd_data_valid),
        .O_mc_app_rdy(mc_rdy), .O_mc_app_wdf_rdy(mc_wdf_rdy),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke), .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n), .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p), .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm), .ddr_dq(ddr_dq), .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n));

    // ---- test FSM (ddr_clk domain) ----
    localparam IDLE=0, WREQ=1, WADV=2, RREQ=3, RWAIT=4, DONE=5;
    reg [2:0] st;
    reg [8:0] idx;
    reg [24:0] addr;
    reg [2:0]  cmd;
    reg        en, wdf_wren, wdf_end;
    reg [127:0] wdata;
    reg        pass, fail;
    reg [1:0]  init_sync;
    reg [25:0] hb;

    wire init_s = init_sync[1];
    wire [127:0] pattern = {idx[7:0], ~idx[7:0],
                            64'h0123456789ABCDEF ^ {8{idx[7:0]}}};

    always @(posedge ddr_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            st<=IDLE; idx<=0; addr<=0; cmd<=0; en<=0; wdf_wren<=0; wdf_end<=0;
            wdata<=0; pass<=0; fail<=0; init_sync<=0; hb<=0;
        end else begin
            init_sync <= {init_sync[0], init_done};
            hb <= hb + 1'b1;
            case (st)
            IDLE: begin
                en<=0; wdf_wren<=0; wdf_end<=0;
                if (init_s) begin idx<=0; st<=WREQ; end
            end
            WREQ: begin
                addr <= {16'd0, idx, 3'd0};
                cmd  <= 3'd0;
                wdata <= pattern;
                en <= 1'b1; wdf_wren <= 1'b1; wdf_end <= 1'b1;
                if (mc_rdy && mc_wdf_rdy) st <= WADV;
            end
            WADV: begin
                en<=0; wdf_wren<=0; wdf_end<=0;
                if (idx == 9'd255) begin idx<=0; st<=RREQ; end
                else begin idx<=idx+1'b1; st<=WREQ; end
            end
            RREQ: begin
                addr <= {16'd0, idx, 3'd0};
                cmd  <= 3'd1;
                en <= 1'b1;
                if (mc_rdy) st <= RWAIT;
            end
            RWAIT: begin
                en <= 1'b0;
                if (mc_rd_data_valid) begin
                    if (mc_rd_data !== pattern) fail <= 1'b1;
                    if (idx == 9'd255) st <= DONE;
                    else begin idx<=idx+1'b1; st<=RREQ; end
                end
            end
            DONE: begin
                en<=0; wdf_wren<=0; wdf_end<=0;
                if (!fail) pass <= 1'b1;
                st <= DONE;
            end
            default: st <= IDLE;
            endcase
        end
    end

    assign mc_addr = addr;
    assign mc_cmd = cmd;
    assign mc_en = en;
    assign mc_wdf_data = wdata;
    assign mc_wdf_end = wdf_end;
    assign mc_wdf_mask = 16'h0;
    assign mc_wdf_wren = wdf_wren;

    assign O_led[0] = init_s;
    assign O_led[1] = pass;
    assign O_led[2] = fail;
    assign O_led[3] = hb[25];
endmodule
