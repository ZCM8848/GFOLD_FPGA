`timescale 1ns/1ps
// ddr2_smoke_top -- M6 DDR2 bring-up, protocol-correct AXI master.
//
// Tests in three stages, each writes a lane-invariant pattern, waits for the
// write pipeline to drain, then reads back and compares:
//   stage 0: 1 word   at 0x0000
//   stage 1: 16 words at 0x1000 (16 idle cycles between writes)
//   stage 2: 256 words at 0x2000 (8 idle cycles between writes)
//
// LED mapping (confirmed by led_walk_top):
//   LED1 D10 = init_calib_complete
//   LED2 E10 = stage 0 (single word) passed
//   LED3 C6  = stage 1 (16 words) passed
//   LED4 D6  = all stages passed (blinks)
//
// If a stage fails (or hangs), the LEDs stop at the last passed stage.
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
    reg [19:0] por_cnt;
    reg        sys_rst_n;
    initial begin por_cnt = 20'd0; sys_rst_n = 1'b0; end
    always @(posedge I_sys_clk) begin
        if (por_cnt < 20'hF_FFFF) begin
            por_cnt  <= por_cnt + 1'b1;
            sys_rst_n <= 1'b0;
        end else begin
            sys_rst_n <= 1'b1;
        end
    end

    wire         ddr_clk;
    wire         init_done;
    wire [24:0]  mc_addr;
    wire [2:0]   mc_cmd;
    wire         mc_en;
    wire [127:0] mc_wdf_data;
    wire         mc_wdf_end;
    wire [15:0]  mc_wdf_mask;
    wire         mc_wdf_wren;
    wire [127:0] mc_rd_data;
    wire         mc_rd_data_end;
    wire         mc_rd_data_valid;
    wire         mc_rdy;
    wire         mc_wdf_rdy;
    wire         mc_awready;
    wire         mc_arready;
    wire         mc_wready;

    ph1p35_324_ddr_wrapper u_ddr (
        .I_sys_clk(I_sys_clk),
        .I_sys_rst_n(sys_rst_n),
        .O_ddr_clk(ddr_clk),
        .O_init_calib_complete(init_done),
        .I_mc_app_addr(mc_addr),
        .I_mc_app_cmd(mc_cmd),
        .I_mc_app_en(mc_en),
        .I_mc_app_wdf_data(mc_wdf_data),
        .I_mc_app_wdf_end(mc_wdf_end),
        .I_mc_app_wdf_mask(mc_wdf_mask),
        .I_mc_app_wdf_wren(mc_wdf_wren),
        .O_mc_app_rd_data(mc_rd_data),
        .O_mc_app_rd_data_end(mc_rd_data_end),
        .O_mc_app_rd_data_valid(mc_rd_data_valid),
        .O_mc_app_rdy(mc_rdy),
        .O_mc_app_wdf_rdy(mc_wdf_rdy),
        .O_mc_app_awready(mc_awready),
        .O_mc_app_arready(mc_arready),
        .O_mc_app_wready(mc_wready),
        .ddr_addr(ddr_addr),
        .ddr_ba(ddr_ba),
        .ddr_cke(ddr_cke),
        .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n),
        .ddr_ras_n(ddr_ras_n),
        .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n),
        .ddr_ck_p(ddr_ck_p),
        .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm),
        .ddr_dq(ddr_dq),
        .ddr_dqs_p(ddr_dqs_p),
        .ddr_dqs_n(ddr_dqs_n)
    );

    // ------------------------------------------------------------------
    // Test sequencer (ddr_clk domain)
    // ------------------------------------------------------------------
    localparam S_IDLE    = 4'd0;
    localparam S_WR_REQ  = 4'd1;
    localparam S_WR_WAIT = 4'd2;
    localparam S_WR_GAP  = 4'd3;
    localparam S_DRAIN   = 4'd4;
    localparam S_RD_REQ  = 4'd5;
    localparam S_RD_WAIT = 4'd6;
    localparam S_RD_DATA = 4'd7;
    localparam S_STAGE   = 4'd8;
    localparam S_DONE    = 4'd9;
    localparam S_FAIL    = 4'd10;

    reg [3:0]  st;
    reg [1:0]  phase;
    reg [8:0]  idx;
    reg [8:0]  nwords;
    reg [7:0]  throttle;
    reg [7:0]  gap_cnt;
    reg [15:0] drain_cnt;
    reg [24:0] base;
    reg        aw_en;
    reg        w_en;
    reg        ar_en;
    reg        aw_done;
    reg        w_done;
    reg        single_pass;
    reg        sixteen_pass;
    reg        all_pass;
    reg        fail;
    reg [1:0]  init_sync;
    reg [25:0] hb;

    wire        in_wr;
    wire        in_rd;
    wire [24:0] word_addr;
    wire [7:0]  pat_byte;
    wire [127:0] pattern;

    assign in_wr    = (st == S_WR_REQ) || (st == S_WR_WAIT);
    assign in_rd    = (st == S_RD_REQ) || (st == S_RD_WAIT) || (st == S_RD_DATA);
    assign word_addr = base + {13'd0, idx, 3'd0};

    // All 16 bytes are identical, so this pattern is invariant to byte/word
    // lane swapping. It still uniquely identifies the address via idx/phase.
    assign pat_byte = 8'hA5 ^ idx[7:0] ^ {6'd0, phase};
    assign pattern  = {16{pat_byte}};

    assign mc_addr     = (in_wr || in_rd) ? word_addr : 25'd0;
    assign mc_cmd      = in_rd ? 3'd1 : 3'd0;
    assign mc_en       = in_wr ? aw_en : (in_rd ? ar_en : 1'b0);
    assign mc_wdf_data = pattern;
    assign mc_wdf_end  = in_wr && w_en;
    assign mc_wdf_mask = 16'h0;
    assign mc_wdf_wren = in_wr && w_en;

    always @(posedge ddr_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            st           <= S_IDLE;
            phase        <= 2'd0;
            idx          <= 9'd0;
            nwords       <= 9'd1;
            throttle     <= 8'd0;
            gap_cnt      <= 8'd0;
            drain_cnt    <= 16'd0;
            base         <= 25'd0;
            aw_en        <= 1'b0;
            w_en         <= 1'b0;
            ar_en        <= 1'b0;
            aw_done      <= 1'b0;
            w_done       <= 1'b0;
            single_pass  <= 1'b0;
            sixteen_pass <= 1'b0;
            all_pass     <= 1'b0;
            fail         <= 1'b0;
            init_sync    <= 2'd0;
            hb           <= 26'd0;
        end else begin
            init_sync <= {init_sync[0], init_done};
            hb        <= hb + 1'b1;

            case (st)
            S_IDLE: begin
                aw_en <= 1'b0;
                w_en  <= 1'b0;
                ar_en <= 1'b0;
                if (init_sync[1]) begin
                    phase    <= 2'd0;
                    idx      <= 9'd0;
                    nwords   <= 9'd1;
                    throttle <= 8'd0;
                    base     <= 25'd0;
                    st       <= S_WR_REQ;
                end
            end

            S_WR_REQ: begin
                // Assert AWVALID and WVALID together; they are tracked
                // independently below because the slave may accept them in
                // different cycles.
                aw_en   <= 1'b1;
                w_en    <= 1'b1;
                aw_done <= 1'b0;
                w_done  <= 1'b0;
                st      <= S_WR_WAIT;
            end

            S_WR_WAIT: begin
                if (aw_en && mc_awready) begin
                    aw_en   <= 1'b0;
                    aw_done <= 1'b1;
                end
                if (w_en && mc_wready) begin
                    w_en   <= 1'b0;
                    w_done <= 1'b1;
                end
                if ((aw_done || (aw_en && mc_awready)) &&
                    (w_done  || (w_en  && mc_wready))) begin
                    st      <= S_WR_GAP;
                    gap_cnt <= throttle;
                end
            end

            S_WR_GAP: begin
                if (gap_cnt != 8'd0) begin
                    gap_cnt <= gap_cnt - 1'b1;
                end else if (idx + 1'b1 == nwords) begin
                    idx       <= 9'd0;
                    drain_cnt <= 16'd20000;
                    st        <= S_DRAIN;
                end else begin
                    idx <= idx + 1'b1;
                    st  <= S_WR_REQ;
                end
            end

            S_DRAIN: begin
                if (drain_cnt != 16'd0) begin
                    drain_cnt <= drain_cnt - 1'b1;
                end else begin
                    idx <= 9'd0;
                    st  <= S_RD_REQ;
                end
            end

            S_RD_REQ: begin
                ar_en <= 1'b1;
                st    <= S_RD_WAIT;
            end

            S_RD_WAIT: begin
                if (ar_en && mc_arready) begin
                    ar_en <= 1'b0;
                    st    <= S_RD_DATA;
                end
            end

            S_RD_DATA: begin
                if (mc_rd_data_valid) begin
                    if (mc_rd_data !== pattern) begin
                        fail <= 1'b1;
                        st   <= S_FAIL;
                    end else if (idx + 1'b1 == nwords) begin
                        st <= S_STAGE;
                    end else begin
                        idx <= idx + 1'b1;
                        st  <= S_RD_REQ;
                    end
                end
            end

            S_STAGE: begin
                case (phase)
                2'd0: begin
                    single_pass <= 1'b1;
                    phase       <= 2'd1;
                    base        <= 25'h1000;
                    nwords      <= 9'd16;
                    throttle    <= 8'd16;
                    idx         <= 9'd0;
                    st          <= S_WR_REQ;
                end
                2'd1: begin
                    sixteen_pass <= 1'b1;
                    phase        <= 2'd2;
                    base         <= 25'h2000;
                    nwords       <= 9'd256;
                    throttle     <= 8'd8;
                    idx          <= 9'd0;
                    st           <= S_WR_REQ;
                end
                default: begin
                    all_pass <= 1'b1;
                    st       <= S_DONE;
                end
                endcase
            end

            S_DONE: begin
                aw_en <= 1'b0;
                w_en  <= 1'b0;
                ar_en <= 1'b0;
            end

            S_FAIL: begin
                aw_en <= 1'b0;
                w_en  <= 1'b0;
                ar_en <= 1'b0;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

    assign O_led[0] = init_sync[1];
    assign O_led[1] = single_pass;
    assign O_led[2] = sixteen_pass;
    assign O_led[3] = all_pass ? hb[25] : 1'b0;
endmodule
