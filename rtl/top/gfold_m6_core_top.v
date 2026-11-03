`timescale 1ns/1ps
// gfold_m6_core_top -- full G-FOLD datapath with DDR2-backed external memory.
// This is the M6 core bring-up top (no Ethernet yet): it exists to close
// synthesis/timing on the complete N=1100/M=2107 solver with the DDR2 memory
// interface in place. The boot image source is tied off; the Ethernet/Flash
// loader is added in the next step.
module gfold_m6_core_top (
    input  wire        I_sys_clk,
    input  wire        start,
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
    reg [19:0] por_cnt;
    reg        sys_rst_n;
    initial begin por_cnt = 20'd0; sys_rst_n = 1'b0; end
    always @(posedge I_sys_clk) begin
        if (por_cnt < 20'hF_FFFF) begin por_cnt <= por_cnt + 1'b1; sys_rst_n <= 1'b0; end
        else sys_rst_n <= 1'b1;
    end

    // 25 MHz solver clock (50/2), through a global buffer.
    reg  sol_div;
    always @(posedge I_sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) sol_div <= 1'b0;
        else sol_div <= ~sol_div;
    end
    wire clk_sol;
    PH1P_LOGIC_BUFG u_bufg_sol(.i(sol_div), .o(clk_sol));

    // DDR2 user clock is asynchronous to clk_sol; the adapter contains the CDC.
    wire ddr_clk, init_done;
    reg [1:0] init_sync;
    always @(posedge clk_sol or negedge sys_rst_n) begin
        if (!sys_rst_n) init_sync <= 2'd0;
        else init_sync <= {init_sync[0], init_done};
    end
    wire rst_sol_n = sys_rst_n & init_sync[1];

    wire mem_req, mem_we;
    wire [17:0] mem_waddr;
    wire [63:0] mem_wdata;
    wire mem_busy;
    wire [63:0] mem_rdata;

    ddr2_sram_ctrl u_mem (
        .clk_sol(clk_sol), .rst_sol_n(rst_sol_n),
        .req(mem_req), .we(mem_we), .waddr(mem_waddr), .wdata(mem_wdata),
        .busy(mem_busy), .rdata(mem_rdata),
        .I_sys_clk(I_sys_clk), .I_sys_rst_n(sys_rst_n),
        .O_ddr_clk(ddr_clk), .O_init_calib_complete(init_done),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke), .ddr_odt(ddr_odt),
        .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n), .ddr_cas_n(ddr_cas_n),
        .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p), .ddr_ck_n(ddr_ck_n),
        .ddr_dm(ddr_dm), .ddr_dq(ddr_dq), .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n)
    );

    reg [1:0] init_sync;
    always @(posedge clk_sol or negedge sys_rst_n) begin
        if (!sys_rst_n) init_sync <= 2'd0;
        else init_sync <= {init_sync[0], init_done};
    end
    wire rst_sol_n = sys_rst_n & init_sync[1];

    // On-chip state + BAND workspace: 23,461 words.
    localparam N = 1100, M = 2107, NNZ = 4783, HB = 17;
    localparam L = N + M + 1;
    localparam LM1 = L - 1;
    localparam BAND_BASE = 5 * L + LM1;
    localparam WS_BASE = BAND_BASE + 2 * M;   // 23,461 for the full problem

    wire [14:0] ram_addr;
    wire [63:0] ram_wdata;
    wire        ram_we;
    reg  [63:0] ram_rdata;

    wire [13:0] kkt_addr;
    wire [63:0] kkt_wdata;
    wire        kkt_we;
    reg  [63:0] kkt_rdata;

    // One dual-port ERAM holds state+BAND (0..23,460) and KKT (24,576..32,767).
    // A single inferred memory avoids TD's one-large-RAM-per-design limitation.
    localparam KKT_BASE = 15'd24576;
    wire [14:0] kkt_mem_addr = KKT_BASE + {2'b0, kkt_addr[12:0]};
    (* ram_style = "bram" *) reg [63:0] mem [0:32767];
    always @(posedge clk_sol) begin
        if (ram_we) mem[ram_addr] <= ram_wdata;
        ram_rdata <= mem[ram_addr];
        if (kkt_we) mem[kkt_mem_addr] <= kkt_wdata;
        kkt_rdata <= mem[kkt_mem_addr];
    end

    wire        refactor, band_valid;
    wire [63:0] band_in;
    wire [15:0] iter;
    wire        scale_valid;
    wire [63:0] scale_r;
    wire        done, idle, band_ready;
    wire [3:0]  verb;
    wire [63:0] scale_cur_out, tau_out;

    wire [19:0] SRAM_ADDR;
    wire [15:0] SRAM_DQ;
    wire        SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N;
    wire [22:0] FL_ADDR;
    wire [7:0]  FL_DQ;
    wire        FL_CE_N, FL_OE_N, FL_WE_N, FL_RESET_N, FL_WP_N;
    wire        boot_req;
    wire [19:0] boot_waddr;
    wire        boot_ready = 1'b1;
    reg         boot_busy_r;
    reg  [63:0] boot_rdata_r;
    reg  [2:0]  boot_cnt;
    always @(posedge clk_sol or negedge rst_sol_n) begin
        if (!rst_sol_n) begin
            boot_busy_r <= 1'b0; boot_rdata_r <= 64'd0; boot_cnt <= 3'd0;
        end else if (boot_req && !boot_busy_r) begin
            boot_busy_r <= 1'b1; boot_rdata_r <= 64'd0; boot_cnt <= 3'd0;
        end else if (boot_busy_r) begin
            if (boot_cnt == 3'd2) boot_busy_r <= 1'b0;
            else boot_cnt <= boot_cnt + 1'b1;
        end
    end

    drs_iter #(
        .N(N), .M(M), .NNZ(NNZ), .HB(HB), .L(L),
        .RAM_AW(15), .SRAM_AW(18), .BOOT_EXT(1), .SCALE_EN(1),
        .EXT_WS(1), .EXT_COEF(1), .MEM_IF(1), .NODE(100)
    ) dut (
        .clk(clk_sol), .rst_n(rst_sol_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid),
        .iter(iter), .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done), .idle(idle), .verb(verb),
        .scale_cur_out(scale_cur_out), .tau_out(tau_out), .band_ready(band_ready),
        .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ),
        .SRAM_CE_N(SRAM_CE_N), .SRAM_OE_N(SRAM_OE_N), .SRAM_WE_N(SRAM_WE_N),
        .SRAM_UB_N(SRAM_UB_N), .SRAM_LB_N(SRAM_LB_N),
        .mem_req(mem_req), .mem_we(mem_we), .mem_waddr(mem_waddr),
        .mem_wdata(mem_wdata), .mem_busy(mem_busy), .mem_rdata(mem_rdata),
        .FL_ADDR(FL_ADDR), .FL_DQ(FL_DQ), .FL_CE_N(FL_CE_N), .FL_OE_N(FL_OE_N),
        .FL_WE_N(FL_WE_N), .FL_RESET_N(FL_RESET_N), .FL_WP_N(FL_WP_N),
        .FL_RY(1'b1),
        .boot_req(boot_req), .boot_waddr(boot_waddr), .boot_rdata(boot_rdata_r),
        .boot_busy(boot_busy_r), .boot_ready(boot_ready)
    );

    assign refactor    = 1'b1;
    assign band_valid  = 1'b0;
    assign band_in     = 64'd0;
    assign iter        = 16'd0;
    assign scale_valid = 1'b0;
    assign scale_r     = 64'h3FF0000000000000;

    assign O_led[0] = init_done;
    assign O_led[1] = done;
    assign O_led[2] = idle;
    assign O_led[3] = verb[0];
endmodule
