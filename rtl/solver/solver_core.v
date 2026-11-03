`timescale 1ns/1ps
// solver_core �?small on-chip PH1P35 demo:
//   * drs_iter (N=10, M=49, HB=4, L=60, NODE=2) with the pool datapath
//   * state RAM / KKT RAM in inferred ERAM
//   * on-chip sram64_ctrl (bram) and boot ROM from the ETH image RAM
//   * runs iteration 0 then iteration 1, copying each v[] to vs_ram
// The full G-FOLD problem path remains the CFI-flash + external-SRAM build.
module solver_core #(
    parameter N = 10, M = 49, NNZ = 147, HB = 4,
    parameter L = 60, LM1 = 59,
    parameter RAM_AW = 10, SRAM_AW = 12,
    parameter NODE = 2,
    parameter [63:0] SQRT_L = 64'h401efbdeb14f4eda,   // sqrt(60)
    parameter RY_FILE = "../../../rtl/data/kkt/small2/r_y.hex",
    parameter DY_FILE = "../../../rtl/data/kkt/small2/Dy.hex"
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,      // pulse: boot + run iter0 + iter1
    output reg         done,
    output wire [9:0]  img_addr,
    input  wire [63:0] img_rdata,
    output reg         vs_we,
    output reg  [7:0]  vs_waddr,
    output reg  [63:0] vs_wdata
);
    // ---- control / drs_iter handshake (declared before the instance) ----
    reg        start_r;
    reg        started;
    reg  [RAM_AW-1:0] ci;
    wire       dut_done;
    wire       dut_idle;
    reg [15:0] iter_r;

    // ---- state RAM ----
    reg [63:0] smem [0:(1<<RAM_AW)-1];
    wire [RAM_AW-1:0] ram_addr;
    wire [63:0] ram_wdata;
    wire        ram_we;
    wire [63:0] ram_rdata;

    // ---- KKT RAM ----
    wire [13:0] kkt_addr;
    wire [63:0] kkt_wdata;
    wire        kkt_we;
    reg  [63:0] kmem [0:255];
    reg  [63:0] kkt_rdata;

    // ---- boot ROM ----
    wire        boot_req;
    wire [19:0] boot_waddr;
    wire [63:0] boot_rdata;
    wire        boot_busy, boot_ready;

    // ---- copy FSM read-port mux (drs_iter idle while copying) ----
    // copy read address is ci (combinational), see ram_addr_m
    reg               copy_en;
    wire [RAM_AW-1:0] ram_addr_m = copy_en ? ci : ram_addr;
    reg  [63:0] ram_rdata_r;

    always @(posedge clk) ram_rdata_r <= smem[ram_addr_m];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    assign ram_rdata = ram_rdata_r;

    always @(posedge clk) kkt_rdata <= kmem[kkt_addr[7:0]];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr[7:0]] <= kkt_wdata;

    drs_iter #(
        .N(N), .M(M), .NNZ(NNZ), .HB(HB), .L(L), .LM1(LM1),
        .RAM_AW(RAM_AW), .SRAM_AW(SRAM_AW), .BOOT_EXT(1), .SCALE_EN(0),
        .NODE(NODE), .SQRT_L(SQRT_L),
        .RY_FILE(RY_FILE), .DY_FILE(DY_FILE)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start_r),
        .refactor(iter_r == 0), .band_in(64'h0), .band_valid(1'b0),
        .iter(iter_r), .scale_valid(1'b0), .scale_r(64'h3FF0000000000000),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we),
        .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we),
        .kkt_rdata(kkt_rdata),
        .done(dut_done), .idle(dut_idle), .band_ready(),
        .SRAM_ADDR(), .SRAM_DQ(), .SRAM_CE_N(), .SRAM_OE_N(), .SRAM_WE_N(),
        .SRAM_UB_N(), .SRAM_LB_N(),
        .FL_ADDR(), .FL_DQ(), .FL_CE_N(), .FL_OE_N(), .FL_WE_N(),
        .FL_RESET_N(), .FL_WP_N(), .FL_RY(1'b1),
        .boot_req(boot_req), .boot_waddr(boot_waddr), .boot_rdata(boot_rdata),
        .boot_busy(boot_busy), .boot_ready(boot_ready)
    );

    boot_rom_ctrl #(.AW(20)) u_boot (
        .clk(clk), .rst_n(rst_n), .req(boot_req), .waddr(boot_waddr),
        .busy(boot_busy), .ready(boot_ready), .rdata(boot_rdata),
        .img_addr(img_addr), .img_rdata(img_rdata)
    );

    // ---- run control: boot, iter0, copy VS1, iter1, copy VS2 ----
    localparam C_IDLE=0, C_BOOT=1, C_S0=2, C_W0=3, C_P0=4, C_CA0=5, C_CP0=6,
               C_S1=7, C_W1=8, C_P1=9, C_CA1=10, C_CP1=11, C_DONE=12;
    reg [3:0]  cst;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cst <= C_IDLE; start_r <= 0; iter_r <= 0; done <= 0; started <= 0;
            vs_we <= 0; vs_waddr <= 0; vs_wdata <= 0;
             copy_en <= 0; ci <= 0;
        end else begin
            start_r <= 0; vs_we <= 0;
            case (cst)
            C_IDLE: begin

                if (start && !started) begin started <= 1; iter_r <= 0; cst <= C_BOOT; end
            end
            C_BOOT: begin
                if (dut_idle) cst <= C_S0;   // boot FSM reached S_IDLE
            end
            C_S0: begin start_r <= 1; iter_r <= 0; cst <= C_W0; end
            C_W0: begin
                if (dut_done) begin ci <= 0; copy_en <= 1;  cst <= C_P0; end
            end
            C_P0: begin cst <= C_CA0; end      // prime the copy read port
            C_CA0: begin  cst <= C_CP0; end
            C_CP0: begin
                vs_we <= 1; vs_waddr <= ci; vs_wdata <= ram_rdata_r;
                if (ci + 1 >= L) begin
                    copy_en <= 0; cst <= C_S1;
                end else begin
                    ci <= ci + 1; cst <= C_CA0;
                end
            end
            C_S1: begin start_r <= 1; iter_r <= 1; cst <= C_W1; end
            C_W1: begin
                if (dut_done) begin ci <= 0; copy_en <= 1;  cst <= C_P1; end
            end
            C_P1: begin cst <= C_CA1; end
            C_CA1: begin  cst <= C_CP1; end
            C_CP1: begin
                vs_we <= 1; vs_waddr <= L + ci; vs_wdata <= ram_rdata_r;
                if (ci + 1 >= L) begin
                    copy_en <= 0; cst <= C_DONE;
                end else begin
                    ci <= ci + 1; cst <= C_CA1;
                end
            end
            C_DONE: begin done <= 1; cst <= C_IDLE; end
            default: cst <= C_IDLE;
            endcase
        end
    end
endmodule
