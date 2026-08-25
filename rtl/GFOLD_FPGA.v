// ============================================================
// GFOLD_FPGA — top-level for DE2-115 (EP4CE115F29C7)
// MINIMAL SYNTACTIC TOP for Quartus resource (M9K) verification:
// instantiates drs_iter + internal main/KKT RAM. Data not yet loaded;
// purpose is to measure real M9K / LE usage before RAM compression.
// ============================================================
`timescale 1ns/1ps
module GFOLD_FPGA (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG
);
    localparam N = 1100, M = 2107, NNZ = 4783, HB = 17;

    wire clk = CLOCK_50;
    wire rst_n = KEY[0];

    // ---- drs_iter ----
    wire start, done;
    wire [15:0] iter;
    wire refactor, band_valid;
    wire [63:0] band_in;
    wire scale_valid;
    wire [63:0] scale_r;
    wire [16:0] ram_addr;
    wire [63:0] ram_wdata;
    wire        ram_we;
    wire [63:0] ram_rdata;
    wire [13:0] kkt_addr;
    wire [63:0] kkt_wdata;
    wire        kkt_we;
    wire [63:0] kkt_rdata;

    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done));

    // ---- main RAM (internal M9K) ----
    reg [63:0] smem [0:131071];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    assign ram_rdata = smem[ram_addr];

    // ---- KKT RAM ----
    reg [63:0] kmem [0:32767];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;
    assign kkt_rdata = kmem[kkt_addr];

    // ---- one-shot start after reset ----
    reg [3:0] cnt = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      cnt <= 0;
        else if (cnt < 10) cnt <= cnt + 1;
    end
    assign start = (cnt == 4'd10) && !done;
    assign refactor   = 1'b1;
    assign band_valid = 1'b0;
    assign band_in    = 64'd0;
    assign iter       = 16'd0;
    assign scale_valid= 1'b0;
    assign scale_r    = 64'h3FF0000000000000;

    // ---- status LEDs ----
    assign LEDR = { done, armed, cnt[2:0], 12'd0 };
    wire armed = (cnt == 4'd10);
    assign LEDG = 9'd0;
endmodule
