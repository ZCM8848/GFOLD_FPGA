`timescale 1ns/1ps
// TB for banded_ldl_fp64_rt: stream N rhs words in, capture N zx words out.
// WHICH=0 -> small (N=8,HB=2, small_f64); WHICH=1 -> full (N=1100,HB=17, full_f64).
// Literal filenames (case) — $sformatf hangs ModelSim 10.5b.
module tb_ldl_rt;
    parameter N = 8, HB = 2, WHICH = 0;
    parameter BAND_FILE = "../data/small_f64/band_f64.hex";
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, rhs_valid = 0;
    reg [63:0] rhs_in;
    wire [63:0] zx_out; wire zx_valid, done; wire [7:0] status;
    banded_ldl_fp64_rt #(.N(N), .HB(HB), .BAND_FILE(BAND_FILE)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),
        .rhs_in(rhs_in),.rhs_valid(rhs_valid),
        .zx_out(zx_out),.zx_valid(zx_valid),.done(done),.status(status));

    reg [63:0] rhs [0:2047];
    reg [63:0] got [0:2047];
    integer k, fd;
    initial begin
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        case (WHICH)
            0: $readmemh("../data/small_f64/rhs_f64.hex", rhs);
            1: $readmemh("../data/full_f64/rhs_f64.hex", rhs);
        endcase
        @(negedge clk); start = 1; rhs_valid = 0;
        @(negedge clk); start = 0;
        for (k = 0; k < N; k = k + 1) begin
            @(negedge clk); rhs_in = rhs[k]; rhs_valid = 1;
        end
        @(negedge clk); rhs_valid = 0;
        k = 0;
        while (k < N) begin
            @(posedge clk);
            if (zx_valid) begin got[k] = zx_out; k = k + 1; end
        end
        while (!done) @(posedge clk);
        $write("CASE 0:");
        for (k = 0; k < N; k = k + 1) $write(" %h", got[k]);
        $write("\n");
        $display("ALL DONE");
        $finish;
    end
endmodule
