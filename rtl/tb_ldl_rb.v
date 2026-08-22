`timescale 1ns/1ps
// TB for banded_ldl_fp64_rb: stream band ((HB+1)*N) then rhs (N) in,
// capture N zx words out, print "CASE 0: ..." for check_ldl_rt.py.
// WHICH=0 -> small (N=8,HB=2, small_f64); WHICH=1 -> full (N=1100,HB=17, full_f64).
// Literal filenames via case(WHICH) — $sformatf hangs ModelSim 10.5b.
module tb_ldl_rb;
    parameter N = 8, HB = 2, WHICH = 0;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, band_valid = 0, rhs_valid = 0;
    reg [63:0] band_in, rhs_in;
    wire [63:0] zx_out; wire zx_valid, done; wire [7:0] status;
    banded_ldl_fp64_rb #(.N(N), .HB(HB)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),
        .band_in(band_in),.band_valid(band_valid),
        .rhs_in(rhs_in),.rhs_valid(rhs_valid),
        .zx_out(zx_out),.zx_valid(zx_valid),.done(done),.status(status));

    reg [63:0] band [0:32767];
    reg [63:0] rhs [0:2047];
    reg [63:0] got [0:2047];
    integer k, fd;
    initial begin
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        case (WHICH)
            0: begin
                $readmemh("../data/small_f64/band_f64.hex", band);
                $readmemh("../data/small_f64/rhs_f64.hex", rhs);
            end
            1: begin
                $readmemh("../data/full_f64/band_f64.hex", band);
                $readmemh("../data/full_f64/rhs_f64.hex", rhs);
            end
        endcase
        @(negedge clk); start = 1; band_valid = 0; rhs_valid = 0;
        @(negedge clk); start = 0;
        for (k = 0; k < (HB+1)*N; k = k + 1) begin @(negedge clk); band_in = band[k]; band_valid = 1; end
        @(negedge clk); band_valid = 0;
        for (k = 0; k < N; k = k + 1) begin @(negedge clk); rhs_in = rhs[k]; rhs_valid = 1; end
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
