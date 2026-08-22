`timescale 1ns/1ps
// TB for banded_ldl_fp64_rb: refactor mode test.
// REF=0: single run refactor=1 (band + rhs), print CASE 0 (compat with
//        check_ldl_rt.py).
// REF=1: two runs — first refactor=1 (stream band + rhs), second refactor=0
//        (rhs only, reuses L/D from run 1); zx must be BIT-IDENTICAL (same
//        band, same rhs, no re-factorization -> same solve). CASE 0 = run 1,
//        CASE 1 = run 2.
// WHICH=0 -> small (N=8,HB=2, small_f64); WHICH=1 -> full (N=1100,HB=17, full_f64).
// Literal filenames via case(WHICH) — $sformatf hangs ModelSim 10.5b.
module tb_ldl_rb;
    parameter N = 8, HB = 2, WHICH = 0, REF = 1;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, refactor = 0, band_valid = 0, rhs_valid = 0;
    reg [63:0] band_in, rhs_in;
    wire [63:0] zx_out; wire zx_valid, done; wire [7:0] status;
    banded_ldl_fp64_rb #(.N(N), .HB(HB)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),.refactor(refactor),
        .band_in(band_in),.band_valid(band_valid),
        .rhs_in(rhs_in),.rhs_valid(rhs_valid),
        .zx_out(zx_out),.zx_valid(zx_valid),.done(done),.status(status));

    reg [63:0] band [0:32767];
    reg [63:0] rhs [0:2047];
    reg [63:0] got1 [0:2047], got2 [0:2047];
    integer k, cnt, nbad;

    task run_solve;
        input rf;
        begin
            @(negedge clk); start = 1; refactor = rf; band_valid = 0; rhs_valid = 0;
            @(negedge clk); start = 0;
            if (rf) begin
                for (k = 0; k < (HB+1)*N; k = k + 1) begin @(negedge clk); band_in = band[k]; band_valid = 1; end
                @(negedge clk); band_valid = 0;
            end
            for (k = 0; k < N; k = k + 1) begin @(negedge clk); rhs_in = rhs[k]; rhs_valid = 1; end
            @(negedge clk); rhs_valid = 0;
            cnt = 0;
            while (cnt < N) begin
                @(posedge clk);
                if (zx_valid) begin
                    if (rf == 1) got1[cnt] = zx_out; else got2[cnt] = zx_out;
                    cnt = cnt + 1;
                end
            end
            while (!done) @(posedge clk);
        end
    endtask

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
        run_solve(1);                       // refactor=1: factor + solve
        $write("CASE 0:");
        for (k = 0; k < N; k = k + 1) $write(" %h", got1[k]);
        $write("\n");
        if (REF) begin
            run_solve(0);                   // refactor=0: reuse L/D, same solve
            $write("CASE 1:");
            for (k = 0; k < N; k = k + 1) $write(" %h", got2[k]);
            $write("\n");
            nbad = 0;
            for (k = 0; k < N; k = k + 1)
                if (got1[k] != got2[k]) nbad = nbad + 1;
            $display("REFACTOR-0 vs REFACTOR-1: %0d bit mismatches of %0d", nbad, N);
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
