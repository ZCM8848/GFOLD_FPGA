`timescale 1ns/1ps
// Parametrized FP64 banded-LDL testbench: drives banded_ldl_fp64, loads exp_zx_f64
// and compares dut.zx (float64 decode). Compares small and full-size cases.
module tb_banded_ldl_fp64;
    parameter N = 8, HB = 2,
              BAND = "../data/small_f64/band_f64.hex",
              RHS  = "../data/small_f64/rhs_f64.hex",
              EXP  = "../data/small_f64/exp_zx_f64.hex";
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    wire done; wire [7:0] status;
    banded_ldl_fp64 #(.N(N), .HB(HB), .BAND_FILE(BAND), .RHS_FILE(RHS))
        dut (.clk(clk), .rst_n(rst_n), .start(start), .done(done), .status(status));

    reg [63:0] exp_zx [0:N-1];
    real max_rel;
    integer fd, di, npass = 0, nfail = 0;

    // IEEE-754 double -> real
    function real f2r(input [63:0] w);
        real s, m, p;
        integer e;
        begin
            s = w[63] ? -1.0 : 1.0;
            e = $signed(w[62:52]) - 1023;
            m = 1.0 + (w[51:0] / 4503599627370496.0);
            p = 2.0 ** e;
            f2r = s * m * p;
        end
    endfunction

    initial $readmemh(EXP, exp_zx);

    task check;
        integer i; real rel, gv, ev;
        begin
            max_rel = 0.0;
            for (i = 0; i < N; i = i + 1) begin
                gv = f2r(dut.zx[i]); ev = f2r(exp_zx[i]);
                rel = (ev == 0.0) ? (gv == 0.0 ? 0.0 : 1.0) : (gv - ev) / ev;
                if (rel < 0.0) rel = -rel;
                if (rel > max_rel) max_rel = rel;
            end
            $display("  N=%0d HB=%0d max_rel=%.3e", N, HB, max_rel);
            if (max_rel < 1e-6) begin npass = npass+1; $display("  PASS max_rel=%.3e", max_rel); end
            else begin nfail = nfail+1; $display("  FAIL max_rel=%.3e", max_rel); end
        end
    endtask

    initial begin
        $display("== tb fp64: BAND=%0s RHS=%0s EXP=%0s N=%0d HB=%0d ==", BAND, RHS, EXP, N, HB);
        #20 rst_n = 1;
        #20 start = 1; @(posedge clk); #1 start = 0;
        wait (done);
        fd = $fopen("zx_fp64_out.mem", "w");
        for (di = 0; di < N; di = di + 1) $fwriteh(fd, dut.zx[di], "\n");
        $fclose(fd);
        check;
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS banded_ldl_fp64 (N=%0d HB=%0d)", N, HB);
        else $display("FAIL banded_ldl_fp64 (N=%0d HB=%0d)", N, HB);
        $finish;
    end
endmodule
