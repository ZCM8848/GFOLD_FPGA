`timescale 1ns/1ps
// Parametrized banded-LDL testbench: points at N/HB and hex file paths via
// module parameters, so the same tb drives both the small (8x8,hb2) and
// full-size (1100x1100,hb17) checks. Compares dut.zx against exp_zx (float32).
module tb_banded_ldl;
    parameter N    = 8,
              HB   = 2,
              WB   = 32,
              BAND = "../data/small/band.hex",
              RHS  = "../data/small/rhs.hex",
              EXP  = "../data/small/exp_zx.hex";
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    wire done;
    wire [7:0] status;
    banded_ldl #(.N(N), .HB(HB), .BAND_FILE(BAND), .RHS_FILE(RHS))
        dut (.clk(clk), .rst_n(rst_n), .start(start), .done(done), .status(status));

    reg [WB-1:0] exp_zx [0:N-1];
    integer npass = 0, nfail = 0;
    real max_rel;
    integer fd, di;

    function real f2r(input [31:0] w);
        real s; integer e; real m;
        begin
            s = w[31] ? -1.0 : 1.0;
            e = w[30:23] - 127;
            m = 1.0 + (w[22:0] / 8388608.0);
            f2r = s * m * (2.0 ** e);
        end
    endfunction

    initial $readmemh(EXP, exp_zx);

    task check;
        integer i; real rel, gv, ev;
        begin
            max_rel = 0.0;
            for (i = 0; i < N; i = i + 1) begin
                gv = f2r(dut.zx[i]);
                ev = f2r(exp_zx[i]);
                rel = (ev == 0.0) ? (gv == 0.0 ? 0.0 : 1.0)
                                  : (gv - ev) / ev;
                if (rel < 0.0) rel = -rel;
                if (rel > max_rel) max_rel = rel;
            end
            $display("  N=%0d HB=%0d max_rel=%.3e (first 6 zx:", N, HB, max_rel);
            for (i = 0; i < 6 && i < N; i = i + 1)
                $display("    zx[%0d]=%.6f exp=%.6f", i, f2r(dut.zx[i]), f2r(exp_zx[i]));
            if (max_rel < 1e-4) begin npass = npass + 1; $display("  PASS max_rel=%.3e", max_rel); end
            else begin nfail = nfail + 1; $display("  FAIL max_rel=%.3e", max_rel); end
        end
    endtask

    initial begin
        $display("== tb: BAND=%0s RHS=%0s EXP=%0s N=%0d HB=%0d ==", BAND, RHS, EXP, N, HB);
        #20 rst_n = 1;
        #20 start = 1; @(posedge clk); #1 start = 0;
        wait (done);
        $display("== banded LDL done (N=%0d HB=%0d), comparing zx ==", N, HB);
        // dump RTL zx as raw hex for Python-side residual/emulation comparison
        fd = $fopen("zx_out.mem", "w");
        for (di = 0; di < N; di = di + 1) $fwriteh(fd, dut.zx[di], "\n");
        $fclose(fd);
        fd = $fopen("B_fact.mem", "w");
        for (di = 0; di < (HB+1)*N; di = di + 1) $fwriteh(fd, dut.B[di], "\n");
        $fclose(fd);
        fd = $fopen("y_out.mem", "w");
        for (di = 0; di < N; di = di + 1) $fwriteh(fd, dut.y[di], "\n");
        $fclose(fd);
        $display("  dumped dut.zx/B_fact/y -> zx_out/B_fact/y_out.mem");
        check;
        $display("  deep zx[%0d]=%.6f exp=%.6f", N/2, f2r(dut.zx[N/2]), f2r(exp_zx[N/2]));
        $display("  deep zx[%0d]=%.6f exp=%.6f", N-1, f2r(dut.zx[N-1]), f2r(exp_zx[N-1]));
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS banded_ldl (N=%0d HB=%0d)", N, HB);
        else $display("FAIL banded_ldl (N=%0d HB=%0d)", N, HB);
        $finish;
    end
endmodule
