`timescale 1ns/1ps
module tb_banded_ldl;
    localparam N = 8, HB = 2, WB = 32;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    wire done;
    wire [7:0] status;
    banded_ldl #(.N(N), .HB(HB)) dut (.clk(clk), .rst_n(rst_n), .start(start),
                                       .done(done), .status(status));

    reg [WB-1:0] exp_zx [0:N-1];
    integer npass = 0, nfail = 0;
    real max_rel;

    function real f2r(input [31:0] w);
        real s; integer e; real m;
        begin
            s = w[31] ? -1.0 : 1.0;
            e = w[30:23] - 127;
            m = 1.0 + (w[22:0] / 8388608.0);
            f2r = s * m * (2.0 ** e);
        end
    endfunction

    initial $readmemh("../data/small/exp_zx.hex", exp_zx);

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
                $display("  zx[%0d] got=%08x (%.6f) exp=%08x (%.6f) rel=%.3e", i,
                         dut.zx[i], gv, exp_zx[i], ev, rel);
            end
            if (max_rel < 1e-5) begin npass = npass + 1; $display("PASS zx max_rel=%.3e", max_rel); end
            else begin nfail = nfail + 1; $display("FAIL zx max_rel=%.3e", max_rel); end
        end
    endtask

    initial begin
        #20 rst_n = 1;
        #20 start = 1; @(posedge clk); #1 start = 0;
        wait (done);
        $display("== banded LDL done, comparing zx ==");
        check;
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS banded_ldl"); else $display("FAIL banded_ldl");
        $finish;
    end
endmodule
