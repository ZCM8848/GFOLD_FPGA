`timescale 1ns/1ps
// Edge-case test for fp32_div: 0/d, small/d, d/0, small/small, 1/d.
module tb_div_edge;
    reg clk = 0; always #5 clk = ~clk;
    reg start = 0;
    reg [31:0] a, b;
    wire done;
    wire [31:0] o;
    fp32_div u (clk, start, a, b, done, o);

    function real f2r(input [31:0] w);
        real s; integer e; real m;
        begin
            s = w[31] ? -1.0 : 1.0;
            e = w[30:23] - 127;
            m = 1.0 + (w[22:0] / 8388608.0);
            f2r = s * m * (2.0 ** e);
        end
    endfunction
    // float->bits helper via $bitstoreal is awkward; use literal hex
    reg [31:0] A, B, EO;
    real av, bv, eov;
    integer npass = 0, nfail = 0;

    task run_case;
        input [31:0] av_in, bv_in;
        input [31:0] exp_o;
        begin
            a = av_in; b = bv_in; start = 1; @(posedge clk); #1 start = 0;
            wait (done);
            if (o === exp_o) begin npass = npass+1; $display("PASS %08x/%08x = %08x", a, b, o); end
            else begin nfail = nfail+1;
                $display("FAIL %08x/%08x = %08x (exp %08x)  got %.4e exp %.4e",
                         a, b, o, exp_o, f2r(o), f2r(exp_o));
            end
        end
    endtask

    initial begin
        // 0.0/d  (0x00000000)
        run_case(32'h00000000, 32'h44FA0000 /*2000*/, 32'h00000000);
        // 5.5/2000  (5.5=0x40B00000, 2000=0x44FA0000) =0x3B343958
        run_case(32'h40B00000, 32'h44FA0000, 32'h3B343958);
        // 2000/5.5 ~363.6  (0x43B5D174)
        run_case(32'h44FA0000, 32'h40B00000, 32'h43B5D174);
        // 0.0/5.5
        run_case(32'h00000000, 32'h40B00000, 32'h00000000);
        // 2000/0.0 -> inf (0x7F800000)
        run_case(32'h44FA0000, 32'h00000000, 32'h7F800000);
        // 1.0/2000 ~0.0005 (0x3A03126F)
        run_case(32'h3F800000, 32'h44FA0000, 32'h3A03126F);
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        $finish;
    end
endmodule
