`timescale 1ns/1ps
module tb_float32;
    reg clk = 0; always #5 clk = ~clk;
    reg [31:0] a, b;
    wire [31:0] pm, pa, ps;
    fp32_mul um(a, b, pm);
    fp32_add ua(a, b, 1'b0, pa);   // a+b
    fp32_add us(a, b, 1'b1, ps);   // a-b
    // div
    reg start = 0;
    wire done;
    wire [31:0] pd;
    fp32_div ud(/*clk*/clk, start, a, b, done, pd);
    integer npass = 0, nfail = 0;
    task check(input [31:0] got, expected, input [40*8:0] name);
        begin
            if (got == expected) begin npass = npass + 1; $display("PASS %0s  %08x", name, got); end
            else begin nfail = nfail + 1; $display("FAIL %0s  got %08x want %08x", name, got, expected); end
        end
    endtask

    initial begin
        // ---- combinational mul / add / sub ----
        a = 32'h3F800000; b = 32'h40000000;  // 1.0 * 2.0
        #1 check(pm, 32'h40000000, "1.0*2.0"); check(pa, 32'h40400000, "1.0+2.0"); check(ps, 32'hBF800000, "1.0-2.0");
        a = 32'h40600000; b = 32'h40000000;  // 3.5 * 2.0
        #1 check(pm, 32'h40E00000, "3.5*2.0"); check(pa, 32'h40B00000, "3.5+2.0"); check(ps, 32'h3FC00000, "3.5-2.0");
        a = 32'h3F000000; b = 32'h3F000000;  // 0.5 * 0.5
        #1 check(pm, 32'h3E800000, "0.5*0.5"); check(pa, 32'h3F800000, "0.5+0.5"); check(ps, 32'h00000000, "0.5-0.5");
        a = 32'hC0600000; b = 32'h40000000;  // -3.5 * 2.0
        #1 check(pm, 32'hC0E00000, "-3.5*2.0");
        a = 32'h40100000; b = 32'h40400000;  // 2.25 + 3.0
        #1 check(pa, 32'h40A80000, "2.25+3.0"); check(ps, 32'hBF400000, "2.25-3.0");
        a = 32'hBF800000; b = 32'h3F800000;  // -1.0 + 1.0
        #1 check(pa, 32'h00000000, "-1.0+1.0");
        // ---- div 7.0 / 2.0 = 3.5 ----
        start = 1; a = 32'h40E00000; b = 32'h40000000;
        @(posedge clk); #1 start = 0;
        wait (done);
        check(pd, 32'h40600000, "7.0/2.0");
        // extra div cases
        start = 1; a = 32'h3F000000; b = 32'h40000000;  // 0.5/2.0 = 0.25
        @(posedge clk); #1 start = 0;
        wait (done);
        check(pd, 32'h3E800000, "0.5/2.0");
        start = 1; a = 32'h40000000; b = 32'hC0000000;  // 2.0/-2.0 = -1.0
        @(posedge clk); #1 start = 0;
        wait (done);
        check(pd, 32'hBF800000, "2.0/-2.0");
        #20;
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS float32 units"); else $display("FAIL float32 units");
        $finish;
    end
endmodule
