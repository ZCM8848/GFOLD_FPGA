`timescale 1ns/1ps
module tb_hello;
    reg clk = 0;
    reg rst_n = 0;
    wire [31:0] cnt;
    hello #(.W(32)) dut (.clk(clk), .rst_n(rst_n), .cnt(cnt));

    always #5 clk = ~clk;  // 100 MHz

    initial begin
        #10 rst_n = 1;
        #50 $display("hello: cnt=%0d (expected ~6-7)", cnt);
        if (cnt >= 5 && cnt <= 8) $display("PASS: hello flow works");
        else begin $display("FAIL: unexpected cnt=%0d", cnt); $fatal; end
        #20 $finish;
    end
endmodule
