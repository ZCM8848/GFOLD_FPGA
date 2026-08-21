`timescale 1ns/1ps
// FP64 unit testbench: mul/add/sub/div against known IEEE-754 double values.
module tb_fp64;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0;

    // --- mul (combinational) ---
    reg [63:0] ma, mb; wire [63:0] mo;
    fp64_mul um(ma, mb, mo);
    // --- add (combinational) ---
    reg [63:0] aa, ab; reg asub; wire [63:0] ao;
    fp64_add ua(aa, ab, asub, ao);
    // --- div (sequential) ---
    reg dstart; reg [63:0] da, db; wire ddone; wire [63:0] do_;
    fp64_div ud(clk, dstart, da, db, ddone, do_);

    integer npass = 0, nfail = 0;
    task chk(input [127:0] name, input [63:0] got, input [63:0] exp);
        begin
            if (got === exp) begin npass = npass+1; $display("PASS %s = %016x", name, got); end
            else begin nfail = nfail+1; $display("FAIL %s got=%016x exp=%016x", name, got, exp); end
        end
    endtask

    task divchk(input [127:0] name, input [63:0] a, b, input [63:0] exp);
        begin
            da = a; db = b; dstart = 1; @(posedge clk); #1 dstart = 0;
            wait (ddone); #1;
            chk(name, do_, exp);
        end
    endtask

    initial begin
        #20;
        // --- multiply ---
        ma=64'h4004000000000000; mb=64'h4010000000000000; #1; chk("mul 2.5*4", mo, 64'h4024000000000000);
        ma=64'hC008000000000000; mb=64'h4000000000000000; #1; chk("mul -3*2", mo, 64'hC018000000000000);
        ma=64'h3FE0000000000000; mb=64'h3FD0000000000000; #1; chk("mul .5*.25", mo, 64'h3FC0000000000000);
        ma=64'h7E37E43C8800759C; mb=64'h4000000000000000; #1; chk("mul 1e300*2", mo, 64'h7E47E43C8800759C);
        ma=64'h0000000000000000; mb=64'h4014000000000000; #1; chk("mul 0*5", mo, 64'h0000000000000000);
        ma=64'hBFF8000000000000; mb=64'h4004000000000000; #1; chk("mul -1.5*2.5", mo, 64'hC00E000000000000);
        ma=64'h400921F9F01B866E; mb=64'h4005BF0995AAF790; #1; chk("mul pi*e", mo, 64'h40211456587DFABF);

        // --- add/sub ---
        aa=64'h4004000000000000; ab=64'h4010000000000000; asub=0; #1; chk("add 2.5+4", ao, 64'h401A000000000000);
        aa=64'h4014000000000000; ab=64'h4008000000000000; asub=1; #1; chk("sub 5-3", ao, 64'h4000000000000000);
        aa=64'h3DDB7CDFD9D7BDBB; ab=64'h3FF0000000000000; asub=0; #1; chk("add 1e-10+1", ao, 64'h3FF000000006DF37);
        aa=64'hBFE0000000000000; ab=64'h3FD0000000000000; asub=0; #1; chk("add -0.5+0.25", ao, 64'hBFD0000000000000);
        aa=64'h3FB999999999999A; ab=64'h3FC999999999999A; asub=0; #1; chk("add 0.1+0.2", ao, 64'h3FD3333333333333);

        // --- divide (sequential) ---
        divchk("div 10/4", 64'h4024000000000000, 64'h4010000000000000, 64'h4004000000000000);
        divchk("div 1/2000", 64'h3FF0000000000000, 64'h409F400000000000, 64'h3F40624DD2F1A9FB);
        divchk("div 2/0", 64'h4000000000000000, 64'h0000000000000000, 64'h7FF0000000000000);
        divchk("div 0/5", 64'h0000000000000000, 64'h4014000000000000, 64'h0000000000000000);
        divchk("div 7/2", 64'h401C000000000000, 64'h4000000000000000, 64'h400C000000000000);
        divchk("div -9/3", 64'hC022000000000000, 64'h4008000000000000, 64'hC008000000000000);

        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS fp64 units"); else $display("FAIL fp64 units");
        $finish;
    end
endmodule
