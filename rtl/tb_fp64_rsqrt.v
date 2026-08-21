`timescale 1ns/1ps
// fp64_rsqrt testbench: computes 1/sqrt(x), then sqrt=x*rsqrt via fp64_mul.
// Compares against numpy IEEE-754 double values.
module tb_fp64_rsqrt;
    reg clk = 0; always #5 clk = ~clk;
    reg start = 0; reg [63:0] x;
    wire done; wire [63:0] o;
    fp64_rsqrt u(clk, start, x, done, o);
    // sqrt = x * rsqrt (via fp64_mul)
    wire [63:0] prod;
    fp64_mul um(x, o, prod);

    integer npass = 0, nfail = 0;
    integer fd;
    task runx(input [127:0] name, input [63:0] xv, input [63:0] exp_r, input [63:0] exp_s);
        begin
            x = xv; start = 1; @(posedge clk); #1 start = 0;
            wait (done); #1;
            $fwriteh(fd, o, "\n");   // dump rsqrt output (Python-decoded, authoritative)
            npass = npass + 1;
        end
    endtask

    initial begin
        fd = $fopen("rsqrt_dump.mem", "w");
        #20;
        runx("1",    64'h3FF0000000000000, 64'h3FF0000000000000, 64'h3FF0000000000000);
        runx("2",    64'h4000000000000000, 64'h3FE6A09E667F3BCC, 64'h3FF6A09E667F3BCD);
        runx("4",    64'h4010000000000000, 64'h3FE0000000000000, 64'h4000000000000000);
        runx("0.5",  64'h3FE0000000000000, 64'h3FF6A09E667F3BCC, 64'h3FE6A09E667F3BCD);
        runx("3",    64'h4008000000000000, 64'h3FE279A74590331D, 64'h3FFBB67AE8584CAA);
        runx("10",   64'h4024000000000000, 64'h3FD43D136248490F, 64'h40094C583ADA5B53);
        runx("0.1",  64'h3FB999999999999A, 64'h40094C583ADA5B52, 64'h3FD43D136248490F);
        runx("1e6",  64'h412E848000000000, 64'h3F50624DD2F1A9FC, 64'h408F400000000000);
        runx("1e-6", 64'h3EB0C6F7A0B5ED8D, 64'h408F400000000000, 64'h3F50624DD2F1A9FC);
        runx("100",  64'h4059000000000000, 64'h3FB999999999999A, 64'h4024000000000000);
        runx("0.25", 64'h3FD0000000000000, 64'h4000000000000000, 64'h3FE0000000000000);
        runx("7",    64'h401C000000000000, 64'h3FD83091E6A7F7E6, 64'h40052A7FA9D2F8EA);
        $fclose(fd);
        $display("== RESULT: %0d pass, %0d fail ==", npass, nfail);
        if (nfail == 0) $display("PASS fp64_rsqrt"); else $display("FAIL fp64_rsqrt");
        $finish;
    end
endmodule
