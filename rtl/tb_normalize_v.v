`timescale 1ns/1ps
// normalize_v testbench (N=4): dumps v_out for Python decode vs numpy.
module tb_normalize_v;
    parameter N = 4;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    reg [N*64-1:0] v_in;
    reg [63:0] sqrtN;
    wire done; wire [N*64-1:0] v_out;
    normalize_v #(.N(N)) dut(.clk(clk), .rst_n(rst_n), .start(start),
                             .v_in(v_in), .sqrtN(sqrtN), .done(done), .v_out(v_out));
    integer fd, j;

    task runcase(input [127:0] name);
        begin
            start = 1; @(posedge clk); #1 start = 0;
            wait (done); #1;
            for (j = 0; j < N; j = j + 1) $fwriteh(fd, v_out[j*64 +: 64], "\n");
            $display("  %0s done", name);
        end
    endtask

    initial begin
        fd = $fopen("normv_out.mem", "w");
        sqrtN = 64'h4000000000000000;   // sqrt(4) = 2.0
        #20 rst_n = 1;
        // v = [1,2,3,4], ||v||=sqrt(30)=5.477, scale=2/5.477=0.3651
        v_in[0*64 +: 64]=64'h3FF0000000000000; v_in[1*64 +: 64]=64'h4000000000000000;
        v_in[2*64 +: 64]=64'h4008000000000000; v_in[3*64 +: 64]=64'h4010000000000000;
        runcase("v1234");
        // v = [-1, 0.5, 2, -3]
        v_in[0*64 +: 64]=64'hBFF0000000000000; v_in[1*64 +: 64]=64'h3FE0000000000000;
        v_in[2*64 +: 64]=64'h4000000000000000; v_in[3*64 +: 64]=64'hC008000000000000;
        runcase("v2");
        $fclose(fd);
        $display("normalize_v dump complete (N=%0d)", N);
        $finish;
    end
endmodule
