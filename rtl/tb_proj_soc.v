`timescale 1ns/1ps
// proj_soc testbench (flattened ports): drives DIM=4 SOC vectors, dumps output.
// Python decodes + compares against numpy proj_soc (authoritative).
module tb_proj_soc;
    parameter DIM = 4;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    reg [DIM*64-1:0] v;
    wire done; wire [DIM*64-1:0] o;
    proj_soc #(.DIM(DIM)) dut(.clk(clk), .rst_n(rst_n), .start(start),
                              .v(v), .done(done), .o(o));
    integer fd, j;

    task setv(input [63:0] a, b, c, d);
        begin
            v[0*64 +: 64] = a; v[1*64 +: 64] = b;
            v[2*64 +: 64] = c; v[3*64 +: 64] = d;
        end
    endtask
    task runcase(input [127:0] name);
        begin
            start = 1; @(posedge clk); #1 start = 0;
            wait (done); #1;
            for (j = 0; j < DIM; j = j + 1) $fwriteh(fd, o[j*64 +: 64], "\n");
            $display("  %0s done", name);
        end
    endtask

    initial begin
        fd = $fopen("proj_soc_out.mem", "w");
        #20 rst_n = 1;
        setv(64'h4024000000000000, 64'h3FF0000000000000, 64'h4000000000000000, 64'h4008000000000000);
        runcase("inside");
        setv(64'hBFF0000000000000, 64'h3FE0000000000000, 64'h3FE0000000000000, 64'h3FE0000000000000);
        runcase("outside");
        setv(64'h3FF0000000000000, 64'h4000000000000000, 64'h4008000000000000, 64'h4010000000000000);
        runcase("project");
        setv(64'h4014000000000000, 64'h4008000000000000, 64'h4010000000000000, 64'h0000000000000000);
        runcase("boundary_inside");
        setv(64'hC024000000000000, 64'h4000000000000000, 64'h4008000000000000, 64'h4010000000000000);
        runcase("outside2");
        $fclose(fd);
        $display("proj_soc dump complete (5 cases, DIM=%0d)", DIM);
        $finish;
    end
endmodule
