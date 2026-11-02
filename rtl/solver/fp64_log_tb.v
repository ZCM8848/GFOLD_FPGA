`timescale 1ns/1ps
module tb_fp64_log;
    reg [63:0] xm [0:1023];
    reg [63:0] em [0:1023];
    reg [63:0] xr;
    integer i;
    wire [63:0] o;
    fp64_log dut(xr, o);
    initial begin
        $readmemh("../data/log/x.hex", xm);
        $readmemh("../data/log/ln.hex", em);
        for (i = 0; i < 1024; i = i + 1) begin
            xr = xm[i];
            #1;
            $display("LOG x=%h o=%h exp=%h", xm[i], o, em[i]);
        end
        $finish;
    end
endmodule
