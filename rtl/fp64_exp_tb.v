`timescale 1ns/1ps
module tb_fp64_exp;
    reg [63:0] xm [0:1023];
    reg [63:0] em [0:1023];
    reg [63:0] xr;
    integer i;
    wire [63:0] o;
    fp64_exp dut(xr, o);
    initial begin
        $readmemh("../data/exp/x.hex", xm);
        $readmemh("../data/exp/e.hex", em);
        for (i = 0; i < 1024; i = i + 1) begin
            xr = xm[i];
            #1;
            $display("EXP x=%h o=%h exp=%h", xm[i], o, em[i]);
        end
        $finish;
    end
endmodule
