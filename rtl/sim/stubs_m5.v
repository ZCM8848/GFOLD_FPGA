`timescale 1ns/1ps
// Simulation-only stubs for Anlogic vendor primitives used by the M5 top.
module PLL(input refclk, output clk0_out, output clk1_out,
           output clk4_out, output clk5_out, output lock, input reset);
    assign clk0_out = refclk;
    assign clk1_out = refclk;
    assign clk4_out = refclk;
    assign clk5_out = refclk;
    assign lock = 1'b1;
endmodule

module PH1P_LOGIC_HD_IDDR #(parameter MODE = "OPPOSITE_EDGE")
    (output q0, output q1, input clk, input d, input rst);
    assign q0 = d;
    assign q1 = d;
endmodule

module PH1P_LOGIC_HD_ODDR
    (output q, input clk, input d0, input d1, input rst);
    assign q = d0;
endmodule
