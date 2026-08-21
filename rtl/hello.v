`timescale 1ns/1ps
// Minimal module to verify the ModelSim command-line flow end-to-end.
module hello #(parameter W = 32) (
    input  wire       clk,
    input  wire       rst_n,
    output reg  [W-1:0] cnt
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cnt <= 0;
        else        cnt <= cnt + 1'b1;
    end
endmodule
