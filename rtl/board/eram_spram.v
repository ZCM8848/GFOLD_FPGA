`timescale 1ns/1ps
// eram_spram -- single-port 64-bit ERAM wrapper with one-cycle read latency.
// Used for the KKT RAM so TD maps it to ERAM instead of distributed LUTRAM.
module eram_spram #(
    parameter ADDR_WIDTH = 13,
    parameter DATA_DEPTH = 8192,
    parameter DATA_WIDTH = 64
)(
    input  wire                  clk,
    input  wire                  ce,
    input  wire                  we,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata
);
    (* ram_style = "bram" *) reg [DATA_WIDTH-1:0] mem [0:DATA_DEPTH-1];
    reg [DATA_WIDTH-1:0] rdata_r;
    always @(posedge clk) begin
        if (we && ce) mem[addr] <= wdata;
        if (ce) rdata_r <= mem[addr];
    end
    assign rdata = rdata_r;
endmodule
