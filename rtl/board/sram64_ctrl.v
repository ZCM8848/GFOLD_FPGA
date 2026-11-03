`timescale 1ns/1ps
// sram64_ctrl (on-chip board variant) — same port list as the DE2-115 external
// SRAM controller, but backed by inferred ERAM. Used for the PH1P35 small
// on-chip solver demo (AW=12 -> 4096 x 64-bit = 32 KB).
// Protocol is unchanged: pulse req; busy rises for one cycle; rdata is valid
// when busy falls (reads). Writes complete when busy falls.
module sram64_ctrl #(parameter AW = 12) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          req,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [63:0]   wdata,
    output reg           busy,
    output reg  [63:0]   rdata,
    // physical SRAM pins: unused on-chip (tied off by the board top)
    output wire [19:0]   SRAM_ADDR,
    inout  wire [15:0]   SRAM_DQ,
    output wire          SRAM_CE_N,
    output wire          SRAM_OE_N,
    output wire          SRAM_WE_N,
    output wire          SRAM_UB_N,
    output wire          SRAM_LB_N
);
    reg [63:0] mem [0:(1<<AW)-1];
    reg        req_p;

    assign SRAM_ADDR = 20'd0;
    assign SRAM_CE_N = 1'b1;
    assign SRAM_OE_N = 1'b1;
    assign SRAM_WE_N = 1'b1;
    assign SRAM_UB_N = 1'b1;
    assign SRAM_LB_N = 1'b1;
    assign SRAM_DQ   = 16'hz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; rdata <= 0; req_p <= 0;
        end else begin
            req_p <= req;
            if (req && !req_p) begin
                busy <= 1;
                if (we) mem[waddr] <= wdata;
                else    rdata <= mem[waddr];
            end else if (busy) begin
                busy <= 0;
            end
        end
    end
endmodule
