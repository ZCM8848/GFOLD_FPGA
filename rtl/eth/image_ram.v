// image_ram.v — 1024×64 双时钟简单双口 RAM
// 写口在 RX_CLK 域（LOAD payload），读口在 clk_125 域（DUMP 回读）。
`timescale 1ns/1ps

module image_ram (
    input  wire        wclk,
    input  wire        wen,
    input  wire [9:0]  waddr,
    input  wire [63:0] wdata,
    input  wire        rclk,
    input  wire [9:0]  raddr,
    output reg  [63:0] rdata
);
    reg [63:0] mem [0:1023];
    always @(posedge wclk) if (wen) mem[waddr] <= wdata;
    always @(posedge rclk) rdata <= mem[raddr];
endmodule
