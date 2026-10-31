// dc_ram_128x8.v — 双时钟简单双口 RAM（写 RX_CLK，读组合）
// 用于 ICMP echo 数据的跨时钟域传递。异步读 → 推断分布式 RAM。
`timescale 1ns/1ps

module dc_ram_128x8 (
    input  wire        wclk,
    input  wire        wen,
    input  wire [6:0]  waddr,
    input  wire [7:0]  wdata,
    input  wire [6:0]  raddr,
    output wire [7:0]  rdata
);
    reg [7:0] mem [0:127];
    always @(posedge wclk) if (wen) mem[waddr] <= wdata;
    assign rdata = mem[raddr];
endmodule
