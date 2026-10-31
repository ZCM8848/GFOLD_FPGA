// crc32_eth_nib.v — 以太网 FCS 的 CRC-32（半字节查表，组合 next + 寄存）
//
// 标准：poly 0xEDB88320（反射），初值 0xFFFFFFFF，结果按位取反。
// 用 16 项半字节表分两步算一个字节，比 256 项字节表省资源、比逐位 8 级快。
//
// 用法：
//   - init 脉冲把 crc 置 0xFFFFFFFF
//   - en=1 时每个时钟吃一个字节
//   - 帧结束时取 ~crc 即为 FCS（小端序：bit[7:0] 先发）
//
`timescale 1ns/1ps

module crc32_eth_nib (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init,
    input  wire        en,
    input  wire [7:0]  data,
    output reg  [31:0] crc,
    output wire [31:0] crc_next
);

    function [31:0] nib(input [3:0] n);
        begin
            case (n)
                4'd0 : nib = 32'h00000000;
                4'd1 : nib = 32'h1DB71064;
                4'd2 : nib = 32'h3B6E20C8;
                4'd3 : nib = 32'h26D930AC;
                4'd4 : nib = 32'h76DC4190;
                4'd5 : nib = 32'h6B6B51F4;
                4'd6 : nib = 32'h4DB26158;
                4'd7 : nib = 32'h5005713C;
                4'd8 : nib = 32'hEDB88320;
                4'd9 : nib = 32'hF00F9344;
                4'd10: nib = 32'hD6D6A3E8;
                4'd11: nib = 32'hCB61B38C;
                4'd12: nib = 32'h9B64C2B0;
                4'd13: nib = 32'h86D3D2D4;
                4'd14: nib = 32'hA00AE278;
                4'd15: nib = 32'hBDBDF21C;
            endcase
        end
    endfunction

    wire [31:0] a = crc ^ { 28'd0, data[3:0] };
    wire [31:0] b = (a >> 4) ^ nib(a[3:0]);
    wire [31:0] c = b ^ { 28'd0, data[7:4] };

    assign crc_next = (c >> 4) ^ nib(c[3:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)   crc <= 32'hFFFFFFFF;
        else if (init) crc <= 32'hFFFFFFFF;
        else if (en)   crc <= crc_next;
    end

endmodule
