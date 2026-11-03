// fp64_pool.v — 全局共享 FP64 运算池（PH1P35 版）
// 3 个乘法 + 3 个加法（组合）+ 1 个除法（时序）
// 加上共享 fp64_rsqrt 内部的 1 个乘法：4 个 53x53 乘法，共 40 DSP。
// 4 个乘法各分 10 DSP，避免某个乘法溢出到 LUT（之前的 4+rsqrt 版本
// 因为 5 个乘法抢 40 DSP，第 4/5 个乘法大量落到 LUT，slice 超预算）。
`timescale 1ns/1ps

module fp64_pool (
    input  wire        clk, rst_n,

    input  wire [63:0]  mul0_a, mul0_b,
    input  wire [63:0]  mul1_a, mul1_b,
    input  wire [63:0]  mul2_a, mul2_b,
    output wire [63:0]  mul0_o, mul1_o, mul2_o,

    input  wire [63:0]  add0_a, add0_b, input wire add0_sub,
    input  wire [63:0]  add1_a, add1_b, input wire add1_sub,
    input  wire [63:0]  add2_a, add2_b, input wire add2_sub,
    output wire [63:0]  add0_o, add1_o, add2_o,

    input  wire        div_start,
    input  wire [63:0] div_a, div_b,
    output wire        div_done,
    output wire [63:0] div_o
);
    fp64_mul u_mul0(.a(mul0_a), .b(mul0_b), .o(mul0_o));
    fp64_mul u_mul1(.a(mul1_a), .b(mul1_b), .o(mul1_o));
    fp64_mul u_mul2(.a(mul2_a), .b(mul2_b), .o(mul2_o));

    fp64_add u_add0(.a(add0_a), .b(add0_b), .sub(add0_sub), .o(add0_o));
    fp64_add u_add1(.a(add1_a), .b(add1_b), .sub(add1_sub), .o(add1_o));
    fp64_add u_add2(.a(add2_a), .b(add2_b), .sub(add2_sub), .o(add2_o));

    fp64_div u_div(.clk(clk), .start(div_start), .a(div_a), .b(div_b),
                   .done(div_done), .o(div_o));
endmodule
