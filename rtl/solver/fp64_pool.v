// fp64_pool.v — 全局共享 FP64 运算池
// 4 个乘法 + 5 个加法（组合，与 fp64_mul/fp64_add 一致）+ 1 个除法（时序）
// 所有 solver 模块通过端口复用这些单元；同一时刻只有一个模块活跃。
//
// DSP 预算：4 × fp64_mul(8 DSP) = 32 DSP；fp64_add 纯 LUT；fp64_div 纯 LUT。
// 加上 root_plus 内部的 fp64_rsqrt(8 DSP) = 共 40 DSP（PH1P35 全部）。
`timescale 1ns/1ps

module fp64_pool (
    input  wire        clk, rst_n,

    // 4 × 乘法（组合）
    input  wire [63:0]  mul0_a, mul0_b,
    input  wire [63:0]  mul1_a, mul1_b,
    input  wire [63:0]  mul2_a, mul2_b,
    input  wire [63:0]  mul3_a, mul3_b,
    output wire [63:0]  mul0_o, mul1_o, mul2_o, mul3_o,

    // 5 × 加/减（组合）
    input  wire [63:0]  add0_a, add0_b, input wire add0_sub,
    input  wire [63:0]  add1_a, add1_b, input wire add1_sub,
    input  wire [63:0]  add2_a, add2_b, input wire add2_sub,
    input  wire [63:0]  add3_a, add3_b, input wire add3_sub,
    input  wire [63:0]  add4_a, add4_b, input wire add4_sub,
    output wire [63:0]  add0_o, add1_o, add2_o, add3_o, add4_o,

    // 1 × 除法（时序，52 拍）
    input  wire        div_start,
    input  wire [63:0] div_a, div_b,
    output wire        div_done,
    output wire [63:0] div_o
);
    fp64_mul u_mul0(.a(mul0_a), .b(mul0_b), .o(mul0_o));
    fp64_mul u_mul1(.a(mul1_a), .b(mul1_b), .o(mul1_o));
    fp64_mul u_mul2(.a(mul2_a), .b(mul2_b), .o(mul2_o));
    fp64_mul u_mul3(.a(mul3_a), .b(mul3_b), .o(mul3_o));

    fp64_add u_add0(.a(add0_a), .b(add0_b), .sub(add0_sub), .o(add0_o));
    fp64_add u_add1(.a(add1_a), .b(add1_b), .sub(add1_sub), .o(add1_o));
    fp64_add u_add2(.a(add2_a), .b(add2_b), .sub(add2_sub), .o(add2_o));
    fp64_add u_add3(.a(add3_a), .b(add3_b), .sub(add3_sub), .o(add3_o));
    fp64_add u_add4(.a(add4_a), .b(add4_b), .sub(add4_sub), .o(add4_o));

    fp64_div u_div(.clk(clk), .start(div_start), .a(div_a), .b(div_b),
                   .done(div_done), .o(div_o));
endmodule
