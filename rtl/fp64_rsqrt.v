`timescale 1ns/1ps
// FP64 reciprocal square root rsqrt(x) = 1/sqrt(x) via multiplicative Newton:
//   y_{n+1} = y_n * (1.5 - 0.5*x*y_n^2)
// Uses only fp64_mul (combinational) + fp64_add; converges quadratically.
// Seed y0 = 2^(floor(e/2)-1) with mantissa 1.0 -> |rel err| <= 0.5 for all x,
// so 6 iterations reach ~5e-20 (better than 53-bit). sqrt(x) = x*rsqrt(x).
// Needed for cone projection (nx) and normalize_v (1/||v||).
module fp64_rsqrt(input  wire        clk,
                  input  wire        start,
                  input  wire [63:0] x,
                  output reg         done,
                  output reg  [63:0] o);
    reg [63:0] y;
    wire [63:0] t1, t2, t3, t4, ynext;
    wire [63:0] c15 = 64'h3FF8000000000000;  // 1.5
    wire [63:0] c05 = 64'h3FE0000000000000;  // 0.5
    fp64_mul u1(y, y, t1);        // y^2
    fp64_mul u2(x, t1, t2);       // x*y^2
    fp64_mul u3(c05, t2, t3);     // 0.5*x*y^2
    fp64_add u4(c15, t3, 1'b1, t4);   // 1.5 - 0.5*x*y^2
    fp64_mul u5(y, t4, ynext);    // y * (1.5 - ...)
    reg [2:0] it;
    reg busy;
    // seed: y0 = 2^(floor(e/2) - 1), mantissa 1.0, e = e_biased-1023 signed.
    wire signed [12:0] etrue = {2'b00, x[62:52]} - 13'd1023;
    wire signed [12:0] etr2 = etrue >>> 1;          // floor(e/2) arithmetic shift
    // se = 1022 - floor(e/2), signed, clamp to [1,2046]
    wire signed [13:0] se_s = 14'sd1022 - {{2{etr2[12]}}, etr2};
    wire [10:0] se = (se_s < 14'sd1) ? 11'd1 : (se_s > 14'sd2046 ? 11'd2046 : se_s[10:0]);
    wire [63:0] seed = {1'b0, se, 52'h0};
    always @(posedge clk) begin
        if (start) begin
            done <= 0; busy <= 1; it <= 0; y <= seed;
        end else if (busy) begin
            if (it == 3'd5) begin
                o <= ynext; done <= 1; busy <= 0;
            end else begin
                y <= ynext; it <= it + 1;
            end
        end
    end
endmodule
