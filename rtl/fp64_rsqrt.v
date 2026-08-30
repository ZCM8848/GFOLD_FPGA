`timescale 1ns/1ps
// FP64 reciprocal square root rsqrt(x) = 1/sqrt(x) via multiplicative Newton:
//   y_{n+1} = y_n * (1.5 - 0.5*x*y_n^2)
// Converges quadratically. Seed y0 = 2^(floor(e/2)-1) with mantissa 1.0 ->
// |rel err| <= 0.5, so 6 iterations reach ~5e-20 (better than 53-bit).
// sqrt(x) = x*rsqrt(x).
// Needed for cone projection (nx) and normalize_v (1/||v||).
//
// Optimized: the per-step data path (5 fp64_mul + 1 fp64_add, ~10k ALUT) is
// time-multiplexed onto ONE fp64_mul + ONE fp64_add (~3.5k ALUT). The 0.5*
// multiply is folded into an exponent decrement. 6 cycles/iter x 6 iters.
module fp64_rsqrt(input  wire        clk,
                  input  wire        start,
                  input  wire [63:0] x,
                  output reg         done,
                  output reg  [63:0] o);
    reg [63:0] y, xl, t1, t2, t4;
    wire [63:0] c15 = 64'h3FF8000000000000;  // 1.5

    reg [63:0] ma, mb; wire [63:0] mpo;
    fp64_mul um(ma, mb, mpo);
    reg [63:0] aa, ab; reg asub; wire [63:0] apo;
    fp64_add ua(aa, ab, asub, apo);
    // 0.5*t2 by exponent decrement (comb); valid one cycle after t2 is sampled
    wire [63:0] t3 = {t2[63], t2[62:52] - 11'd1, t2[51:0]};

    reg [2:0] it;
    reg [2:0] st;
    reg busy;

    // seed: y0 = 2^(floor(e/2) - 1), mantissa 1.0, e = e_biased-1023 signed.
    wire signed [12:0] etrue = {2'b00, x[62:52]} - 13'd1023;
    wire signed [12:0] etr2 = etrue >>> 1;          // floor(e/2) arithmetic shift
    wire signed [13:0] se_s = 14'sd1022 - {{2{etr2[12]}}, etr2};
    wire [10:0] se = (se_s < 14'sd1) ? 11'd1 : (se_s > 14'sd2046 ? 11'd2046 : se_s[10:0]);
    wire [63:0] seed = {1'b0, se, 52'h0};

    localparam S_IDLE=0, S0=1, S1=2, S2=3, S3=4, S4=5;
    always @(posedge clk) begin
        if (start) begin
            done <= 0; busy <= 1; it <= 0; xl <= x; y <= seed; st <= S0;
            ma <= seed; mb <= seed;                     // t1 = y*y
        end else if (busy) begin
            case (st)
            S0: begin t1 <= mpo; st <= S1;
                ma <= xl; mb <= mpo; end                // t2 = x*t1
            S1: begin t2 <= mpo; st <= S2; end          // sample t2 (t3 comb next cycle)
            S2: begin st <= S3; aa <= c15; ab <= t3; asub <= 1'b1; end  // t4 = 1.5 - t3
            S3: begin t4 <= apo; st <= S4;
                ma <= y; mb <= apo; end                 // ynext = y*t4
            S4: begin y <= mpo; st <= S0;
                if (it == 3'd5) begin o <= mpo; done <= 1; busy <= 0; st <= S_IDLE; end
                else begin it <= it + 1; ma <= mpo; mb <= mpo; end  // next t1 = ynext^2
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
