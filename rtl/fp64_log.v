`timescale 1ns/1ps
// fp64_log: IEEE-754 FP64 natural log (combinational), for the adaptive-scale
// decision logic (needs log(rel_pri) - log(rel_dual)). Accuracy ~1e-7 on
// m in [1,2). Method: x = m*2^e (m in [1,2)), ln(x) = e*ln2 + ln(m), with
// ln(m) via a deg-8 polynomial in t = m-1 (Chebyshev fit, ~9e-8). int->double
// for the exponent e via i2d. Slow combinational chain (sim-only; pipeline for
// synthesis if timing misses).
module fp64_log(input wire [63:0] x, output wire [63:0] o);
    localparam LN2 = 64'h3FE62E42FEFA39EF;  // ln(2)
    // deg8 power coeffs of ln(1+t), t in [0,1] (c0..c8, ascending)
    localparam C0 = 64'h3E7833F29EA1D91B;
    localparam C1 = 64'h3FEFFFEE2172E7B5;
    localparam C2 = 64'hBFDFFCBF3824CB66;
    localparam C3 = 64'h3FD53499FEA00753;
    localparam C4 = 64'hBFCE9DF2263A68F1;
    localparam C5 = 64'h3FC517DBFCA03D72;
    localparam C6 = 64'hBFB7A24C7ACFA642;
    localparam C7 = 64'h3FA19FB5B7BCD890;
    localparam C8 = 64'hBF78E28D5F586CC0;

    wire [10:0] e = x[62:52] - 11'd1023;          // signed exponent
    wire [63:0] m = {x[63], 11'd1023, x[51:0]};   // 1.0 + frac (normalized)
    wire [63:0] t;                                 // m - 1.0
    fp64_add ua_t(m, 64'h3FF0000000000000, 1'b1, t);

    wire [63:0] e_dbl, e_ln2;
    i2d ui(e, e_dbl);
    fp64_mul um_e(e_dbl, LN2, e_ln2);

    // Horner: ((((C8*t+C7)*t+C6)*t+... +C1)*t + C0)
    wire [63:0] a0,a1,a2,a3,a4,a5,a6,a7;
    wire [63:0] p1,p2,p3,p4,p5,p6,p7,p8;
    fp64_mul um0(t, C8, a0); fp64_add ua0(a0, C7, 1'b0, p1);
    fp64_mul um1(p1, t, a1); fp64_add ua1(a1, C6, 1'b0, p2);
    fp64_mul um2(p2, t, a2); fp64_add ua2(a2, C5, 1'b0, p3);
    fp64_mul um3(p3, t, a3); fp64_add ua3(a3, C4, 1'b0, p4);
    fp64_mul um4(p4, t, a4); fp64_add ua4(a4, C3, 1'b0, p5);
    fp64_mul um5(p5, t, a5); fp64_add ua5(a5, C2, 1'b0, p6);
    fp64_mul um6(p6, t, a6); fp64_add ua6(a6, C1, 1'b0, p7);
    fp64_mul um7(p7, t, a7); fp64_add ua7(a7, C0, 1'b0, p8);
    fp64_add ua8(e_ln2, p8, 1'b0, o);
endmodule

// signed 11-bit int -> FP64 double (combinational). e in [-1023,1023].
module i2d(input wire signed [10:0] e, output reg [63:0] o);
    reg s; reg [10:0] mag; reg [3:0] k;
    always @* begin
        s = e[10];
        mag = s ? (~e + 1) : e;
        // find leading-1 position k (0..10); mag=0 -> k=0 (result +0)
        if      (mag[10]) k = 10;
        else if (mag[9])  k = 9;
        else if (mag[8])  k = 8;
        else if (mag[7])  k = 7;
        else if (mag[6])  k = 6;
        else if (mag[5])  k = 5;
        else if (mag[4])  k = 4;
        else if (mag[3])  k = 3;
        else if (mag[2])  k = 2;
        else if (mag[1])  k = 1;
        else              k = 0;
        if (mag == 0)
            o = 64'h0;                       // +0.0
        else begin
            // mag = 2^k + rest; mantissa frac = (mag - 2^k) * 2^(52-k)
            o[63]   = s;
            o[62:52]= k + 11'd1023;
            o[51:0] = ((mag - (11'd1 << k)) << (52 - k));
        end
    end
endmodule
