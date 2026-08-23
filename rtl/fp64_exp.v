`timescale 1ns/1ps
// fp64_exp: IEEE-754 FP64 exp (combinational), for the adaptive-scale decision
// (new_scale = scale * exp(sum_log/(2*n_log))). exp(x) = 2^(x*log2e). Split
// y=x*log2e into n=round(y), f=y-n in [-0.5,0.5]; 2^f via deg-7 polynomial
// (~1e-10), 2^n by direct exponent construction. Valid for small |x| (n within
// double exponent range); the scale factor exp(sum_log/(2n)) has |arg|~<3.
module fp64_exp(input wire [63:0] x, output wire [63:0] o);
    localparam LOG2E = 64'h3FF71547652B82FE;  // 1.4426950408889634
    localparam HALF  = 64'h3FE0000000000000;
    localparam NHALF = 64'hBFE0000000000000;
    // deg7 coeffs of 2^f on f in [-0.5,0.5] (E0..E7 ascending)
    localparam E0 = 64'h3FEFFFFFFFFC1D58;
    localparam E1 = 64'h3FE62E42FEF8CD01;
    localparam E2 = 64'h3FCEBFBE083E1E47;
    localparam E3 = 64'h3FAC6B08DC3D17E7;
    localparam E4 = 64'h3F83B29F7217856A;
    localparam E5 = 64'h3F55D875BCD0CD0D;
    localparam E6 = 64'h3F24455175F06368;
    localparam E7 = 64'h3EF00CE056A57E75;

    wire [63:0] y;
    fp64_mul umy(x, LOG2E, y);
    wire [63:0] half = y[63] ? NHALF : HALF;     // 0.5*sign
    wire [63:0] y_adj;
    fp64_add uay(y, half, 1'b0, y_adj);          // y + 0.5*sign (round to nearest)
    wire signed [10:0] n;
    wire [63:0] n_dbl, f, poly, pow2n;
    fp64_trunc ut(y_adj, n);
    i2d un(n, n_dbl);
    fp64_add uaf(y, n_dbl, 1'b1, f);             // f = y - n in [-0.5,0.5]

    wire [63:0] a0,a1,a2,a3,a4,a5,a6;
    wire [63:0] p1,p2,p3,p4,p5,p6;
    fp64_mul um0(f, E7, a0); fp64_add ua0(a0, E6, 1'b0, p1);
    fp64_mul um1(p1, f, a1); fp64_add ua1(a1, E5, 1'b0, p2);
    fp64_mul um2(p2, f, a2); fp64_add ua2(a2, E4, 1'b0, p3);
    fp64_mul um3(p3, f, a3); fp64_add ua3(a3, E3, 1'b0, p4);
    fp64_mul um4(p4, f, a4); fp64_add ua4(a4, E2, 1'b0, p5);
    fp64_mul um5(p5, f, a5); fp64_add ua5(a5, E1, 1'b0, p6);
    fp64_mul um6(p6, f, a6); fp64_add ua6(a6, E0, 1'b0, poly);   // 2^f
    i2pow2 up(n, pow2n);
    fp64_mul uo(poly, pow2n, o);
endmodule

// trunc(v) toward zero -> signed int n (valid |v|<2^10).
module fp64_trunc(input wire [63:0] v, output reg signed [10:0] n);
    reg s; reg [10:0] e; reg [51:0] man; reg [10:0] shift; reg [10:0] intp;
    always @* begin
        s = v[63]; e = v[62:52]; man = v[51:0];
        if (e >= 11'd1023 && e < 11'd1034) begin        // 1 <= |v| < 2^11
            shift = e - 11'd1023;                        // actual exponent (0..10)
            intp = (11'd1 << shift) | (man >> (11'd52 - shift));
            n = s ? -intp : intp;
        end else if (e >= 11'd1034) begin
            n = s ? -11'd2047 : 11'd2047;                // saturate (out of range)
        end else begin                                   // |v| < 1
            n = 0;
        end
    end
endmodule

// 2^n as FP64 double (n in signed 11-bit; exponent = n+1023).
module i2pow2(input wire signed [10:0] n, output wire [63:0] o);
    assign o[63]   = 1'b0;
    assign o[62:52] = n + 11'd1023;
    assign o[51:0]  = 52'd0;
endmodule
