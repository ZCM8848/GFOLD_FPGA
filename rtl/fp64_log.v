`timescale 1ns/1ps
// fp64_log: IEEE-754 FP64 natural log (SEQUENTIAL, time-multiplexed), for the
// adaptive-scale decision (needs log(rel_pri) - log(rel_dual)).
// x = m*2^e (m in [1,2)), ln(x) = e*ln2 + ln(m), ln(m) via a deg-8 polynomial
// in t = m-1 (Chebyshev fit, ~9e-8).
//
// Combinational version used 9 fp64_mul + 9 fp64_add (~30k ALUT). This version
// shares ONE fp64_mul + ONE fp64_add across a ~21-cycle FSM (~3.5k ALUT); log is
// only needed once every 25 iterations (scale decision).
module fp64_log(input  wire        clk,
                input  wire        start,
                input  wire [63:0] x,
                output reg         done,
                output reg  [63:0] o);
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

    reg [63:0] ma, mb; wire [63:0] mpo;
    fp64_mul um(ma, mb, mpo);
    reg [63:0] aa, ab; reg asub; wire [63:0] apo;
    fp64_add ua(aa, ab, asub, apo);

    reg [63:0] xl, t, e_ln2, poly;
    reg [4:0] st;

    wire [10:0] e = xl[62:52] - 11'd1023;          // signed exponent
    wire [63:0] m = {x[63], 11'd1023, x[51:0]};    // 1.0 + frac (normalized, from input x)
    wire [63:0] e_dbl;
    i2d ui(e, e_dbl);

    localparam S_IDLE=0, S_T=1, S_ELN2=2, S_M0=3, S_A0=4, S_M1=5, S_A1=6,
               S_M2=7, S_A2=8, S_M3=9, S_A3=10, S_M4=11, S_A4=12, S_M5=13, S_A5=14,
               S_M6=15, S_A6=16, S_M7=17, S_A7=18, S_O=19, S_DONE=20;
    always @(posedge clk) begin
        if (start) begin
            done <= 0; xl <= x; st <= S_T;
            aa <= m; ab <= 64'h3FF0000000000000; asub <= 1'b1;   // t = m - 1.0
        end else case (st)
        S_IDLE: done <= 0;
        S_T: begin t <= apo; st <= S_ELN2;
            ma <= e_dbl; mb <= LN2; end                          // e_ln2 = e*ln2
        S_ELN2: begin e_ln2 <= mpo; st <= S_M0;
            ma <= t; mb <= C8; end                               // a = t*C8
        S_M0: begin st <= S_A0; aa <= mpo; ab <= C7; asub <= 1'b0; end   // p = a+C7
        S_A0: begin st <= S_M1; ma <= apo; mb <= t; end
        S_M1: begin st <= S_A1; aa <= mpo; ab <= C6; asub <= 1'b0; end
        S_A1: begin st <= S_M2; ma <= apo; mb <= t; end
        S_M2: begin st <= S_A2; aa <= mpo; ab <= C5; asub <= 1'b0; end
        S_A2: begin st <= S_M3; ma <= apo; mb <= t; end
        S_M3: begin st <= S_A3; aa <= mpo; ab <= C4; asub <= 1'b0; end
        S_A3: begin st <= S_M4; ma <= apo; mb <= t; end
        S_M4: begin st <= S_A4; aa <= mpo; ab <= C3; asub <= 1'b0; end
        S_A4: begin st <= S_M5; ma <= apo; mb <= t; end
        S_M5: begin st <= S_A5; aa <= mpo; ab <= C2; asub <= 1'b0; end
        S_A5: begin st <= S_M6; ma <= apo; mb <= t; end
        S_M6: begin st <= S_A6; aa <= mpo; ab <= C1; asub <= 1'b0; end
        S_A6: begin st <= S_M7; ma <= apo; mb <= t; end
        S_M7: begin st <= S_A7; aa <= mpo; ab <= C0; asub <= 1'b0; end
        S_A7: begin poly <= apo; st <= S_O;
            aa <= e_ln2; ab <= apo; asub <= 1'b0; end             // o = e_ln2 + poly
        S_O: begin st <= S_DONE; end
        S_DONE: begin o <= apo; done <= 1; st <= S_IDLE; end
        default: st <= S_IDLE;
        endcase
    end
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
