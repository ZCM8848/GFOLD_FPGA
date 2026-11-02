`timescale 1ns/1ps
// fp64_exp: IEEE-754 FP64 exp (SEQUENTIAL, time-multiplexed), for the
// adaptive-scale decision (new_scale = scale * exp(sum_log/(2*n_log))).
// exp(x) = 2^(x*log2e). Split y=x*log2e into n=round(y), f=y-n in [-0.5,0.5];
// 2^f via deg-7 polynomial (~1e-10), 2^n by direct exponent construction.
//
// Combinational version used 9 fp64_mul + 7 fp64_add (~35k ALUT). This version
// shares ONE fp64_mul + ONE fp64_add across a ~24-cycle FSM (~3.5k ALUT), since
// exp is only needed once every 25 iterations (scale decision).
module fp64_exp(input  wire        clk,
                input  wire        start,
                input  wire [63:0] x,
                output reg         done,
                output reg  [63:0] o);
    localparam LOG2E = 64'h3FF71547652B82FE;  // 1.4426950408889634
    localparam HALF  = 64'h3FE0000000000000;
    localparam NHALF = 64'hBFE0000000000000;
    localparam E0 = 64'h3FEFFFFFFFFC1D58;
    localparam E1 = 64'h3FE62E42FEF8CD01;
    localparam E2 = 64'h3FCEBFBE083E1E47;
    localparam E3 = 64'h3FAC6B08DC3D17E7;
    localparam E4 = 64'h3F83B29F7217856A;
    localparam E5 = 64'h3F55D875BCD0CD0D;
    localparam E6 = 64'h3F24455175F06368;
    localparam E7 = 64'h3EF00CE056A57E75;

    reg [63:0] ma, mb; wire [63:0] mpo;
    fp64_mul um(ma, mb, mpo);
    reg [63:0] aa, ab; reg asub; wire [63:0] apo;
    fp64_add ua(aa, ab, asub, apo);

    reg [63:0] y, f, y_adj_r;
    reg [4:0] st;
    wire [63:0] half = y[63] ? NHALF : HALF;
    // combinational helpers (small, kept from the original)
    wire signed [10:0] n_tr;
    fp64_trunc ut(y_adj_r, n_tr);
    wire [63:0] n_dbl;
    i2d un(n_tr, n_dbl);
    wire [63:0] pow2n;
    i2pow2 up(n_tr, pow2n);

    localparam S_IDLE=0, S_MUL1=1, S_Y=2, S_ADD1=3, S_YADJ=4, S_YADJ2=5,
               S_ADDF=6, S_F=7, S_H0=8, S_H1=9, S_H2=10, S_H3=11, S_H4=12,
               S_H5=13, S_H6=14, S_H7=15, S_H8=16, S_H9=17, S_H10=18, S_H11=19,
               S_H12=20, S_H13=21, S_MULP=22, S_DONE=23;
    always @(posedge clk) begin
        if (start) begin
            done <= 0; st <= S_MUL1;
            ma <= x; mb <= LOG2E;                     // y = x*LOG2E
        end else case (st)
        S_IDLE: done <= 0;
        S_MUL1: begin st <= S_Y; end
        S_Y: begin y <= mpo; st <= S_ADD1;
            aa <= mpo; ab <= half; asub <= 1'b0; end  // y_adj = y + half
        S_ADD1: begin st <= S_YADJ; end
        S_YADJ: begin y_adj_r <= apo; st <= S_YADJ2; end   // sample y_adj (n/n_dbl/pow2n comb next cycle)
        S_YADJ2: begin aa <= y; ab <= n_dbl; asub <= 1'b1; st <= S_ADDF; end  // f = y - n_dbl
        S_ADDF: begin st <= S_F; end
        S_F: begin f <= apo; st <= S_H0;
            ma <= apo; mb <= E7; end                  // a = f*E7
        S_H0: begin st <= S_H1; aa <= mpo; ab <= E6; asub <= 1'b0; end   // p = a+E6
        S_H1: begin st <= S_H2; ma <= apo; mb <= f; end
        S_H2: begin st <= S_H3; aa <= mpo; ab <= E5; asub <= 1'b0; end
        S_H3: begin st <= S_H4; ma <= apo; mb <= f; end
        S_H4: begin st <= S_H5; aa <= mpo; ab <= E4; asub <= 1'b0; end
        S_H5: begin st <= S_H6; ma <= apo; mb <= f; end
        S_H6: begin st <= S_H7; aa <= mpo; ab <= E3; asub <= 1'b0; end
        S_H7: begin st <= S_H8; ma <= apo; mb <= f; end
        S_H8: begin st <= S_H9; aa <= mpo; ab <= E2; asub <= 1'b0; end
        S_H9: begin st <= S_H10; ma <= apo; mb <= f; end
        S_H10: begin st <= S_H11; aa <= mpo; ab <= E1; asub <= 1'b0; end
        S_H11: begin st <= S_H12; ma <= apo; mb <= f; end
        S_H12: begin st <= S_H13; aa <= mpo; ab <= E0; asub <= 1'b0; end   // p = poly
        S_H13: begin st <= S_MULP; ma <= apo; mb <= pow2n; end              // o = poly * 2^n
        S_MULP: begin st <= S_DONE; end
        S_DONE: begin o <= mpo; done <= 1; st <= S_IDLE; end
        default: st <= S_IDLE;
        endcase
    end
endmodule

// trunc(v) toward zero -> signed int n (valid |v|<2^10). (combinational, kept)
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

// 2^n as FP64 double (n in signed 11-bit; exponent = n+1023). (combinational, kept)
module i2pow2(input wire signed [10:0] n, output wire [63:0] o);
    assign o[63]   = 1'b0;
    assign o[62:52] = n + 11'd1023;
    assign o[51:0]  = 52'd0;
endmodule
