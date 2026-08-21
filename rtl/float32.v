`timescale 1ns/1ps
// IEEE 754 single-precision (float32) arithmetic units for Stage-A RTL.
// mul / add / sub / div (Newton-Raphson reciprocal). Combinational.
// NOTE: Stage-A uses float to validate the LDL control logic against the float
// oracle; Stage B replaces these with BFP arithmetic (no FP on Cyclone IV).

// ---------------- float32 multiply ----------------
module fp32_mul(input  wire [31:0] a, b,
               output wire [31:0] o);
    wire sa = a[31], sb = b[31];
    wire [7:0] ea = a[30:23], eb = b[30:23];
    wire [22:0] fa = a[22:0], fb = b[22:0];
    wire za = ~|a[30:0], zb = ~|b[30:0];
    wire [23:0] ma = {1'b1, fa}, mb = {1'b1, fb};
    wire [47:0] m = ma * mb;
    wire shift = m[47];
    wire [8:0] e = {1'b0, ea} + {1'b0, eb} + (shift ? 9'd1 : 9'd0) - 9'd127;
    wire [22:0] mant = shift ? m[46:24] : m[45:23];
    assign o[31]   = sa ^ sb;
    assign o[30:23] = za|zb ? 8'h00 : e[7:0];
    assign o[22:0]  = za|zb ? 23'h0 : mant;
endmodule

// ---------------- float32 add/sub (o = a + b, or a - b when sub=1) ----------------
module fp32_add(input  wire [31:0] a, b,
                input  wire sub,
                output reg  [31:0] o);
    // operand b sign flipped for subtract
    wire [31:0] bo = {b[31]^sub, b[30:0]};
    wire sa = a[31], sb = bo[31];
    wire [7:0] ea = a[30:23], eb = bo[30:23];
    wire [22:0] fa = a[22:0], fb = bo[22:0];
    wire za = ~|a[30:0], zb = ~|bo[30:0];
    wire same = (sa == sb);
    // align: put larger exponent in a side
    wire [8:0] diff = {1'b0, ea} - {1'b0, eb};
    wire swap = diff[8] | za | (diff==0 & (fa < fb));  // b has larger exp (or a is zero)
    wire [7:0] el = swap ? eb : ea;
    wire [7:0] es = swap ? ea : eb;
    wire [22:0] fl = swap ? fb : fa;
    wire [22:0] fs = swap ? fa : fb;
    wire sl = swap ? sb : sa;
    wire [8:0] sh = {1'b0, el} - {1'b0, es};   // shift amount
    reg [26:0] ml, ms;
    always @(*) begin
        ml = {3'b0, 1'b1, fl};                  // 27-bit: guard+round+sticky headroom
        if (sh > 9'd27) ms = 27'b0; else ms = {3'b0, 1'b1, fs} >> sh;
    end
    wire [26:0] msum = same ? (ml + ms) : (ml - ms);   // both positive mantissas
    wire msign = msum[26];                              // negative result (borrow)
    wire [26:0] mabs = msign ? (~msum + 27'd1) : msum;
    // normalize
    wire [4:0] lzc;
    // leading-zero count on mabs[25:0]
    function [4:0] clz26;
        input [25:0] x;
        integer i;
        begin
            clz26 = 5'd26;
            for (i = 25; i >= 0; i = i - 1) if (x[i]) begin clz26 = (25 - i); i = -1; end
        end
    endfunction
    assign lzc = (mabs == 0) ? 5'd26 : clz26(mabs[25:0]);
    wire [7:0] en = el - lzc + 5'd2;   // leading-1 at bit 23 -> en=el
    wire [24:0] mn = (mabs << lzc);     // now 1.x
    always @(*) begin
        if (za) o = bo;
        else if (zb) o = a;
        else if (mabs == 0) o = 32'h0;
        else begin
            o[31] = same ? sl : (msign ? (swap ? sa : sb) : sl);
            o[30:23] = en[7:0];
            o[22:0] = mn[24:2];                     // round-to-nearest-ish (drop guard)
        end
    end
endmodule

// ---------------- float32 divide o = a/b (restoring mantissa division) -------
// Mantissa result = 1.f (24 bits: 1 implicit + 23 frac). Normalize dividend D =
// ma (if ma>=mb) or 2*ma (if ma<mb) so D/mb in [1,2); exponent adjusted by -1
// in the latter case. Then R = D - mb, and 23 restoring iterations compute the
// 23 fraction bits: f = (D-mb)*2^23 / mb.
module fp32_div(input  wire clk,
                input  wire start,
                input  wire [31:0] a, b,
                output reg  done,
                output reg  [31:0] o);
    reg [4:0] it;
    reg [24:0] R;         // restoring remainder (25 bits, < mb after each step)
    reg [22:0] Q;         // fraction bits
    reg [23:0] divs;      // divisor mantissa
    reg [24:0] divd;      // normalized dividend (ma or 2*ma, 25 bits)
    reg [7:0] expo;       // result exponent
    reg sgn;
    reg busy;
    wire [24:0] Rsh = {R[23:0], 1'b0};   // shifted remainder (compare against this)
    always @(posedge clk) begin
        if (start) begin
            it   <= 0; done <= 0; busy <= 1;
            sgn  <= a[31] ^ b[31];
            divs <= {1'b1, b[22:0]};
            if ({1'b1, a[22:0]} >= {1'b1, b[22:0]}) begin
                divd <= {1'b0, 1'b1, a[22:0]};
                expo <= a[30:23] - b[30:23] + 8'd127;
            end else begin
                divd <= {1'b1, a[22:0], 1'b0};      // 2*ma
                expo <= a[30:23] - b[30:23] + 8'd126;  // -1 for the x2
            end
            R <= 0; Q <= 0;
        end else if (busy) begin
            if (it == 5'd0) begin
                R <= divd - {1'b0, divs};           // extract leading 1
                it <= it + 1'b1;
            end else if (it == 5'd24) begin
                o[31]   <= sgn;
                o[30:23] <= expo;
                o[22:0]  <= Q;
                done <= 1; busy <= 0;
            end else begin
                if (Rsh >= {1'b0, divs}) begin      // compare SHIFTED remainder
                    R <= Rsh - {1'b0, divs};
                    Q[23 - it] <= 1'b1;             // fraction bit (it=1..23 -> bits 22..0)
                end else begin
                    R <= Rsh;
                end
                it <= it + 1'b1;
            end
        end
    end
endmodule
