`timescale 1ns/1ps
// IEEE-754 double-precision (FP64) arithmetic units for Stage-B RTL.
// mul / add / sub / div, mirroring float32.v but with 53-bit mantissa (52 frac
// + 1 implicit) and 11-bit exponent. Cyclone IV has no hard FP -> these are
// built from logic + DSP multipliers (53x53 mantissa).
// NOTE: truncating (no round-to-nearest); 53-bit mantissa gives rel ~2^-52,
// utterly negligible for the solver, so truncation is fine.

// ---------------- double multiply ----------------
module fp64_mul(input  wire [63:0] a, b,
                output reg  [63:0] o);
    wire sa = a[63], sb = b[63];
    wire [10:0] ea = a[62:52], eb = b[62:52];
    wire [51:0] fa = a[51:0], fb = b[51:0];
    wire za = ~|a[62:0], zb = ~|b[62:0];
    wire [52:0] ma = {1'b1, fa}, mb = {1'b1, fb};
    wire [105:0] m = ma * mb;
    wire shift = m[105];
    wire [11:0] e = {1'b0, ea} + {1'b0, eb} + (shift ? 12'd1 : 12'd0) - 12'd1023;
    wire [51:0] mant = shift ? m[104:53] : m[103:52];
    always @(*) begin
        if (za | zb) o = 64'h0;
        else o = {sa ^ sb, e[10:0], mant};
    end
endmodule

// ---------------- double add/sub (o = a + b, or a - b when sub=1) ----------------
module fp64_add(input  wire [63:0] a, b,
                input  wire sub,
                output reg  [63:0] o);
    wire [63:0] bo = {b[63] ^ sub, b[62:0]};
    wire sa = a[63], sb = bo[63];
    wire [10:0] ea = a[62:52], eb = bo[62:52];
    wire [51:0] fa = a[51:0], fb = bo[51:0];
    wire za = ~|a[62:0], zb = ~|bo[62:0];
    wire same = (sa == sb);
    wire [11:0] diff = {1'b0, ea} - {1'b0, eb};
    wire swap = diff[11] | za | (diff == 0 & (fa < fb));
    wire [10:0] el = swap ? eb : ea;
    wire [10:0] es = swap ? ea : eb;
    wire [51:0] fl = swap ? fb : fa;
    wire [51:0] fs = swap ? fa : fb;
    wire sl = swap ? sb : sa;
    wire [11:0] sh = {1'b0, el} - {1'b0, es};
    reg [52:0] ml, ms;                    // mantissas in [2^52, 2^53)
    always @(*) begin
        ml = {1'b1, fl};
        if (sh > 12'd53) ms = 53'b0;
        else ms = {1'b1, fs} >> sh;
    end
    reg [53:0] msum;                      // same: [2^53,2^54); diff: [0,2^53)
    always @(*) msum = same ? ({1'b0, ml} + {1'b0, ms}) : ({1'b0, ml} - {1'b0, ms});
    // leading-zero count on msum[52:0] (diff-sign path)
    reg [5:0] lzc;
    integer i;
    always @(*) begin
        if (msum[52])      lzc = 6'd0;
        else if (msum[51])      lzc = 6'd1;
        else if (msum[50])      lzc = 6'd2;
        else if (msum[49])      lzc = 6'd3;
        else if (msum[48])      lzc = 6'd4;
        else if (msum[47])      lzc = 6'd5;
        else if (msum[46])      lzc = 6'd6;
        else if (msum[45])      lzc = 6'd7;
        else if (msum[44])      lzc = 6'd8;
        else if (msum[43])      lzc = 6'd9;
        else if (msum[42])      lzc = 6'd10;
        else if (msum[41])      lzc = 6'd11;
        else if (msum[40])      lzc = 6'd12;
        else if (msum[39])      lzc = 6'd13;
        else if (msum[38])      lzc = 6'd14;
        else if (msum[37])      lzc = 6'd15;
        else if (msum[36])      lzc = 6'd16;
        else if (msum[35])      lzc = 6'd17;
        else if (msum[34])      lzc = 6'd18;
        else if (msum[33])      lzc = 6'd19;
        else if (msum[32])      lzc = 6'd20;
        else if (msum[31])      lzc = 6'd21;
        else if (msum[30])      lzc = 6'd22;
        else if (msum[29])      lzc = 6'd23;
        else if (msum[28])      lzc = 6'd24;
        else if (msum[27])      lzc = 6'd25;
        else if (msum[26])      lzc = 6'd26;
        else if (msum[25])      lzc = 6'd27;
        else if (msum[24])      lzc = 6'd28;
        else if (msum[23])      lzc = 6'd29;
        else if (msum[22])      lzc = 6'd30;
        else if (msum[21])      lzc = 6'd31;
        else if (msum[20])      lzc = 6'd32;
        else if (msum[19])      lzc = 6'd33;
        else if (msum[18])      lzc = 6'd34;
        else if (msum[17])      lzc = 6'd35;
        else if (msum[16])      lzc = 6'd36;
        else if (msum[15])      lzc = 6'd37;
        else if (msum[14])      lzc = 6'd38;
        else if (msum[13])      lzc = 6'd39;
        else if (msum[12])      lzc = 6'd40;
        else if (msum[11])      lzc = 6'd41;
        else if (msum[10])      lzc = 6'd42;
        else if (msum[9])      lzc = 6'd43;
        else if (msum[8])      lzc = 6'd44;
        else if (msum[7])      lzc = 6'd45;
        else if (msum[6])      lzc = 6'd46;
        else if (msum[5])      lzc = 6'd47;
        else if (msum[4])      lzc = 6'd48;
        else if (msum[3])      lzc = 6'd49;
        else if (msum[2])      lzc = 6'd50;
        else if (msum[1])      lzc = 6'd51;
        else if (msum[0])      lzc = 6'd52;
        else               lzc = 6'd53;
    end
    always @(*) begin
        if (za) o = bo;
        else if (zb) o = a;
        else if (!same && msum[53:0] == 0) o = 64'h0;
        else if (same) begin
            // msum = ml+ms in [2^52,2^54); carry (bit 53) if ms contributed
            if (msum[53]) begin
                o[63] = sl; o[62:52] = el + 11'd1; o[51:0] = msum[52:1];
            end else begin
                o[63] = sl; o[62:52] = el; o[51:0] = msum[51:0];
            end
        end else begin
            // msum in [0,2^53), leading 1 at bit 52-lzc; shift left lzc -> 1.frac
            o[63] = sl;
            o[62:52] = el - lzc;
            o[51:0] = (msum << lzc) & 52'hFFFFFFFFFFFFF;   // low 52 bits (frac)
        end
    end
endmodule

// ---------------- double divide o = a/b (restoring mantissa division) --------
// 53-bit result: 1 implicit + 52 fraction bits, 52 restoring iterations.
module fp64_div(input  wire clk,
                input  wire start,
                input  wire [63:0] a, b,
                output reg  done,
                output reg  [63:0] o);
    reg [5:0] it;
    reg [53:0] R;          // restoring remainder
    reg [51:0] Q;          // fraction bits
    reg [52:0] divs;       // divisor mantissa
    reg [53:0] divd;       // normalized dividend
    reg [10:0] expo;       // result exponent
    reg sgn;
    reg busy;
    reg zflag;
    wire [53:0] Rsh = {R[52:0], 1'b0};
    always @(posedge clk) begin
        if (start) begin
            it <= 0; done <= 0; busy <= 1; zflag <= 0;
            sgn <= a[63] ^ b[63];
            if (a[62:0] == 0 || b[62:0] == 0) begin
                zflag <= 1;
                if (b[62:0] == 0 && a[62:0] != 0)
                    o <= {a[63] ^ b[63], 11'h7FF, 52'h0};   // ±inf
                else
                    o <= {a[63] ^ b[63], 63'h0};             // ±0
            end else begin
                divs <= {1'b1, b[51:0]};
                if ({1'b1, a[51:0]} >= {1'b1, b[51:0]}) begin
                    divd <= {1'b0, 1'b1, a[51:0]};
                    expo <= a[62:52] - b[62:52] + 11'd1023;
                end else begin
                    divd <= {1'b1, a[51:0], 1'b0};
                    expo <= a[62:52] - b[62:52] + 11'd1022;
                end
                R <= 0; Q <= 0;
            end
        end else if (busy) begin
            if (zflag) begin
                done <= 1; busy <= 0; zflag <= 0;
            end else if (it == 6'd0) begin
                R <= divd - {1'b0, divs};
                it <= it + 1'b1;
            end else if (it == 6'd53) begin
                o[63] <= sgn;
                o[62:52] <= expo;
                o[51:0] <= Q;
                done <= 1; busy <= 0;
            end else begin
                if (Rsh >= {1'b0, divs}) begin
                    R <= Rsh - {1'b0, divs};
                    Q[52 - it] <= 1'b1;
                end else begin
                    R <= Rsh;
                end
                it <= it + 1'b1;
            end
        end
    end
endmodule
