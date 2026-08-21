`timescale 1ns/1ps
// IEEE-754 double comparator: outputs a<b, a<=b, a>b, a>=b, a==b (signed).
// Compares sign, then biased exponent, then mantissa.
module fp64_cmp(input  wire [63:0] a, b,
                output wire al, ale, ag, age, ae);
    wire sa = a[63], sb = b[63];
    wire [10:0] ea = a[62:52], eb = b[62:52];
    wire [51:0] fa = a[51:0], fb = b[51:0];
    wire aneg = sa;                 // a negative
    // same-sign comparison (magnitude)
    wire maga_lt = (ea < eb) || (ea == eb && fa < fb);
    wire maga_eq = (ea == eb && fa == fb);
    // a<b:
    //  - a negative, b positive -> a<b (unless a==-0,b==0; treat -0==0)
    //  - both positive: a<b iff maga_lt
    //  - both negative: a<b iff magb_lt (i.e., !maga_lt && !maga_eq, i.e. b larger magnitude)
    wire az = (a[62:0]==0), bz = (b[62:0]==0);
    wire bothz = az && bz;
    wire lt = bothz ? 1'b0 :
              (aneg && !sb) ? 1'b1 :           // - vs +
              (!aneg && sb) ? 1'b0 :           // + vs -
              (!aneg)       ? maga_lt :         // both +
              (maga_lt)     ? 1'b0 : (maga_eq ? 1'b0 : 1'b1); // both -: a<b iff maga>magb
    assign al = lt;
    assign ae = (a == b) || bothz;   // bit equal or both zero (+0 == -0)
    assign ale = lt || ae;
    assign age = !lt;                // a>=b = not(a<b)
    assign ag = !(lt || ae);         // a>b = not(a<=b), no NaN
endmodule
