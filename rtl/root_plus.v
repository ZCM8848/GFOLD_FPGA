`timescale 1ns/1ps
// root_plus: R-weighted 1D root for tau (SCS C root_plus), FP64, truncating.
//   Inputs vectors r,g,p,mu (length L) and scalar eta:
//     gg = sum g^2 r ; mug = sum mu g r ; pg = sum p g r ; pp = sum p^2 r ; pmu=sum p mu r
//     a = TAU + gg ; b = mug - 2pg - eta*TAU ; cc = pp - pmu
//     rad = b*b - 4*a*cc
//     if rad<0  tau = -b/(2a)
//     sq = sqrt(rad)
//     if b<=0   tau = (-b+sq)/(2a)
//     q = -0.5*(b+sq) ; tau = cc/q  (0 if q==0)
// Stream: one (r,g,p,mu) per cycle with din_valid during S_ACC (L elements).
// TAU = 10.0 default (0x4024000000000000).
module root_plus #(parameter L = 64,
                   parameter [63:0] TAU = 64'h4024000000000000) (
    input  wire             clk, rst_n, start,
    input  wire [63:0]      r_in, g_in, p_in, mu_in,
    input  wire             din_valid,
    input  wire [63:0]      eta,
    output reg              done,
    output reg  [63:0]      tau
);
    // ---- accumulation combinational pool ----
    wire [63:0] gr, gg_t, mug_t, pg_t, p2, pp_t, pmu2, pmu_t;
    wire [63:0] rr_o;          // rsqrt output (used by m_sq, declared up top)
    reg  [63:0] sq;            // sqrt(rad) (used by add_num, declared up top)
    fp64_mul m_gr  (.a(r_in),  .b(g_in),  .o(gr));
    fp64_mul m_gg  (.a(gr),    .b(g_in),  .o(gg_t));   // gr*g
    fp64_mul m_mug (.a(gr),    .b(mu_in), .o(mug_t));  // gr*mu
    fp64_mul m_pg  (.a(gr),    .b(p_in),  .o(pg_t));   // gr*p
    fp64_mul m_p2  (.a(p_in),  .b(p_in),  .o(p2));
    fp64_mul m_pp  (.a(p2),    .b(r_in),  .o(pp_t));   // p^2*r
    fp64_mul m_pmu2(.a(p_in),  .b(mu_in), .o(pmu2));
    fp64_mul m_pmu (.a(pmu2),  .b(r_in),  .o(pmu_t));  // p*mu*r
    reg [63:0] gg, mug, pg, pp, pmu;                    // accumulators
    wire [63:0] agg, amug, apg, app, apmu;
    fp64_add add_gg (.a(gg), .b(gg_t),  .sub(1'b0), .o(agg));
    fp64_add add_mug(.a(mug),.b(mug_t), .sub(1'b0), .o(amug));
    fp64_add add_pg (.a(pg), .b(pg_t),  .sub(1'b0), .o(apg));
    fp64_add add_pp (.a(pp), .b(pp_t),  .sub(1'b0), .o(app));
    fp64_add add_pmu(.a(pmu),.b(pmu_t), .sub(1'b0), .o(apmu));

    // ---- tail combinational pool (inputs = accum regs + latched sq) ----
    wire [63:0] cc, a_, twopg, et10;
    fp64_add add_cc  (.a(pp), .b(pmu), .sub(1'b1), .o(cc));   // pp - pmu
    fp64_add add_a   (.a(TAU),.b(gg),  .sub(1'b0), .o(a_));   // TAU + gg
    fp64_add add_2pg (.a(pg), .b(pg),  .sub(1'b0), .o(twopg));// 2pg
    fp64_mul m_et10  (.a(eta),.b(TAU), .o(et10));             // eta*TAU
    wire [63:0] b1, b_;
    fp64_add add_b1  (.a(mug),.b(twopg),.sub(1'b1), .o(b1));  // mug-2pg
    fp64_add add_b   (.a(b1), .b(et10), .sub(1'b1), .o(b_));  // b1-eta*TAU
    wire [63:0] b2, a_cc, four_ac;
    fp64_mul m_b2    (.a(b_), .b(b_),    .o(b2));
    fp64_mul m_acc   (.a(a_), .b(cc),    .o(a_cc));
    fp64_mul m_x4    (.a(a_cc),.b(64'h4010000000000000), .o(four_ac)); // *4
    wire [63:0] rad_;
    fp64_add add_rad (.a(b2), .b(four_ac), .sub(1'b1), .o(rad_)); // b2-4ac
    wire [63:0] sqmul, bplussq, q_;
    fp64_mul m_sq    (.a(rad_),.b(rr_o), .o(sqmul));        // rad*rsqrt(rad)
    fp64_add add_bps (.a(b_), .b(sq),   .sub(1'b0), .o(bplussq)); // b+sq
    fp64_mul m_q     (.a(bplussq),.b(64'hBFE0000000000000), .o(q_)); // -0.5*(b+sq)
    wire rad_neg, b_le0, q_zero;
    fp64_cmp cmp_rad(.a(rad_),.b(64'h0),.al(rad_neg),.ale(),.ag(),.age(),.ae());
    fp64_cmp cmp_b  (.a(b_),   .b(64'h0),.al(),.ale(b_le0),.ag(),.age(),.ae());
    fp64_cmp cmp_q  (.a(q_),   .b(64'h0),.al(),.ale(),.ag(),.age(),.ae(q_zero));
    wire [63:0] denom;
    fp64_mul m_den  (.a(a_), .b(64'h4000000000000000), .o(denom)); // 2a
    wire [63:0] negb_sq;                    // -b + sq
    reg  [63:0] nb;
    fp64_add add_num(.a({nb[63]^1'b1, nb[62:0]}), .b(sq), .sub(1'b0), .o(negb_sq));

    // ---- rsqrt: sq = rad*rsqrt(rad) ----
    reg  rr_start; reg [63:0] rr_in; wire rr_done;
    fp64_rsqrt ur(.clk(clk), .start(rr_start), .x(rr_in), .done(rr_done), .o(rr_o));
    // ---- divider ----
    reg div_start; reg [63:0] diva, divb; wire div_done; wire [63:0] divo;
    fp64_div ud(.clk(clk), .start(div_start), .a(diva), .b(divb), .done(div_done), .o(divo));

    reg [31:0] cnt;
    localparam S0=0, S_ACC=1, S_W1=2, S_RAD=3, S_RSQ=4, S_RSQW=5,
               S_SQ=6, S_SQW=7, S_DIV=8, S_DIVW=9, S_DONE=10;
    reg [3:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; cnt <= 0; rr_start <= 0; div_start <= 0;
            gg <= 0; mug <= 0; pg <= 0; pp <= 0; pmu <= 0; tau <= 0;
            nb <= 0; sq <= 0;
        end else begin
            rr_start <= 0; div_start <= 0;
            case (st)
            S0: begin
                done <= 0;
                if (start) begin gg <= 0; mug <= 0; pg <= 0; pp <= 0; pmu <= 0;
                    cnt <= 0; st <= S_ACC; end
            end
            S_ACC: begin
                if (din_valid) begin
                    gg <= agg; mug <= amug; pg <= apg; pp <= app; pmu <= apmu;
                    if (cnt + 1 >= L) st <= S_W1;
                    else cnt <= cnt + 1;
                end
            end
            S_W1: begin              // comb chain settles
                nb <= b_;
                st <= S_RAD;
            end
            S_RAD: begin
                if (rad_neg) begin
                    diva <= {nb[63]^1'b1, nb[62:0]}; divb <= denom;
                    div_start <= 1; st <= S_DIV;
                end else begin
                    rr_start <= 1; rr_in <= rad_; st <= S_RSQ;
                end
            end
            S_RSQ: st <= S_RSQW;
            S_RSQW: if (rr_done) st <= S_SQ;
            S_SQ: begin sq <= sqmul; st <= S_SQW; end
            S_SQW: begin
                if (b_le0) begin
                    diva <= negb_sq; divb <= denom; div_start <= 1; st <= S_DIV;
                end else if (q_zero) begin
                    tau <= 0; st <= S_DONE;
                end else begin
                    diva <= cc; divb <= q_; div_start <= 1; st <= S_DIV;
                end
            end
            S_DIV: st <= S_DIVW;
            S_DIVW: if (div_done) begin tau <= divo; st <= S_DONE; end
            S_DONE: begin done <= 1; st <= S0; end
            default: st <= S0;
            endcase
        end
    end
endmodule
