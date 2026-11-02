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
//
// Both the accumulation (2-stage mul + add) and the tail (~6-deep chain) are
// pipelined so each cycle holds at most ONE fp64 op (meets 30 MHz).
module root_plus #(parameter L = 64,
                   parameter [63:0] TAU = 64'h4024000000000000) (
    input  wire             clk, rst_n, start,
    input  wire [63:0]      r_in, g_in, p_in, mu_in,
    input  wire             din_valid,
    input  wire [63:0]      eta,
    output reg              done,
    output reg  [63:0]      tau
);
    wire [63:0] rr_o;          // rsqrt output (used by m_sq)
    reg  [63:0] sq;            // sqrt(rad)

    // ---- accumulation: stage-1 muls (gr, p2, pmu2) ----
    wire [63:0] gr, p2, pmu2;
    fp64_mul m_gr   (.a(r_in),  .b(g_in),  .o(gr));
    fp64_mul m_p2   (.a(p_in),  .b(p_in),  .o(p2));
    fp64_mul m_pmu2 (.a(p_in),  .b(mu_in), .o(pmu2));
    // stage-1 latches + 1-cycle input delay
    reg [63:0] gr_r, p2_r, pmu2_r;
    reg [63:0] r_d1, g_d1, p_d1, mu_d1;
    // ---- accumulation: stage-2 muls ----
    wire [63:0] gg_t, mug_t, pg_t, pp_t, pmu_t;
    fp64_mul m_gg  (.a(gr_r),   .b(g_d1),  .o(gg_t));   // gr*g
    fp64_mul m_mug (.a(gr_r),   .b(mu_d1), .o(mug_t));  // gr*mu
    fp64_mul m_pg  (.a(gr_r),   .b(p_d1),  .o(pg_t));   // gr*p
    fp64_mul m_pp  (.a(p2_r),   .b(r_d1),  .o(pp_t));   // p^2*r
    fp64_mul m_pmu (.a(pmu2_r), .b(r_d1),  .o(pmu_t));  // p*mu*r
    // stage-2 latches
    reg [63:0] gg_t_r, mug_t_r, pg_t_r, pp_t_r, pmu_t_r;
    // ---- accumulation: add ----
    reg [63:0] gg, mug, pg, pp, pmu;
    wire [63:0] agg, amug, apg, app, apmu;
    fp64_add add_gg (.a(gg), .b(gg_t_r),  .sub(1'b0), .o(agg));
    fp64_add add_mug(.a(mug),.b(mug_t_r), .sub(1'b0), .o(amug));
    fp64_add add_pg (.a(pg), .b(pg_t_r),  .sub(1'b0), .o(apg));
    fp64_add add_pp (.a(pp), .b(pp_t_r),  .sub(1'b0), .o(app));
    fp64_add add_pmu(.a(pmu),.b(pmu_t_r), .sub(1'b0), .o(apmu));

    // ---- pipelined tail (one fp64 op per cycle) ----
    reg [63:0] twopg_r, et10_r, cc_r, a_r;
    reg [63:0] b1_r, b_r, a_cc_r;
    reg [63:0] b2_r, four_ac_r, rad_r;
    reg [63:0] denom_r, bplussq_r, q_r;

    // stage 1 (parallel): twopg, et10, cc, a_
    wire [63:0] twopg, et10, cc, a_;
    fp64_add add_2pg (.a(pg),  .b(pg),  .sub(1'b0), .o(twopg));   // 2pg
    fp64_mul m_et10  (.a(eta), .b(TAU), .o(et10));                 // eta*TAU
    fp64_add add_cc  (.a(pp),  .b(pmu), .sub(1'b1), .o(cc));       // pp - pmu
    fp64_add add_a   (.a(TAU), .b(gg),  .sub(1'b0), .o(a_));       // TAU + gg

    // stage 2: b1 = mug - twopg_r ; denom = a_r*2
    wire [63:0] b1, denom;
    fp64_add add_b1  (.a(mug), .b(twopg_r), .sub(1'b1), .o(b1));   // mug-2pg
    fp64_mul m_den   (.a(a_r), .b(64'h4000000000000000), .o(denom)); // 2a

    // stage 3: b_ = b1_r - et10_r ; a_cc = a_r * cc_r
    wire [63:0] b_, a_cc;
    fp64_add add_b   (.a(b1_r), .b(et10_r), .sub(1'b1), .o(b_));   // b1-eta*TAU
    fp64_mul m_acc   (.a(a_r), .b(cc_r), .o(a_cc));

    // stage 4: b2 = b_r^2 ; four_ac = a_cc_r * 4
    wire [63:0] b2, four_ac;
    fp64_mul m_b2    (.a(b_r), .b(b_r), .o(b2));
    fp64_mul m_x4    (.a(a_cc_r), .b(64'h4010000000000000), .o(four_ac)); // *4

    // stage 5: rad = b2_r - four_ac_r
    wire [63:0] rad;
    fp64_add add_rad (.a(b2_r), .b(four_ac_r), .sub(1'b1), .o(rad)); // b2-4ac

    // post-rsqrt: sqmul = rad_r * rr_o
    wire [63:0] sqmul;
    fp64_mul m_sq    (.a(rad_r), .b(rr_o), .o(sqmul));

    // S_SQW: bplussq = b_r + sq ; negb_sq = -b_r + sq ; b_le0 = b_r<=0
    wire [63:0] bplussq, negb_sq;
    fp64_add add_bps (.a(b_r), .b(sq), .sub(1'b0), .o(bplussq));    // b+sq
    fp64_add add_num (.a({b_r[63]^1'b1, b_r[62:0]}), .b(sq), .sub(1'b0), .o(negb_sq));
    wire b_le0;
    fp64_cmp cmp_b  (.a(b_r), .b(64'h0), .al(), .ale(b_le0), .ag(), .age(), .ae());

    // S_SQWb: q_ = bplussq_r * -0.5
    wire [63:0] q_;
    fp64_mul m_q     (.a(bplussq_r), .b(64'hBFE0000000000000), .o(q_)); // -0.5*(b+sq)

    // S_RADc / S_SQWc compares (registered inputs)
    wire rad_neg, q_zero;
    fp64_cmp cmp_rad(.a(rad_r), .b(64'h0), .al(rad_neg), .ale(), .ag(), .age(), .ae());
    fp64_cmp cmp_q  (.a(q_r), .b(64'h0), .al(), .ale(), .ag(), .age(), .ae(q_zero));

    // ---- rsqrt: sq = rad*rsqrt(rad) ----
    reg  rr_start; reg [63:0] rr_in; wire rr_done;
    fp64_rsqrt ur(.clk(clk), .start(rr_start), .x(rr_in), .done(rr_done), .o(rr_o));
    // ---- divider ----
    reg div_start; reg [63:0] diva, divb; wire div_done; wire [63:0] divo;
    fp64_div ud(.clk(clk), .start(div_start), .a(diva), .b(divb), .done(div_done), .o(divo));

    reg [31:0] cnt;
    localparam S0=0, S_ACC=1, S_ACC_D1=2, S_ACC_D2=3, S_W1=4, S_W1b=5, S_W1c=6,
               S_RAD=7, S_RADb=8, S_RADc=9,
               S_RSQ=10, S_RSQW=11, S_SQ=12, S_SQW=13, S_SQWb=14, S_SQWc=15,
               S_DIV=16, S_DIVW=17, S_DONE=18;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; cnt <= 0; rr_start <= 0; div_start <= 0;
            gg <= 0; mug <= 0; pg <= 0; pp <= 0; pmu <= 0; tau <= 0; sq <= 0;
            gr_r <= 0; p2_r <= 0; pmu2_r <= 0;
            r_d1 <= 0; g_d1 <= 0; p_d1 <= 0; mu_d1 <= 0;
            gg_t_r <= 0; mug_t_r <= 0; pg_t_r <= 0; pp_t_r <= 0; pmu_t_r <= 0;
            twopg_r <= 0; et10_r <= 0; cc_r <= 0; a_r <= 0;
            b1_r <= 0; b_r <= 0; a_cc_r <= 0;
            b2_r <= 0; four_ac_r <= 0; rad_r <= 0;
            denom_r <= 0; bplussq_r <= 0; q_r <= 0;
        end else begin
            rr_start <= 0; div_start <= 0;
            case (st)
            S0: begin
                done <= 0;
                if (start) begin gg <= 0; mug <= 0; pg <= 0; pp <= 0; pmu <= 0;
                    gr_r <= 0; p2_r <= 0; pmu2_r <= 0;
                    gg_t_r <= 0; mug_t_r <= 0; pg_t_r <= 0; pp_t_r <= 0; pmu_t_r <= 0;
                    cnt <= 0; st <= S_ACC; end
            end
            S_ACC: begin
                if (din_valid) begin
                    gr_r <= gr; p2_r <= p2; pmu2_r <= pmu2;
                    r_d1 <= r_in; g_d1 <= g_in; p_d1 <= p_in; mu_d1 <= mu_in;
                    gg_t_r <= gg_t; mug_t_r <= mug_t; pg_t_r <= pg_t; pp_t_r <= pp_t; pmu_t_r <= pmu_t;
                    gg <= agg; mug <= amug; pg <= apg; pp <= app; pmu <= apmu;
                    if (cnt + 1 >= L) st <= S_ACC_D1;
                    else cnt <= cnt + 1;
                end
            end
            S_ACC_D1: begin
                gg_t_r <= gg_t; mug_t_r <= mug_t; pg_t_r <= pg_t; pp_t_r <= pp_t; pmu_t_r <= pmu_t;
                gg <= agg; mug <= amug; pg <= apg; pp <= app; pmu <= apmu;
                st <= S_ACC_D2;
            end
            S_ACC_D2: begin
                gg <= agg; mug <= amug; pg <= apg; pp <= app; pmu <= apmu;
                st <= S_W1;
            end
            S_W1: begin
                twopg_r <= twopg; et10_r <= et10; cc_r <= cc; a_r <= a_;
                st <= S_W1b;
            end
            S_W1b: begin
                b1_r <= b1; denom_r <= denom;
                st <= S_W1c;
            end
            S_W1c: begin
                b_r <= b_; a_cc_r <= a_cc;
                st <= S_RAD;
            end
            S_RAD: begin
                b2_r <= b2; four_ac_r <= four_ac;
                st <= S_RADb;
            end
            S_RADb: begin
                rad_r <= rad;
                st <= S_RADc;
            end
            S_RADc: begin
                if (rad_neg) begin
                    diva <= {b_r[63]^1'b1, b_r[62:0]}; divb <= denom_r;
                    div_start <= 1; st <= S_DIV;
                end else begin
                    rr_start <= 1; rr_in <= rad_r; st <= S_RSQ;
                end
            end
            S_RSQ: st <= S_RSQW;
            S_RSQW: if (rr_done) st <= S_SQ;
            S_SQ: begin sq <= sqmul; st <= S_SQW; end
            S_SQW: begin
                bplussq_r <= bplussq;
                if (b_le0) begin
                    diva <= negb_sq; divb <= denom_r; div_start <= 1; st <= S_DIV;
                end else begin
                    st <= S_SQWb;
                end
            end
            S_SQWb: begin
                q_r <= q_;
                st <= S_SQWc;
            end
            S_SQWc: begin
                if (q_zero) begin tau <= 0; st <= S_DONE; end
                else begin diva <= cc_r; divb <= q_r; div_start <= 1; st <= S_DIV; end
            end
            S_DIV: st <= S_DIVW;
            S_DIVW: if (div_done) begin tau <= divo; st <= S_DONE; end
            S_DONE: begin done <= 1; st <= S0; end
            default: st <= S0;
            endcase
        end
    end
endmodule
