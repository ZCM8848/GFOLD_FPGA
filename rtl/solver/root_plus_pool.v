// root_plus.v �?R-weighted 1D root for tau (SCS C root_plus), FP64 truncating.
// 重写版：使用全局 fp64_pool�? mul + 5 add）替代直接实例�?// 累加每元�?4 拍（原版 1 拍流水线），尾链与原版相同的顺序�?// 数值路径完全一致（�?fp64_mul/fp64_add 模块，同操作顺序），保证 bit-exact�?`timescale 1ns/1ps

module root_plus #(
    parameter L = 64,
    parameter [63:0] TAU = 64'h4024000000000000
)(
    input  wire        clk, rst_n, start,
    input  wire [63:0] r_in, g_in, p_in, mu_in,
    input  wire        din_valid,
    input  wire [63:0] eta,
    output reg         done,
    output reg  [63:0] tau,

    // Pool mul channels (组合)
    output reg [63:0] mul0_a, mul0_b, output reg [63:0] mul1_a, mul1_b,
    output reg [63:0] mul2_a, mul2_b, output reg [63:0] mul3_a, mul3_b,
    input  wire [63:0] mul0_o, mul1_o, mul2_o, mul3_o,
    // Pool add channels (组合)
    output reg [63:0] add0_a, add0_b, output reg add0_sub,
    output reg [63:0] add1_a, add1_b, output reg add1_sub,
    output reg [63:0] add2_a, add2_b, output reg add2_sub,
    output reg [63:0] add3_a, add3_b, output reg add3_sub,
    output reg [63:0] add4_a, add4_b, output reg add4_sub,
    input  wire [63:0] add0_o, add1_o, add2_o, add3_o, add4_o,
    // Pool div (时序)
    output reg         div_start,
    output reg [63:0] div_a, div_b,
    input  wire        div_done,
    input  wire [63:0] div_o,
    // Shared rsqrt (owned by drs_iter)
    output reg         rr_start,
    output reg  [63:0] rr_in,
    input  wire        rr_done,
    input  wire [63:0] rr_o
);
    // rsqrt is shared (ports above)

    // 比较器（�?LUT�?
    wire b_le0, rad_neg, q_zero;
    fp64_cmp cmp_b  (.a(b_r), .b(64'h0), .al(), .ale(b_le0), .ag(), .age(), .ae());
    fp64_cmp cmp_rad(.a(rad_r), .b(64'h0), .al(rad_neg), .ale(), .ag(), .age(), .ae());
    fp64_cmp cmp_q  (.a(q_r), .b(64'h0), .al(), .ale(), .ag(), .age(), .ae(q_zero));

    // 累加寄存�?
    reg [63:0] gr_r, p2_r, pmu2_r;           // stage-1 结果
    reg [63:0] r_d, g_d, p_d, mu_d;          // 延迟输入
    reg [63:0] gg_t, mug_t, pg_t, pp_t, pmu_t; // stage-2 结果
    reg [63:0] gg, mug, pg, pp, pmu;         // 累加�?
    reg [63:0] sq;

    // 尾链寄存�?
    reg [63:0] twopg_r, et10_r, cc_r, a_r;
    reg [63:0] b1_r, b_r, a_cc_r;
    reg [63:0] b2_r, four_ac_r, rad_r;
    reg [63:0] denom_r, bplussq_r, q_r;

    reg [31:0] cnt;
    reg [4:0] st;
    localparam S_IDLE=0, S_ACC_W=1, S_ACC1=2, S_ACC2=3, S_ACC3=4, S_ACC4=5,
               S_W1=6, S_W1b=7, S_W1c=8, S_RAD=9, S_RADb=10, S_RADc=11,
               S_RSQ=12, S_RSQW=13, S_SQ=14, S_SQW=15, S_SQWb=16, S_SQWc=17,
               S_DIV=18, S_DIVW=19, S_DONE=20;

    // combinational pool operands
    always @(*) begin
        mul0_a=0; mul0_b=0; mul1_a=0; mul1_b=0; mul2_a=0; mul2_b=0; mul3_a=0; mul3_b=0;
        add0_a=0; add0_b=0; add0_sub=0; add1_a=0; add1_b=0; add1_sub=0;
        add2_a=0; add2_b=0; add2_sub=0; add3_a=0; add3_b=0; add3_sub=0;
        add4_a=0; add4_b=0; add4_sub=0;
        div_start=0; div_a=0; div_b=0; rr_start=0; rr_in=0;
        case (st)
        S_ACC_W: begin
            // consume the streamed element in the SAME cycle din_valid is high
            mul0_a=r_in;  mul0_b=g_in;   // gr = r*g
            mul1_a=p_in;  mul1_b=p_in;   // p2 = p*p
            mul2_a=p_in;  mul2_b=mu_in;  // pmu2 = p*mu
        end
        S_ACC2: begin
            mul0_a=gr_r;  mul0_b=g_d;    // gg = gr*g
            mul1_a=gr_r;  mul1_b=mu_d;   // mug = gr*mu
            mul2_a=gr_r;  mul2_b=p_d;    // pg = gr*p
            mul3_a=p2_r;  mul3_b=r_d;    // pp = p2*r
        end
        S_ACC3: begin
            mul0_a=pmu2_r; mul0_b=r_d;   // pmu = pmu2*r
        end
        S_ACC4: begin
            add0_a=gg;  add0_b=gg_t;  add0_sub=0;  // gg += gg_t
            add1_a=mug; add1_b=mug_t; add1_sub=0;  // mug += mug_t
            add2_a=pg;  add2_b=pg_t;  add2_sub=0;  // pg += pg_t
            add3_a=pp;  add3_b=pp_t;  add3_sub=0;  // pp += pp_t
            add4_a=pmu; add4_b=pmu_t; add4_sub=0;  // pmu += pmu_t
        end
        S_W1: begin
            mul0_a=eta;  mul0_b=TAU;                    // et10 = eta*TAU
            add0_a=pg;   add0_b=pg;   add0_sub=0;       // 2pg
            add1_a=pp;   add1_b=pmu;  add1_sub=1;       // cc = pp-pmu
            add2_a=TAU;  add2_b=gg;   add2_sub=0;       // a = TAU+gg
        end
        S_W1b: begin
            mul0_a=a_r;  mul0_b=64'h4000000000000000;   // denom = 2a
            add0_a=mug;  add0_b=twopg_r; add0_sub=1;    // b1 = mug-2pg
        end
        S_W1c: begin
            mul0_a=a_r;  mul0_b=cc_r;                   // a_cc = a*cc
            add0_a=b1_r; add0_b=et10_r; add0_sub=1;     // b = b1-et10
        end
        S_RAD: begin
            mul0_a=b_r;  mul0_b=b_r;                    // b2 = b^2
            mul1_a=a_cc_r; mul1_b=64'h4010000000000000; // four_ac = a_cc*4
        end
        S_RADb: begin
            add0_a=b2_r; add0_b=four_ac_r; add0_sub=1;  // rad = b2-4ac
        end
        S_RADc: begin
            if (rad_neg) begin
                div_a={b_r[63]^1'b1, b_r[62:0]}; div_b=denom_r; div_start=1; // -b/2a
            end else begin
                rr_start=1; rr_in=rad_r;
            end
        end
        S_SQ: begin
            mul0_a=rad_r; mul0_b=rr_o;                  // sq = rad*rsqrt(rad)
        end
        S_SQW: begin
            add0_a=b_r;  add0_b=sq;  add0_sub=0;        // b+sq
            add1_a={b_r[63]^1'b1, b_r[62:0]}; add1_b=sq; add1_sub=0; // -b+sq
            if (b_le0) begin div_a=add1_o; div_b=denom_r; div_start=1; end // -b+sq / 2a
        end
        S_SQWb: begin
            mul0_a=bplussq_r; mul0_b=64'hBFE0000000000000; // q = -0.5*(b+sq)
        end
        S_SQWc: begin
            if (!q_zero) begin
                div_a=cc_r; div_b=q_r; div_start=1;     // cc/q
            end
        end
        default: ;
        endcase
    end

    // 时序：FSM + 寄存�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_IDLE; done<=0; cnt<=0;
            gg<=0; mug<=0; pg<=0; pp<=0; pmu<=0; tau<=0; sq<=0;
            gr_r<=0; p2_r<=0; pmu2_r<=0; r_d<=0; g_d<=0; p_d<=0; mu_d<=0;
            gg_t<=0; mug_t<=0; pg_t<=0; pp_t<=0; pmu_t<=0;
            twopg_r<=0; et10_r<=0; cc_r<=0; a_r<=0;
            b1_r<=0; b_r<=0; a_cc_r<=0;
            b2_r<=0; four_ac_r<=0; rad_r<=0;
            denom_r<=0; bplussq_r<=0; q_r<=0;
        end else begin

            case (st)
            S_IDLE: begin
                done<=0;
                if (start) begin
                    gg<=0; mug<=0; pg<=0; pp<=0; pmu<=0; cnt<=0;
                    st<=S_ACC_W;
                end
            end
            S_ACC_W: begin
                if (din_valid) begin
                    gr_r<=mul0_o; p2_r<=mul1_o; pmu2_r<=mul2_o;
                    r_d<=r_in; g_d<=g_in; p_d<=p_in; mu_d<=mu_in;
                    st<=S_ACC2;
                end
            end
            S_ACC2: begin
                gg_t<=mul0_o; mug_t<=mul1_o; pg_t<=mul2_o; pp_t<=mul3_o;
                st<=S_ACC3;
            end
            S_ACC3: begin
                pmu_t<=mul0_o;
                st<=S_ACC4;
            end
            S_ACC4: begin
                gg<=add0_o; mug<=add1_o; pg<=add2_o; pp<=add3_o; pmu<=add4_o;
                if (cnt+1>=L) st<=S_W1;
                else begin cnt<=cnt+1; st<=S_ACC_W; end
            end
            S_W1: begin
                twopg_r<=add0_o; et10_r<=mul0_o; cc_r<=add1_o; a_r<=add2_o;
                st<=S_W1b;
            end
            S_W1b: begin
                b1_r<=add0_o; denom_r<=mul0_o;
                st<=S_W1c;
            end
            S_W1c: begin
                b_r<=add0_o; a_cc_r<=mul0_o;
                st<=S_RAD;
            end
            S_RAD: begin
                b2_r<=mul0_o; four_ac_r<=mul1_o;
                st<=S_RADb;
            end
            S_RADb: begin
                rad_r<=add0_o;
                st<=S_RADc;
            end
            S_RADc: begin
                if (rad_neg) st<=S_DIV;
                else st<=S_RSQ;
            end
            S_RSQ: st<=S_RSQW;
            S_RSQW: if (rr_done) st<=S_SQ;
            S_SQ: begin sq<=mul0_o; st<=S_SQW; end
            S_SQW: begin
                bplussq_r<=add0_o;
                if (b_le0) st<=S_DIV;        // div_a/b/div_start set in comb
                else st<=S_SQWb;
            end
            S_SQWb: begin
                q_r<=mul0_o;
                st<=S_SQWc;
            end
            S_SQWc: begin
                if (q_zero) begin tau<=0; st<=S_DONE; end
                else st<=S_DIV;              // div set in comb
            end
            S_DIV: st<=S_DIVW;
            S_DIVW: if (div_done) begin tau<=div_o; st<=S_DONE; end
            S_DONE: begin done<=1; st<=S_IDLE; end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
