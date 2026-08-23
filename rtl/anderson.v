`timescale 1ns/1ps
// anderson: Type-I Anderson acceleration orchestrator (SCS wiring), regularized
// normal equations + dense Cholesky, per the SKILL design and software/gen_anderson.py
// (class AndersonNaive mirrors this state machine exactly).
//
// One "apply call" = accelerate one DRS output f given the previous DR input x:
//   call 0 (iter==0):  xarr<-x, farr<-f, g_prev<-x-f ; PASS (no acceleration)
//   call >=1:          idx=(iter-1)%MEM
//                      S[:,idx]=x-xarr, D[:,idx]=f-farr, g=x-f, Y[:,idx]=g-g_prev
//                      xarr<-x, farr<-f, g_prev<-g ; acc_s/acc_y[idx] += col sq-sums
//                      if iter<MEM  -> PASS (f_out=f)
//                      else solve regularized normal equations and APPLY:
//                          rreg = AA_R * ||S||_F * ||Y||_F
//                          G = S^T S + rreg*I ; rhs = S^T g
//                          gamma = G^-1 rhs (Cholesky, via chol10)
//                          f_out = f - D @ gamma
//   iter++ at end of each call.
//
// KEY simplification (validated): aa_frob(v)=m*sqrt(sum((v/m)^2)) == ||v||_2, and
// ||nrm_s||_2 = ||S||_F = sqrt(sum_i acc_s[i]) where acc_s[i] = column i's sum of
// squares. So the column norms are never materialized: frob is computed directly
// from the persistent acc_s/acc_y arrays (no nrm_s/nrm_y, no per-column sqrt).
//
// The regularized normal equations + dense Cholesky (10x10) is the USER-approved
// Anderson decision (NOT pivoted-QR). aa_gram + chol10 are the validated sub-modules.
//
// Interface: pulse `start`, then stream DIM (x[i],f[i]) pairs on the rdy/din_valid
// handshake; result f_out[i] comes out on o_valid (DIM words), `done` pulses at the
// end of the call. All state persists across calls (reset only on rst_n).
module anderson #(
    parameter DIM = 8,
    parameter MEM = 10
)(
    input  wire             clk, rst_n,
    input  wire             aa_reset,        // clear persistent state (scale change)
    // ---- one apply call ----
    input  wire             start,
    input  wire [63:0]      x_in, f_in,
    input  wire             din_valid,
    output reg              rdy,             // high when the DUT will accept the next element
    // ---- result ----
    output reg  [63:0]      f_out,
    output reg              o_valid,
    output reg              done
);
    // ---------- persistent state (across apply calls) ----------
    reg [63:0] xarr  [0:DIM-1];          // previous DR inputs
    reg [63:0] farr  [0:DIM-1];          // current DR outputs (= f, refreshed each call)
    reg [63:0] garr  [0:DIM-1];          // current residual g = x - f
    reg [63:0] gprev [0:DIM-1];          // previous residual
    reg [63:0] S     [0:DIM*MEM-1];      // S[p*MEM + col]  (row p, column col)
    reg [63:0] D     [0:DIM*MEM-1];
    reg [63:0] Y     [0:DIM*MEM-1];
    reg [63:0] acc_s [0:MEM-1];          // column sum-of-squares of S (persistent)
    reg [63:0] acc_y [0:MEM-1];          // column sum-of-squares of Y
    reg [63:0] Gbuf  [0:MEM*MEM-1];      // aa_gram output capture
    reg [63:0] rhsbuf[0:MEM-1];
    reg [63:0] gamma [0:MEM-1];
    reg [31:0] iter;

    // ---------- shared FP64 arithmetic (combinational mul/add, seq rsqrt) ----------
    // A0: S = x - xarr ; A1: D = f - farr ; A2: g = x - f ; A3: Y = g - g_prev ; A4: accumulation
    reg  [63:0] a0a,a0b; wire [63:0] a0o; fp64_add A0(.a(a0a),.b(a0b),.sub(1'b1),.o(a0o));
    reg  [63:0] a1a,a1b; wire [63:0] a1o; fp64_add A1(.a(a1a),.b(a1b),.sub(1'b1),.o(a1o));
    reg  [63:0] a2a,a2b; wire [63:0] a2o; fp64_add A2(.a(a2a),.b(a2b),.sub(1'b1),.o(a2o));
    reg  [63:0] a3a,a3b; wire [63:0] a3o; fp64_add A3(.a(a3a),.b(a3b),.sub(1'b1),.o(a3o));
    reg  [63:0] a4a,a4b; reg a4sub; wire [63:0] a4o; fp64_add A4(.a(a4a),.b(a4b),.sub(a4sub),.o(a4o));
    reg  [63:0] m0a,m0b; wire [63:0] m0o; fp64_mul M0(.a(m0a),.b(m0b),.o(m0o));
    reg  [63:0] m1a,m1b; wire [63:0] m1o; fp64_mul M1(.a(m1a),.b(m1b),.o(m1o));
    reg rs_start; reg [63:0] rs_x; wire rs_done; wire [63:0] rs_o;
    fp64_rsqrt UR(.clk(clk),.start(rs_start),.x(rs_x),.done(rs_done),.o(rs_o));

    // latched intermediate values
    reg [63:0] Sreg, Dreg, Greg, Yreg, sq_s, sq_y, fsum, frob_s, frob_y, rreg, appacc;

    // ---------- sub-modules ----------
    reg ag_start, ag_dv; reg [63:0] ag_s, ag_g; wire ag_done, ag_o_valid; wire [63:0] ag_gout, ag_rout;
    aa_gram #(.DIM(DIM),.MEM(MEM)) UAG(.clk(clk),.rst_n(rst_n),.start(ag_start),
        .s_in(ag_s),.g_in(ag_g),.rreg_in(rreg),.din_valid(ag_dv),
        .done(ag_done),.g_out(ag_gout),.r_out(ag_rout),.o_valid(ag_o_valid));
    reg ch_start, ch_dv; reg [63:0] ch_g, ch_r; wire ch_done, ch_o_valid; wire [63:0] ch_gamma;
    chol10 #(.MEM(MEM)) UCH(.clk(clk),.rst_n(rst_n),.start(ch_start),
        .g_in(ch_g),.r_in(ch_r),.din_valid(ch_dv),
        .done(ch_done),.gamma_out(ch_gamma),.o_valid(ch_o_valid));

    // ---------- counters ----------
    reg [31:0] idx, wcnt, fi, agw, agcnt, chcnt, appi, appj, p_i;

    localparam S_IDLE=0, S_L0=1, S_L0B=2, S_LRST=3, S_LA=4, S_LB=5, S_LC=6, S_LD=7, S_LE=8,
               S_FS0=9, S_FS1=10, S_FS2=11, S_FSRT0=12, S_FSRT0A=13, S_FSRT0W=14, S_FSRT0SQ=15,
               S_FY0=16, S_FY1=17, S_FY2=18, S_FSRT1=19, S_FSRT1A=20, S_FSRT1W=21, S_FSRT1SQ=22,
               S_RREG0=23, S_RREG1=24, S_RREG2=25,
               S_AG_LOAD=26, S_AG_STREAM=27, S_AG_CAP=28,
               S_CH_LOAD=29, S_CH_STREAM=30, S_CH_CAP=31,
               S_APP1=32, S_APP2=33, S_APP3=34, S_APP4=35,
               S_PASS0=36, S_PASS1=37, S_DONE=38, S_ACLR=39;
    reg [5:0] st;
    reg [19:0] aclr;

    localparam [63:0] AA_R = 64'h3E45798EE2308C3A;   // 1e-8

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; rdy <= 0;
            iter <= 0; idx <= 0; wcnt <= 0; fi <= 0; agw <= 0; agcnt <= 0; chcnt <= 0;
            appi <= 0; appj <= 0; p_i <= 0; aclr <= 0;
            rs_start <= 0; ag_start <= 0; ch_start <= 0; ag_dv <= 0; ch_dv <= 0;
            a0a <= 0; a0b <= 0; a1a <= 0; a1b <= 0; a2a <= 0; a2b <= 0;
            a3a <= 0; a3b <= 0; a4a <= 0; a4b <= 0; a4sub <= 0;
            m0a <= 0; m0b <= 0; m1a <= 0; m1b <= 0; rs_x <= 0;
            Sreg <= 0; Dreg <= 0; Greg <= 0; Yreg <= 0; sq_s <= 0; sq_y <= 0;
            fsum <= 0; frob_s <= 0; frob_y <= 0; rreg <= 0; appacc <= 0;
        end else begin
            done <= 0; o_valid <= 0; rdy <= 0; rs_start <= 0; ag_start <= 0; ch_start <= 0;
            case (st)
            // ================= IDLE: dispatch =================
            S_IDLE: begin
                done <= 0; o_valid <= 0; rdy <= 0;
                if (aa_reset) begin aclr <= 0; st <= S_ACLR; end
                else if (start) begin
                    wcnt <= 0;
                    if (iter == 0) st <= S_L0;
                    else begin idx <= (iter - 1) % MEM; st <= S_LRST; end
                end
            end
            // ============ scale-change reset: clear all persistent arrays ============
            S_ACLR: begin
                // flat-index clear of x/f/g/gp(4*DIM) + S/D/Y(3*DIM*MEM) + acc_s/acc_y(2*MEM)
                if (aclr < DIM) xarr[aclr] <= 0;
                else if (aclr < 2*DIM) farr[aclr - DIM] <= 0;
                else if (aclr < 3*DIM) garr[aclr - 2*DIM] <= 0;
                else if (aclr < 4*DIM) gprev[aclr - 3*DIM] <= 0;
                else if (aclr < 4*DIM + DIM*MEM) S[aclr - 4*DIM] <= 0;
                else if (aclr < 4*DIM + 2*DIM*MEM) D[aclr - 4*DIM - DIM*MEM] <= 0;
                else if (aclr < 4*DIM + 3*DIM*MEM) Y[aclr - 4*DIM - 2*DIM*MEM] <= 0;
                else if (aclr < 4*DIM + 3*DIM*MEM + MEM) acc_s[aclr - 4*DIM - 3*DIM*MEM] <= 0;
                else acc_y[aclr - 4*DIM - 3*DIM*MEM - MEM] <= 0;
                if (aclr + 1 >= 4*DIM + 3*DIM*MEM + 2*MEM) begin iter <= 0; done <= 1; st <= S_IDLE; end
                else aclr <= aclr + 1;
            end
            // ================= iter==0 load: xarr=x, farr=f, g_prev=x-f =================
            S_L0: begin
                rdy <= 1;
                if (din_valid) begin
                    rdy <= 0;
                    xarr[wcnt] <= x_in; farr[wcnt] <= f_in;
                    a2a <= x_in; a2b <= f_in;         // g = x - f
                    st <= S_L0B;
                end
            end
            S_L0B: begin
                gprev[wcnt] <= a2o;
                if (wcnt + 1 >= DIM) st <= S_PASS0;
                else begin wcnt <= wcnt + 1; st <= S_L0; end
            end
            // ================= iter>=1 load (reset current column sum) =================
            S_LRST: begin
                acc_s[idx] <= 0; acc_y[idx] <= 0;
                st <= S_LA;
            end
            S_LA: begin
                rdy <= 1;
                if (din_valid) begin
                    rdy <= 0;
                    // S=x-xarr, D=f-farr, g=x-f (combinational, valid next cycle)
                    a0a <= x_in; a0b <= xarr[wcnt];
                    a1a <= f_in; a1b <= farr[wcnt];
                    a2a <= x_in; a2b <= f_in;
                    xarr[wcnt] <= x_in; farr[wcnt] <= f_in;
                    st <= S_LB;
                end
            end
            S_LB: begin
                Sreg <= a0o; Dreg <= a1o; Greg <= a2o;
                // Y = g - g_prev ; sq_s = S^2
                a3a <= a2o; a3b <= gprev[wcnt];
                m0a <= a0o; m0b <= a0o;
                S[wcnt*MEM+idx] <= a0o; D[wcnt*MEM+idx] <= a1o;
                garr[wcnt] <= a2o; gprev[wcnt] <= a2o;
                st <= S_LC;
            end
            S_LC: begin
                Yreg <= a3o; sq_s <= m0o;
                m1a <= a3o; m1b <= a3o;                 // sq_y = Y^2
                a4a <= acc_s[idx]; a4b <= m0o; a4sub <= 0;   // acc_s += S^2
                Y[wcnt*MEM+idx] <= a3o;
                st <= S_LD;
            end
            S_LD: begin
                sq_y <= m1o;
                acc_s[idx] <= a4o;
                a4a <= acc_y[idx]; a4b <= m1o; a4sub <= 0;   // acc_y += Y^2
                st <= S_LE;
            end
            S_LE: begin
                acc_y[idx] <= a4o;
                if (wcnt + 1 >= DIM) begin
                    if (iter >= MEM) st <= S_FS0;       // enough history -> solve
                    else st <= S_PASS0;                  // else pass through
                end else begin wcnt <= wcnt + 1; st <= S_LA; end
            end
            // ================= frob_s = ||S||_F = sqrt(sum_i acc_s[i]) =================
            S_FS0: begin fsum <= 0; fi <= 0; st <= S_FS1; end
            S_FS1: begin a4a <= fsum; a4b <= acc_s[fi]; a4sub <= 0; st <= S_FS2; end
            S_FS2: begin
                fsum <= a4o;
                if (fi + 1 < MEM) begin fi <= fi + 1; st <= S_FS1; end
                else st <= S_FSRT0;
            end
            S_FSRT0: begin rs_start <= 1; rs_x <= fsum; st <= S_FSRT0A; end
            S_FSRT0A: if (!rs_done) st <= S_FSRT0W;    // arm: wait stale rsqrt done to clear
            S_FSRT0W: if (rs_done) begin m0a <= fsum; m0b <= rs_o; st <= S_FSRT0SQ; end
            S_FSRT0SQ: begin frob_s <= m0o; st <= S_FY0; end
            // ================= frob_y = ||Y||_F = sqrt(sum_i acc_y[i]) =================
            S_FY0: begin fsum <= 0; fi <= 0; st <= S_FY1; end
            S_FY1: begin a4a <= fsum; a4b <= acc_y[fi]; a4sub <= 0; st <= S_FY2; end
            S_FY2: begin
                fsum <= a4o;
                if (fi + 1 < MEM) begin fi <= fi + 1; st <= S_FY1; end
                else st <= S_FSRT1;
            end
            S_FSRT1: begin rs_start <= 1; rs_x <= fsum; st <= S_FSRT1A; end
            S_FSRT1A: if (!rs_done) st <= S_FSRT1W;    // arm
            S_FSRT1W: if (rs_done) begin m0a <= fsum; m0b <= rs_o; st <= S_FSRT1SQ; end
            S_FSRT1SQ: begin frob_y <= m0o; st <= S_RREG0; end
            // ================= rreg = AA_R * frob_s * frob_y =================
            S_RREG0: begin m0a <= frob_s; m0b <= frob_y; st <= S_RREG1; end   // frob_s*frob_y
            S_RREG1: begin m1a <= AA_R; m1b <= m0o; st <= S_RREG2; end        // AA_R * tmp
            S_RREG2: begin rreg <= m1o; st <= S_AG_LOAD; end
            // ================= drive aa_gram: S (col-major MEM*DIM) + g (DIM) =================
            S_AG_LOAD: begin ag_start <= 1; ag_dv <= 0; agw <= 0; st <= S_AG_STREAM; end
            S_AG_STREAM: begin
                ag_start <= 0; ag_dv <= 1;
                if (agw < MEM*DIM) ag_s <= S[(agw%DIM)*MEM + (agw/DIM)];   // col-major S[col*DIM+p]
                else ag_g <= garr[agw - MEM*DIM];
                if (agw + 1 >= MEM*DIM + DIM) begin agcnt <= 0; st <= S_AG_CAP; end
                else agw <= agw + 1;
            end
            S_AG_CAP: begin
                if (ag_o_valid) begin
                    if (agcnt < MEM*MEM) begin Gbuf[agcnt] <= ag_gout; agcnt <= agcnt + 1; end
                    else begin rhsbuf[agcnt - MEM*MEM] <= ag_rout; agcnt <= agcnt + 1; end
                end
                if (ag_done) begin chcnt <= 0; st <= S_CH_LOAD; end
            end
            // ================= drive chol10: G (row-major MEM*MEM) + rhs (MEM) =================
            S_CH_LOAD: begin ch_start <= 1; ch_dv <= 0; chcnt <= 0; st <= S_CH_STREAM; end
            S_CH_STREAM: begin
                ch_start <= 0; ch_dv <= 1;
                if (chcnt < MEM*MEM) ch_g <= Gbuf[chcnt];
                else ch_r <= rhsbuf[chcnt - MEM*MEM];
                if (chcnt + 1 >= MEM*MEM + MEM) begin chcnt <= 0; st <= S_CH_CAP; end
                else chcnt <= chcnt + 1;
            end
            S_CH_CAP: begin
                if (ch_o_valid) begin gamma[chcnt] <= ch_gamma; chcnt <= chcnt + 1; end
                if (ch_done) begin appi <= 0; st <= S_APP1; end
            end
            // ================= APPLY: f_out = farr - D @ gamma =================
            S_APP1: begin appacc <= farr[appi]; appj <= 0; st <= S_APP2; end
            S_APP2: begin m0a <= D[appi*MEM+appj]; m0b <= gamma[appj]; st <= S_APP3; end
            S_APP3: begin a4a <= appacc; a4b <= m0o; a4sub <= 1; st <= S_APP4; end
            S_APP4: begin
                appacc <= a4o;
                if (appj + 1 < MEM) begin appj <= appj + 1; st <= S_APP2; end
                else begin
                    f_out <= a4o; o_valid <= 1;
                    if (appi + 1 >= DIM) st <= S_DONE;
                    else begin appi <= appi + 1; st <= S_APP1; end
                end
            end
            // ================= PASS: f_out = farr (no acceleration) =================
            S_PASS0: begin p_i <= 0; st <= S_PASS1; end
            S_PASS1: begin
                f_out <= farr[p_i]; o_valid <= 1;
                if (p_i + 1 >= DIM) st <= S_DONE;
                else begin p_i <= p_i + 1; st <= S_PASS1; end
            end
            // ================= DONE =================
            S_DONE: begin done <= 1; iter <= iter + 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
