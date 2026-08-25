`timescale 1ns/1ps
// anderson: Type-I Anderson acceleration (regularized normal-equations + 10x10
// Cholesky via aa_gram + chol10), the USER-APPROVED design.
//   call 0 (iter==0):  xarr<-x, farr<-f, g_prev<-x-f ; PASS (no acceleration)
//   call >=1:          idx=(iter-1)%MEM
//                      S[:,idx]=x-xarr, D[:,idx]=f-farr, g=x-f, Y[:,idx]=g-g_prev
//                      xarr<-x, farr<-f, g_prev<-g ; acc_s/acc_y[idx] += col sq-sums
//                      if iter<MEM  -> PASS (f_out=f)
//                      else rreg=AA_R*||S||_F*||Y||_F ; G=S'S+rreg*I ; rhs=S'g
//                      gamma = G^-1 rhs (Cholesky, via chol10)
//                      f_out = f - D @ gamma
// SAFEGUARD (HW semantics): on reject (||diff|| > ||x-f||), roll back v<-f, v_prev<-x.
// Interface: pulse `start`, then stream DIM (x[i],f[i]) pairs on the rdy/din_valid
// handshake; result f_out[i] comes out on o_valid (DIM words), `done` pulses at the
// end of the call. All state persists across calls (reset only on rst_n).
//
// SRAM PORT (2026-08-25): the big persistent arrays (xarr/farr/garr/gprev/S/D/Y,
// 109072 x 64-bit words = 872 KB at DIM=3208) live in the EXTERNAL SRAM via the
// sram64_ctrl word interface (one 64-bit access = 6 cycles). acc_s/acc_y/Gbuf/
// rhsbuf/gamma stay in M9K (small). Main-FSM access pattern: set sram_wa/sram_wdata/
// sram_we, pulse sram_start, wait sram_done (rdata valid for reads).
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
    output reg              done,
    // ---- external SRAM (64-bit word interface via sram64_ctrl) ----
    output reg              sram_req, sram_we,
    output reg  [17:0]      sram_waddr,
    output reg  [63:0]      sram_wdata,
    input  wire             sram_busy,
    input  wire  [63:0]     sram_rdata
);
    // ---------- SRAM address map ----------
    localparam XARR_BASE  = 0;
    localparam FARR_BASE  = DIM;
    localparam GARR_BASE  = 2*DIM;
    localparam GPREV_BASE = 3*DIM;
    localparam S_BASE     = 4*DIM;
    localparam D_BASE     = 4*DIM + DIM*MEM;
    localparam Y_BASE     = 4*DIM + 2*DIM*MEM;
    localparam TOT_SRAM   = 4*DIM + 3*DIM*MEM;

    // ---------- persistent state in M9K (small) ----------
    reg [63:0] acc_s [0:MEM-1];          // column sum-of-squares of S (persistent)
    reg [63:0] acc_y [0:MEM-1];          // column sum-of-squares of Y
    reg [63:0] Gbuf  [0:MEM*MEM-1];      // aa_gram output capture
    reg [63:0] rhsbuf[0:MEM-1];
    reg [63:0] gamma [0:MEM-1];
    reg [31:0] iter;

    // ---------- SRAM access handshake (sub-FSM) ----------
    // Main FSM sets sram_wa/sram_wdata/sram_we + pulses sram_start; sram_done
    // pulses when the access completes (rdata valid for reads).
    reg sram_start;
    reg sram_done_r;
    reg [1:0] sst;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sst <= 0; sram_req <= 0; sram_done_r <= 0; end
        else begin
            sram_done_r <= 0;
            case (sst)
            0: begin
                if (sram_start) begin sram_req <= 1; sst <= 1; end
                else sram_req <= 0;
            end
            1: begin
                sram_req <= 0;
                if (sram_busy) sst <= 2;
            end
            2: begin
                if (!sram_busy) begin sram_done_r <= 1; sst <= 0; end
            end
            endcase
        end
    end
    wire sram_done = sram_done_r;

    // ---------- shared FP64 arithmetic (combinational mul/add, seq rsqrt) ----------
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

    localparam S_IDLE=0, S_ACLR=1, S_ACLRW=2,
               S_L0=3, S_L0W1=4, S_L0W2=5, S_L0W3=6,
               S_LRST=7, S_LA=8, S_LAW1=9, S_LAW2=10, S_LAW3=11, S_LAW4=12,
               S_LB=13, S_LBW1=14, S_LBW2=15, S_LBW3=16, S_LBW4=17, S_LBW5=18,
               S_LC=19, S_LCW=20, S_LD=21, S_LE=22,
               S_FS0=23, S_FS1=24, S_FS2=25, S_FSRT0=26, S_FSRT0A=27, S_FSRT0W=28, S_FSRT0SQ=29,
               S_FY0=30, S_FY1=31, S_FY2=32, S_FSRT1=33, S_FSRT1A=34, S_FSRT1W=35, S_FSRT1SQ=36,
               S_RREG0=37, S_RREG1=38, S_RREG2=39,
               S_AG_LOAD=40, S_AG_STREAM=41, S_AG_WAIT=42, S_AG_CAP=43,
               S_CH_LOAD=44, S_CH_STREAM=45, S_CH_CAP=46,
               S_APP1=47, S_APP1W=48, S_APP2=49, S_APP2W=50, S_APP3=51, S_APP4=52,
               S_PASS0=53, S_PASS1=54, S_PASS1W=55, S_DONE=56;
    reg [5:0] st;
    reg [31:0] aclr;

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
            sram_start <= 0; sram_we <= 0; sram_waddr <= 0; sram_wdata <= 0;
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
                // flat-index clear: SRAM words (TOT_SRAM) then acc_s/acc_y (2*MEM, M9K)
                if (aclr < TOT_SRAM) begin
                    sram_waddr <= aclr[17:0]; sram_wdata <= 0; sram_we <= 1;
                    sram_start <= 1; st <= S_ACLRW;
                end else if (aclr < TOT_SRAM + MEM) begin
                    acc_s[aclr - TOT_SRAM] <= 0;
                    if (aclr + 1 >= TOT_SRAM + 2*MEM) begin iter <= 0; done <= 1; st <= S_IDLE; end
                    else aclr <= aclr + 1;
                end else begin
                    acc_y[aclr - TOT_SRAM - MEM] <= 0;
                    if (aclr + 1 >= TOT_SRAM + 2*MEM) begin iter <= 0; done <= 1; st <= S_IDLE; end
                    else aclr <= aclr + 1;
                end
            end
            S_ACLRW: begin
                sram_start <= 0;
                if (sram_done) begin aclr <= aclr + 1; st <= S_ACLR; end
            end
            // ================= iter==0 load: xarr=x, farr=f, g_prev=x-f =================
            S_L0: begin
                rdy <= 1;
                if (din_valid) begin
                    rdy <= 0;
                    a2a <= x_in; a2b <= f_in;         // g = x - f
                    sram_waddr <= XARR_BASE + wcnt; sram_wdata <= x_in; sram_we <= 1;
                    sram_start <= 1; st <= S_L0W1;
                end
            end
            S_L0W1: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= FARR_BASE + wcnt; sram_wdata <= f_in; sram_we <= 1;
                    sram_start <= 1; st <= S_L0W2;
                end
            end
            S_L0W2: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= GPREV_BASE + wcnt; sram_wdata <= a2o; sram_we <= 1;
                    sram_start <= 1; st <= S_L0W3;
                end
            end
            S_L0W3: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (wcnt + 1 >= DIM) st <= S_PASS0;
                    else begin wcnt <= wcnt + 1; st <= S_L0; end
                end
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
                    a2a <= x_in; a2b <= f_in;         // g = x - f
                    sram_waddr <= XARR_BASE + wcnt; sram_we <= 0;
                    sram_start <= 1; st <= S_LAW1;    // read xarr[wcnt]
                end
            end
            S_LAW1: begin
                sram_start <= 0;
                if (sram_done) begin
                    a0a <= x_in; a0b <= sram_rdata;   // S = x - xarr
                    sram_waddr <= FARR_BASE + wcnt; sram_we <= 0;
                    sram_start <= 1; st <= S_LAW2;    // read farr[wcnt]
                end
            end
            S_LAW2: begin
                sram_start <= 0;
                if (sram_done) begin
                    a1a <= f_in; a1b <= sram_rdata;   // D = f - farr
                    sram_waddr <= XARR_BASE + wcnt; sram_wdata <= x_in; sram_we <= 1;
                    sram_start <= 1; st <= S_LAW3;    // write xarr[wcnt]
                end
            end
            S_LAW3: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= FARR_BASE + wcnt; sram_wdata <= f_in; sram_we <= 1;
                    sram_start <= 1; st <= S_LAW4;    // write farr[wcnt]
                end
            end
            S_LAW4: begin
                sram_start <= 0;
                if (sram_done) st <= S_LB;           // farr write done -> LB
            end
            S_LB: begin
                Sreg <= a0o; Dreg <= a1o; Greg <= a2o;
                m0a <= a0o; m0b <= a0o;               // sq_s = S^2
                sram_waddr <= GPREV_BASE + wcnt; sram_we <= 0;
                sram_start <= 1; st <= S_LBW1;        // read gprev[wcnt]
            end
            S_LBW1: begin
                sram_start <= 0;
                if (sram_done) begin
                    a3a <= a2o; a3b <= sram_rdata;    // Y = g - g_prev
                    sram_waddr <= S_BASE + wcnt*MEM + idx; sram_wdata <= a0o; sram_we <= 1;
                    sram_start <= 1; st <= S_LBW2;    // write S
                end
            end
            S_LBW2: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= D_BASE + wcnt*MEM + idx; sram_wdata <= a1o; sram_we <= 1;
                    sram_start <= 1; st <= S_LBW3;    // write D
                end
            end
            S_LBW3: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= GARR_BASE + wcnt; sram_wdata <= a2o; sram_we <= 1;
                    sram_start <= 1; st <= S_LBW4;    // write garr
                end
            end
            S_LBW4: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= GPREV_BASE + wcnt; sram_wdata <= a2o; sram_we <= 1;
                    sram_start <= 1; st <= S_LBW5;    // write gprev
                end
            end
            S_LBW5: begin
                sram_start <= 0;
                if (sram_done) st <= S_LC;           // gprev write done -> LC
            end
            S_LC: begin
                Yreg <= a3o; sq_s <= m0o;
                m1a <= a3o; m1b <= a3o;               // sq_y = Y^2
                a4a <= acc_s[idx]; a4b <= m0o; a4sub <= 0;   // acc_s += S^2
                sram_waddr <= Y_BASE + wcnt*MEM + idx; sram_wdata <= a3o; sram_we <= 1;
                sram_start <= 1; st <= S_LCW;         // write Y
            end
            S_LCW: begin
                sram_start <= 0;
                if (sram_done) st <= S_LD;
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
            S_RREG0: begin m0a <= frob_s; m0b <= frob_y; st <= S_RREG1; end
            S_RREG1: begin m1a <= AA_R; m1b <= m0o; st <= S_RREG2; end
            S_RREG2: begin rreg <= m1o; st <= S_AG_LOAD; end
            // ================= drive aa_gram: S (col-major MEM*DIM) + g (DIM) =================
            S_AG_LOAD: begin ag_start <= 1; ag_dv <= 0; agw <= 0; st <= S_AG_STREAM; end
            S_AG_STREAM: begin
                ag_start <= 0; ag_dv <= 0;   // clear dv immediately (else 2-cycle pulse -> double-sample)
                if (agw < MEM*DIM) begin
                    sram_waddr <= S_BASE + (agw%DIM)*MEM + (agw/DIM); sram_we <= 0;
                    sram_start <= 1; st <= S_AG_WAIT; // read S col-major
                end else begin
                    sram_waddr <= GARR_BASE + (agw - MEM*DIM); sram_we <= 0;
                    sram_start <= 1; st <= S_AG_WAIT; // read garr
                end
            end
            S_AG_WAIT: begin
                sram_start <= 0; ag_dv <= 0;
                if (sram_done) begin
                    ag_dv <= 1;                  // data-valid pulse, one cycle
                    if (agw < MEM*DIM) ag_s <= sram_rdata;
                    else ag_g <= sram_rdata;
                    if (agw + 1 >= MEM*DIM + DIM) begin agcnt <= 0; st <= S_AG_CAP; end
                    else begin agw <= agw + 1; st <= S_AG_STREAM; end
                end
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
            S_APP1: begin
                sram_waddr <= FARR_BASE + appi; sram_we <= 0;
                sram_start <= 1; st <= S_APP1W;       // read farr[appi]
            end
            S_APP1W: begin
                sram_start <= 0;
                if (sram_done) begin
                    appacc <= sram_rdata; appj <= 0; st <= S_APP2;
                end
            end
            S_APP2: begin
                sram_waddr <= D_BASE + appi*MEM + appj; sram_we <= 0;
                sram_start <= 1; st <= S_APP2W;       // read D[appi][appj]
            end
            S_APP2W: begin
                sram_start <= 0;
                if (sram_done) begin
                    m0a <= sram_rdata; m0b <= gamma[appj]; st <= S_APP3;
                end
            end
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
                sram_waddr <= FARR_BASE + p_i; sram_we <= 0;
                sram_start <= 1; st <= S_PASS1W;      // read farr[p_i]
            end
            S_PASS1W: begin
                sram_start <= 0;
                if (sram_done) begin
                    f_out <= sram_rdata; o_valid <= 1;
                    if (p_i + 1 >= DIM) st <= S_DONE;
                    else begin p_i <= p_i + 1; st <= S_PASS1; end
                end
            end
            // ================= DONE =================
            S_DONE: begin done <= 1; iter <= iter + 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
