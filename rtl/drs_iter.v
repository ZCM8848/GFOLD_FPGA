`timescale 1ns/1ps
// drs_iter: ONE SCS DRS iteration on the REORDERED (node-major) problem
// (Phase 4b core orchestrator). Static scale, no Anderson (added later).
// Mirrors drs_reference.one_iteration EXACTLY (i >= FEAS=1 behavior):
//   1. normalize: v *= sqrt(L)/||v||              (iter >= 1)
//   2. KKT solve: u_t[:l-1] = KKT^-1 [rho_x v[:n]; -r_y v[n:l-1]]
//                  via kkt_solve (vx=v[:n], vy=v[n:l-1])   [refactor=1:
//                  band streamed in first, re-factorizes; =0: reuse L/D]
//   3. u_t[l-1] = (iter==0) ? 1.0 : root_plus(diag_r,g,u_t,v,eta=v[l-1])
//   4. u_t[:l-1] -= u_t[l-1] * g        (ALSO for iter==0: tau = 1.0)
//   5. u = 2*u_t - v ;  u[n:l-1] = proj_dual_cone_rb(u[n:l-1])  (always)
//   6. u[l-1] = (iter==0) ? 1.0 : max(u[l-1], 0)
//   7. rsk = (v + u - 2*u_t) * diag_r
//   8. v += 1.5 * (u - u_t)
// State RAM layout (RAM_AW=15, single port, sync write / async read):
//   v[0,L)  u_t[L,2L)  u[2L,3L)  rsk[3L,4L)  g[4L,4L+l-1)  diag_r[4L+l-1, 4L+l-1+L)
// kkt_solve gets its OWN RAM (kkt_* port, tb-provided). proj_dual_cone_rb
// shares the state RAM through a port mux with base offset U_BASE+N.
//
// WRITE-ADDRESS DISCIPLINE (hit + fixed): every element loop that writes RAM
// ends in a write state (own_addr<=DEST+i, we<=1) followed by a PURE ADVANCE
// state (no own_addr assignment) — the write lands on the advance state's
// posedge using the write address. Assigning the next READ address in the
// write state (or jumping straight into a read state) overwrites the write
// address before the write lands -> data written to the wrong region (the
// RSK loop was overwriting v[] this way).
module drs_iter #(
    parameter N = 1100, M = 2107, NNZ = 4783, HB = 17,
    parameter L = N + M + 1,
    parameter LM1 = L - 1,
    parameter RAM_AW = 17,
    parameter SQRT_L = 64'h404c51d19a043914,   // sqrt(L), L=3208
    parameter AROW_FILE = "../data/kkt/full/Arow.hex",
    parameter ACOL_FILE = "../data/kkt/full/Acol.hex",
    parameter AVAL_FILE = "../data/kkt/full/Aval.hex"
)(
    input  wire              clk, rst_n, start,
    input  wire              refactor,        // 1: stream band + re-factorize
    input  wire [63:0]       band_in,
    input  wire              band_valid,
    input  wire [15:0]       iter,            // 0 = first iteration (FEAS)
    input  wire              scale_valid,     // pulse: recompute diag_r/D_y, rebuild band, g, aa.reset, v remap
    input  wire [63:0]       scale_r,         // new scale for this update
    // ---- state RAM port (drs_iter owns; pdc shares via mux) ----
    output reg  [RAM_AW-1:0] ram_addr,
    output reg  [63:0]       ram_wdata,
    output reg               ram_we,
    input  wire [63:0]       ram_rdata,
    // ---- kkt_solve's own RAM port (external, tb-provided) ----
    output wire [13:0]       kkt_addr,
    output wire [63:0]       kkt_wdata,
    output wire              kkt_we,
    input  wire [63:0]       kkt_rdata,
    output reg               done,
    // ---- LDL band ready (back-pressure to the upstream band streamer) ----
    output wire              band_ready,
    // ---- external SRAM (sram64_ctrl; shared by Anderson + LDL via arbiter) ----
    output wire [19:0]       SRAM_ADDR,
    inout  wire [15:0]       SRAM_DQ,
    output wire              SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N
);
    localparam V_BASE  = 0;
    localparam UT_BASE = L;
    localparam U_BASE  = 2 * L;
    localparam RSK_BASE= 3 * L;
    localparam G_BASE  = 4 * L;
    localparam DR_BASE = 4 * L + LM1;
    localparam VPR_BASE= 5 * L + LM1;      // v_prev (AA apply input x)
    localparam SAFEV_BASE = 6 * L + LM1;   // v before AA (apply input f backup)
    localparam SAFEVP_BASE = 7 * L + LM1;  // v_prev before AA (apply input x backup)
    localparam CB_BASE = 8 * L + 2 * LM1 + L;      // c[0..n) + (-b)[n..n+m) for g recompute
    localparam BAND_BASE = CB_BASE + LM1;          // s_build band output (19800 words)
    localparam DY_BASE = BAND_BASE + (HB + 1) * N; // D_y for s_build (M words)
    localparam ZMASK_BASE = DY_BASE;  // boot zmask data shares DY area (loaded to reg bits at reset)
    localparam LMAX = (N > M) ? N : M;             // = M for the G-FOLD problem
    // spmv workspace for the adaptive-scale residual: reused for A^Ty then Ax.
    // needs LMAX + LMAX words (spmv x@[0,LENX) + out@[LMAX,LMAX+LENO)).
    localparam AA_SAFEGUARD = 64'h3FF0000000000000;  // 1.0 (scs_faithful AA_SAFEGUARD)
    localparam NM_B = 64'h40A2C00000000000;  // max|b| = 2400.0 (reordered frame)
    localparam NM_C = 64'h3FF0000000000000;  // max|c| = 1.0
    localparam LN10 = 64'h40026BB1BBB55516;  // ln(10) = 2.302585093
    localparam SC_MIN = 64'h3EB0C6F7A0B5ED8D;  // 1e-6
    localparam SC_MAX = 64'h412E848000000000;  // 1e6

    // ---- FSM state definitions (moved up so sub-block ports can use st) ----
    localparam S_IDLE=0, S_NM0=1, S_NM1=2, S_NM2=3, S_NM3=4, S_NM4=67, S_NMR=5, S_NMRW=6,
               S_NMRX=7, S_NMV0=8, S_NMV1=9, S_NMV2=10, S_NMV3=11, S_NMV4=12,
               S_KSSTART=13, S_KSBAND=14, S_KSVX=15, S_KSVXW=16,
               S_KSVY=17, S_KSVYW=18, S_KSCAP=19, S_KSDONE=20, S_UTT=21, S_KSDONE_ETA=22,
 S_RP0=23, S_RP1=24, S_RP2=25, S_RP3=26, S_RP4=27, S_RP5=68, S_RPW=28, S_UTT2=29,
 S_GT0=30, S_GT1=31, S_GT2=32, S_GT3=33, S_GT4=34, S_GT5=35, S_GT6=36,
 S_U0=37, S_U1=38, S_U2=39, S_U3=40, S_U4=41, S_U5=42,
 S_CONE=43, S_CONEW=44, S_UM0=45, S_UM1=46, S_UMW=47,
 S_RSK0=48, S_RSK1=49, S_RSK2=50, S_RSK3=51, S_RSK4=52,
 S_RSK5=53, S_RSK6=54, S_RSK7=55, S_RSK8=56, S_RSK9=57,
 S_V0=58, S_V1=59, S_V2=60, S_V3=61, S_V4=62, S_V5=63,
 S_V6=64, S_V7=65, S_DONE=66,
 S_AA0=69, S_AA1=70, S_AA2=71, S_AA3=72, S_AA4=73, S_AA5=74,
 S_AA6=75, S_AA7=76, S_AA8=77, S_AA9=78, S_AA10=79, S_AAW=80,
 S_VPC0=81, S_VPC1=82, S_VPC2=83, S_VPC3=84,
 S_SG0=85, S_SG1=86, S_SG2=87, S_SG3=88, S_SG4=89, S_SG5=90,
 S_SG6=91, S_SG7=92, S_SG8=93, S_SG9=94,
 S_SC0=95, S_SC1=96, S_SC2=97, S_SC3=98,
 S_SCP0=99, S_SCP1=100, S_SCP2=101, S_SCP3=102,
 S_SX0=110, S_SX1=111, S_SX2=112, S_SX3=113, S_SX4=114, S_SX5=115, S_SX6=138,
 S_SB=116, S_SBW=117,
 S_GKS0=118, S_GKSB=119, S_GKSVX=120, S_GKSVXW=121, S_GKSVY=122, S_GKSVYW=123,
 S_GKSZ=124, S_GKSD=125,
 S_AAR=126, S_AARW=127,
 S_VR0=128, S_VR1=129, S_VR2=130, S_VR2B=131, S_VR3=132, S_VR4=133, S_VR5=134, S_VR6=135,
 S_VR7=136, S_VR8=137,
 S_GKSVX0=139, S_GKSVX1=140, S_GKSVY0=141, S_GKSVY1=142, S_GKSVY2=143,
 S_GKSRY0=144, S_GKSDY0=145,
S_INIT0=146, S_INIT1=147, S_INIT2=148,
 S_SCL0=150, S_SCLY0=151, S_SCLY1=152, S_SCLT=153,
 S_SCLD0=154, S_SCLD1=155, S_SCLD2=156, S_SCLD3=157, S_SCLD4=158, S_SCLD5=159,
 S_SCLX0=160, S_SCLX1=161,
 S_SCLP0=162, S_SCLP1=163, S_SCLP2=164, S_SCLP3=165, S_SCLP4=166, S_SCLP5=167,
 S_SCLP6=168, S_SCLP7=169, S_SCLP8=170,
 S_SCLR0=171, S_SCLR1=172, S_SCLR2=173, S_SCLR3=174, S_SCLR4=175, S_SCLR5=176,
 S_SCLR6=177, S_SCLR7=178, S_SCLR8=179, S_SCLR9=180, S_SCLR10=181,
 S_SCLY2=182, S_SCLX2=183, S_SCLY3=184, S_SCLX3=185, S_SCLY4=186, S_SCLX4=187,
 S_SCLD1b=188, S_SCLP2b=189,
 S_SCLB0=190, S_SCLB1=191, S_SCLB2=192, S_SCLB3=193, S_SCLB4=194,
 S_SCLB5=195, S_SCLB6=196, S_SCLB7=197, S_SCLB8=198, S_SCLB9=199,
 S_SX0A=200, S_SX0B=201,
    S_W1=210, S_W2=211, S_W3=212, S_W4=213, S_W5=214, S_W6=215, S_W7=216, S_W8=217,
    S_W9=218, S_W10=219, S_W11=220, S_W12=221, S_W13=222, S_W14=223, S_W15=224, S_W16=225,
    S_W17=226, S_W18=227, S_W19=228, S_W20=229, S_W21=230, S_W22=231, S_W23=232, S_W24=233,
    S_W25=234, S_W26=235, S_W27=236, S_W28=237, S_W29=238, S_W30=239, S_W31=240, S_W32=241,
    S_W33=242, S_W34=243, S_W35=244, S_W36=245, S_W37=246, S_W38=247, S_W39=248, S_W40=249,
    S_W41=250, S_W42=251, S_W43=252, S_W44=253, S_W45=254, S_W46=255,S_W47=257,S_W48=258,S_W49=259,S_W50=260,S_W51=261,S_W52=262,S_W53=263,S_W54=264,S_W55=265,S_W56=266,S_W57=267,S_W58=268,S_W59=269,S_W60=270,S_W61=271,S_W62=272,S_W63=273,S_W64=274,S_W65=275,S_W66=276,S_W67=277,S_W68=278,S_W69=279,S_W70=280,S_W71=281,S_W72=282,S_W73=283,S_W74=284,S_W75=285,S_W76=286,S_W77=287,S_W78=288,S_W79=289,S_W80=290,S_W81=291,S_W82=292,S_W83=293,S_W84=294, S_W85=295;
    reg [8:0] st;   // 0..261 fits in 9 bits (sync-read waits S_W1..W46)
    // force refactor=1 during a scale-update g recompute (band comes from s_build)
    reg gks_active;
    wire ks_refactor = refactor || gks_active;

    // ---- sub-blocks ----
    reg  ks_start, ks_band_valid, ks_din_valid;
    reg  ks_par_update, ks_ry_valid, ks_dy_valid;
    reg  [63:0] ks_band_in, ks_x_in, ks_ry_in, ks_dy_in;
    wire [63:0] ks_z_out;
    wire        ks_o_valid, ks_done;
    wire ks_sram_req, ks_sram_we; wire [17:0] ks_sram_waddr; wire [63:0] ks_sram_wdata;
    wire ks_sram_busy; wire [63:0] ks_sram_rdata;
    wire ks_band_ready;
    reg band_valid_p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) band_valid_p <= 0;
        else band_valid_p <= band_valid;
    end
    wire band_valid_rise = band_valid && !band_valid_p;   // one sample per tb word
    kkt_solve #(.N(N), .M(M), .NNZ(NNZ), .HB(HB),
                .AA_BASE(4*L + 3*L*10),   // SRAM word offset past the Anderson region (109072)
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE),
                .RY_FILE("../data/kkt/full/r_y_r.hex"), .DY_FILE("../data/kkt/full/Dy_r.hex")) u_ks(
        .clk(clk), .rst_n(rst_n), .start(ks_start),
        .band_in(ks_band_in), .band_valid(ks_band_valid), .refactor(ks_refactor),
        .par_update(ks_par_update), .ry_in(ks_ry_in), .dy_in(ks_dy_in),
        .ry_valid(ks_ry_valid), .dy_valid(ks_dy_valid),
        .x_in(ks_x_in), .din_valid(ks_din_valid),
        .ram_addr(kkt_addr), .ram_wdata(kkt_wdata), .ram_we(kkt_we),
        .ram_rdata(kkt_rdata),
        .z_out(ks_z_out), .o_valid(ks_o_valid), .done(ks_done),
        .ldl_sram_req(ks_sram_req), .ldl_sram_we(ks_sram_we), .ldl_sram_waddr(ks_sram_waddr),
        .ldl_sram_wdata(ks_sram_wdata), .ldl_sram_busy(ks_sram_busy), .ldl_sram_rdata(ks_sram_rdata),
        .band_ready(ks_band_ready));

    // ---- s_build (scale update: rebuild S band from D_y + A COO) ----
    reg  sb_start;
    wire sb_done;
    wire [RAM_AW-1:0] sb_addr;
    wire [63:0] sb_wdata;
    wire sb_we;
    s_build #(.N(N), .M(M), .NNZ(NNZ), .HB(HB), .MAXROW(6), .RAM_AW(RAM_AW),
              .BAND_OFFSET(BAND_BASE), .DY_OFFSET(DY_BASE),
              .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE)) u_sb(
        .clk(clk), .rst_n(rst_n), .start(sb_start),
        .ram_addr(sb_addr), .ram_wdata(sb_wdata), .ram_we(sb_we), .ram_rdata(ram_rdata),
        .done(sb_done));

    // ---- spmv for adaptive-scale residual: A^T y (transpose=1) then A x
    //      (transpose=0), both reusing BAND_BASE workspace (band idle outside scale rebuild) (sequential) ----
    reg  saty_start, saty_din_valid; reg [63:0] saty_x_in;
    wire [12:0] saty_addr; wire [63:0] saty_wdata; wire saty_we, saty_done;   // 13-bit: spmv ram_addr width
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(1),
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE)) u_saty(
        .clk(clk), .rst_n(rst_n), .start(saty_start), .x_in(saty_x_in), .din_valid(saty_din_valid),
        .ram_addr(saty_addr), .ram_wdata(saty_wdata), .ram_we(saty_we), .ram_rdata(ram_rdata),
        .out_out(), .o_valid(), .done(saty_done));
    reg  sax_start, sax_din_valid; reg [63:0] sax_x_in;
    wire [12:0] sax_addr; wire [63:0] sax_wdata; wire sax_we, sax_done;   // 13-bit: spmv ram_addr width
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(0),
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE)) u_sax(
        .clk(clk), .rst_n(rst_n), .start(sax_start), .x_in(sax_x_in), .din_valid(sax_din_valid),
        .ram_addr(sax_addr), .ram_wdata(sax_wdata), .ram_we(sax_we), .ram_rdata(ram_rdata),
        .out_out(), .o_valid(), .done(sax_done));

    reg  rp_start, rp_dv;
    reg  [63:0] rp_r, rp_g, rp_p, rp_mu, rp_eta;
    wire rp_done;
    wire [63:0] rp_tau;
    root_plus #(.L(LM1)) u_rp(.clk(clk), .rst_n(rst_n), .start(rp_start),
        .r_in(rp_r), .g_in(rp_g), .p_in(rp_p), .mu_in(rp_mu), .din_valid(rp_dv),
        .eta(rp_eta), .done(rp_done), .tau(rp_tau));

    reg  pdc_start;
    wire [11:0] pdc_addr;
    wire [63:0] pdc_wdata;
    wire pdc_we;
    wire [63:0] pdc_rdata;
    wire pdc_done;
    proj_dual_cone_rb u_pdc(.clk(clk), .rst_n(rst_n), .start(pdc_start),
        .addr(pdc_addr), .wdata(pdc_wdata), .we(pdc_we), .rdata(pdc_rdata),
        .done(pdc_done));

    // ---- Anderson accelerator (apply call: x=v_prev, f=v; f_out = accel f) ----
    reg  aa_start, aa_dv;
    reg  [63:0] aa_x, aa_f;
    wire aa_rdy;
    wire [63:0] aa_fout;
    wire aa_o_valid, aa_done;
    reg  aa_reset_s;
    wire aa_sram_req, aa_sram_we; wire [17:0] aa_sram_waddr; wire [63:0] aa_sram_wdata;
    wire aa_sram_busy; wire [63:0] aa_sram_rdata;
    anderson #(.DIM(L), .MEM(10)) u_aa(.clk(clk), .rst_n(rst_n), .aa_reset(aa_reset_s), .start(aa_start),
        .x_in(aa_x), .f_in(aa_f), .din_valid(aa_dv), .rdy(aa_rdy),
        .f_out(aa_fout), .o_valid(aa_o_valid), .done(aa_done),
        .sram_req(aa_sram_req), .sram_we(aa_sram_we), .sram_waddr(aa_sram_waddr),
        .sram_wdata(aa_sram_wdata), .sram_busy(aa_sram_busy), .sram_rdata(aa_sram_rdata));

    // ---- external SRAM arbiter (Anderson + LDL -> sram64_ctrl) ----
    wire sram_req, sram_we; wire [17:0] sram_waddr; wire [63:0] sram_wdata;
    wire sram_busy; wire [63:0] sram_rdata;
    reg a_grant, l_grant;
    reg a_fall_p, l_fall_p;
    reg sram_busy_p;
    always @(posedge clk) sram_busy_p <= sram_busy;
    // release ONLY on the busy falling edge (a grant issued while the ctrl was
    // idle would otherwise be dropped before it starts; client busy routing
    // then reads 0 and the client stalls forever waiting for busy)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin a_grant <= 0; l_grant <= 0; a_fall_p <= 0; l_fall_p <= 0; end
        else begin
            // fall markers: 1 cycle AFTER the busy falling edge, so the client's
            // read data (sampled when sram_done pulses) still sees the grant held
            // and gets the real rdata instead of the routed 0
            a_fall_p <= (a_grant && sram_busy_p && !sram_busy);
            l_fall_p <= (l_grant && sram_busy_p && !sram_busy);
            if (a_fall_p) a_grant <= 0;
            if (l_fall_p) l_grant <= 0;
            if (!a_grant && !l_grant) begin
                if (aa_sram_req) a_grant <= 1;
                else if (ks_sram_req) l_grant <= 1;
            end
        end
    end
    assign sram_req   = a_grant || l_grant;   // held during grant; ctrl uses rising edge
    assign sram_we    = a_grant ? aa_sram_we : ks_sram_we;
    assign sram_waddr = a_grant ? aa_sram_waddr : ks_sram_waddr;
    assign sram_wdata = a_grant ? aa_sram_wdata : ks_sram_wdata;
    assign aa_sram_busy  = a_grant ? sram_busy : 1'b0;
    assign aa_sram_rdata = a_grant ? sram_rdata : 64'h0;
    assign ks_sram_busy  = l_grant ? sram_busy : 1'b0;
    assign ks_sram_rdata = l_grant ? sram_rdata : 64'h0;
    assign band_ready = (st == S_KSBAND) && ks_band_ready;
    sram64_ctrl #(.AW(18)) u_sram(.clk(clk), .rst_n(rst_n), .req(sram_req), .we(sram_we),
        .waddr(sram_waddr), .wdata(sram_wdata), .busy(sram_busy), .rdata(sram_rdata),
        .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ), .SRAM_CE_N(SRAM_CE_N),
        .SRAM_OE_N(SRAM_OE_N), .SRAM_WE_N(SRAM_WE_N), .SRAM_UB_N(SRAM_UB_N),
        .SRAM_LB_N(SRAM_LB_N));
    // AA round: iter>0 and iter%10==0
    wire aa_round = (iter != 0) && (iter % 10 == 0);
    // FP64 comparison (a > b), combinational, no NaN/denormal handling needed here
    reg  [63:0] cmp_a, cmp_b;
    wire cmp_gt = (cmp_a[63] != cmp_b[63]) ? (~cmp_a[63]) :
                  (cmp_a[63] == 0) ? ((cmp_a[62:0] > cmp_b[62:0]) ? 1'b1 : 1'b0) :
                                     ((cmp_a[62:0] < cmp_b[62:0]) ? 1'b1 : 1'b0);

    // ---- own FP64 combinational pool ----
    reg [63:0] a1, b1;
    wire [63:0] po1;
    fp64_mul um1(a1, b1, po1);
    reg [63:0] a2, b2;
    wire [63:0] po2;
    fp64_mul um2(a2, b2, po2);
    reg [63:0] sa, sb;
    reg ssub;
    wire [63:0] so1;
    fp64_add ua1(sa, sb, ssub, so1);
    reg [63:0] sc, sd;
    reg ssub2;
    wire [63:0] so2;
    fp64_add ua2(sc, sd, ssub2, so2);

    reg  div_start;
    reg  [63:0] div_a, div_b;
    wire div_done; wire [63:0] div_o;
    fp64_div udiv(.clk(clk), .start(div_start), .a(div_a), .b(div_b), .done(div_done), .o(div_o));
    reg  rs_start;
    reg  [63:0] rs_in;
    wire rs_done;
    wire [63:0] rs_o;
    fp64_rsqrt urs(clk, rs_start, rs_in, rs_done, rs_o);

    // ---- RAM port mux: own | pdc (base U_BASE+N) | sb | saty | sax ----
    reg [RAM_AW-1:0] own_addr;
    reg [63:0] own_wdata;
    reg own_we;
    reg saty_active, sax_active;   // set by the scale-residual FSM while a spmv drives RAM

    // ---- FSM ---- (state defs at module top; see S_IDLE etc above)
    reg [15:0] i;
    reg [63:0] vv, utv, gv, drv, tau_r, norm_r, sqacc, v_eta;
    reg [M-1:0] zmask_bits;
    reg [63:0] gacc, norm_g_r, aa_xr, aa_fr;
    reg sflag;
 reg aa_done_r;
 reg rs_done_p, rs_rise;
 reg [15:0] aa_cnt;
 reg [63:0] rinv, zm, vr, vd, vu, vv2;
 reg scale_valid_p;   // prev scale_valid, for rising-edge detection (level-safe trigger)
 reg scale_flow;      // 1 while the scale-update chain runs (suppress residual check at its S_DONE)
 // ---- adaptive-scale residual / decision regs ----
 reg [63:0] max_aty, max_px, max_ax, max_s, max_axs;
 reg [63:0] denom_pri, denom_dual, rel_pri, rel_dual;
 reg [63:0] sum_log_r, scale_cur, tau_scl, new_scale;
 reg signed [15:0] n_log_r, last_scale_iter;
 reg [63:0] aty_i, c_i, ax_i, s_i, nb_i;
 reg [63:0] saty_pf, sax_pf;
 wire [63:0] log_relp, log_reld, exp_arg, exp_out, newscale_mul;
 fp64_log ulogp(rel_pri, log_relp);      // log(rel_pri)   (combinational)
 fp64_log ulogd(rel_dual, log_reld);     // log(rel_dual)
 wire [63:0] n_log_dbl;
 i2d ui2d(n_log_r[10:0], n_log_dbl);     // n_log -> double
 assign exp_arg = div_o;                 // exp arg = sum_log/(2*n_log) from the trigger div
 fp64_exp  uexp(exp_arg, exp_out);       // exp(sum_log/(2n))
 fp64_mul unm(scale_cur, exp_out, newscale_mul);   // scale * exp(...)
 wire pdc_own = (st == S_CONE || st == S_CONEW);
    wire sb_own = (st == S_SB || st == S_SBW);
    always @* begin
        if (sb_own) begin
            ram_addr = sb_addr; ram_wdata = sb_wdata; ram_we = sb_we;
        end else if (pdc_own) begin
            ram_addr = U_BASE + N + pdc_addr;
            ram_wdata = pdc_wdata;
            ram_we = pdc_we;
        end else if (saty_active) begin
            ram_addr = BAND_BASE + saty_addr;
            ram_wdata = saty_wdata;
            ram_we = saty_we;
        end else if (sax_active) begin
            ram_addr = BAND_BASE + sax_addr;
            ram_wdata = sax_wdata;
            ram_we = sax_we;
        end else begin
            ram_addr = own_addr;
            ram_wdata = own_wdata;
            ram_we = own_we;
        end
    end
    assign pdc_rdata = ram_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_INIT0; done <= 0;
            own_addr <= 0; own_wdata <= 0; own_we <= 0;
            ks_start <= 0; ks_band_valid <= 0; ks_din_valid <= 0;
            ks_band_in <= 0; ks_x_in <= 0;
            ks_par_update <= 0; ks_ry_valid <= 0; ks_dy_valid <= 0;
            ks_ry_in <= 0; ks_dy_in <= 0;
            rp_start <= 0; rp_dv <= 0; rp_r <= 0; rp_g <= 0; rp_p <= 0;
            rp_mu <= 0; rp_eta <= 0;
            pdc_start <= 0;
            a1 <= 0; b1 <= 0; a2 <= 0; b2 <= 0;
            sa <= 0; sb <= 0; ssub <= 0; sc <= 0; sd <= 0; ssub2 <= 0;
            rs_start <= 0; rs_in <= 0;
            i <= 0; vv <= 0; utv <= 0; gv <= 0; drv <= 0;
            tau_r <= 0; norm_r <= 0; sqacc <= 0; v_eta <= 0;
            aa_start <= 0; aa_dv <= 0; aa_x <= 0; aa_f <= 0;
            gacc <= 0; norm_g_r <= 0; aa_xr <= 0; aa_fr <= 0; sflag <= 0;
            cmp_a <= 0; cmp_b <= 0; aa_done_r <= 0;
            rs_done_p <= 0; rs_rise <= 0; aa_cnt <= 0;
            rinv <= 0; zm <= 0; vr <= 0; vd <= 0; vu <= 0; vv2 <= 0;
            sb_start <= 0; aa_reset_s <= 0; gks_active <= 0; scale_valid_p <= 0; scale_flow <= 0;
            saty_start <= 0; saty_din_valid <= 0; saty_x_in <= 0; saty_active <= 0;
            sax_start <= 0; sax_din_valid <= 0; sax_x_in <= 0; sax_active <= 0;
            max_aty <= 0; max_px <= 0; max_ax <= 0; max_s <= 0; max_axs <= 0;
            denom_pri <= 0; denom_dual <= 0; rel_pri <= 0; rel_dual <= 0;
            sum_log_r <= 0; scale_cur <= 64'h3FF0000000000000; tau_scl <= 0; new_scale <= 0;  // scale_cur init 1.0
            n_log_r <= 0; last_scale_iter <= -16'd100; aty_i <= 0; c_i <= 0; ax_i <= 0; s_i <= 0; nb_i <= 0;
            zmask_bits <= 0;
        end else begin
            ks_start <= 0; ks_band_valid <= 0; ks_din_valid <= 0;
            ks_ry_valid <= 0; ks_dy_valid <= 0; ks_par_update <= 0;
            rp_start <= 0; rp_dv <= 0; pdc_start <= 0; rs_start <= 0;
            div_start <= 0; sb_start <= 0;
            saty_start <= 0; saty_din_valid <= 0;
            sax_start <= 0; sax_din_valid <= 0;
            aa_start <= 0; aa_dv <= 0;
            if (ks_band_valid && !ks_band_ready) ks_band_valid <= 0;   // level-held: clear after LDL sampled
            own_we <= 0;
            scale_valid_p <= scale_valid;
            case (st)
            // ---- boot: load zmask bits from DY area (tb preloaded zmask data) ----
            S_INIT0: begin i <= 0; own_addr <= ZMASK_BASE; st <= S_W11; end
            S_INIT1: begin zm <= ram_rdata; st <= S_INIT2; end
            S_INIT2: begin
                zmask_bits[i] <= (zm != 64'h0);
                if (i + 1 >= M) st <= S_IDLE;
                else begin i <= i + 1; own_addr <= ZMASK_BASE + i + 1; st <= S_W12; end
            end
            S_IDLE: begin
                done <= 0;
                scale_flow <= 0;
                if (scale_valid && !scale_valid_p) begin scale_cur <= scale_r; i <= 0; own_addr <= ZMASK_BASE; st <= S_W1; end
                else if (start) begin
                    if (iter == 0) begin i <= 0; own_addr <= V_BASE; st <= S_W13; end  // FEAS: v_prev=v0, no normalize
                    else if (aa_round) begin i <= 0; own_addr <= VPR_BASE; st <= S_W14; end  // AA apply first
                    else begin i <= 0; own_addr <= V_BASE; st <= S_W15; end
                end
            end
            // ============ normalize v (iter>=1): pass1 ||v||, pass2 scale ====
            S_NM0: begin i <= 0; sqacc <= 0; own_addr <= V_BASE; st <= S_W16; end
            S_NM1: begin
                vv <= ram_rdata; own_addr <= V_BASE + i + 1; st <= S_W17;
            end
            S_NM2: begin a1 <= vv; b1 <= vv; st <= S_NM3; end       // vv^2
            S_NM3: begin
                sa <= sqacc; sb <= po1; ssub <= 0; st <= S_NM4;
            end
            // sqacc must sample so1 AFTER sa/sb updated (S_NM3) — same-cycle
            // sa<=sqacc + sqacc<=so1 would accumulate every other element.
            S_NM4: begin
                sqacc <= so1;
                if (i + 1 >= L) st <= S_NMR;
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_W18; end
            end
            S_NMR: begin
                sqacc <= so1; rs_start <= 1; rs_in <= so1; st <= S_NMRW;
            end
            S_NMRW: begin
                // rs_done RISING-EDGE only: the residue from the previous
                // normalize (iter>=1) would otherwise exit on cycle 1 with the
                // STALE rs_o (norm_r = previous iteration's scale).
                rs_done_p <= rs_done;
                if (rs_done && !rs_done_p) begin a1 <= SQRT_L; b1 <= rs_o; st <= S_NMRX; end
            end
            S_NMRX: begin norm_r <= po1; st <= S_NMV0; end
            S_NMV0: begin i <= 0; own_addr <= V_BASE; st <= S_W19; end
            S_NMV1: begin
                vv <= ram_rdata; own_addr <= V_BASE + i + 1; st <= S_W20;
            end
            S_NMV2: begin a1 <= vv; b1 <= norm_r; st <= S_NMV3; end   // v*norm
            S_NMV3: begin
                own_addr <= V_BASE + i; own_wdata <= po1; own_we <= 1;
                st <= S_NMV4;
            end
            S_NMV4: begin
                if (i + 1 >= L) begin i <= 0; own_addr <= V_BASE; st <= S_W21; end
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_W22; end
            end
            // ============ Anderson apply (aa_round): stream (x=v_prev, f=v),
            // back up both to SAFEV/SAFEVP, accumulate ||x-f||^2 for norm_g ====
            S_AA0: begin aa_start <= 1; gacc <= 0; aa_done_r <= 0; aa_cnt <= aa_cnt + 1; i <= 0; own_addr <= VPR_BASE; st <= S_W23; end
            S_AA1: begin aa_xr <= ram_rdata; own_addr <= V_BASE + i; st <= S_W24; end
            S_AA2: begin aa_fr <= ram_rdata; sa <= aa_xr; sb <= aa_fr; ssub <= 1; st <= S_AA3; end
            S_AA3: begin
                a1 <= so1; b1 <= so1; aa_x <= aa_xr; aa_f <= aa_fr; aa_dv <= 1; st <= S_AA4;
            end
            S_AA4: begin sc <= gacc; sd <= po1; ssub2 <= 0; st <= S_AA5; end
            S_AA5: begin gacc <= so2; st <= S_AA6; end
            S_AA6: begin
                own_addr <= SAFEV_BASE + i; own_wdata <= aa_fr; own_we <= 1; st <= S_AA7;
            end
            S_AA7: begin
                own_addr <= SAFEVP_BASE + i; own_wdata <= aa_xr; own_we <= 1;
                if (aa_rdy || i + 1 >= L) begin
                    aa_dv <= 0;   // clear after Anderson sampled (level-held valid)
                    if (i + 1 >= L) begin i <= 0; st <= S_AA8; end
                    else begin i <= i + 1; own_addr <= VPR_BASE + i + 1; st <= S_W25; end
                end
            end
            S_AA8: begin
                aa_dv <= 0;
                if (aa_o_valid) begin
                    own_addr <= V_BASE + i; own_wdata <= aa_fout; own_we <= 1;
                    if (i + 1 >= L) st <= S_AA9; else i <= i + 1;
                end
            end
            S_AA9: begin
                // aa_done pulses here (1 cycle after the last f_out) — latch it
                // NOW; S_AA10 samples it one cycle too late (S_IDLE clears it).
                rs_start <= 1; rs_in <= gacc; rs_rise <= 0;
                if (aa_done) aa_done_r <= 1;
                st <= S_AA10;
            end
            S_AA10: begin
                if (aa_done) aa_done_r <= 1;
                rs_done_p <= rs_done;
                if (rs_done && !rs_done_p) begin norm_g_r <= rs_o; rs_rise <= 1; end
                if (aa_done_r && rs_rise) begin
                    aa_done_r <= 0; rs_rise <= 0; i <= 0; own_addr <= V_BASE; st <= S_W26;
                end
            end
            // ============ v_prev <- v (after AA apply / normalize) ============
            S_VPC0: begin i <= 0; own_addr <= V_BASE; st <= S_W7; end
            S_W7: begin st <= S_VPC1; end
            S_VPC1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_W27; end
            S_VPC2: begin
                own_addr <= VPR_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_VPC3;
            end
            S_VPC3: begin
                if (i + 1 >= L) st <= S_KSSTART;
                else begin i <= i + 1; own_addr <= V_BASE + i; st <= S_W8; end
            end
            S_W8: begin st <= S_VPC1; end
            // ============ KKT solve ============
            S_KSSTART: begin
                ks_start <= 1; i <= 0;
                if (refactor) st <= S_KSBAND; else st <= S_KSVX;
            end
            S_KSBAND: begin
                if (band_valid_rise && ks_band_ready && !ks_band_valid) begin
                    ks_band_valid <= 1; ks_band_in <= band_in;
                    if (i + 1 >= (HB+1)*N) begin i <= 0; st <= S_KSVX; end
                    else i <= i + 1;
                end
            end
            S_KSVX: begin own_addr <= V_BASE + i; st <= S_W9; end
            S_W9: begin st <= S_KSVXW; end
            S_KSVXW: begin
                ks_x_in <= ram_rdata; ks_din_valid <= 1;
                if (i + 1 >= N) begin i <= 0; st <= S_KSVY; end
                else begin i <= i + 1; st <= S_KSVX; end
            end
            S_KSVY: begin own_addr <= V_BASE + N + i; st <= S_W10; end
            S_W10: begin st <= S_KSVYW; end
            S_KSVYW: begin
                ks_x_in <= ram_rdata; ks_din_valid <= 1;
                if (i + 1 >= M) begin i <= 0; st <= S_KSCAP; end
                else begin i <= i + 1; st <= S_KSVY; end
            end
            // capture zx(N) + zy(M) -> u_t[0..l-2]  (one per ks_o_valid; write
            // addr updated every cycle, write lands on the NEXT posedge with
            // the CURRENT cycle's address — correct for a continuous stream)
            S_KSCAP: begin
                if (ks_o_valid) begin
                    own_addr <= UT_BASE + i; own_wdata <= ks_z_out; own_we <= 1;
                    if (i + 1 >= LM1) st <= S_KSDONE; else i <= i + 1;
                end
            end
            S_KSDONE: begin
                if (ks_done) begin
                    if (iter == 0) begin
                        // u_t[l-1] = 1.0 (FEAS), tau = 1.0 for the -= tau*g pass
                        tau_r <= 64'h3FF0000000000000;
                        own_addr <= UT_BASE + LM1; own_wdata <= 64'h3FF0000000000000; own_we <= 1;
                        st <= S_UTT;
                    end else begin own_addr <= V_BASE + LM1; st <= S_W28; end
                end
            end
            S_UTT: begin st <= S_GT0; end      // u_t[l-1] write lands here
            S_KSDONE_ETA: begin
                v_eta <= ram_rdata; i <= 0; st <= S_RP0;
            end
            // ============ root_plus (iter>=1): stream LM1 4-tuples ===========
            S_RP0: begin
                rp_start <= 1; rp_eta <= v_eta; i <= 0; own_addr <= G_BASE; st <= S_W29;
            end
            S_RP1: begin gv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_W30; end
            S_RP2: begin utv <= ram_rdata; own_addr <= V_BASE + i; st <= S_W31; end
            S_RP3: begin vv <= ram_rdata; own_addr <= DR_BASE + i; st <= S_W32; end
            S_RP4: begin
                drv <= ram_rdata; st <= S_RP5;
            end
            S_RP5: begin
                rp_r <= drv; rp_g <= gv; rp_p <= utv; rp_mu <= vv; rp_dv <= 1;
                if (i + 1 >= LM1) st <= S_RPW;
                else begin i <= i + 1; own_addr <= G_BASE + i + 1; st <= S_W33; end
            end
            S_RPW: begin
                if (rp_done) begin
                    tau_r <= rp_tau;
                    own_addr <= UT_BASE + LM1; own_wdata <= rp_tau; own_we <= 1;
                    st <= S_UTT2;
                end
            end
            S_UTT2: begin st <= S_GT0; end      // u_t[l-1] write lands here
            // ============ u_t[:l-1] -= tau*g ============
            S_GT0: begin i <= 0; own_addr <= UT_BASE; st <= S_W34; end
            S_GT1: begin utv <= ram_rdata; own_addr <= G_BASE + i; st <= S_W35; end
            S_GT2: begin gv <= ram_rdata; st <= S_GT3; end
            S_GT3: begin a1 <= tau_r; b1 <= gv; st <= S_GT4; end
            S_GT4: begin sa <= utv; sb <= po1; ssub <= 1; st <= S_GT5; end
            S_GT5: begin
                own_addr <= UT_BASE + i; own_wdata <= so1; own_we <= 1; st <= S_GT6;
            end
            S_GT6: begin
                if (i + 1 >= LM1) st <= S_U0;
                else begin i <= i + 1; own_addr <= UT_BASE + i + 1; st <= S_W36; end
            end
            // ============ u = 2*u_t - v ============
            S_U0: begin i <= 0; own_addr <= UT_BASE; st <= S_W37; end
            S_U1: begin utv <= ram_rdata; own_addr <= V_BASE + i; st <= S_W38; end
            S_U2: begin vv <= ram_rdata; a1 <= utv; b1 <= 64'h4000000000000000; st <= S_U3; end
            S_U3: begin sa <= po1; sb <= vv; ssub <= 1; st <= S_U4; end
            S_U4: begin
                own_addr <= U_BASE + i; own_wdata <= so1; own_we <= 1;
                st <= S_U5;
            end
            S_U5: begin
                if (i + 1 >= L) st <= S_CONE;
                else begin i <= i + 1; own_addr <= UT_BASE + i + 1; st <= S_W39; end
            end
            // ============ cone projection (u[n:l-1], always) ============
            S_CONE: begin pdc_start <= 1; st <= S_CONEW; end
            S_CONEW: begin
                if (pdc_done) begin
                    if (iter == 0) begin
                        own_addr <= U_BASE + LM1; own_wdata <= 64'h3FF0000000000000;
                        own_we <= 1; st <= S_UMW;
                    end else begin own_addr <= U_BASE + LM1; st <= S_W40; end
                end
            end
            // ============ u[l-1] = max(u[l-1], 0) (iter>=1) ============
            S_UM0: begin vv <= ram_rdata; st <= S_UM1; end
            S_UM1: begin
                own_addr <= U_BASE + LM1; own_wdata <= (vv[63] ? 64'h0 : vv);
                own_we <= 1; st <= S_UMW;
            end
            S_UMW: begin st <= S_RSK0; end
            // ============ rsk = (v+u-2*u_t)*diag_r ============
            S_RSK0: begin i <= 0; own_addr <= V_BASE; st <= S_W41; end
            S_RSK1: begin vv <= ram_rdata; own_addr <= U_BASE + i; st <= S_W42; end
            S_RSK2: begin utv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_W43; end
            S_RSK3: begin gv <= ram_rdata; own_addr <= DR_BASE + i; st <= S_W44; end
            S_RSK4: begin
                drv <= ram_rdata; sc <= vv; sd <= utv; ssub2 <= 0; st <= S_RSK5;
            end
            S_RSK5: begin a1 <= gv; b1 <= 64'h4000000000000000; st <= S_RSK6; end
            S_RSK6: begin sa <= so2; sb <= po1; ssub <= 1; st <= S_RSK7; end
            S_RSK7: begin a2 <= so1; b2 <= drv; st <= S_RSK8; end
            S_RSK8: begin
                own_addr <= RSK_BASE + i; own_wdata <= po2; own_we <= 1;
                st <= S_RSK9;
            end
            S_RSK9: begin
                if (i + 1 >= L) st <= S_V0;
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_W45; end
            end
            // ============ v += 1.5*(u - u_t) ============
            S_V0: begin i <= 0; own_addr <= U_BASE; st <= S_W46; end
            S_V1: begin utv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_W47; end
            S_V2: begin gv <= ram_rdata; own_addr <= V_BASE + i; st <= S_W48; end
            S_V3: begin vv <= ram_rdata; sa <= utv; sb <= gv; ssub <= 1; st <= S_V4; end
            S_V4: begin a1 <= so1; b1 <= 64'h3FF8000000000000; st <= S_V5; end
            S_V5: begin sc <= vv; sd <= po1; ssub2 <= 0; st <= S_V6; end
            S_V6: begin
                own_addr <= V_BASE + i; own_wdata <= so2; own_we <= 1;
                st <= S_V7;
            end
            S_V7: begin
                if (i + 1 >= L) begin
                    if (aa_round) begin i <= 0; own_addr <= V_BASE; st <= S_W49; end
                    else st <= S_DONE;
                end
                else begin i <= i + 1; own_addr <= U_BASE + i + 1; st <= S_W50; end
            end
            // ============ AA safeguard: ||v - v_prev|| vs AA_SAFEGUARD*norm_g ===
            S_SG0: begin i <= 0; gacc <= 0; own_addr <= V_BASE; st <= S_W51; end
            S_SG1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_W52; end
            S_SG2: begin gv <= ram_rdata; sa <= vv; sb <= gv; ssub <= 1; st <= S_SG3; end
            S_SG3: begin a1 <= so1; b1 <= so1; st <= S_SG4; end
            S_SG4: begin sc <= gacc; sd <= po1; ssub2 <= 0; st <= S_SG5; end
            S_SG5: begin
                gacc <= so2;
                if (i + 1 >= L) st <= S_SG6;
                else begin i <= i + 1; own_addr <= V_BASE + i; st <= S_W53; end
            end
            S_SG6: begin rs_start <= 1; rs_in <= gacc; st <= S_SG7; end
            S_SG7: begin
                rs_done_p <= rs_done;
                if (rs_done && !rs_done_p) begin
                    // safeguard reject iff ||diff|| > AA_SAFEGUARD*||x-f|| (scs_faithful).
                    // norm_g_r = 1/||x-f|| and rs_o = 1/||diff|| are rsqrt RECIPROCALS,
                    // so reject iff 1/||x-f|| > 1/||diff||  i.e. cmp_a(=1/||x-f||)
                    // > cmp_b(=1/||diff||). AA_SAFEGUARD=1.0 so norm_g_r*AA_SAFEGUARD
                    // == norm_g_r. FIX: cmp_a<=po2(threshold side), cmp_b<=rs_o(diff side).
                    a2 <= norm_g_r; b2 <= AA_SAFEGUARD; cmp_b <= rs_o; st <= S_SG8;
                end
            end
            S_SG8: begin cmp_a <= po2; st <= S_SG9; end
            S_SG9: begin
                // scs_faithful.safeguard returns early unless the LAST apply
                // actually solved (self.success), which happens only from the
                // 11th apply onward (aa.iter >= AA_MINLEN=10). aa_cnt is 1-based:
                // checks are meaningful from aa_cnt >= 11.
                if (cmp_gt && aa_cnt >= 11) st <= S_SC0;      // reject: roll back
                else st <= S_DONE;
            end
            // ============ rollback: SAFEV->v, SAFEVP->v_prev ============
            S_SC0: begin i <= 0; own_addr <= SAFEV_BASE; st <= S_W54; end
            S_SC1: begin vv <= ram_rdata; own_addr <= V_BASE + i; st <= S_W55; end
            S_SC2: begin
                own_addr <= V_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_SC3;
            end
            S_SC3: begin
                if (i + 1 >= L) st <= S_SCP0;
                // forward-addr MUST be BASE+i+1 (non-blocking i and addr update
                // in the same cycle with the OLD i; +1 pre-reads the NEXT element)
                else begin i <= i + 1; own_addr <= SAFEV_BASE + i + 1; st <= S_W56; end
            end
            S_SCP0: begin i <= 0; own_addr <= SAFEVP_BASE; st <= S_W57; end
            S_SCP1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_W58; end
            S_SCP2: begin
                own_addr <= VPR_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_SCP3;
            end
            S_SCP3: begin
                if (i + 1 >= L) st <= S_DONE;
                else begin i <= i + 1; own_addr <= SAFEVP_BASE + i + 1; st <= S_W59; end
            end

            // ============ scale update: recompute diag_r[n:n+m] + D_y, then
            // rebuild band (s_build), g = KKT^-1[c;-b], aa.reset, v remap ============
            S_SX0: begin
                // rinv = 1/scale (one division); scale_cur holds the scale to use
                // (set in S_IDLE for an external scale_valid, or by the residual
                // trigger path, so both flows drive the same downstream chain).
                // ARM the div: a stale div_done (e.g. the residual sum_log/(2n) div
                // that triggered this path) would otherwise be captured as rinv.
                scale_flow <= 1;
                div_a <= 64'h3FF0000000000000; div_b <= scale_cur; div_start <= 1;
                i <= 0; st <= S_SX0A;
            end
            S_SX0A: begin if (!div_done) st <= S_SX0B; else st <= S_SX0A; end   // wait done drop
            S_SX0B: begin if (div_done) begin rinv <= div_o; st <= S_SX1; end else st <= S_SX0B; end
            S_SX1: begin zm <= {63'b0, zmask_bits[i]}; st <= S_SX2; end
            S_SX2: begin zm <= {63'b0, zmask_bits[i]}; st <= S_SX3; end      // zero-cone row mask[i]
            S_SX3: begin
                // zmask=1.0 -> zero-cone row: r_y = rinv*1e-3, D_y = scale*1000
                // zmask=0.0 -> other rows:      r_y = rinv*1,    D_y = scale*1
                a1 <= rinv;
                b1 <= (zm == 64'h0) ? 64'h3FF0000000000000 : 64'h3F50624DD2F1A9FC;
                a2 <= scale_cur;
                b2 <= (zm == 64'h0) ? 64'h3FF0000000000000 : 64'h408F400000000000;
                st <= S_SX4;
            end
            S_SX4: begin
                own_addr <= DR_BASE + N + i; own_wdata <= po1; own_we <= 1; st <= S_SX5;
            end
            S_SX5: begin
                // write D_y — must be its OWN state: the else-branch ZMASK forward
                // address would override DY_BASE+i in the same cycle (double assign).
                own_addr <= DY_BASE + i; own_wdata <= po2; own_we <= 1; st <= S_SX6;
            end
            S_SX6: begin
                if (i + 1 >= M) begin sb_start <= 1; st <= S_SB; end
                else begin i <= i + 1; st <= S_SX2; end
            end
            // ---- s_build: rebuild S band into BAND_BASE ----
            S_SB: begin sb_start <= 1; st <= S_SBW; end
            S_SBW: begin
                if (sb_done) begin i <= 0; own_addr <= BAND_BASE; st <= S_W60; end
            end
            // ---- KKT: g = KKT^-1[c;-b]  (band from BAND_BASE, rhs c/-b from CB_BASE) ----
            // NOTE: kkt_solve solves M·z = [rho_x·v_x; -R_y·v_y]. To get g=M^-1[c;-b]
            // we must feed v_x = c/rho_x (=c*1e6) and v_y = D_y·b (= -D_y·nb_r), NOT
            // (c,-b). Feeding (c,-b) yields M^-1[rho_x·c; R_y·b] -> wrong g (g[0]=2.5
            // vs SW -449.98). Also stream the NEW r_y/D_y (par_update) so zy uses the
            // rescaled D_y, not the static scale=1 arrays.
            S_GKS0: begin
                ks_start <= 1; gks_active <= 1; ks_par_update <= 1;
                i <= 0; own_addr <= DR_BASE + N; st <= S_W61;
            end
            // stream M r_y (DR_BASE+N) then M D_y (DY_BASE) into kkt_solve's arrays
            S_GKSRY0: begin
                ks_ry_in <= ram_rdata; ks_ry_valid <= 1;
                if (i + 1 >= M) begin i <= 0; own_addr <= DY_BASE; st <= S_W62; end
                else begin i <= i + 1; own_addr <= DR_BASE + N + i + 1; st <= S_W63; end
            end
            S_GKSDY0: begin
                ks_dy_in <= ram_rdata; ks_dy_valid <= 1;
                if (i + 1 >= M) begin i <= 0; own_addr <= BAND_BASE; st <= S_W2; end
                else begin i <= i + 1; own_addr <= DY_BASE + i + 1; st <= S_W64; end
            end
            S_GKSB: begin
                ks_band_in <= ram_rdata; ks_band_valid <= 1;
                if (i + 1 >= (HB + 1) * N) begin i <= 0; own_addr <= CB_BASE; st <= S_W3; end
                else begin i <= i + 1; own_addr <= BAND_BASE + i + 1; st <= S_W65; end
            end
            // v_x[i] = c[i]/rho_x = c[i]*1e6  (1/rho_x = 1e6)
            S_GKSVX0: begin a2 <= ram_rdata; b2 <= 64'h412E848000000000; st <= S_GKSVX1; end
            S_GKSVX1: begin
                ks_x_in <= po2; ks_din_valid <= 1;
                if (i + 1 >= N) begin i <= 0; own_addr <= CB_BASE + N; st <= S_W66; end
                else begin i <= i + 1; own_addr <= CB_BASE + i + 1; st <= S_W67; end
            end
            // v_y[i] = D_y[i]*b[i] = -D_y[i]*nb_r[i]  (nb_r = -b stored at CB_BASE+N)
            S_GKSVY0: begin vd <= ram_rdata; own_addr <= DY_BASE + i; st <= S_W68; end
            S_GKSVY1: begin
                a1 <= vd; b1 <= (ram_rdata ^ 64'h8000000000000000); st <= S_GKSVY2;
            end
            S_GKSVY2: begin
                ks_x_in <= po1; ks_din_valid <= 1;
                if (i + 1 >= M) begin i <= 0; st <= S_GKSZ; end
                else begin i <= i + 1; own_addr <= CB_BASE + N + i + 1; st <= S_W69; end
            end
            S_GKSZ: begin
                if (ks_o_valid) begin
                    own_addr <= G_BASE + i; own_wdata <= ks_z_out; own_we <= 1;
                    if (i + 1 >= LM1) st <= S_GKSD; else i <= i + 1;
                end
            end
            S_GKSD: begin
                if (ks_done) begin gks_active <= 0; aa_reset_s <= 1; st <= S_AAR; end
            end
            // ---- anderson reset (clear history, aa_cnt) ----
            S_AAR: begin aa_reset_s <= 1; st <= S_AARW; end
            S_AARW: begin
                if (aa_done) begin
                    aa_reset_s <= 0; aa_cnt <= 0; i <= 0; own_addr <= RSK_BASE; st <= S_W70;
                end
            end
            // ---- v remap: v = rsk/diag_r + 2*u_t - u (all l-1 elements) ----
            S_VR0: begin i <= 0; own_addr <= RSK_BASE; st <= S_W71; end
            S_VR1: begin vr <= ram_rdata; own_addr <= DR_BASE + i; st <= S_W72; end
            S_VR2: begin
                vd <= ram_rdata; div_a <= vr;
                own_addr <= UT_BASE + i; st <= S_W73;
            end
            S_VR2B: begin
                // div_b must use the fresh vd (vd updates at S_VR2) — one cycle later
                div_b <= vd; div_start <= 1; st <= S_VR3;
            end
            S_VR3: begin vu <= ram_rdata; own_addr <= U_BASE + i; st <= S_W74; end
            S_VR4: begin vv2 <= ram_rdata; a1 <= vu; b1 <= 64'h4000000000000000; st <= S_VR5; end
            S_VR5: begin sa <= po1; sb <= vv2; ssub <= 1; st <= S_VR6; end
            S_VR6: begin
                if (div_done) begin sc <= so1; sd <= div_o; ssub2 <= 0; st <= S_VR7; end
            end
            S_VR7: begin
                own_addr <= V_BASE + i; own_wdata <= so2; own_we <= 1; st <= S_VR8;
            end
            S_VR8: begin
                if (i + 1 >= L) st <= S_DONE;
                else begin i <= i + 1; own_addr <= RSK_BASE + i + 1; st <= S_W75; end
            end

            S_DONE: begin
                if (iter % 25 == 0 && iter > 0 && !scale_flow) begin
                    // run the residual+scale chain THIS iteration before done,
                    // otherwise it bleeds into the next iter and skips its DRS step.
                    done <= 0;
                    i <= 0; own_addr <= U_BASE + N; st <= S_W4;
                end else begin
                    done <= 1; st <= S_IDLE;
                end
            end
            // ============ adaptive-scale residual (every 25 iters) ============
            // After a completed iteration (u/rsk updated): compute
            //   max_aty=max|A^Ty|, max_px=max|A^Ty+tau*c|  (dual, over N)
            //   max_ax=max|Ax|, max_s=max|s|, max_axs=max|Ax+s+tau*nb|  (primal, over M)
            //   denom_pri=max(max_ax,max_s,nm_b*tau); rel_pri=max_axs/denom_pri
            //   denom_dual=max(max_aty,nm_c*tau);      rel_dual=max_px/denom_dual
            // (b stored as nb=-b at CB_BASE+N, so ax_s_btau = Ax+s+tau*nb)
            S_SCL0: begin done <= 0; saty_start <= 1; i <= 0; own_addr <= U_BASE + N; st <= S_W76; end
            // 4-cycle feed per element: RD(read u, own mux) -> FEED(spmv x_in) ->
            // SAMP(spmv samples din_valid, mux held) -> WR(spmv ram_we takes effect,
            // mux held) so the delayed spmv write and the u-read never collide.
            S_SCLY0: begin saty_pf <= ram_rdata; st <= S_SCLY1; end
            S_SCLY1: begin saty_x_in <= saty_pf; saty_din_valid <= 1; saty_active <= 1; st <= S_SCLY2; end
            S_SCLY2: begin saty_active <= 1; st <= S_SCLY3; end
            S_SCLY3: begin
                if (i + 1 >= M) begin i <= 0; saty_active <= 1; st <= S_SCLY4; end
                else begin saty_active <= 0; i <= i + 1; own_addr <= U_BASE + N + i + 1; st <= S_W77; end
            end
            S_SCLY4: begin
                if (saty_done) begin
                    saty_active <= 0; max_aty <= 0; max_px <= 0;
                    i <= 0; own_addr <= U_BASE + LM1; st <= S_W5;
                end
            end
            S_SCLT: begin tau_scl <= ram_rdata; i <= 0; own_addr <= BAND_BASE + LMAX; st <= S_W78; end
            S_SCLD0: begin aty_i <= ram_rdata; own_addr <= CB_BASE + i; st <= S_W79; end
            S_SCLD1: begin c_i <= ram_rdata; a1 <= tau_scl; st <= S_SCLD1b; end
            S_SCLD1b: begin b1 <= c_i; st <= S_SCLD2; end
            S_SCLD2: begin sa <= aty_i; sb <= po1; ssub <= 0; st <= S_SCLD3; end
            S_SCLD3: begin cmp_a <= {1'b0, aty_i[62:0]}; cmp_b <= max_aty; st <= S_SCLD4; end
            S_SCLD4: begin
                max_aty <= cmp_gt ? {1'b0, aty_i[62:0]} : max_aty;
                cmp_a <= {1'b0, so1[62:0]}; cmp_b <= max_px; st <= S_SCLD5;
            end
            S_SCLD5: begin
                max_px <= cmp_gt ? {1'b0, so1[62:0]} : max_px;
                if (i + 1 >= N) begin i <= 0; sax_start <= 1; own_addr <= U_BASE; st <= S_W80; end
                else begin i <= i + 1; own_addr <= BAND_BASE + LMAX + i + 1; st <= S_W81; end
            end
            S_SCLX0: begin sax_pf <= ram_rdata; st <= S_SCLX1; end
            S_SCLX1: begin sax_x_in <= sax_pf; sax_din_valid <= 1; sax_active <= 1; st <= S_SCLX2; end
            S_SCLX2: begin sax_active <= 1; st <= S_SCLX3; end
            S_SCLX3: begin
                if (i + 1 >= N) begin i <= 0; sax_active <= 1; st <= S_SCLX4; end
                else begin sax_active <= 0; i <= i + 1; own_addr <= U_BASE + i + 1; st <= S_W82; end
            end
            S_SCLX4: begin
                if (sax_done) begin
                    sax_active <= 0; max_ax <= 0; max_s <= 0; max_axs <= 0;
                    i <= 0; own_addr <= BAND_BASE + LMAX; st <= S_W6;
                end
            end
            S_SCLP0: begin ax_i <= ram_rdata; own_addr <= RSK_BASE + N + i; st <= S_W83; end
            S_SCLP1: begin s_i <= ram_rdata; own_addr <= CB_BASE + N + i; st <= S_W84; end
            S_SCLP2: begin nb_i <= ram_rdata; a1 <= tau_scl; st <= S_SCLP2b; end
            S_SCLP2b: begin b1 <= nb_i; st <= S_SCLP3; end
            S_SCLP3: begin sa <= s_i; sb <= po1; ssub <= 0; st <= S_SCLP4; end      // s + tau*nb -> so1
            S_SCLP4: begin sc <= ax_i; sd <= so1; ssub2 <= 0; st <= S_SCLP5; end    // axs = ax + so1 -> so2
            S_SCLP5: begin cmp_a <= {1'b0, ax_i[62:0]}; cmp_b <= max_ax; st <= S_SCLP6; end
            S_SCLP6: begin
                max_ax <= cmp_gt ? {1'b0, ax_i[62:0]} : max_ax;
                cmp_a <= {1'b0, s_i[62:0]}; cmp_b <= max_s; st <= S_SCLP7;
            end
            S_SCLP7: begin
                max_s <= cmp_gt ? {1'b0, s_i[62:0]} : max_s;
                cmp_a <= {1'b0, so2[62:0]}; cmp_b <= max_axs; st <= S_SCLP8;
            end
            S_SCLP8: begin
                max_axs <= cmp_gt ? {1'b0, so2[62:0]} : max_axs;
                if (i + 1 >= M) st <= S_SCLR0;
                else begin i <= i + 1; own_addr <= BAND_BASE + LMAX + i + 1; st <= S_W85; end
            end
            // rel_pri / rel_dual (fp64_div, ARM'd for the level-done)
            S_SCLR0: begin a1 <= NM_B; b1 <= tau_scl; st <= S_SCLR1; end
            S_SCLR1: begin cmp_a <= max_ax; cmp_b <= max_s; st <= S_SCLR2; end
            S_SCLR2: begin
                denom_pri <= (cmp_gt ? max_ax : max_s);
                cmp_a <= po1; cmp_b <= (cmp_gt ? max_ax : max_s); st <= S_SCLR3;
            end
            S_SCLR3: begin denom_pri <= (cmp_gt ? po1 : denom_pri); st <= S_SCLR4; end
            S_SCLR4: begin div_a <= max_axs; div_b <= denom_pri; div_start <= 1; st <= S_SCLR5; end
            S_SCLR5: begin if (!div_done) st <= S_SCLR6; else st <= S_SCLR5; end
            S_SCLR6: begin
                if (div_done) begin rel_pri <= div_o; a1 <= NM_C; b1 <= tau_scl; st <= S_SCLR7; end
                else st <= S_SCLR6;
            end
            S_SCLR7: begin cmp_a <= max_aty; cmp_b <= po1; st <= S_SCLR8; end
            S_SCLR8: begin
                denom_dual <= (cmp_gt ? max_aty : po1);
                div_a <= max_px; div_b <= (cmp_gt ? max_aty : po1); div_start <= 1; st <= S_SCLR9;
            end
            S_SCLR9: begin if (!div_done) st <= S_SCLR10; else st <= S_SCLR9; end
            S_SCLR10: begin
                if (div_done) begin rel_dual <= div_o; st <= S_SCLB0; end
                else st <= S_SCLR10;
            end
            // ---- adaptive-scale decision: sum_log += log(rel_pri)-log(rel_dual) ----
            S_SCLB0: begin sa <= log_relp; sb <= log_reld; ssub <= 1; st <= S_SCLB1; end
            S_SCLB1: begin
                sc <= sum_log_r; sd <= so1; ssub2 <= 0;
                n_log_r <= n_log_r + 1; st <= S_SCLB2;      // sum_log + dlog -> so2
            end
            S_SCLB2: begin sum_log_r <= so2; a2 <= n_log_dbl; b2 <= LN10; st <= S_SCLB3; end
            S_SCLB3: begin
                cmp_a <= {1'b0, sum_log_r[62:0]}; cmp_b <= po2; st <= S_SCLB4;   // |sum_log| vs n_log*ln10
            end
            S_SCLB4: begin
                // trigger = |sum_log|>n_log*ln10  AND  iters since last scale >= 100
                if (cmp_gt && ($signed(iter) - last_scale_iter >= 16'd100)) begin
                    a1 <= n_log_dbl; b1 <= 64'h4000000000000000; st <= S_SCLB5;  // 2*n_log -> po1
                end else begin done <= 1; st <= S_IDLE; end
            end
            S_SCLB5: begin div_a <= sum_log_r; div_b <= po1; div_start <= 1; st <= S_SCLB6; end  // sum_log/(2n)
            S_SCLB6: begin if (!div_done) st <= S_SCLB7; else st <= S_SCLB6; end   // ARM
            S_SCLB7: begin
                if (div_done) begin
                    cmp_a <= newscale_mul; cmp_b <= SC_MIN; st <= S_SCLB8;          // clamp low
                end else st <= S_SCLB7;
            end
            S_SCLB8: begin
                new_scale <= (!cmp_gt) ? SC_MIN : newscale_mul;
                cmp_a <= newscale_mul; cmp_b <= SC_MAX; st <= S_SCLB9;              // clamp high
            end
            S_SCLB9: begin
                new_scale <= cmp_gt ? SC_MAX : new_scale;
                last_scale_iter <= iter;
                scale_cur <= new_scale;
                sum_log_r <= 0; n_log_r <= 0;   // reset accumulator (scs_faithful reset)
                st <= S_SX0;            // run the scale-update chain with new scale
            end
            default: st <= S_IDLE;

            // ---- sync-RAM read wait states ----
            S_W1: begin st <= S_SX0; end
            S_W2: begin st <= S_GKSB; end
            S_W3: begin st <= S_GKSVX0; end
            S_W4: begin st <= S_SCL0; end
            S_W5: begin st <= S_SCLT; end
            S_W6: begin st <= S_SCLP0; end

            // ---- sync-RAM read wait states ----
            S_W11: begin st <= S_INIT1; end
            S_W12: begin st <= S_INIT1; end
            S_W13: begin st <= S_VPC0; end
            S_W14: begin st <= S_AA0; end
            S_W15: begin st <= S_NM0; end
            S_W16: begin st <= S_NM1; end
            S_W17: begin st <= S_NM2; end
            S_W18: begin st <= S_NM1; end
            S_W19: begin st <= S_NMV1; end
            S_W20: begin st <= S_NMV2; end
            S_W21: begin st <= S_VPC0; end
            S_W22: begin st <= S_NMV1; end
            S_W23: begin st <= S_AA1; end
            S_W24: begin st <= S_AA2; end
            S_W25: begin st <= S_AA1; end
            S_W26: begin st <= S_NM0; end
            S_W27: begin st <= S_VPC2; end
            S_W28: begin st <= S_KSDONE_ETA; end
            S_W29: begin st <= S_RP1; end
            S_W30: begin st <= S_RP2; end
            S_W31: begin st <= S_RP3; end
            S_W32: begin st <= S_RP4; end
            S_W33: begin st <= S_RP1; end
            S_W34: begin st <= S_GT1; end
            S_W35: begin st <= S_GT2; end
            S_W36: begin st <= S_GT1; end
            S_W37: begin st <= S_U1; end
            S_W38: begin st <= S_U2; end
            S_W39: begin st <= S_U1; end
            S_W40: begin st <= S_UM0; end
            S_W41: begin st <= S_RSK1; end
            S_W42: begin st <= S_RSK2; end
            S_W43: begin st <= S_RSK3; end
            S_W44: begin st <= S_RSK4; end
            S_W45: begin st <= S_RSK1; end
            S_W46: begin st <= S_V1; end
            S_W47: begin st <= S_V2; end
            S_W48: begin st <= S_V3; end
            S_W49: begin st <= S_SG0; end
            S_W50: begin st <= S_V1; end
            S_W51: begin st <= S_SG1; end
            S_W52: begin st <= S_SG2; end
            S_W53: begin st <= S_SG1; end
            S_W54: begin st <= S_SC1; end
            S_W55: begin st <= S_SC2; end
            S_W56: begin st <= S_SC1; end
            S_W57: begin st <= S_SCP1; end
            S_W58: begin st <= S_SCP2; end
            S_W59: begin st <= S_SCP1; end
            S_W60: begin st <= S_GKS0; end
            S_W61: begin st <= S_GKSRY0; end
            S_W62: begin st <= S_GKSDY0; end
            S_W63: begin st <= S_GKSRY0; end
            S_W64: begin st <= S_GKSDY0; end
            S_W65: begin st <= S_GKSB; end
            S_W66: begin st <= S_GKSVY0; end
            S_W67: begin st <= S_GKSVX0; end
            S_W68: begin st <= S_GKSVY1; end
            S_W69: begin st <= S_GKSVY0; end
            S_W70: begin st <= S_VR0; end
            S_W71: begin st <= S_VR1; end
            S_W72: begin st <= S_VR2; end
            S_W73: begin st <= S_VR2B; end
            S_W74: begin st <= S_VR4; end
            S_W75: begin st <= S_VR1; end
            S_W76: begin st <= S_SCLY0; end
            S_W77: begin st <= S_SCLY0; end
            S_W78: begin st <= S_SCLD0; end
            S_W79: begin st <= S_SCLD1; end
            S_W80: begin st <= S_SCLX0; end
            S_W81: begin st <= S_SCLD0; end
            S_W82: begin st <= S_SCLX0; end
            S_W83: begin st <= S_SCLP1; end
            S_W84: begin st <= S_SCLP2; end
            S_W85: begin st <= S_SCLP0; end
            endcase
        end
    end
endmodule
