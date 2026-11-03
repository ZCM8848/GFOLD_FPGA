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
// state (no own_addr assignment) �?the write lands on the advance state's
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
    parameter AVAL_FILE = "../data/kkt/full/Aval.hex",
    parameter BOOT_EXT = 0   // 0: on-chip CFI flash controller; 1: external boot-memory port
    ,parameter SRAM_AW = 18   // external SRAM word-address width (18 = 2MB; 12 = on-chip demo)
    ,parameter SCALE_EN = 1   // 0: omit adaptive-scale datapath (s_build + 2 spmv)
    ,parameter RY_FILE = "../data/kkt/full/r_y_r.hex",
    parameter DY_FILE = "../data/kkt/full/Dy_r.hex"
    ,parameter NODE = 100, Z0 = 14, ZP = 7, NP = 3
    ,parameter NS4 = 2, DIM4 = 4, NS3 = 1, DIM3 = 3
    ,parameter TAIL_NN = 1, TAIL_Z = 6
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
    output wire              idle,             // high in S_IDLE (boot done / iteration complete)
    output reg  [3:0]        verb,            // current FSM phase (AGC-style VERB)
    output wire [63:0]       scale_cur_out,   // adaptive scale (for display)
    output wire [63:0]       tau_out,         // root_plus tau (for display)
    // ---- LDL band ready (back-pressure to the upstream band streamer) ----
    output wire              band_ready,
    // ---- external SRAM (sram64_ctrl; shared by Anderson + LDL via arbiter) ----
    output wire [19:0]       SRAM_ADDR,
    inout  wire [15:0]       SRAM_DQ,
    output wire              SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N,
    // ---- CFI Flash (boot: solver input burned in at power-up) ----
    output wire [22:0]       FL_ADDR,
    inout  wire [7:0]        FL_DQ,
    output wire              FL_CE_N, FL_OE_N, FL_WE_N,
    output wire              FL_RESET_N, FL_WP_N,
    input  wire              FL_RY,
    // ---- external boot memory (used when BOOT_EXT=1) ----
    output wire              boot_req,
    output wire [19:0]       boot_waddr,
    input  wire [63:0]       boot_rdata,
    input  wire              boot_busy,
    input  wire              boot_ready
);
    localparam V_BASE  = 0;
    localparam LMAX = (N > M) ? N : M;             // = M for the G-FOLD problem
    localparam UT_BASE = L;
    localparam U_BASE  = 2 * L;
    localparam RSK_BASE= 3 * L;
    localparam G_BASE  = 4 * L;
    localparam DR_BASE = 4 * L + LM1;
    localparam VPR_BASE= 5 * L + LM1;      // v_prev
    localparam CB_BASE = 6 * L + LM1;      // c[0..n) + (-b)[n..n+m) for g recompute (packed after VPR)
    localparam BAND_BASE = CB_BASE + LM1;          // spmv workspace (2*LMAX words; band moved to SRAM)
    localparam DY_BASE = BAND_BASE + 2 * LMAX;     // D_y for s_build (M words)
    localparam ZMASK_BASE = DY_BASE;  // boot zmask data shares DY area (loaded to reg bits at reset)
    localparam COO_BASE = 0;                       // packed COO in SRAM (word[2k]={Acol,Arow}, word[2k+1]=Aval)
    localparam AA_BASE   = 4*L + 3*L*10;           // LDL array base in SRAM (109072); s_build writes band here
    // ---- Boot image segment bases/lengths (derived from the problem size, so
    // the same RTL works for the full G-FOLD instance and the small on-chip
    // demo). Word addresses; layout: COO | c | nb | zmask | band | g.
    localparam FL_COO_W = 20'd0;                   // COO segment
    localparam FL_C_W   = FL_COO_W + 2*NNZ;        // c segment
    localparam FL_NB_W  = FL_C_W + N;              // nb segment
    localparam FL_ZM_W  = FL_NB_W + M;             // zmask segment
    localparam FL_BAND_W= FL_ZM_W + M;             // band segment
    localparam FL_G_W   = FL_BAND_W + (HB+1)*N;    // g segment
    // spmv workspace for the adaptive-scale residual: reused for A^Ty then Ax.
    // needs LMAX + LMAX words (spmv x@[0,LENX) + out@[LMAX,LMAX+LENO)).
    localparam NM_B = 64'h40A2C00000000000;  // max|b| = 2400.0 (reordered frame)
    localparam NM_C = 64'h3FF0000000000000;  // max|c| = 1.0
    localparam RHO_X = 64'h3EB0C6F7A0B5ED8D;  // 1e-6 (diag_r[:n])
    localparam TAU_FACTOR = 64'h4024000000000000;  // 10.0 (diag_r[n+m])
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
 S_VPC0=81, S_VPC1=82, S_VPC2=83, S_VPC3=84,
 S_SG0=85, S_SG1=86, S_SG2=87, S_SG3=88, S_SG4=89, S_SG5=90,
 S_SG6=91, S_SG7=92, S_SG8=93, S_SG9=94,
 S_SC0=95, S_SC1=96, S_SC2=97, S_SC3=98,
 S_SCP0=99, S_SCP1=100, S_SCP2=101, S_SCP3=102,
 S_SX0=110, S_SX1=111, S_SX2=112, S_SX3=113, S_SX4=114, S_SX5=115, S_SX6=138,
 S_SB=116, S_SBW=117,
 S_GKS0=118, S_GKSB=119, S_GKSVX=120, S_GKSVXW=121, S_GKSVY=122, S_GKSVYW=123,
 S_GKSZ=124, S_GKSD=125,
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
  S_SCLB_LOGW=202, S_SCLB_EXPW=203,
    S_W1=210, S_W2=211, S_W3=212, S_W4=213, S_W5=214, S_W6=215, S_W7=216, S_W8=217,
    S_W9=218, S_W10=219, S_W11=220, S_W12=221, S_W13=222, S_W14=223, S_W15=224, S_W16=225,
    S_W17=226, S_W18=227, S_W19=228, S_W20=229, S_W21=230, S_W22=231, S_W23=232, S_W24=233,
    S_W25=234, S_W26=235, S_W27=236, S_W28=237, S_W29=238, S_W30=239, S_W31=240, S_W32=241,
    S_W33=242, S_W34=243, S_W35=244, S_W36=245, S_W37=246, S_W38=247, S_W39=248, S_W40=249,
    S_W41=250, S_W42=251, S_W43=252, S_W44=253, S_W45=254, S_W46=255,S_W47=257,S_W48=258,S_W49=259,S_W50=260,S_W51=261,S_W52=262,S_W53=263,S_W54=264,S_W55=265,S_W56=266,S_W57=267,S_W58=268,S_W59=269,S_W60=270,S_W61=271,S_W62=272,S_W63=273,S_W64=274,S_W65=275,S_W66=276,S_W67=277,S_W68=278,S_W69=279,S_W70=280,S_W71=281,S_W72=282,S_W73=283,S_W74=284,S_W75=285,S_W76=286,S_W77=287,S_W78=288,S_W79=289,S_W80=290,S_W81=291,S_W82=292,S_W83=293,    S_W84=294, S_W85=295, S_GKSBH=296,
    S_BOOT_WAIT=297, S_BOOT_F=298, S_BOOT_FW=299, S_BOOT_W=300, S_BOOT_WW=301, S_BOOT_V0=302, S_BOOT_DR=303;
    reg [8:0] st;   // 0..261 fits in 9 bits (sync-read waits S_W1..W46)
    assign idle = (st == S_IDLE);
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
    wire ks_sp1_sram_req, ks_sp1_sram_we; wire [17:0] ks_sp1_sram_waddr; wire [63:0] ks_sp1_sram_wdata;
    wire ks_sp1_sram_busy; wire [63:0] ks_sp1_sram_rdata;
    wire ks_sp2_sram_req, ks_sp2_sram_we; wire [17:0] ks_sp2_sram_waddr; wire [63:0] ks_sp2_sram_wdata;
    wire ks_sp2_sram_busy; wire [63:0] ks_sp2_sram_rdata;
    wire ks_band_ready;
    reg band_valid_p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) band_valid_p <= 0;
        else band_valid_p <= band_valid;
    end
    wire band_valid_rise = band_valid && !band_valid_p;   // one sample per tb word
    kkt_solve #(.N(N), .M(M), .NNZ(NNZ), .HB(HB),
                .AA_BASE(AA_BASE), .COO_BASE(COO_BASE),
                .RY_FILE(RY_FILE), .DY_FILE(DY_FILE)) u_ks(
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
        .sp1_sram_req(ks_sp1_sram_req), .sp1_sram_we(ks_sp1_sram_we), .sp1_sram_waddr(ks_sp1_sram_waddr),
        .sp1_sram_wdata(ks_sp1_sram_wdata), .sp1_sram_busy(ks_sp1_sram_busy), .sp1_sram_rdata(ks_sp1_sram_rdata),
        .sp2_sram_req(ks_sp2_sram_req), .sp2_sram_we(ks_sp2_sram_we), .sp2_sram_waddr(ks_sp2_sram_waddr),
        .sp2_sram_wdata(ks_sp2_sram_wdata), .sp2_sram_busy(ks_sp2_sram_busy), .sp2_sram_rdata(ks_sp2_sram_rdata),
        .band_ready(ks_band_ready),
        .mul0_a(ks_mul0_a), .mul0_b(ks_mul0_b), .mul0_o(pm0_o),
        .add0_a(ks_add0_a), .add0_b(ks_add0_b), .add0_sub(ks_add0_sub), .add0_o(pa0_o),
        .div_start(ks_div_start), .div_a(ks_div_a), .div_b(ks_div_b),
        .div_done(pdiv_done), .div_o(pdiv_o));

    // ---- s_build (scale update: rebuild S band from D_y + A COO) ----
    reg  sb_start;
    wire sb_done;
    wire [RAM_AW-1:0] sb_addr;
    wire [63:0] sb_wdata;
    wire sb_we;
    wire sb_sram_req, sb_sram_we; wire [17:0] sb_sram_waddr; wire [63:0] sb_sram_wdata;
    wire sb_sram_busy; wire [63:0] sb_sram_rdata;
    generate if (SCALE_EN) begin : g_sb
    s_build #(.N(N), .M(M), .NNZ(NNZ), .HB(HB), .MAXROW(6), .RAM_AW(RAM_AW),
              .DY_OFFSET(DY_BASE), .COO_BASE(COO_BASE), .BAND_SRAM_BASE(AA_BASE)) u_sb(
        .clk(clk), .rst_n(rst_n), .start(sb_start),
        .ram_addr(sb_addr), .ram_wdata(sb_wdata), .ram_we(sb_we), .ram_rdata(ram_rdata),
        .sram_req(sb_sram_req), .sram_we(sb_sram_we), .sram_waddr(sb_sram_waddr),
        .sram_wdata(sb_sram_wdata), .sram_busy(sb_sram_busy), .sram_rdata(sb_sram_rdata),
        .done(sb_done),
        .mul0_a(sb_mul0_a), .mul0_b(sb_mul0_b), .mul0_o(pm0_o),
        .add0_a(sb_add0_a), .add0_b(sb_add0_b), .add0_sub(sb_add0_sub), .add0_o(pa0_o));
    end else begin : g_sb_off
        assign sb_done = 1'b0;
        assign sb_addr = {RAM_AW{1'b0}};
        assign sb_wdata = 64'h0;
        assign sb_we = 1'b0;
        assign sb_sram_req = 1'b0;
        assign sb_sram_we = 1'b0;
        assign sb_sram_waddr = 18'h0;
        assign sb_sram_wdata = 64'h0;
    end endgenerate

    // ---- spmv for adaptive-scale residual: A^T y (transpose=1) then A x
    //      (transpose=0), both reusing BAND_BASE workspace (band idle outside scale rebuild) (sequential) ----
    reg  saty_start, saty_din_valid; reg [63:0] saty_x_in;
    wire [12:0] saty_addr; wire [63:0] saty_wdata; wire saty_we, saty_done;   // 13-bit: spmv ram_addr width
    wire saty_sram_req, saty_sram_we; wire [17:0] saty_sram_waddr; wire [63:0] saty_sram_wdata;
    wire saty_sram_busy; wire [63:0] saty_sram_rdata;
    generate if (SCALE_EN) begin : g_saty
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(1), .COO_BASE(COO_BASE)) u_saty(
        .clk(clk), .rst_n(rst_n), .start(saty_start), .x_in(saty_x_in), .din_valid(saty_din_valid),
        .ram_addr(saty_addr), .ram_wdata(saty_wdata), .ram_we(saty_we), .ram_rdata(ram_rdata),
        .out_out(), .o_valid(), .done(saty_done),
        .sram_req(saty_sram_req), .sram_we(saty_sram_we), .sram_waddr(saty_sram_waddr),
        .sram_wdata(saty_sram_wdata), .sram_busy(saty_sram_busy), .sram_rdata(saty_sram_rdata),
        .mul0_a(sy_mul0_a), .mul0_b(sy_mul0_b), .mul0_o(pm0_o),
        .add0_a(sy_add0_a), .add0_b(sy_add0_b), .add0_sub(sy_add0_sub), .add0_o(pa0_o));
    end else begin : g_saty_off
        assign saty_done = 1'b0;
        assign saty_addr = 13'h0;
        assign saty_wdata = 64'h0;
        assign saty_we = 1'b0;
        assign saty_sram_req = 1'b0;
        assign saty_sram_we = 1'b0;
        assign saty_sram_waddr = 18'h0;
        assign saty_sram_wdata = 64'h0;
    end endgenerate
    reg  sax_start, sax_din_valid; reg [63:0] sax_x_in;
    wire [12:0] sax_addr; wire [63:0] sax_wdata; wire sax_we, sax_done;   // 13-bit: spmv ram_addr width
    wire sax_sram_req, sax_sram_we; wire [17:0] sax_sram_waddr; wire [63:0] sax_sram_wdata;
    wire sax_sram_busy; wire [63:0] sax_sram_rdata;
    generate if (SCALE_EN) begin : g_sax
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(0), .COO_BASE(COO_BASE)) u_sax(
        .clk(clk), .rst_n(rst_n), .start(sax_start), .x_in(sax_x_in), .din_valid(sax_din_valid),
        .ram_addr(sax_addr), .ram_wdata(sax_wdata), .ram_we(sax_we), .ram_rdata(ram_rdata),
        .out_out(), .o_valid(), .done(sax_done),
        .sram_req(sax_sram_req), .sram_we(sax_sram_we), .sram_waddr(sax_sram_waddr),
        .sram_wdata(sax_sram_wdata), .sram_busy(sax_sram_busy), .sram_rdata(sax_sram_rdata),
        .mul0_a(sx_mul0_a), .mul0_b(sx_mul0_b), .mul0_o(pm0_o),
        .add0_a(sx_add0_a), .add0_b(sx_add0_b), .add0_sub(sx_add0_sub), .add0_o(pa0_o));
    end else begin : g_sax_off
        assign sax_done = 1'b0;
        assign sax_addr = 13'h0;
        assign sax_wdata = 64'h0;
        assign sax_we = 1'b0;
        assign sax_sram_req = 1'b0;
        assign sax_sram_we = 1'b0;
        assign sax_sram_waddr = 18'h0;
        assign sax_sram_wdata = 64'h0;
    end endgenerate

    reg  rp_start, rp_dv;
    reg  [63:0] rp_r, rp_g, rp_p, rp_mu, rp_eta;
    wire rp_done;
    wire [63:0] rp_tau;
    root_plus #(.L(LM1)) u_rp(.clk(clk), .rst_n(rst_n), .start(rp_start),
        .r_in(rp_r), .g_in(rp_g), .p_in(rp_p), .mu_in(rp_mu), .din_valid(rp_dv),
        .eta(rp_eta), .done(rp_done), .tau(rp_tau),
        .mul0_a(rp_m0a), .mul0_b(rp_m0b), .mul1_a(rp_m1a), .mul1_b(rp_m1b),
        .mul2_a(rp_m2a), .mul2_b(rp_m2b),
        .mul0_o(pm0_o), .mul1_o(pm1_o), .mul2_o(pm2_o),
        .add0_a(rp_a0a), .add0_b(rp_a0b), .add0_sub(rp_a0s),
        .add1_a(rp_a1a), .add1_b(rp_a1b), .add1_sub(rp_a1s),
        .add2_a(rp_a2a), .add2_b(rp_a2b), .add2_sub(rp_a2s),


        .add0_o(pa0_o), .add1_o(pa1_o), .add2_o(pa2_o),
        .div_start(rp_div_start), .div_a(rp_div_a), .div_b(rp_div_b),
        .div_done(pdiv_done), .div_o(pdiv_o),
        .rr_start(rp_rr_start), .rr_in(rp_rr_in),
        .rr_done(srs_done), .rr_o(srs_o));

    reg  pdc_start;
    wire [11:0] pdc_addr;
    wire [63:0] pdc_wdata;
    wire pdc_we;
    wire [63:0] pdc_rdata;
    wire pdc_done;
    proj_dual_cone_rb #(.M(M), .NODE(NODE), .Z0(Z0), .ZP(ZP), .NP(NP), .NS4(NS4), .DIM4(DIM4), .NS3(NS3), .DIM3(DIM3), .TAIL_NN(TAIL_NN), .TAIL_Z(TAIL_Z)) u_pdc(.clk(clk), .rst_n(rst_n), .start(pdc_start),
        .addr(pdc_addr), .wdata(pdc_wdata), .we(pdc_we), .rdata(pdc_rdata),
        .done(pdc_done),
        .add0_a(pdc_a0a), .add0_b(pdc_a0b), .add0_sub(pdc_a0s), .add0_o(pa0_o),
        .mul0_a(pdc_m0a), .mul0_b(pdc_m0b), .mul1_a(pdc_m1a), .mul1_b(pdc_m1b),
        .mul2_a(pdc_m2a), .mul2_b(pdc_m2b),
        .mul0_o(pm0_o), .mul1_o(pm1_o), .mul2_o(pm2_o),
        .rr_start(pdc_rr_start), .rr_in(pdc_rr_in),
        .rr_done(srs_done), .rr_o(srs_o));

    // ---- boot copy FSM regs (declared before the arbiter that uses them) ----
    reg [2:0] boot_seg;        // 0=COO,1=c,2=nb,3=zmask
    reg       boot_sram_req;   // boot write to SRAM (COO)
    reg [17:0] boot_sram_waddr;
    reg [63:0] boot_sram_wdata;
    reg       booting;         // 1 while the boot g-recompute runs (alters S_GKSD exit)
    reg       flash_req; reg [19:0] flash_waddr;
    // ---- external SRAM: 7-user arbiter (boot + 6 runtime, all serial -> priority mux) ----
    wire sram_req, sram_we; wire [17:0] sram_waddr; wire [63:0] sram_wdata;
    wire sram_busy; wire [63:0] sram_rdata;
    wire g_boot= boot_sram_req;
    wire g_ldl = ks_sram_req     && !g_boot;
    wire g_sp1 = ks_sp1_sram_req && !g_boot && !g_ldl;
    wire g_sp2 = ks_sp2_sram_req && !g_boot && !g_ldl && !g_sp1;
    wire g_sb  = sb_sram_req     && !g_boot && !g_ldl && !g_sp1 && !g_sp2;
    wire g_saty= saty_sram_req   && !g_boot && !g_ldl && !g_sp1 && !g_sp2 && !g_sb;
    wire g_sax = sax_sram_req    && !g_boot && !g_ldl && !g_sp1 && !g_sp2 && !g_sb && !g_saty;
    assign sram_req = g_boot | g_ldl | g_sp1 | g_sp2 | g_sb | g_saty | g_sax;
    reg sram_we_m; reg [17:0] sram_waddr_m; reg [63:0] sram_wdata_m;
    always @(*) begin
        if (g_boot) begin sram_we_m = 1'b1; sram_waddr_m = boot_sram_waddr; sram_wdata_m = boot_sram_wdata; end
        else if (g_ldl) begin sram_we_m = ks_sram_we; sram_waddr_m = ks_sram_waddr; sram_wdata_m = ks_sram_wdata; end
        else if (g_sp1) begin sram_we_m = ks_sp1_sram_we; sram_waddr_m = ks_sp1_sram_waddr; sram_wdata_m = ks_sp1_sram_wdata; end
        else if (g_sp2) begin sram_we_m = ks_sp2_sram_we; sram_waddr_m = ks_sp2_sram_waddr; sram_wdata_m = ks_sp2_sram_wdata; end
        else if (g_sb) begin sram_we_m = sb_sram_we; sram_waddr_m = sb_sram_waddr; sram_wdata_m = sb_sram_wdata; end
        else if (g_saty) begin sram_we_m = saty_sram_we; sram_waddr_m = saty_sram_waddr; sram_wdata_m = saty_sram_wdata; end
        else begin sram_we_m = sax_sram_we; sram_waddr_m = sax_sram_waddr; sram_wdata_m = sax_sram_wdata; end
    end
    assign sram_we = sram_we_m;
    assign sram_waddr = sram_waddr_m;
    assign sram_wdata = sram_wdata_m;
    // busy/rdata broadcast: only the active user issues req and reads rdata
    assign ks_sram_busy     = sram_busy; assign ks_sram_rdata     = sram_rdata;
    assign ks_sp1_sram_busy = sram_busy; assign ks_sp1_sram_rdata = sram_rdata;
    assign ks_sp2_sram_busy = sram_busy; assign ks_sp2_sram_rdata = sram_rdata;
    assign sb_sram_busy     = sram_busy; assign sb_sram_rdata     = sram_rdata;
    assign saty_sram_busy   = sram_busy; assign saty_sram_rdata   = sram_rdata;
    assign sax_sram_busy    = sram_busy; assign sax_sram_rdata    = sram_rdata;
    assign band_ready = (st == S_KSBAND) && ks_band_ready;
    sram64_ctrl #(.AW(SRAM_AW)) u_sram(.clk(clk), .rst_n(rst_n), .req(sram_req), .we(sram_we),
        .waddr(sram_waddr), .wdata(sram_wdata), .busy(sram_busy), .rdata(sram_rdata),
        .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ), .SRAM_CE_N(SRAM_CE_N),
        .SRAM_OE_N(SRAM_OE_N), .SRAM_WE_N(SRAM_WE_N), .SRAM_UB_N(SRAM_UB_N),
        .SRAM_LB_N(SRAM_LB_N));
    // ---- Boot memory: CFI Flash controller (BOOT_EXT=0) or external
    // boot-memory port (BOOT_EXT=1, e.g. on-chip image RAM for the PH1P35
    // small-problem demo). Both present the same req/busy/ready/rdata timing.
    wire flash_busy, flash_ready; wire [63:0] flash_rdata;
    generate if (BOOT_EXT) begin : g_ext_boot
        assign boot_req   = flash_req;
        assign boot_waddr = flash_waddr;
        assign flash_rdata = boot_rdata;
        assign flash_busy  = boot_busy;
        assign flash_ready = boot_ready;
    end else begin : g_flash_boot
        flash_ctrl #(.AW(20)) u_flash(.clk(clk), .rst_n(rst_n), .req(flash_req), .waddr(flash_waddr),
            .busy(flash_busy), .ready(flash_ready), .rdata(flash_rdata),
            .FL_ADDR(FL_ADDR), .FL_DQ(FL_DQ), .FL_CE_N(FL_CE_N), .FL_OE_N(FL_OE_N),
            .FL_WE_N(FL_WE_N), .FL_RESET_N(FL_RESET_N), .FL_WP_N(FL_WP_N), .FL_RY(FL_RY));
        assign boot_req = 1'b0;
        assign boot_waddr = 20'd0;
    end endgenerate
    // busy falling-edge detects for the boot copy loop
    reg flash_busy_p, sram_busy_p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin flash_busy_p <= 0; sram_busy_p <= 0; end
        else begin flash_busy_p <= flash_busy; sram_busy_p <= sram_busy; end
    end
    wire [19:0] boot_fl_base = (boot_seg == 0) ? FL_COO_W : (boot_seg == 1) ? FL_C_W :
                               (boot_seg == 2) ? FL_NB_W : (boot_seg == 3) ? FL_ZM_W :
                               (boot_seg == 4) ? FL_BAND_W : FL_G_W;
    wire [15:0] boot_fl_len  = (boot_seg == 0) ? (2*NNZ) : (boot_seg == 1) ? N :
                               (boot_seg == 2) ? M : (boot_seg == 3) ? M :
                               (boot_seg == 4) ? ((HB+1)*N) : (L-1);
    // FP64 comparison (a > b), combinational, no NaN/denormal handling needed here
    reg  [63:0] cmp_a, cmp_b;
    wire cmp_gt = (cmp_a[63] != cmp_b[63]) ? (~cmp_a[63]) :
                  (cmp_a[63] == 0) ? ((cmp_a[62:0] > cmp_b[62:0]) ? 1'b1 : 1'b0) :
                                     ((cmp_a[62:0] < cmp_b[62:0]) ? 1'b1 : 1'b0);

    // ---- own FP64 combinational results (driven by the global pool) ----
    reg [63:0] a1, b1;
    reg [63:0] a2, b2;
    reg [63:0] sa, sb;
    reg ssub;
    reg [63:0] sc, sd;
    reg ssub2;
    wire [63:0] po1, po2, so1, so2;   // assigned to pool outputs below
    reg  div_start;
    reg  [63:0] div_a, div_b;
    wire div_done;
    wire [63:0] div_o;
    reg  rs_start;
    reg  [63:0] rs_in;
    wire rs_done;
    wire [63:0] rs_o;
    reg saty_active, sax_active;   // set by the scale-residual FSM while a spmv drives RAM

    // ================= global FP64 pool (3 mul + 3 add + 1 div) =================
    reg  [63:0] pm0_a, pm0_b, pm1_a, pm1_b, pm2_a, pm2_b;
    wire [63:0] pm0_o, pm1_o, pm2_o;
    reg  [63:0] pa0_a, pa0_b, pa1_a, pa1_b, pa2_a, pa2_b;
    reg         pa0_sub, pa1_sub, pa2_sub;
    wire [63:0] pa0_o, pa1_o, pa2_o;
    reg         pdiv_start;
    reg  [63:0] pdiv_a, pdiv_b;
    wire        pdiv_done;
    wire [63:0] pdiv_o;

    fp64_pool upool(.clk(clk), .rst_n(rst_n),
        .mul0_a(pm0_a), .mul0_b(pm0_b), .mul1_a(pm1_a), .mul1_b(pm1_b),
        .mul2_a(pm2_a), .mul2_b(pm2_b),
        .mul0_o(pm0_o), .mul1_o(pm1_o), .mul2_o(pm2_o),
        .add0_a(pa0_a), .add0_b(pa0_b), .add0_sub(pa0_sub),
        .add1_a(pa1_a), .add1_b(pa1_b), .add1_sub(pa1_sub),
        .add2_a(pa2_a), .add2_b(pa2_b), .add2_sub(pa2_sub),
        .add0_o(pa0_o), .add1_o(pa1_o), .add2_o(pa2_o),
        .div_start(pdiv_start), .div_a(pdiv_a), .div_b(pdiv_b),
        .div_done(pdiv_done), .div_o(pdiv_o));

    // single shared rsqrt (root_plus / proj_soc / own normalize)
    wire        srs_done;  wire [63:0] srs_o;
    reg         srs_start; reg  [63:0] srs_in;
    fp64_rsqrt usrs(.clk(clk), .start(srs_start), .x(srs_in),
                    .done(srs_done), .o(srs_o));

    // pool-port wires from the submodules
    wire [63:0] ks_mul0_a, ks_mul0_b, ks_add0_a, ks_add0_b;
    wire        ks_add0_sub, ks_div_start;
    wire [63:0] ks_div_a, ks_div_b;
    wire [63:0] sb_mul0_a, sb_mul0_b, sb_add0_a, sb_add0_b;
    wire        sb_add0_sub;
    wire [63:0] sy_mul0_a, sy_mul0_b, sy_add0_a, sy_add0_b;
    wire        sy_add0_sub;
    wire [63:0] sx_mul0_a, sx_mul0_b, sx_add0_a, sx_add0_b;
    wire        sx_add0_sub;
    wire [63:0] rp_m0a,rp_m0b,rp_m1a,rp_m1b,rp_m2a,rp_m2b;
    wire [63:0] rp_a0a,rp_a0b,rp_a1a,rp_a1b,rp_a2a,rp_a2b;
    wire rp_a0s,rp_a1s,rp_a2s;
    wire rp_div_start; wire [63:0] rp_div_a, rp_div_b;
    wire rp_rr_start; wire [63:0] rp_rr_in;
    wire [63:0] pdc_m0a,pdc_m0b,pdc_m1a,pdc_m1b,pdc_m2a,pdc_m2b;
    wire [63:0] pdc_a0a,pdc_a0b; wire pdc_a0s;
    wire pdc_rr_start; wire [63:0] pdc_rr_in;

    // Sticky "module owns the pool" flags: the streamer spends most of its
    // time in shared S_W* wait states, so a plain state-range window would
    // disconnect the pool from the active module. Track start..done instead.
    reg ks_busy, rp_busy, pdc_busy, sb_busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ks_busy<=0; rp_busy<=0; pdc_busy<=0; sb_busy<=0; end
        else begin
            if (ks_start)  ks_busy <= 1;  else if (ks_done)  ks_busy <= 0;
            if (rp_start)  rp_busy <= 1;  else if (rp_done)  rp_busy <= 0;
            if (pdc_start) pdc_busy <= 1; else if (pdc_done) pdc_busy <= 0;
            if (sb_start)  sb_busy <= 1;  else if (sb_done)  sb_busy <= 0;
        end
    end
    wire ks_active  = ks_busy;
    wire rp_active  = rp_busy;
    wire pdc_active = pdc_busy;
    wire sb_active  = sb_busy;

    always @(*) begin
        // default: drs_iter's own arithmetic
        pm0_a=a1; pm0_b=b1; pm1_a=a2; pm1_b=b2; pm2_a=64'h0; pm2_b=64'h0;

        pa0_a=sa; pa0_b=sb; pa0_sub=ssub;
        pa1_a=sc; pa1_b=sd; pa1_sub=ssub2;
        pa2_a=64'h0; pa2_b=64'h0; pa2_sub=1'b0;


        pdiv_start=div_start; pdiv_a=div_a; pdiv_b=div_b;
        srs_start=rs_start; srs_in=rs_in;

        if (pdc_active) begin
            pm0_a=pdc_m0a; pm0_b=pdc_m0b; pm1_a=pdc_m1a; pm1_b=pdc_m1b;
            pm2_a=pdc_m2a; pm2_b=pdc_m2b;
            pa0_a=pdc_a0a; pa0_b=pdc_a0b; pa0_sub=pdc_a0s;
            srs_start=pdc_rr_start; srs_in=pdc_rr_in;
        end else if (rp_active) begin
            pm0_a=rp_m0a; pm0_b=rp_m0b; pm1_a=rp_m1a; pm1_b=rp_m1b;
            pm2_a=rp_m2a; pm2_b=rp_m2b;

            pa0_a=rp_a0a; pa0_b=rp_a0b; pa0_sub=rp_a0s;
            pa1_a=rp_a1a; pa1_b=rp_a1b; pa1_sub=rp_a1s;
            pa2_a=rp_a2a; pa2_b=rp_a2b; pa2_sub=rp_a2s;


            pdiv_start=rp_div_start; pdiv_a=rp_div_a; pdiv_b=rp_div_b;
            srs_start=rp_rr_start; srs_in=rp_rr_in;
        end else if (ks_active) begin
            pm0_a=ks_mul0_a; pm0_b=ks_mul0_b;
            pa0_a=ks_add0_a; pa0_b=ks_add0_b; pa0_sub=ks_add0_sub;
            pdiv_start=ks_div_start; pdiv_a=ks_div_a; pdiv_b=ks_div_b;
        end else if (sb_active) begin
            pm0_a=sb_mul0_a; pm0_b=sb_mul0_b;
            pa0_a=sb_add0_a; pa0_b=sb_add0_b; pa0_sub=sb_add0_sub;
        end else if (saty_active) begin
            pm0_a=sy_mul0_a; pm0_b=sy_mul0_b;
            pa0_a=sy_add0_a; pa0_b=sy_add0_b; pa0_sub=sy_add0_sub;
        end else if (sax_active) begin
            pm0_a=sx_mul0_a; pm0_b=sx_mul0_b;
            pa0_a=sx_add0_a; pa0_b=sx_add0_b; pa0_sub=sx_add0_sub;
        end
    end

    assign po1 = pm0_o;  assign po2 = pm1_o;
    assign so1 = pa0_o;  assign so2 = pa1_o;
    assign div_done = pdiv_done;  assign div_o = pdiv_o;
    assign rs_done  = srs_done;   assign rs_o  = srs_o;

    // ---- RAM port mux: own | pdc (base U_BASE+N) | sb | saty | sax ----
    reg [RAM_AW-1:0] own_addr;
    reg [63:0] own_wdata;
    reg own_we;

    // ---- FSM ---- (state defs at module top; see S_IDLE etc above)
    reg [15:0] i;
    reg [63:0] vv, utv, gv, drv, tau_r, norm_r, sqacc, v_eta;
    reg [M-1:0] zmask_bits;
    reg sflag;
 reg rs_done_p;
 reg [63:0] rinv, zm, vr, vd, vu, vv2;
 reg scale_valid_p;   // prev scale_valid, for rising-edge detection (level-safe trigger)
 reg scale_flow;      // 1 while the scale-update chain runs (suppress residual check at its S_DONE)
 // ---- adaptive-scale residual / decision regs ----
 reg [63:0] max_aty, max_px, max_ax, max_s, max_axs;
 reg [63:0] denom_pri, denom_dual, rel_pri, rel_dual;
 reg [63:0] sum_log_r, scale_cur, tau_scl, new_scale;
 reg signed [15:0] n_log_r, last_scale_iter;
 reg [4:0] mod25_cnt;                  // iter mod 25 (avoids a 16-bit comb % divider)
 reg [63:0] aty_i, c_i, ax_i, s_i, nb_i;

 // ---- display outputs (AGC-style VERB phase + adaptive scale + tau) ----
 assign scale_cur_out = scale_cur;
 assign tau_out = rp_tau;
 always @(*) begin
     if (st == S_IDLE)                       verb = 4'd0;
     else if (st >= S_BOOT_WAIT)             verb = 4'd1;   // 297-303 BOOT
     else if (st == S_INIT0 || st == S_INIT1 || st == S_INIT2) verb = 4'd1;  // 146-148 BOOT
     else if (st == S_GKSBH)                 verb = 4'd3;   // 296 KKT band handshake
     else if (st >= S_SCL0 && st <= S_SCLB_EXPW) verb = 4'd8;  // 150-203 SCALE
     else if (st >= S_GKSVX0 && st <= S_GKSDY0) verb = 4'd3;   // 139-145 KKT
     else if (st == S_SX6)                   verb = 4'd8;      // 138 SCALE
     else if (st >= S_VR0 && st <= S_VR8)    verb = 4'd8;      // 128-137 SCALE
     else if (st >= S_GKS0 && st <= S_GKSD)  verb = 4'd3;      // 118-125 KKT
     else if (st >= S_SX0 && st <= S_SBW)    verb = 4'd8;      // 110-117 SCALE
     else if (st >= S_SG0 && st <= S_SCP3)   verb = 4'd8;      // 85-102 SCALE
     else if (st >= S_VPC0 && st <= S_VPC3)  verb = 4'd2;      // 81-84 NORM
     else if (st == S_NM4)                   verb = 4'd2;      // 67 NORM
     else if (st == S_RP5)                   verb = 4'd4;      // 68 ROOT
     else if (st == S_DONE)                  verb = 4'd9;      // 66 DONE
     else if (st >= S_V0 && st <= S_V7)      verb = 4'd5;      // 58-65 UPDT
     else if (st >= S_RSK0 && st <= S_RSK9)  verb = 4'd7;      // 48-57 RESID
     else if (st >= S_UM0 && st <= S_UMW)    verb = 4'd5;      // 45-47 UPDT
     else if (st == S_CONE || st == S_CONEW) verb = 4'd6;      // 43-44 CONE
     else if (st >= S_UTT2 && st <= S_U5)    verb = 4'd5;      // 29-42 UPDT
     else if (st >= S_RP0 && st <= S_RPW)    verb = 4'd4;      // 23-28 ROOT
     else if (st == S_KSDONE_ETA)            verb = 4'd3;      // 22 KKT
     else if (st == S_UTT)                   verb = 4'd5;      // 21 UPDT
     else if (st >= S_KSSTART && st <= S_KSDONE) verb = 4'd3;  // 13-20 KKT
     else if (st >= S_NM0 && st <= S_NMV4)   verb = 4'd2;      // 1-12 NORM
     else                                    verb = 4'd0;      // wait states -> IDLE
 end
 reg [63:0] saty_pf, sax_pf;
    // Milestone 5: adaptive scaling is not exercised in 2 iterations, so the
    // exp/log datapaths are not instantiated (they would add 9 multipliers).
    // Tie their results off; the scale FSM consequently stays inert.
    wire [63:0] log_relp = 64'h0, log_reld = 64'h0, exp_out = 64'h0;
    wire [63:0] newscale_mul = 64'h0;
    reg  logp_start, logd_start, exp_start;
    wire logp_done = 1'b0, logd_done = 1'b0, exp_done = 1'b0;
    wire [63:0] n_log_dbl;
    i2d ui2d(n_log_r[10:0], n_log_dbl);     // n_log -> double
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
            st <= S_BOOT_WAIT; done <= 0;
            boot_seg <= 0; boot_sram_req <= 0; boot_sram_waddr <= 0; boot_sram_wdata <= 0;
            booting <= 0; flash_req <= 0; flash_waddr <= 0;
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
            sflag <= 0;
            cmp_a <= 0; cmp_b <= 0;
            rs_done_p <= 0;
            rinv <= 0; zm <= 0; vr <= 0; vd <= 0; vu <= 0; vv2 <= 0;
            sb_start <= 0; gks_active <= 0; scale_valid_p <= 0; scale_flow <= 0;
            saty_start <= 0; saty_din_valid <= 0; saty_x_in <= 0; saty_active <= 0;
            sax_start <= 0; sax_din_valid <= 0; sax_x_in <= 0; sax_active <= 0;
            max_aty <= 0; max_px <= 0; max_ax <= 0; max_s <= 0; max_axs <= 0;
            denom_pri <= 0; denom_dual <= 0; rel_pri <= 0; rel_dual <= 0;
            sum_log_r <= 0; scale_cur <= 64'h3FF0000000000000; tau_scl <= 0; new_scale <= 0;  // scale_cur init 1.0
            n_log_r <= 0; last_scale_iter <= -16'd100; mod25_cnt <= 0; aty_i <= 0; c_i <= 0; ax_i <= 0; s_i <= 0; nb_i <= 0;
            logp_start <= 0; logd_start <= 0; exp_start <= 0;
            zmask_bits <= 0;
        end else begin
            ks_start <= 0; ks_band_valid <= 0; ks_din_valid <= 0;
            ks_ry_valid <= 0; ks_dy_valid <= 0; ks_par_update <= 0;
            rp_start <= 0; rp_dv <= 0; pdc_start <= 0; rs_start <= 0;
            div_start <= 0; sb_start <= 0;
            saty_start <= 0; saty_din_valid <= 0;
            sax_start <= 0; sax_din_valid <= 0;
            logp_start <= 0; logd_start <= 0; exp_start <= 0;
            flash_req <= 0; boot_sram_req <= 0;
            if (ks_band_valid && !ks_band_ready) ks_band_valid <= 0;   // level-held: clear after LDL sampled
            own_we <= 0;
            rs_done_p <= rs_done;   // track every cycle: shared rsqrt keeps done high between calls
            scale_valid_p <= scale_valid;
            case (st)
            // ============ boot: Flash -> SRAM/smem, then compute diag_r/D_y/band/g ============
            S_BOOT_WAIT: begin
                if (flash_ready) begin boot_seg <= 0; i <= 0; st <= S_BOOT_F; end
            end
            S_BOOT_F: begin
                flash_waddr <= boot_fl_base + i; flash_req <= 1; st <= S_BOOT_FW;
            end
            S_BOOT_FW: begin
                if (flash_busy_p && !flash_busy) begin   // flash word ready in flash_rdata
                    case (boot_seg)
                    0: begin boot_sram_req <= 1; boot_sram_waddr <= COO_BASE + i; boot_sram_wdata <= flash_rdata; end
                    1: begin own_addr <= CB_BASE + i; own_wdata <= flash_rdata; own_we <= 1; end
                    2: begin own_addr <= CB_BASE + N + i; own_wdata <= flash_rdata; own_we <= 1; end
                    3: begin own_addr <= ZMASK_BASE + i; own_wdata <= flash_rdata; own_we <= 1; end
                    4: begin boot_sram_req <= 1; boot_sram_waddr <= AA_BASE + i; boot_sram_wdata <= flash_rdata; end
                    default: begin own_addr <= G_BASE + i; own_wdata <= flash_rdata; own_we <= 1; end
                    endcase
                    st <= S_BOOT_W;
                end
            end
            S_BOOT_W: begin
                if (boot_seg == 0 || boot_seg == 4) begin
                    if (sram_busy_p && !sram_busy) st <= S_BOOT_WW;   // sram write done
                end else st <= S_BOOT_WW;                             // ram write (1 cycle)
            end
            S_BOOT_WW: begin
                if (i + 1 >= boot_fl_len) begin
                    if (boot_seg == 5) begin i <= 0; st <= S_BOOT_V0; end
                    else begin boot_seg <= boot_seg + 1; i <= 0; st <= S_BOOT_F; end
                end else begin i <= i + 1; st <= S_BOOT_F; end
            end
            // ---- v0 = [0..0, 1.0] (homogeneous self-dual embedding initial point) ----
            S_BOOT_V0: begin
                own_addr <= i; own_wdata <= (i == LM1) ? 64'h3FF0000000000000 : 64'h0; own_we <= 1;
                if (i + 1 >= L) begin i <= 0; st <= S_BOOT_DR; end
                else begin i <= i + 1; st <= S_BOOT_V0; end
            end
            // ---- diag_r[:n] = rho_x, diag_r[n+m] = tau_factor (r_y is filled by S_SX) ----
            S_BOOT_DR: begin
                if (i < N) begin own_addr <= DR_BASE + i; own_wdata <= RHO_X; own_we <= 1; end
                else begin own_addr <= DR_BASE + N + M; own_wdata <= TAU_FACTOR; own_we <= 1; end
                if (i + 1 >= N + 1) begin booting <= 1; st <= S_INIT0; end
                else begin i <= i + 1; st <= S_BOOT_DR; end
            end
            // ---- boot: load zmask bits from DY area (just written from Flash) ----
            S_INIT0: begin i <= 0; own_addr <= ZMASK_BASE; st <= S_W11; end
            S_INIT1: begin zm <= ram_rdata; st <= S_INIT2; end
            S_INIT2: begin
                zmask_bits[i] <= (zm != 64'h0);
                if (i + 1 >= M) st <= (booting ? S_SX0 : S_IDLE);
                else begin i <= i + 1; own_addr <= ZMASK_BASE + i + 1; st <= S_W12; end
            end
            S_IDLE: begin
                done <= 0;
                scale_flow <= 0;
                booting <= 0;
                if (scale_valid && !scale_valid_p) begin scale_cur <= scale_r; i <= 0; own_addr <= ZMASK_BASE; st <= S_W1; end
                else if (start) begin
                    if (iter == 0) begin i <= 0; own_addr <= V_BASE; st <= S_W13; end  // FEAS: v_prev=v0, no normalize
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
            // sqacc must sample so1 AFTER sa/sb updated (S_NM3) �?same-cycle
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
                // rs_done RISING-EDGE only (tracked every cycle above).
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
            // ============ v_prev <- v (after normalize) ============
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
                st <= S_KSVX;   // band pre-loaded into LDL SRAM (refactor) or reused
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
            // the CURRENT cycle's address �?correct for a continuous stream)
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
                if (i + 1 >= L) st <= S_DONE;
                else begin i <= i + 1; own_addr <= U_BASE + i + 1; st <= S_W50; end
            end
            // ============ scale update: recompute diag_r[n:n+m] + D_y, then
            // rebuild band (s_build), g = KKT^-1[c;-b], v remap ============
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
                // write D_y �?must be its OWN state: the else-branch ZMASK forward
                // address would override DY_BASE+i in the same cycle (double assign).
                own_addr <= DY_BASE + i; own_wdata <= po2; own_we <= 1; st <= S_SX6;
            end
            S_SX6: begin
                if (i + 1 >= M) begin
                    if (booting) st <= S_IDLE;   // boot: diag_r/D_y done; band pre-loaded from Flash
                    else begin sb_start <= 1; st <= S_SB; end
                end
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
                if (i + 1 >= M) begin i <= 0; own_addr <= CB_BASE; st <= S_W3; end  // band pre-loaded; skip S_GKSB
                else begin i <= i + 1; own_addr <= DY_BASE + i + 1; st <= S_W64; end
            end
            S_GKSB: begin
                // phase 1: wait for LDL ready (in S_BAND), then present word_i
                if (ks_band_ready) begin
                    ks_band_in <= ram_rdata; ks_band_valid <= 1;
                    st <= S_GKSBH;
                end
                // else: LDL busy -> hold without presenting (valid stays low)
            end
            // phase 2: word_i presented; hold valid until LDL consumes it (ready falls)
            S_GKSBH: begin
                ks_band_valid <= 1;
                if (!ks_band_ready) begin
                    ks_band_valid <= 0;
                    if (i + 1 >= (HB + 1) * N) begin i <= 0; own_addr <= CB_BASE; st <= S_W3; end
                    else begin i <= i + 1; own_addr <= BAND_BASE + i + 1; st <= S_W65; end
                end
                // else: LDL still in S_BAND -> keep holding valid
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
                if (ks_done) begin gks_active <= 0; i <= 0; own_addr <= RSK_BASE; st <= S_W70; end
            end
            // ---- v remap: v = rsk/diag_r + 2*u_t - u (all l-1 elements) ----
            S_VR0: begin i <= 0; own_addr <= RSK_BASE; st <= S_W71; end
            S_VR1: begin vr <= ram_rdata; own_addr <= DR_BASE + i; st <= S_W72; end
            S_VR2: begin
                vd <= ram_rdata; div_a <= vr;
                own_addr <= UT_BASE + i; st <= S_W73;
            end
            S_VR2B: begin
                // div_b must use the fresh vd (vd updates at S_VR2) �?one cycle later
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
                if (mod25_cnt == 5'd0 && iter > 0 && !scale_flow) begin
                    // run the residual+scale chain THIS iteration before done,
                    // otherwise it bleeds into the next iter and skips its DRS step.
                    done <= 0;
                    i <= 0; own_addr <= U_BASE + N; st <= S_W4;
                end else begin
                    mod25_cnt <= (mod25_cnt == 5'd24) ? 5'd0 : mod25_cnt + 5'd1;
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
                if (div_done) begin rel_dual <= div_o; logp_start <= 1; logd_start <= 1; st <= S_SCLB_LOGW; end
                else st <= S_SCLR10;
            end
            S_SCLB_LOGW: begin
                if (logp_done && logd_done) st <= S_SCLB0;
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
                if (div_done) begin exp_start <= 1; st <= S_SCLB_EXPW; end
                else st <= S_SCLB7;
            end
            S_SCLB_EXPW: begin
                if (exp_done) begin
                    cmp_a <= newscale_mul; cmp_b <= SC_MIN; st <= S_SCLB8;          // clamp low
                end
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
            S_W15: begin st <= S_NM0; end
            S_W16: begin st <= S_NM1; end
            S_W17: begin st <= S_NM2; end
            S_W18: begin st <= S_NM1; end
            S_W19: begin st <= S_NMV1; end
            S_W20: begin st <= S_NMV2; end
            S_W21: begin st <= S_VPC0; end
            S_W22: begin st <= S_NMV1; end
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
            S_W50: begin st <= S_V1; end
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
