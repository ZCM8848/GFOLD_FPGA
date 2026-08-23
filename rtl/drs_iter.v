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
    parameter RAM_AW = 15,
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
    output reg               done
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
    localparam AA_SAFEGUARD = 64'h3FF0000000000000;  // 1.0 (scs_faithful AA_SAFEGUARD)

    // ---- sub-blocks ----
    reg  ks_start, ks_band_valid, ks_din_valid;
    reg  [63:0] ks_band_in, ks_x_in;
    wire [63:0] ks_z_out;
    wire        ks_o_valid, ks_done;
    kkt_solve #(.N(N), .M(M), .NNZ(NNZ), .HB(HB),
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE),
                .RY_FILE("../data/kkt/full/r_y_r.hex"), .DY_FILE("../data/kkt/full/Dy_r.hex")) u_ks(
        .clk(clk), .rst_n(rst_n), .start(ks_start),
        .band_in(ks_band_in), .band_valid(ks_band_valid), .refactor(refactor),
        .x_in(ks_x_in), .din_valid(ks_din_valid),
        .ram_addr(kkt_addr), .ram_wdata(kkt_wdata), .ram_we(kkt_we),
        .ram_rdata(kkt_rdata),
        .z_out(ks_z_out), .o_valid(ks_o_valid), .done(ks_done));

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
    anderson #(.DIM(L), .MEM(10)) u_aa(.clk(clk), .rst_n(rst_n), .start(aa_start),
        .x_in(aa_x), .f_in(aa_f), .din_valid(aa_dv), .rdy(aa_rdy),
        .f_out(aa_fout), .o_valid(aa_o_valid), .done(aa_done));
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
    reg  rs_start;
    reg  [63:0] rs_in;
    wire rs_done;
    wire [63:0] rs_o;
    fp64_rsqrt urs(clk, rs_start, rs_in, rs_done, rs_o);

    // ---- RAM port mux: own | pdc (base U_BASE+N) ----
    reg [RAM_AW-1:0] own_addr;
    reg [63:0] own_wdata;
    reg own_we;

    // ---- FSM ----
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
 S_SCP0=99, S_SCP1=100, S_SCP2=101, S_SCP3=102;
 reg [6:0] st;   // 0..102 fits in 7 bits
 reg [15:0] i;
 reg [63:0] vv, utv, gv, drv, tau_r, norm_r, sqacc, v_eta;
 reg [63:0] gacc, norm_g_r, aa_xr, aa_fr;
 reg sflag;
 reg aa_done_r;
 reg rs_done_p, rs_rise;
 reg [15:0] aa_cnt;
    wire pdc_own = (st == S_CONE || st == S_CONEW);
    always @* begin
        if (pdc_own) begin
            ram_addr = U_BASE + N + pdc_addr;
            ram_wdata = pdc_wdata;
            ram_we = pdc_we;
        end else begin
            ram_addr = own_addr;
            ram_wdata = own_wdata;
            ram_we = own_we;
        end
    end
    assign pdc_rdata = ram_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0;
            own_addr <= 0; own_wdata <= 0; own_we <= 0;
            ks_start <= 0; ks_band_valid <= 0; ks_din_valid <= 0;
            ks_band_in <= 0; ks_x_in <= 0;
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
        end else begin
            ks_start <= 0; ks_band_valid <= 0; ks_din_valid <= 0;
            rp_start <= 0; rp_dv <= 0; pdc_start <= 0; rs_start <= 0;
            aa_start <= 0; aa_dv <= 0;
            own_we <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    if (iter == 0) begin i <= 0; own_addr <= V_BASE; st <= S_VPC0; end  // FEAS: v_prev=v0, no normalize
                    else if (aa_round) begin i <= 0; own_addr <= VPR_BASE; st <= S_AA0; end  // AA apply first
                    else begin i <= 0; own_addr <= V_BASE; st <= S_NM0; end
                end
            end
            // ============ normalize v (iter>=1): pass1 ||v||, pass2 scale ====
            S_NM0: begin i <= 0; sqacc <= 0; own_addr <= V_BASE; st <= S_NM1; end
            S_NM1: begin
                vv <= ram_rdata; own_addr <= V_BASE + i + 1; st <= S_NM2;
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
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_NM1; end
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
            S_NMV0: begin i <= 0; own_addr <= V_BASE; st <= S_NMV1; end
            S_NMV1: begin
                vv <= ram_rdata; own_addr <= V_BASE + i + 1; st <= S_NMV2;
            end
            S_NMV2: begin a1 <= vv; b1 <= norm_r; st <= S_NMV3; end   // v*norm
            S_NMV3: begin
                own_addr <= V_BASE + i; own_wdata <= po1; own_we <= 1;
                st <= S_NMV4;
            end
            S_NMV4: begin
                if (i + 1 >= L) begin i <= 0; own_addr <= V_BASE; st <= S_VPC0; end
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_NMV1; end
            end
            // ============ Anderson apply (aa_round): stream (x=v_prev, f=v),
            // back up both to SAFEV/SAFEVP, accumulate ||x-f||^2 for norm_g ====
            S_AA0: begin aa_start <= 1; gacc <= 0; aa_done_r <= 0; aa_cnt <= aa_cnt + 1; i <= 0; own_addr <= VPR_BASE; st <= S_AA1; end
            S_AA1: begin aa_xr <= ram_rdata; own_addr <= V_BASE + i; st <= S_AA2; end
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
                if (i + 1 >= L) begin i <= 0; st <= S_AA8; end
                else begin i <= i + 1; own_addr <= VPR_BASE + i; st <= S_AA1; end
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
                    aa_done_r <= 0; rs_rise <= 0; i <= 0; own_addr <= V_BASE; st <= S_NM0;
                end
            end
            // ============ v_prev <- v (after AA apply / normalize) ============
            S_VPC0: begin i <= 0; own_addr <= V_BASE; st <= S_VPC1; end
            S_VPC1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_VPC2; end
            S_VPC2: begin
                own_addr <= VPR_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_VPC3;
            end
            S_VPC3: begin
                if (i + 1 >= L) st <= S_KSSTART;
                else begin i <= i + 1; own_addr <= V_BASE + i; st <= S_VPC1; end
            end
            // ============ KKT solve ============
            S_KSSTART: begin
                ks_start <= 1; i <= 0;
                if (refactor) st <= S_KSBAND; else st <= S_KSVX;
            end
            S_KSBAND: begin
                if (band_valid) begin
                    ks_band_valid <= 1; ks_band_in <= band_in;
                    if (i + 1 >= (HB+1)*N) begin i <= 0; st <= S_KSVX; end
                    else i <= i + 1;
                end
            end
            S_KSVX: begin own_addr <= V_BASE + i; st <= S_KSVXW; end
            S_KSVXW: begin
                ks_x_in <= ram_rdata; ks_din_valid <= 1;
                if (i + 1 >= N) begin i <= 0; st <= S_KSVY; end
                else begin i <= i + 1; st <= S_KSVX; end
            end
            S_KSVY: begin own_addr <= V_BASE + N + i; st <= S_KSVYW; end
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
                    end else begin own_addr <= V_BASE + LM1; st <= S_KSDONE_ETA; end
                end
            end
            S_UTT: begin st <= S_GT0; end      // u_t[l-1] write lands here
            S_KSDONE_ETA: begin
                v_eta <= ram_rdata; i <= 0; st <= S_RP0;
            end
            // ============ root_plus (iter>=1): stream LM1 4-tuples ===========
            S_RP0: begin
                rp_start <= 1; rp_eta <= v_eta; i <= 0; own_addr <= G_BASE; st <= S_RP1;
            end
            S_RP1: begin gv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_RP2; end
            S_RP2: begin utv <= ram_rdata; own_addr <= V_BASE + i; st <= S_RP3; end
            S_RP3: begin vv <= ram_rdata; own_addr <= DR_BASE + i; st <= S_RP4; end
            S_RP4: begin
                drv <= ram_rdata; st <= S_RP5;
            end
            S_RP5: begin
                rp_r <= drv; rp_g <= gv; rp_p <= utv; rp_mu <= vv; rp_dv <= 1;
                if (i + 1 >= LM1) st <= S_RPW;
                else begin i <= i + 1; own_addr <= G_BASE + i + 1; st <= S_RP1; end
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
            S_GT0: begin i <= 0; own_addr <= UT_BASE; st <= S_GT1; end
            S_GT1: begin utv <= ram_rdata; own_addr <= G_BASE + i; st <= S_GT2; end
            S_GT2: begin gv <= ram_rdata; st <= S_GT3; end
            S_GT3: begin a1 <= tau_r; b1 <= gv; st <= S_GT4; end
            S_GT4: begin sa <= utv; sb <= po1; ssub <= 1; st <= S_GT5; end
            S_GT5: begin
                own_addr <= UT_BASE + i; own_wdata <= so1; own_we <= 1; st <= S_GT6;
            end
            S_GT6: begin
                if (i + 1 >= LM1) st <= S_U0;
                else begin i <= i + 1; own_addr <= UT_BASE + i + 1; st <= S_GT1; end
            end
            // ============ u = 2*u_t - v ============
            S_U0: begin i <= 0; own_addr <= UT_BASE; st <= S_U1; end
            S_U1: begin utv <= ram_rdata; own_addr <= V_BASE + i; st <= S_U2; end
            S_U2: begin vv <= ram_rdata; a1 <= utv; b1 <= 64'h4000000000000000; st <= S_U3; end
            S_U3: begin sa <= po1; sb <= vv; ssub <= 1; st <= S_U4; end
            S_U4: begin
                own_addr <= U_BASE + i; own_wdata <= so1; own_we <= 1;
                st <= S_U5;
            end
            S_U5: begin
                if (i + 1 >= L) st <= S_CONE;
                else begin i <= i + 1; own_addr <= UT_BASE + i + 1; st <= S_U1; end
            end
            // ============ cone projection (u[n:l-1], always) ============
            S_CONE: begin pdc_start <= 1; st <= S_CONEW; end
            S_CONEW: begin
                if (pdc_done) begin
                    if (iter == 0) begin
                        own_addr <= U_BASE + LM1; own_wdata <= 64'h3FF0000000000000;
                        own_we <= 1; st <= S_UMW;
                    end else begin own_addr <= U_BASE + LM1; st <= S_UM0; end
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
            S_RSK0: begin i <= 0; own_addr <= V_BASE; st <= S_RSK1; end
            S_RSK1: begin vv <= ram_rdata; own_addr <= U_BASE + i; st <= S_RSK2; end
            S_RSK2: begin utv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_RSK3; end
            S_RSK3: begin gv <= ram_rdata; own_addr <= DR_BASE + i; st <= S_RSK4; end
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
                else begin i <= i + 1; own_addr <= V_BASE + i + 1; st <= S_RSK1; end
            end
            // ============ v += 1.5*(u - u_t) ============
            S_V0: begin i <= 0; own_addr <= U_BASE; st <= S_V1; end
            S_V1: begin utv <= ram_rdata; own_addr <= UT_BASE + i; st <= S_V2; end
            S_V2: begin gv <= ram_rdata; own_addr <= V_BASE + i; st <= S_V3; end
            S_V3: begin vv <= ram_rdata; sa <= utv; sb <= gv; ssub <= 1; st <= S_V4; end
            S_V4: begin a1 <= so1; b1 <= 64'h3FF8000000000000; st <= S_V5; end
            S_V5: begin sc <= vv; sd <= po1; ssub2 <= 0; st <= S_V6; end
            S_V6: begin
                own_addr <= V_BASE + i; own_wdata <= so2; own_we <= 1;
                st <= S_V7;
            end
            S_V7: begin
                if (i + 1 >= L) begin
                    if (aa_round) begin i <= 0; own_addr <= V_BASE; st <= S_SG0; end
                    else st <= S_DONE;
                end
                else begin i <= i + 1; own_addr <= U_BASE + i + 1; st <= S_V1; end
            end
            // ============ AA safeguard: ||v - v_prev|| vs AA_SAFEGUARD*norm_g ===
            S_SG0: begin i <= 0; gacc <= 0; own_addr <= V_BASE; st <= S_SG1; end
            S_SG1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_SG2; end
            S_SG2: begin gv <= ram_rdata; sa <= vv; sb <= gv; ssub <= 1; st <= S_SG3; end
            S_SG3: begin a1 <= so1; b1 <= so1; st <= S_SG4; end
            S_SG4: begin sc <= gacc; sd <= po1; ssub2 <= 0; st <= S_SG5; end
            S_SG5: begin
                gacc <= so2;
                if (i + 1 >= L) st <= S_SG6;
                else begin i <= i + 1; own_addr <= V_BASE + i; st <= S_SG1; end
            end
            S_SG6: begin rs_start <= 1; rs_in <= gacc; st <= S_SG7; end
            S_SG7: begin
                if (rs_done) begin
                    a2 <= norm_g_r; b2 <= AA_SAFEGUARD; cmp_a <= rs_o; st <= S_SG8;
                end
            end
            S_SG8: begin cmp_b <= po2; st <= S_SG9; end
            S_SG9: begin
                // scs_faithful.safeguard returns early unless the LAST apply
                // actually solved (self.success), which happens only from the
                // 11th apply onward (aa.iter >= AA_MINLEN=10). aa_cnt is 1-based:
                // checks are meaningful from aa_cnt >= 11.
                if (cmp_gt && aa_cnt >= 11) st <= S_SC0;      // reject: roll back
                else st <= S_DONE;
            end
            // ============ rollback: SAFEV->v, SAFEVP->v_prev ============
            S_SC0: begin i <= 0; own_addr <= SAFEV_BASE; st <= S_SC1; end
            S_SC1: begin vv <= ram_rdata; own_addr <= V_BASE + i; st <= S_SC2; end
            S_SC2: begin
                own_addr <= V_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_SC3;
            end
            S_SC3: begin
                if (i + 1 >= L) st <= S_SCP0;
                else begin i <= i + 1; own_addr <= SAFEV_BASE + i; st <= S_SC1; end
            end
            S_SCP0: begin i <= 0; own_addr <= SAFEVP_BASE; st <= S_SCP1; end
            S_SCP1: begin vv <= ram_rdata; own_addr <= VPR_BASE + i; st <= S_SCP2; end
            S_SCP2: begin
                own_addr <= VPR_BASE + i; own_wdata <= vv; own_we <= 1; st <= S_SCP3;
            end
            S_SCP3: begin
                if (i + 1 >= L) st <= S_DONE;
                else begin i <= i + 1; own_addr <= SAFEVP_BASE + i; st <= S_SCP1; end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
