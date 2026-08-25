`timescale 1ns/1ps
// kkt_solve: complete Schur block-elimination KKT solve (Phase 1d/1e + 4a).
// Given iterate v_x (N), v_y (M), static r_y/D_y, computes the KKT solve
//   S z_x = rho_x v_x - A^T v_y          (S = rho_x I + A^T D_y A, banded LDL^T)
//   z_y   = D_y (A z_x + r_y v_y)
// by chaining the validated blocks: 2x spmv_fp64 (A^T and A) + 1x
// banded_ldl_fp64_rb, orchestrated by this FSM. All arithmetic IEEE-754 FP64.
// Phase 4a: the S band is now STREAMED IN (band_in/band_valid, (HB+1)*N words)
// instead of $readmemh — the band can come from s_build at runtime, so the
// adaptive-scaling loop can re-factorize with a rebuilt S (new D_y).
//
// RAM-port design (proj_dual_cone / spmv pattern): one shared external RAM
// (sync write, async read) holds all state vectors. Address map (LMAX=max(N,M)):
//   [0, LMAX)            vy            (spmv1 x region, BASE1=0)
//   [LMAX, 2*LMAX)       A^T vy        (spmv1 out region; dead after rhs stream)
//   [2*LMAX, 3*LMAX)     vx -> zx      (spmv2 x region; vx overwritten by zx)
//   [3*LMAX, 4*LMAX)     az -> zy      (spmv2 out region; zy written in place)
// spmv2's physical addresses are offset by 2*LMAX in the port mux; spmv1 uses
// BASE1=0 so its addresses pass through untouched (its 13-bit ram_addr is fine:
// max 2*LMAX-1 = 4213 < 8192; the mux widens to RAM_AW=14).
//
// Protocol: pulse start, stream (HB+1)*N band words (band_valid, 1/cycle),
// then N vx words then M vy words (din_valid, 1/cycle), wait done. On
// completion, N zx words then M zy words stream out on z_out (o_valid, 1/cycle),
// then done pulses.
module kkt_solve #(
    parameter N = 10, M = 20, NNZ = 41, HB = 4,
    parameter AA_BASE = 0,   // SRAM word offset for LDL arrays (past Anderson region)
    parameter RAM_AW = 14,
    parameter AROW_FILE = "../data/kkt/small/Arow.hex",
    parameter ACOL_FILE = "../data/kkt/small/Acol.hex",
    parameter AVAL_FILE = "../data/kkt/small/Aval.hex",
    parameter RY_FILE   = "../data/kkt/small/r_y.hex",
    parameter DY_FILE   = "../data/kkt/small/Dy.hex"
)(
    input  wire          clk, rst_n, start,
    // runtime band input (streamed, (HB+1)*N words; only when refactor=1)
    input  wire [63:0]   band_in,
    input  wire          band_valid,
    // 1: stream band + re-factorize (scale change); 0: reuse last L/D (per-iter)
    input  wire          refactor,
    // runtime D_y / r_y update (streamed M words each, when par_update=1 at start):
    // lets a scale change feed the NEW D_y/r_y into zy's computation (else the
    // static $readmemh arrays hold the scale=1 values and zy is wrong after rescale)
    input  wire          par_update,
    input  wire [63:0]   ry_in, dy_in,
    input  wire          ry_valid, dy_valid,
    input  wire [63:0]   x_in,
    input  wire          din_valid,
    // shared external RAM port (sync write, async read)
    output reg  [RAM_AW-1:0] ram_addr,
    output reg  [63:0]   ram_wdata,
    output reg           ram_we,
    input  wire [63:0]   ram_rdata,
    // output stream: N zx words then M zy words
    output reg  [63:0]   z_out,
    output reg           o_valid,
    output reg           done,
    // ---- LDL external SRAM (arbiter-routed at drs_iter level) ----
    output wire           ldl_sram_req, ldl_sram_we,
    output wire  [17:0]   ldl_sram_waddr,
    output wire  [63:0]   ldl_sram_wdata,
    input  wire          ldl_sram_busy,
    input  wire [63:0]   ldl_sram_rdata,
    // ---- LDL band ready (back-pressure to the upstream band streamer) ----
    output wire           band_ready
);
    localparam LMAX = (N > M) ? N : M;
    localparam AD_ATVY = LMAX;          // spmv1 out region (A^T vy, N words)
    localparam AD_VX   = 2 * LMAX;      // vx region (later overwritten by zx)
    localparam AD_AZ   = 3 * LMAX;      // az region (later zy, M words in place)
    localparam RHO_X   = 64'h3eb0c6f7a0b5ed8d;  // 1e-6 (bit-exact double)

    // ---- static coefficient arrays (aa_gram pattern: $readmemh, sequential index) ----
    reg [63:0] r_y_arr [0:M-1];
    reg [63:0] Dy_arr  [0:M-1];
    initial begin
`ifndef SYNTHESIS
        $readmemh(RY_FILE, r_y_arr);
`endif
`ifndef SYNTHESIS
        $readmemh(DY_FILE, Dy_arr);
`endif
    end
    reg [63:0] zx_arr [0:N-1];          // zx captured from LDL, fed to spmv2

    // ---- sub-blocks ----
    wire sp1_done_w;
    wire [12:0] sp1_addr;
    wire [63:0] sp1_wdata;
    wire        sp1_we;
    reg  sp1_start, sp1_din_valid;
    reg  [63:0] sp1_x_in;
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(1),
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE)) u_spmv1(
        .clk(clk), .rst_n(rst_n), .start(sp1_start),
        .x_in(sp1_x_in), .din_valid(sp1_din_valid),
        .ram_addr(sp1_addr), .ram_wdata(sp1_wdata), .ram_we(sp1_we),
        .ram_rdata(ram_rdata), .out_out(), .o_valid(), .done(sp1_done_w));

    wire [12:0] sp2_addr;
    wire [63:0] sp2_wdata;
    wire        sp2_we;
    reg  sp2_start, sp2_din_valid;
    reg  [63:0] sp2_x_in;
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(0),
                .AROW_FILE(AROW_FILE), .ACOL_FILE(ACOL_FILE), .AVAL_FILE(AVAL_FILE)) u_spmv2(
        .clk(clk), .rst_n(rst_n), .start(sp2_start),
        .x_in(sp2_x_in), .din_valid(sp2_din_valid),
        .ram_addr(sp2_addr), .ram_wdata(sp2_wdata), .ram_we(sp2_we),
        .ram_rdata(ram_rdata), .out_out(), .o_valid(), .done(sp2_done_w));

    reg  ldl_start, ldl_band_valid, ldl_rhs_valid;
    reg  [63:0] ldl_band_in, ldl_rhs_in;
    wire [63:0] ldl_zx_out;
    wire        ldl_zx_valid, ldl_done_w;
    wire        ldl_band_ready_w, ldl_rhs_ready_w;
    banded_ldl_fp64_rb #(.N(N), .HB(HB), .AA_BASE(AA_BASE)) u_ldl(
        .clk(clk), .rst_n(rst_n), .start(ldl_start), .refactor(refactor),
        .band_in(ldl_band_in), .band_valid(ldl_band_valid), .band_ready(ldl_band_ready_w),
        .rhs_in(ldl_rhs_in), .rhs_valid(ldl_rhs_valid), .rhs_ready(ldl_rhs_ready_w),
        .zx_out(ldl_zx_out), .zx_valid(ldl_zx_valid), .done(ldl_done_w), .status(),
        .sram_req(ldl_sram_req), .sram_we(ldl_sram_we), .sram_waddr(ldl_sram_waddr),
        .sram_wdata(ldl_sram_wdata), .sram_busy(ldl_sram_busy), .sram_rdata(ldl_sram_rdata));
    assign band_ready = ldl_band_ready_w;   // back-pressure to the band streamer

    // ---- own FP64 arithmetic (combinational units, spmv pattern) ----
    // rhs_x = rho_x*vx - A^T vy  (all combinational from regs vx_i/atvy_i)
    reg [63:0] vx_i, atvy_i;
    wire [63:0] po_rhs, so_rhs;
    fp64_mul um_rhs(vx_i, RHO_X, po_rhs);
    fp64_add ua_rhs(po_rhs, atvy_i, 1'b1, so_rhs);
    // zy = Dy * (az + r_y*vy)    (all combinational from regs az_r/vy_r/ry_reg/dy_reg)
    reg [63:0] az_r, vy_r, ry_reg, dy_reg;
    wire [63:0] po_zy1, so_zy1, po_zy2;
    fp64_mul um_zy1(ry_reg, vy_r, po_zy1);
    fp64_add ua_zy1(az_r, po_zy1, 1'b0, so_zy1);
    fp64_mul um_zy2(dy_reg, so_zy1, po_zy2);

    // ---- RAM port mux: spmv1 (BASE1=0) | spmv2 (BASE2=2*LMAX) | own ----
    localparam S_IDLE=0, S_BAND=1, S_VX=2, S_VXW=3, S_VY=4,
               S_SP1WAIT=5, S_RHS0=6, S_RHS1=7, S_RHS1R=27, S_RHS2=8, S_RHS2R=28, S_RHS3=9,
               S_ZCAP=10, S_ZCAPDONE=11,
               S_SP2START=12, S_SP2FEED=13, S_SP2WAIT=14,
               S_ZY0=15, S_ZY1=16, S_ZY1R=29, S_ZY2=17, S_ZY2R=30, S_ZY3=18, S_ZY4=19,
               S_ZOUT0=20, S_ZOUT1=21, S_ZOUT1R=31, S_ZYOUT0=22, S_ZYOUT1=23, S_ZYOUT1R=32, S_DONE=24,
               S_DYRY=25, S_DYDY=26;
    reg [5:0] st;
    reg [15:0] wp, i, r, zo;
    wire sp1_own = (st == S_VY || st == S_SP1WAIT);
    wire sp2_own = (st == S_SP2FEED || st == S_SP2WAIT);
    reg [RAM_AW-1:0] own_addr;
    reg [63:0] own_wdata;
    reg own_we;
    always @* begin
        if (sp1_own) begin
            ram_addr = sp1_addr;
            ram_wdata = sp1_wdata;
            ram_we = sp1_we;
        end else if (sp2_own) begin
            ram_addr = 2 * LMAX + sp2_addr;
            ram_wdata = sp2_wdata;
            ram_we = sp2_we;
        end else begin
            ram_addr = own_addr;
            ram_wdata = own_wdata;
            ram_we = own_we;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0;
            own_addr <= 0; own_wdata <= 0; own_we <= 0;
            sp1_start <= 0; sp1_din_valid <= 0; sp1_x_in <= 0;
            sp2_start <= 0; sp2_din_valid <= 0; sp2_x_in <= 0;
            ldl_start <= 0; ldl_band_valid <= 0; ldl_band_in <= 0;
            ldl_rhs_valid <= 0; ldl_rhs_in <= 0;
            z_out <= 0; wp <= 0; i <= 0; r <= 0; zo <= 0;
            vx_i <= 0; atvy_i <= 0; az_r <= 0; vy_r <= 0;
            ry_reg <= 0; dy_reg <= 0;
        end else begin
            o_valid <= 0; own_we <= 0;
            sp1_din_valid <= 0; sp2_din_valid <= 0;
            ldl_rhs_valid <= 0; ldl_start <= 0;
            if (ldl_band_valid && !ldl_band_ready_w) ldl_band_valid <= 0;   // clear after LDL sampled
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    ldl_start <= 1; wp <= 0;
                    // par_update: stream new r_y + D_y into the arrays first
                    if (par_update) st <= S_DYRY;
                    else if (refactor) st <= S_BAND; else st <= S_VX;
                end
            end
            // ---- stream M r_y + M D_y words (scale change: refresh zy's D_y/r_y) ----
            S_DYRY: begin
                if (ry_valid) begin
                    r_y_arr[wp] <= ry_in;
                    if (wp + 1 >= M) begin wp <= 0; st <= S_DYDY; end
                    else wp <= wp + 1;
                end
            end
            S_DYDY: begin
                if (dy_valid) begin
                    Dy_arr[wp] <= dy_in;
                    if (wp + 1 >= M) begin
                        wp <= 0;
                        if (refactor) st <= S_BAND; else st <= S_VX;
                    end else wp <= wp + 1;
                end
            end
            // ---- stream (HB+1)*N band words into LDL (only when refactor=1) ----
            S_BAND: begin
                if (band_valid && ldl_band_ready_w && !ldl_band_valid) begin
                    ldl_band_valid <= 1; ldl_band_in <= band_in;
                    if (wp + 1 >= (HB+1)*N) begin wp <= 0; st <= S_VX; end
                    else wp <= wp + 1;
                end
            end
            // ---- stream N vx words into RAM[AD_VX + wp] ----
            S_VX: begin
                if (din_valid) begin
                    own_addr <= AD_VX + wp; own_wdata <= x_in; own_we <= 1;
                    if (wp + 1 >= N) begin sp1_start <= 1; wp <= 0; st <= S_VXW; end
                    else wp <= wp + 1;
                end
            end
            // ---- hold one cycle so the LAST vx write lands while the mux is
            //      still on "own" (switching to spmv1 same-cycle would drop it:
            //      ram_we becomes sp1_we=0 and the write is lost). This cycle
            //      ALSO forwards the first vy word to spmv1 (it sampled start
            //      this cycle) so the continuous vx+vy stream loses no word. ----
            S_VXW: begin
                sp1_start <= 0;
                if (din_valid) begin
                    sp1_din_valid <= 1; sp1_x_in <= x_in;
                    if (wp + 1 >= M) begin wp <= 0; st <= S_SP1WAIT; end
                    else begin wp <= wp + 1; st <= S_VY; end
                end
            end
            // ---- forward remaining vy words straight into spmv1 (it writes
            //      them to RAM[0..M), its x region; BASE1=0 so mux passes
            //      addresses) ----
            S_VY: begin
                sp1_start <= 0;
                if (din_valid) begin
                    sp1_din_valid <= 1; sp1_x_in <= x_in;
                    if (wp + 1 >= M) begin wp <= 0; st <= S_SP1WAIT; end
                    else wp <= wp + 1;
                end
            end
            // ---- wait A^T vy done (spmv1 wrote it to RAM[LMAX..LMAX+N));
            //      LDL already got start + band, now waiting in S_RHS ----
            S_SP1WAIT: begin
                if (sp1_done_w) begin i <= 0; st <= S_RHS0; end
            end
            // ---- stream rhs_x[i] = rho_x*vx[i] - A^Tvy[i] into LDL, 4 cyc/elem ----
            S_RHS0: begin own_addr <= AD_VX + i; st <= S_RHS1; end
            S_RHS1: begin st <= S_RHS1R; end   // sync-read wait: kmem rdata for AD_VX+i
            S_RHS1R: begin vx_i <= ram_rdata; own_addr <= AD_ATVY + i; st <= S_RHS2; end
            S_RHS2: begin st <= S_RHS2R; end   // sync-read wait: kmem rdata for AD_ATVY+i
            S_RHS2R: begin atvy_i <= ram_rdata; st <= S_RHS3; end
            S_RHS3: begin
                if (ldl_rhs_ready_w) begin
                    ldl_rhs_in <= so_rhs; ldl_rhs_valid <= 1;
                    if (i + 1 >= N) begin zo <= 0; st <= S_ZCAP; end
                    else begin i <= i + 1; st <= S_RHS0; end
                end
            end
            // ---- capture zx from LDL into zx_arr + RAM[AD_VX + zo] (vx dead).
            //      LDL streams one word/cycle, so capture must take ONE state
            //      per word (a 2-state capture misses every other word) ----
            S_ZCAP: begin
                if (ldl_zx_valid) begin
                    zx_arr[zo] <= ldl_zx_out;
                    own_addr <= AD_VX + zo; own_wdata <= ldl_zx_out; own_we <= 1;
                    if (zo + 1 >= N) st <= S_ZCAPDONE; else zo <= zo + 1;
                end
            end
            S_ZCAPDONE: begin
                if (ldl_done_w) begin sp2_start <= 1; i <= 0; st <= S_SP2START; end
            end
            S_SP2START: begin sp2_start <= 0; st <= S_SP2FEED; end
            // ---- re-stream zx from zx_arr into spmv2 (writes same values back
            //      to RAM[2*LMAX..], its x region) ----
            S_SP2FEED: begin
                sp2_din_valid <= 1; sp2_x_in <= zx_arr[i];
                if (i + 1 >= N) st <= S_SP2WAIT; else i <= i + 1;
            end
            // ---- wait A zx done (az in RAM[3*LMAX..3*LMAX+M)) ----
            S_SP2WAIT: begin
                if (sp2_done_w) begin r <= 0; st <= S_ZY0; end
            end
            // ---- zy[r] = Dy[r]*(az[r] + r_y[r]*vy[r]), 4 cyc/elem, in place ----
            S_ZY0: begin
                ry_reg <= r_y_arr[r]; dy_reg <= Dy_arr[r];
                own_addr <= AD_AZ + r; st <= S_ZY1;
            end
            S_ZY1: begin st <= S_ZY1R; end   // sync-read wait: kmem rdata for AD_AZ+r
            S_ZY1R: begin az_r <= ram_rdata; own_addr <= r; st <= S_ZY2; end
            S_ZY2: begin st <= S_ZY2R; end   // sync-read wait: kmem rdata for r
            S_ZY2R: begin vy_r <= ram_rdata; st <= S_ZY3; end
            S_ZY3: begin
                own_addr <= AD_AZ + r; own_wdata <= po_zy2; own_we <= 1;
                st <= S_ZY4;
            end
            S_ZY4: begin
                if (r + 1 >= M) begin i <= 0; st <= S_ZOUT0; end
                else begin r <= r + 1; st <= S_ZY0; end
            end
            // ---- stream N zx words (RAM[AD_VX..]) then M zy words (RAM[AD_AZ..]) ----
            S_ZOUT0: begin own_addr <= AD_VX + i; st <= S_ZOUT1; end
            S_ZOUT1: begin st <= S_ZOUT1R; end   // sync-read wait: kmem rdata for AD_VX+i
            S_ZOUT1R: begin
                z_out <= ram_rdata; o_valid <= 1;
                if (i + 1 >= N) begin i <= 0; st <= S_ZYOUT0; end
                else begin i <= i + 1; st <= S_ZOUT0; end
            end
            S_ZYOUT0: begin own_addr <= AD_AZ + i; st <= S_ZYOUT1; end
            S_ZYOUT1: begin st <= S_ZYOUT1R; end   // sync-read wait: kmem rdata for AD_AZ+i
            S_ZYOUT1R: begin
                z_out <= ram_rdata; o_valid <= 1;
                if (i + 1 >= M) st <= S_DONE; else begin i <= i + 1; st <= S_ZYOUT0; end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
