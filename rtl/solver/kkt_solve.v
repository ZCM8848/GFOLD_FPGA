`timescale 1ns/1ps
// kkt_solve: complete Schur block-elimination KKT solve (Phase 1d/1e + 4a).
// Given iterate v_x (N), v_y (M), static r_y/D_y, computes the KKT solve
//   S z_x = rho_x v_x - A^T v_y          (S = rho_x I + A^T D_y A, banded LDL^T)
//   z_y   = D_y (A z_x + r_y v_y)
// by chaining the validated blocks: 2x spmv_fp64 (A^T and A) + 1x
// banded_ldl_fp64_rb, orchestrated by this FSM. All arithmetic IEEE-754 FP64.
// Phase 4a: the S band is now STREAMED IN (band_in/band_valid, (HB+1)*N words)
// instead of $readmemh �?the band can come from s_build at runtime, so the
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
    parameter COO_BASE = 0,  // SRAM word base of the packed COO (shared by spmv1/spmv2)
    parameter RAM_AW = 14,
    parameter EXT_COEF = 0,          // 1: r_y/D_y live in external SRAM
    parameter [17:0] COEF_RY_BASE = 18'h12000,
    parameter [17:0] COEF_DY_BASE = 18'h14000,
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
    // ---- spmv external SRAM (COO read, arbiter-routed at drs_iter level) ----
    output wire           sp1_sram_req, sp1_sram_we,
    output wire  [17:0]   sp1_sram_waddr,
    output wire  [63:0]   sp1_sram_wdata,
    input  wire          sp1_sram_busy,
    input  wire [63:0]   sp1_sram_rdata,
    output wire           sp2_sram_req, sp2_sram_we,
    output wire  [17:0]   sp2_sram_waddr,
    output wire  [63:0]   sp2_sram_wdata,
    input  wire          sp2_sram_busy,
    input  wire [63:0]   sp2_sram_rdata,
    // ---- external r_y / D_y coefficients (EXT_COEF=1) ----
    output reg           coef_req, coef_we,
    output reg  [17:0]   coef_addr,
    output reg  [63:0]   coef_wdata,
    input  wire          coef_busy,
    input  wire [63:0]   coef_rdata,
    // ---- LDL band ready (back-pressure to the upstream band streamer) ----
    output wire           band_ready,
    // Pool channels (muxed to own/spmv1/spmv2/ldl internally)
    output wire [63:0]  mul0_a, mul0_b,
    input  wire [63:0]  mul0_o,
    output wire [63:0]  add0_a, add0_b,
    output wire         add0_sub,
    input  wire [63:0]  add0_o,
    output wire         div_start,
    output wire [63:0] div_a, div_b,
    input  wire         div_done,
    input  wire [63:0] div_o
);
    localparam LMAX = (N > M) ? N : M;
    localparam AD_ATVY = LMAX;          // spmv1 out region (A^T vy, N words)
    localparam AD_VX   = 2 * LMAX;      // vx region (later overwritten by zx)
    // A^T vy is dead after the RHS stream, so az/zy reuses its region.
    // This reduces the KKT RAM from 4*LMAX to 3*LMAX words.
    localparam AD_AZ   = LMAX;          // az region (later zy, M words in place)
    localparam RHO_X   = 64'h3eb0c6f7a0b5ed8d;  // 1e-6 (bit-exact double)

    // ---- static coefficient arrays (aa_gram pattern: $readmemh, sequential index) ----
    (* ram_style = "bram" *) reg [63:0] r_y_arr [0:M-1];
    (* ram_style = "bram" *) reg [63:0] Dy_arr  [0:M-1];
    initial begin
`ifndef SYNTHESIS
        $readmemh(RY_FILE, r_y_arr);
`endif
`ifndef SYNTHESIS
        $readmemh(DY_FILE, Dy_arr);
`endif
    end
    (* ram_style = "bram" *) reg [63:0] zx_arr [0:N-1];          // zx captured from LDL, fed to spmv2

    // ---- sub-blocks ----
    wire sp1_done_w;
    wire [12:0] sp1_addr;
    wire [63:0] sp1_wdata;
    wire        sp1_we;
    reg  sp1_start, sp1_din_valid;
    reg  [63:0] sp1_x_in;
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(1), .COO_BASE(COO_BASE)) u_spmv1(
        .clk(clk), .rst_n(rst_n), .start(sp1_start),
        .x_in(sp1_x_in), .din_valid(sp1_din_valid),
        .ram_addr(sp1_addr), .ram_wdata(sp1_wdata), .ram_we(sp1_we),
        .ram_rdata(ram_rdata), .out_out(), .o_valid(), .done(sp1_done_w),
        .sram_req(sp1_sram_req), .sram_we(sp1_sram_we), .sram_waddr(sp1_sram_waddr),
        .sram_wdata(sp1_sram_wdata), .sram_busy(sp1_sram_busy), .sram_rdata(sp1_sram_rdata),
    .mul0_a(sp1_m0a), .mul0_b(sp1_m0b), .mul0_o(sp1_m0o),
    .add0_a(sp1_a0a), .add0_b(sp1_a0b), .add0_sub(sp1_a0s), .add0_o(sp1_a0o));

    wire [12:0] sp2_addr;
    wire [63:0] sp2_wdata;
    wire        sp2_we;
    reg  sp2_start, sp2_din_valid;
    reg  [63:0] sp2_x_in;
    spmv_fp64 #(.N(N), .M(M), .NNZ(NNZ), .transpose(0), .COO_BASE(COO_BASE)) u_spmv2(
        .clk(clk), .rst_n(rst_n), .start(sp2_start),
        .x_in(sp2_x_in), .din_valid(sp2_din_valid),
        .ram_addr(sp2_addr), .ram_wdata(sp2_wdata), .ram_we(sp2_we),
        .ram_rdata(ram_rdata), .out_out(), .o_valid(), .done(sp2_done_w),
        .sram_req(sp2_sram_req), .sram_we(sp2_sram_we), .sram_waddr(sp2_sram_waddr),
        .sram_wdata(sp2_sram_wdata), .sram_busy(sp2_sram_busy), .sram_rdata(sp2_sram_rdata),
    .mul0_a(sp2_m0a), .mul0_b(sp2_m0b), .mul0_o(sp2_m0o),
    .add0_a(sp2_a0a), .add0_b(sp2_a0b), .add0_sub(sp2_a0s), .add0_o(sp2_a0o));

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
        .sram_wdata(ldl_sram_wdata), .sram_busy(ldl_sram_busy), .sram_rdata(ldl_sram_rdata),
        .mul_a(ldl_ma), .mul_b(ldl_mb), .mul_o(ldl_mo),
        .add_a(ldl_aa), .add_b(ldl_ab), .add_sub(ldl_asub), .add_o(ldl_ao),
        .div_start(ldl_ds), .div_a(ldl_da), .div_b(ldl_db), .div_done(ldl_ddone), .div_o(ldl_do));
    assign band_ready = ldl_band_ready_w;   // back-pressure to the band streamer

    // ---- external coefficient handshake (EXT_COEF=1) ----
    reg        coef_start;
    reg        coef_done_r;
    reg [1:0]  cst;
    wire       coef_done = coef_done_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coef_req <= 1'b0; cst <= 2'd0; coef_done_r <= 1'b0;
        end else begin
            coef_done_r <= 1'b0;
            case (cst)
            2'd0: begin
                if (coef_start) begin coef_req <= 1'b1; cst <= 2'd1; end
                else coef_req <= 1'b0;
            end
            2'd1: begin
                coef_req <= 1'b0;
                if (coef_busy) cst <= 2'd2;
            end
            2'd2: begin
                if (!coef_busy) begin coef_done_r <= 1'b1; cst <= 2'd0; end
            end
            endcase
        end
    end

    // ---- own FP64 arithmetic via pool (rhs + zy, sequential) ----
    reg [63:0] vx_i, atvy_i, po_rhs_r;
    reg [63:0] az_r, vy_r, ry_reg, dy_reg, zy1_r, zy2_r;
    // Pool mux: own vs spmv1 vs spmv2 vs ldl
    wire sp1_own = (st == S_VY || st == S_SP1WAIT);
    wire sp2_own = (st == S_SP2FEED || st == S_SP2WAIT);
    wire own_arith = (st==S_RHS0||st==S_RHS1||st==S_RHS1R||st==S_RHS2||st==S_RHS2R||
                      st==S_RHS3||st==S_RHS3b||st==S_ZY0||st==S_ZY1||st==S_ZY1R||
                      st==S_ZY2||st==S_ZY2R||st==S_ZY3||st==S_ZY3b||st==S_ZY4);
    // Sub-module pool wires
    wire [63:0] sp1_m0a, sp1_m0b, sp1_m0o, sp1_a0a, sp1_a0b, sp1_a0o; wire sp1_a0s;
    wire [63:0] sp2_m0a, sp2_m0b, sp2_m0o, sp2_a0a, sp2_a0b, sp2_a0o; wire sp2_a0s;
    wire [63:0] ldl_ma, ldl_mb, ldl_mo, ldl_aa, ldl_ab, ldl_ao; wire ldl_asub;
    wire ldl_ds; wire [63:0] ldl_da, ldl_db, ldl_do; wire ldl_ddone;
    // Mux operands to pool
    assign mul0_a = own_arith ? pa_own : sp1_own ? sp1_m0a : sp2_own ? sp2_m0a : ldl_ma;
    assign mul0_b = own_arith ? pb_own : sp1_own ? sp1_m0b : sp2_own ? sp2_m0b : ldl_mb;
    assign add0_a = own_arith ? sa_own : sp1_own ? sp1_a0a : sp2_own ? sp2_a0a : ldl_aa;
    assign add0_b = own_arith ? sb_own : sp1_own ? sp1_a0b : sp2_own ? sp2_a0b : ldl_ab;
    assign add0_sub = own_arith ? ss_own : sp1_own ? sp1_a0s : sp2_own ? sp2_a0s : ldl_asub;
    assign div_start = ldl_ds; assign div_a = ldl_da; assign div_b = ldl_db;
    // Broadcast results to all consumers
    assign sp1_m0o = mul0_o; assign sp2_m0o = mul0_o; assign ldl_mo = mul0_o;
    assign sp1_a0o = add0_o; assign sp2_a0o = add0_o; assign ldl_ao = add0_o;
    assign ldl_ddone = div_done; assign ldl_do = div_o;
    // Own arithmetic operand mux (by state)
    reg [63:0] pa_own, pb_own, sa_own, sb_own; reg ss_own;
    always @(*) begin
        pa_own=0; pb_own=0; sa_own=0; sb_own=0; ss_own=0;
        case (st)
        // NOTE: the original design computes these combinationally and latches
        // one/two states later, so the operands must be selected in the LATCH
        // state, not in the earlier read state.
        S_RHS3: begin pa_own=vx_i; pb_own=RHO_X; end                 // latch rho_x*vx
        S_RHS3b: begin sa_own=po_rhs_r; sb_own=atvy_i; ss_own=1; end // po_rhs_r - atvy
        S_ZY3: begin pa_own=ry_reg; pb_own=vy_r; end                 // latch r_y*vy
        S_ZY3b: begin sa_own=az_r; sb_own=zy1_r; ss_own=0; end       // latch az+r_y*vy
        S_ZY4: begin pa_own=dy_reg; pb_own=zy2_r; end                // Dy*(az+r_y*vy)
        default: ;
        endcase
    end
    wire [63:0] po_rhs = mul0_o, so_rhs = add0_o;
    wire [63:0] po_zy1 = mul0_o, so_zy1 = add0_o, po_zy2 = mul0_o;

    // ---- RAM port mux: spmv1 (BASE1=0) | spmv2 (BASE2=2*LMAX) | own ----
    localparam S_IDLE=0, S_BAND=1, S_VX=2, S_VXW=3, S_VY=4,
               S_SP1WAIT=5, S_RHS0=6, S_RHS1=7, S_RHS1R=27, S_RHS2=8, S_RHS2R=28, S_RHS3=9, S_RHS3b=34,
               S_ZCAP=10, S_ZCAPDONE=11,
               S_SP2START=12, S_SP2FEED=13, S_SP2WAIT=14,
                S_ZY0=15, S_ZY1=16, S_ZY1R=29, S_ZY2=17, S_ZY2R=30, S_ZY3=18, S_ZY3b=33, S_ZY4=19,
               S_ZOUT0=20, S_ZOUT1=21, S_ZOUT1R=31, S_ZYOUT0=22, S_ZYOUT1=23, S_ZYOUT1R=32, S_DONE=24,
               S_DYRY=25, S_DYDY=26, S_ZY0W=35, S_ZY0D=36;
    reg [5:0] st;
    reg [15:0] wp, i, r, zo;
    reg [RAM_AW-1:0] own_addr;
    reg [63:0] own_wdata;
    reg own_we;
    always @* begin
        if (sp1_own) begin
            ram_addr = sp1_addr;
            ram_wdata = sp1_wdata;
            ram_we = sp1_we;
        end else if (sp2_own) begin
            // x region at 2*LMAX, out region reuses A^Tvy at LMAX.
            ram_addr = (sp2_addr < LMAX) ? (2 * LMAX + sp2_addr)
                                         : (LMAX + (sp2_addr - LMAX));
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
            coef_start <= 0; coef_we <= 0; coef_addr <= 0; coef_wdata <= 0;
            sp1_start <= 0; sp1_din_valid <= 0; sp1_x_in <= 0;
            sp2_start <= 0; sp2_din_valid <= 0; sp2_x_in <= 0;
            ldl_start <= 0; ldl_band_valid <= 0; ldl_band_in <= 0;
            ldl_rhs_valid <= 0; ldl_rhs_in <= 0;
            z_out <= 0; wp <= 0; i <= 0; r <= 0; zo <= 0;
            vx_i <= 0; atvy_i <= 0; az_r <= 0; vy_r <= 0;
            ry_reg <= 0; dy_reg <= 0; zy1_r <= 0; zy2_r <= 0; po_rhs_r <= 0;
        end else begin
            o_valid <= 0; own_we <= 0;
            coef_start <= 0; coef_we <= 0;
            sp1_din_valid <= 0; sp2_din_valid <= 0;
            ldl_rhs_valid <= 0; ldl_start <= 0;
            if (ldl_band_valid && !ldl_band_ready_w) ldl_band_valid <= 0;   // clear after LDL sampled
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    ldl_start <= 1; wp <= 0;
                    // par_update: stream new r_y + D_y into the arrays first.
                    // band is pre-loaded into LDL's SRAM B[] by s_build (when
                    // refactor=1), so it is no longer streamed in here.
                    if (par_update) st <= S_DYRY;
                    else st <= S_VX;
                end
            end
            // ---- stream M r_y + M D_y words (scale change: refresh zy's D_y/r_y) ----
            S_DYRY: begin
                if (ry_valid) begin
                    if (!EXT_COEF) r_y_arr[wp] <= ry_in;
                    if (wp + 1 >= M) begin wp <= 0; st <= S_DYDY; end
                    else wp <= wp + 1;
                end
            end
            S_DYDY: begin
                if (dy_valid) begin
                    if (!EXT_COEF) Dy_arr[wp] <= dy_in;
                    if (wp + 1 >= M) begin
                        wp <= 0;
                        st <= S_VX;   // band pre-loaded into LDL SRAM; never streamed
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
            S_RHS3: begin po_rhs_r <= po_rhs; st <= S_RHS3b; end   // latch rho_x*vx
            S_RHS3b: begin
                if (ldl_rhs_ready_w) begin
                    ldl_rhs_in <= so_rhs; ldl_rhs_valid <= 1;   // rho_x*vx - A^Tvy
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
                if (EXT_COEF) begin
                    coef_start <= 1; coef_we <= 0; coef_addr <= COEF_RY_BASE + r;
                    st <= S_ZY0W;
                end else begin
                    ry_reg <= r_y_arr[r]; dy_reg <= Dy_arr[r];
                    own_addr <= AD_AZ + r; st <= S_ZY1;
                end
            end
            S_ZY0W: begin
                if (coef_done) begin
                    ry_reg <= coef_rdata;
                    coef_start <= 1; coef_we <= 0; coef_addr <= COEF_DY_BASE + r;
                    st <= S_ZY0D;
                end
            end
            S_ZY0D: begin
                if (coef_done) begin
                    dy_reg <= coef_rdata;
                    own_addr <= AD_AZ + r; st <= S_ZY1;
                end
            end
            S_ZY1: begin st <= S_ZY1R; end   // sync-read wait: kmem rdata for AD_AZ+r
            S_ZY1R: begin az_r <= ram_rdata; own_addr <= r; st <= S_ZY2; end
            S_ZY2: begin st <= S_ZY2R; end   // sync-read wait: kmem rdata for r
            S_ZY2R: begin vy_r <= ram_rdata; st <= S_ZY3; end
            S_ZY3: begin zy1_r <= po_zy1; st <= S_ZY3b; end      // latch r_y*vy
            S_ZY3b: begin zy2_r <= so_zy1; st <= S_ZY4; end      // latch az + r_y*vy
            S_ZY4: begin
                own_addr <= AD_AZ + r; own_wdata <= po_zy2; own_we <= 1;   // Dy*(az+r_y*vy)
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
