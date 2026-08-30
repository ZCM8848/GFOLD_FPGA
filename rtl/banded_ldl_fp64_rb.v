`timescale 1ns/1ps
// banded_ldl_fp64_rb: Stage-B banded LDL^T solver, IEEE-754 FP64, with RUNTIME
// BAND INPUT + REFACTOR MODE (Phase 2 + 4b).
// Protocol: pulse start with refactor held; if refactor=1 stream (HB+1)*N
// band words (band_valid + band_ready handshake); then N rhs words (rhs_valid
// + rhs_ready); self-timed solve; N zx words out (zx_valid, 1 per ~6 cycles);
// done pulses.
//
// SYNTH FIXES (2026-08-25):
//  - Rev4: ALL array reads through ONE clocked read port per array -> Quartus
//    infers block RAM (combinational reads exploded elaboration at N=1100).
//  - Rev5 (A2): the arrays (B/rhs/y/x/zx = 24200 x 64-bit = 193KB at N=1100)
//    move to the EXTERNAL SRAM via the sram64_ctrl word interface (same bridge
//    as Anderson). Access pattern: set sram_waddr/sram_wdata/sram_we, pulse
//    sram_start, wait sram_done (rdata valid for reads). band_ready/rhs_ready
//    back-pressure the upstream streaming (1 word per ~6 cycles now).
module banded_ldl_fp64_rb #(
    parameter N = 8, HB = 2,
    parameter AA_BASE = 0   // SRAM word offset of LDL arrays (must be past the Anderson region)
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire       refactor,     // 1: re-factorize (stream band); 0: reuse L/D
    // runtime band input (streamed, only when refactor=1)
    input  wire [63:0] band_in,
    input  wire        band_valid,
    output wire        band_ready,  // LDL can accept one band word now (comb)
    // runtime rhs input (streamed)
    input  wire [63:0] rhs_in,
    input  wire        rhs_valid,
    output wire         rhs_ready,   // LDL can accept one rhs word now (comb)
    // zx output (streamed)
    output reg  [63:0] zx_out,
    output reg         zx_valid,
    output reg         done,
    output reg  [7:0]  status,
    // ---- external SRAM (64-bit word interface via sram64_ctrl) ----
    output reg         sram_req, sram_we,
    output reg  [17:0] sram_waddr,
    output reg  [63:0] sram_wdata,
    input  wire        sram_busy,
    input  wire [63:0] sram_rdata
);
    // ---------- SRAM address map ----------
    // ---- SRAM layout (word addresses; offset by AA_BASE to clear the Anderson region) ----
    localparam B_BASE   = AA_BASE;
    localparam RHS_BASE = AA_BASE + (HB+1)*N;
    localparam Y_BASE   = RHS_BASE + N;
    localparam X_BASE   = Y_BASE + N;
    localparam ZX_BASE  = X_BASE + N;
    localparam TOT      = ZX_BASE + N;      // (HB+1)*N + 4N = 24200 @ N=1100

    // ---------- SRAM access handshake (sub-FSM) ----------
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

    // fp64 units (mul/add combinational, div sequential)
    wire [63:0] mul_a, mul_b, mul_o;
    fp64_mul um(mul_a, mul_b, mul_o);
    wire [63:0] add_a, add_b, add_o;
    reg add_sub;
    fp64_add ua(add_a, add_b, add_sub, add_o);
    reg div_start;
    reg [63:0] div_a, div_b;
    wire div_done;
    wire [63:0] div_o;
    fp64_div udiv(clk, div_start, div_a, div_b, div_done, div_o);

    // pipelined operand regs (sampled from SRAM reads)
    reg [63:0] mul_a_r, mul_b_r, add_a_b, rd_tmp;
    assign mul_a = mul_a_r;
    assign mul_b = mul_b_r;
    assign add_b = mul_o;

    localparam S_IDLE=0, S_BAND=1, S_BANDW=2, S_RHS=3, S_RHSW=4,
               S_SETUP0=5, S_SETUP1=6, S_SETUP2=7,
               S_TDIV_ST=8, S_TDIV_ST1=9, S_TDIV_ST2=10, S_TDIV_ARM=11, S_TDIV_W=12,
               S_UPDATE0=13, S_UPDATE1=14, S_UPDATE2=15, S_UPDATE3=16, S_UPDATE4=17,
               S_UPDATE5=18, S_UPDATE6=19,
               S_LCONV_ST=20, S_LCONV_ST1=21, S_LCONV_ST2=22, S_LCONV_ARM=23,
               S_LCONV_W=24, S_LCONV_W2=25,
               S_FWD_E=26, S_FWD_E1=27, S_FWD_E2=28,
               S_FWD_L0=29, S_FWD_L1=30, S_FWD_L4=31, S_FWD_L3=32,
               S_FWD_LW=33, S_FWD_L2=34, S_FWD_L5=35,
               S_DDIV_ST=36, S_DDIV_W=37, S_DDIV_W1=38, S_DDIV_W2=39,
               S_DDIV_ARM=40, S_DDIV_D=41, S_DDIV_DW=42,
               S_BACK_E=43, S_BACK_E1=44, S_BACK_E2=45,
               S_BACK_L0=46, S_BACK_L1=47, S_BACK_L4=48, S_BACK_L3=49,
               S_BACK_LX=50, S_BACK_LZ=51, S_BACK_L2=52, S_BACK_L5=53,
               S_ZOUT=54, S_ZOUTW=55, S_DONE=56;
    reg [5:0] st;
    reg [15:0] k, i_off, j, i_, wp, zo;
    reg [63:0] d, t, acc;
    wire [15:0] r_cur = k + i_off;
    // ---- moved after state/reg declarations (resolve forward references) ----
    wire [63:0] add_a_w = (st == S_UPDATE5) ? add_a_b : acc;
    assign add_a = add_a_w;
    assign band_ready = (st == S_BAND);
    assign rhs_ready  = (st == S_RHS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; zx_valid <= 0; status <= 0;
            div_start <= 0; add_sub <= 1;
            k <= 0; i_off <= 0; j <= 0; i_ <= 0; wp <= 0; zo <= 0;
            acc <= 0; t <= 0; d <= 0; zx_out <= 0;
            mul_a_r <= 0; mul_b_r <= 0; add_a_b <= 0; rd_tmp <= 0;
            sram_start <= 0; sram_we <= 0; sram_waddr <= 0; sram_wdata <= 0;
        end else begin
            div_start <= 0; zx_valid <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    wp <= 0;
                    // band is pre-loaded into SRAM B[] by s_build (refactor) or
                    // reused from the last factorization (no refactor); it is no
                    // longer streamed in — go straight to rhs.
                    k <= 0; st <= S_RHS;
                end
            end
            // ---- stream (HB+1)*N band words into B[] (1 word per ~6 cyc) ----
            S_BAND: begin
                if (band_valid) begin
                    sram_waddr <= B_BASE + wp; sram_wdata <= band_in; sram_we <= 1;
                    sram_start <= 1; st <= S_BANDW;
                end
            end
            S_BANDW: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (wp + 1 >= (HB+1)*N) begin wp <= 0; st <= S_RHS; end
                    else begin wp <= wp + 1; st <= S_BAND; end
                end
            end
            // ---- stream N rhs words into rhs[] ----
            S_RHS: begin
                if (rhs_valid) begin
                    sram_waddr <= RHS_BASE + wp; sram_wdata <= rhs_in; sram_we <= 1;
                    sram_start <= 1; st <= S_RHSW;
                end
            end
            S_RHSW: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (wp + 1 >= N) begin
                        k <= 0;
                        if (refactor) st <= S_SETUP0;
                        else st <= S_FWD_E;
                    end else begin wp <= wp + 1; st <= S_RHS; end
                end
            end
            // ---- factorize ----
            S_SETUP0: begin
                sram_waddr <= B_BASE + k*(HB+1); sram_we <= 0;
                sram_start <= 1; i_off <= 1; st <= S_SETUP1;
            end
            S_SETUP1: begin
                sram_start <= 0;
                if (sram_done) st <= S_SETUP2;
            end
            S_SETUP2: begin d <= sram_rdata; st <= S_TDIV_ST; end
            S_TDIV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    sram_waddr <= B_BASE + k*(HB+1) + i_off; sram_we <= 0;
                    sram_start <= 1; st <= S_TDIV_ST1;
                end else begin i_off <= 1; st <= S_LCONV_ST; end
            end
            S_TDIV_ST1: begin
                sram_start <= 0;
                if (sram_done) st <= S_TDIV_ST2;
            end
            S_TDIV_ST2: begin
                div_a <= sram_rdata; div_b <= d;
                div_start <= 1; st <= S_TDIV_ARM;
            end
            S_TDIV_ARM: st <= S_TDIV_W;
            S_TDIV_W: begin
                if (div_done) begin t <= div_o; j <= k + i_off; st <= S_UPDATE0; end
            end
            // ---- rank-1 update: B[r_cur][j-r_cur] -= B[k][j-k] * t ----
            S_UPDATE0: begin
                sram_waddr <= B_BASE + k*(HB+1) + (j-k); sram_we <= 0;
                sram_start <= 1; mul_b_r <= t; st <= S_UPDATE1;   // read B[k][j-k]
            end
            S_UPDATE1: begin
                sram_start <= 0;
                if (sram_done) st <= S_UPDATE2;
            end
            S_UPDATE2: begin
                mul_a_r <= sram_rdata;
                sram_waddr <= B_BASE + r_cur*(HB+1) + (j-r_cur); sram_we <= 0;
                sram_start <= 1; st <= S_UPDATE3;   // read B[r_cur][j-r_cur]
            end
            S_UPDATE3: begin
                sram_start <= 0;
                if (sram_done) st <= S_UPDATE4;
            end
            S_UPDATE4: begin
                add_a_b <= sram_rdata; st <= S_UPDATE5;
            end
            S_UPDATE5: begin
                add_sub <= 1;
                sram_waddr <= B_BASE + r_cur*(HB+1) + (j-r_cur); sram_wdata <= add_o; sram_we <= 1;
                sram_start <= 1; st <= S_UPDATE6;   // write B[r_cur][j-r_cur]
            end
            S_UPDATE6: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (j >= k + HB || j >= N - 1) begin i_off <= i_off + 1; st <= S_TDIV_ST; end
                    else begin j <= j + 1; st <= S_UPDATE0; end
                end
            end
            S_LCONV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    sram_waddr <= B_BASE + k*(HB+1) + i_off; sram_we <= 0;
                    sram_start <= 1; st <= S_LCONV_ST1;
                end else begin
                    if (k + 1 >= N) st <= S_FWD_E; else begin k <= k + 1; st <= S_SETUP0; end
                end
            end
            S_LCONV_ST1: begin
                sram_start <= 0;
                if (sram_done) st <= S_LCONV_ST2;
            end
            S_LCONV_ST2: begin
                div_a <= sram_rdata; div_b <= d;
                div_start <= 1; st <= S_LCONV_ARM;
            end
            S_LCONV_ARM: st <= S_LCONV_W;
            S_LCONV_W: begin
                if (div_done) begin
                    sram_waddr <= B_BASE + k*(HB+1) + i_off; sram_wdata <= div_o; sram_we <= 1;
                    sram_start <= 1; st <= S_LCONV_W2;
                end
            end
            S_LCONV_W2: begin
                sram_start <= 0;
                if (sram_done) begin i_off <= i_off + 1; st <= S_LCONV_ST; end
            end
            // ---- forward solve: acc = rhs[k] - sum L[k][k-i]*y[k-i] ----
            S_FWD_E: begin
                k <= 0; i_ <= 1;
                sram_waddr <= RHS_BASE; sram_we <= 0;
                sram_start <= 1; st <= S_FWD_E1;    // read rhs[0]
            end
            S_FWD_E1: begin
                sram_start <= 0;
                if (sram_done) st <= S_FWD_E2;
            end
            S_FWD_E2: begin acc <= sram_rdata; st <= S_FWD_L0; end
            S_FWD_L0: begin
                if (i_ <= HB && i_ <= k) begin
                    sram_waddr <= B_BASE + (k-i_)*(HB+1) + i_; sram_we <= 0;
                    sram_start <= 1; st <= S_FWD_L1;   // read L[k][k-i]
                end else begin
                    sram_waddr <= Y_BASE + k; sram_wdata <= acc; sram_we <= 1;
                    sram_start <= 1; st <= S_FWD_LW;   // write y[k]
                end
            end
            S_FWD_L1: begin
                sram_start <= 0;
                if (sram_done) begin
                    mul_a_r <= sram_rdata;
                    sram_waddr <= Y_BASE + (k-i_); sram_we <= 0;
                    sram_start <= 1; st <= S_FWD_L4;   // read y[k-i]
                end
            end
            S_FWD_L4: begin
                sram_start <= 0;
                if (sram_done) begin
                    mul_b_r <= sram_rdata; st <= S_FWD_L3;
                end
            end
            S_FWD_L3: begin
                add_sub <= 1;
                acc <= add_o;   // acc - L*y
                i_ <= i_ + 1;
                st <= S_FWD_L0;
            end
            S_FWD_LW: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (k + 1 >= N) st <= S_DDIV_ST;
                    else begin
                        k <= k + 1; i_ <= 1;
                        sram_waddr <= RHS_BASE + k + 1; sram_we <= 0;
                        sram_start <= 1; st <= S_FWD_L2;   // read rhs[k+1]
                    end
                end
            end
            S_FWD_L2: begin
                sram_start <= 0;
                if (sram_done) st <= S_FWD_L5;
            end
            S_FWD_L5: begin acc <= sram_rdata; st <= S_FWD_L0; end
            S_DDIV_ST: begin k <= 0; st <= S_DDIV_W; end
            S_DDIV_W: begin
                sram_waddr <= Y_BASE + k; sram_we <= 0;
                sram_start <= 1; st <= S_DDIV_W1;   // read y[k]
            end
            S_DDIV_W1: begin
                sram_start <= 0;
                if (sram_done) begin
                    rd_tmp <= sram_rdata;
                    sram_waddr <= B_BASE + k*(HB+1); sram_we <= 0;
                    sram_start <= 1; st <= S_DDIV_W2;   // read B[k][0]
                end
            end
            S_DDIV_W2: begin
                sram_start <= 0;
                if (sram_done) begin
                    div_a <= rd_tmp; div_b <= sram_rdata;
                    div_start <= 1; st <= S_DDIV_ARM;
                end
            end
            S_DDIV_ARM: st <= S_DDIV_D;
            S_DDIV_D: begin
                if (div_done) begin
                    sram_waddr <= Y_BASE + k; sram_wdata <= div_o; sram_we <= 1;
                    sram_start <= 1; st <= S_DDIV_DW;   // write y[k]
                end
            end
            S_DDIV_DW: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (k + 1 >= N) st <= S_BACK_E; else begin k <= k + 1; st <= S_DDIV_W; end
                end
            end
            // ---- back solve: x[k] = y[k] - sum L[k+i][k]*x[k+i] ----
            S_BACK_E: begin
                k <= N - 1; i_ <= 1;
                sram_waddr <= Y_BASE + (N-1); sram_we <= 0;
                sram_start <= 1; st <= S_BACK_E1;   // read y[N-1]
            end
            S_BACK_E1: begin
                sram_start <= 0;
                if (sram_done) st <= S_BACK_E2;
            end
            S_BACK_E2: begin acc <= sram_rdata; st <= S_BACK_L0; end
            S_BACK_L0: begin
                if (i_ <= HB && (k + i_) < N) begin
                    sram_waddr <= B_BASE + k*(HB+1) + i_; sram_we <= 0;
                    sram_start <= 1; st <= S_BACK_L1;   // read L[k+i][k]
                end else begin
                    sram_waddr <= X_BASE + k; sram_wdata <= acc; sram_we <= 1;
                    sram_start <= 1; st <= S_BACK_LX;   // write x[k]
                end
            end
            S_BACK_L1: begin
                sram_start <= 0;
                if (sram_done) begin
                    mul_a_r <= sram_rdata;
                    sram_waddr <= X_BASE + (k+i_); sram_we <= 0;
                    sram_start <= 1; st <= S_BACK_L4;   // read x[k+i]
                end
            end
            S_BACK_L4: begin
                sram_start <= 0;
                if (sram_done) begin
                    mul_b_r <= sram_rdata; st <= S_BACK_L3;
                end
            end
            S_BACK_L3: begin
                add_sub <= 1;
                acc <= add_o;   // acc - L*x
                i_ <= i_ + 1;
                st <= S_BACK_L0;
            end
            S_BACK_LX: begin
                sram_start <= 0;
                if (sram_done) begin
                    sram_waddr <= ZX_BASE + k; sram_wdata <= acc; sram_we <= 1;
                    sram_start <= 1; st <= S_BACK_LZ;   // write zx[k]
                end
            end
            S_BACK_LZ: begin
                sram_start <= 0;
                if (sram_done) begin
                    if (k == 0) begin zo <= 0; st <= S_ZOUT; end
                    else begin
                        k <= k - 1; i_ <= 1;
                        sram_waddr <= Y_BASE + k - 1; sram_we <= 0;
                        sram_start <= 1; st <= S_BACK_L2;   // read y[k-1]
                    end
                end
            end
            S_BACK_L2: begin
                sram_start <= 0;
                if (sram_done) st <= S_BACK_L5;
            end
            S_BACK_L5: begin acc <= sram_rdata; st <= S_BACK_L0; end
            S_ZOUT: begin
                sram_waddr <= ZX_BASE + zo; sram_we <= 0;
                sram_start <= 1; st <= S_ZOUTW;
            end
            S_ZOUTW: begin
                sram_start <= 0;
                if (sram_done) begin
                    zx_out <= sram_rdata; zx_valid <= 1;
                    if (zo + 1 >= N) begin zo <= zo + 1; st <= S_DONE; end
                    else begin zo <= zo + 1; st <= S_ZOUT; end
                end
            end
            S_DONE: begin done <= 1; status <= 8'h01; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
