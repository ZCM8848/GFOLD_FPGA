`timescale 1ns/1ps
// banded_ldl_fp64_rb: Stage-B banded LDL^T solver, IEEE-754 FP64, with RUNTIME
// BAND INPUT + REFACTOR MODE (Phase 2 + 4b).
// Protocol: pulse start with refactor held; if refactor=1 stream (HB+1)*N
// band words (band_valid, 1/cycle); then N rhs words (rhs_valid, 1/cycle);
// self-timed solve; N zx words out (zx_valid, 1/cycle); done pulses.
//
// SYNTH FIX (2026-08-25): ALL array reads go through ONE dedicated read port
// per array (`X_rd <= X[X_ra]`, clocked) so Quartus infers 1R1W block RAM
// (combinational variable-index reads exploded elaboration at N=1100).
// PITFALL FIXED IN THIS REV: the read port registers data ONE cycle after the
// address is set, so a state that sets X_ra must be followed by a WAIT state
// before the sampled data is consumed (address@T -> data@T+1 -> consume@T+2).
// Rev3 sampled 1 cycle early (stale data) — masked at iter0 (zero rhs) but
// exploded at the iter25 scale-update KKT (non-zero rhs).
module banded_ldl_fp64_rb #(parameter N = 8, HB = 2) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire       refactor,     // 1: re-factorize (stream band); 0: reuse L/D
    input  wire [63:0] band_in,
    input  wire        band_valid,
    input  wire [63:0] rhs_in,
    input  wire        rhs_valid,
    output reg  [63:0] zx_out,
    output reg         zx_valid,
    output reg         done,
    output reg  [7:0]  status
);
    reg [63:0] B   [0:(HB+1)*N-1];
    reg [63:0] rhs [0:N-1];
    reg [63:0] y   [0:N-1];
    reg [63:0] x   [0:N-1];
    reg [63:0] zx  [0:N-1];

    // ---- ONE read port per array (RAM-inferable: single clocked read point) ----
    reg [15:0] B_ra, Y_ra, R_ra, X_ra;
    reg [63:0] B_rd, Y_rd, R_rd, X_rd;
    always @(posedge clk) begin
        B_rd <= B[B_ra];
        Y_rd <= y[Y_ra];
        R_rd <= rhs[R_ra];
        X_rd <= x[X_ra];
    end

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

    // pipelined operand regs (sampled from the read ports, AFTER the wait cycle)
    reg [63:0] mul_a_r, mul_b_r, add_a_b;
    wire [63:0] add_a_w = (st == S_UPDATE5) ? add_a_b : acc;
    assign mul_a = mul_a_r;
    assign mul_b = mul_b_r;
    assign add_a = add_a_w;
    assign add_b = mul_o;

    localparam S_IDLE=0, S_BAND=1, S_RHS=2,
               S_SETUP0=3, S_SETUP1=4, S_SETUP2=5,
               S_TDIV_ST=6, S_TDIV_ST1=7, S_TDIV_ST2=8, S_TDIV_ARM=9, S_TDIV_W=10,
               S_UPDATE0=11, S_UPDATE1=12, S_UPDATE2=13, S_UPDATE3=14,
               S_UPDATE4=15, S_UPDATE5=16,
               S_LCONV_ST=17, S_LCONV_ST1=18, S_LCONV_ST2=19, S_LCONV_ARM=20,
               S_LCONV_W=21,
               S_FWD_E=22, S_FWD_E1=23, S_FWD_E2=24,
               S_FWD_L0=25, S_FWD_L1=26, S_FWD_L4=27, S_FWD_L3=28,
               S_FWD_L2=29, S_FWD_L5=30,
               S_DDIV_ST=31, S_DDIV_W=32, S_DDIV_W1=33, S_DDIV_W2=34,
               S_DDIV_ARM=35, S_DDIV_D=36,
               S_BACK_E=37, S_BACK_E1=38, S_BACK_E2=39,
               S_BACK_L0=40, S_BACK_L1=41, S_BACK_L4=42, S_BACK_L3=43,
               S_BACK_L2=44, S_BACK_L5=45,
               S_ZOUT=46, S_DONE=47;
    reg [5:0] st;
    reg [15:0] k, i_off, j, i_, wp, zo;
    reg [63:0] d, t, acc;
    wire [15:0] r_cur = k + i_off;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; zx_valid <= 0; status <= 0;
            div_start <= 0; add_sub <= 1;
            k <= 0; i_off <= 0; j <= 0; i_ <= 0; wp <= 0; zo <= 0;
            acc <= 0; t <= 0; d <= 0; zx_out <= 0;
            mul_a_r <= 0; mul_b_r <= 0; add_a_b <= 0;
            B_ra <= 0; Y_ra <= 0; R_ra <= 0; X_ra <= 0;
        end else begin
            div_start <= 0; zx_valid <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin
                    wp <= 0;
                    if (refactor) st <= S_BAND;
                    else begin k <= 0; st <= S_RHS; end
                end
            end
            // ---- stream (HB+1)*N band words into B[] ----
            S_BAND: begin
                if (band_valid) begin
                    B[wp] <= band_in;
                    if (wp + 1 >= (HB+1)*N) begin wp <= 0; st <= S_RHS; end
                    else wp <= wp + 1;
                end
            end
            // ---- stream N rhs words into rhs[] ----
            S_RHS: begin
                if (rhs_valid) begin
                    rhs[wp] <= rhs_in;
                    if (wp + 1 >= N) begin
                        k <= 0;
                        if (refactor) st <= S_SETUP0;
                        else st <= S_FWD_E;
                    end else wp <= wp + 1;
                end
            end
            // ---- factorize ----
            S_SETUP0: begin B_ra <= k*(HB+1); i_off <= 1; st <= S_SETUP1; end
            S_SETUP1: begin st <= S_SETUP2; end      // wait: B_rd now B[k*(HB+1)]
            S_SETUP2: begin d <= B_rd; st <= S_TDIV_ST; end
            S_TDIV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    B_ra <= k*(HB+1) + i_off; st <= S_TDIV_ST1;
                end else begin i_off <= 1; st <= S_LCONV_ST; end
            end
            S_TDIV_ST1: begin st <= S_TDIV_ST2; end  // wait
            S_TDIV_ST2: begin
                div_a <= B_rd; div_b <= d;
                div_start <= 1; st <= S_TDIV_ARM;
            end
            S_TDIV_ARM: st <= S_TDIV_W;
            S_TDIV_W: begin
                if (div_done) begin t <= div_o; j <= k + i_off; st <= S_UPDATE0; end
            end
            // ---- rank-1 update: B[r_cur][j-r_cur] -= B[k][j-k] * t ----
            S_UPDATE0: begin
                B_ra <= k*(HB+1) + (j-k);
                mul_b_r <= t;
                st <= S_UPDATE1;
            end
            S_UPDATE1: begin st <= S_UPDATE2; end    // wait: B_rd = B[k][j-k]
            S_UPDATE2: begin
                mul_a_r <= B_rd;
                B_ra <= r_cur*(HB+1) + (j-r_cur);
                st <= S_UPDATE3;
            end
            S_UPDATE3: begin st <= S_UPDATE4; end    // wait: B_rd = B[r_cur][j-r_cur]
            S_UPDATE4: begin
                add_a_b <= B_rd;
                st <= S_UPDATE5;
            end
            S_UPDATE5: begin
                add_sub <= 1;
                B[r_cur*(HB+1) + (j-r_cur)] <= add_o;
                if (j >= k + HB || j >= N - 1) begin i_off <= i_off + 1; st <= S_TDIV_ST; end
                else begin j <= j + 1; st <= S_UPDATE0; end
            end
            S_LCONV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    B_ra <= k*(HB+1) + i_off; st <= S_LCONV_ST1;
                end else begin
                    if (k + 1 >= N) st <= S_FWD_E; else begin k <= k + 1; st <= S_SETUP0; end
                end
            end
            S_LCONV_ST1: begin st <= S_LCONV_ST2; end // wait
            S_LCONV_ST2: begin
                div_a <= B_rd; div_b <= d;
                div_start <= 1; st <= S_LCONV_ARM;
            end
            S_LCONV_ARM: st <= S_LCONV_W;
            S_LCONV_W: begin
                if (div_done) begin B[k*(HB+1) + i_off] <= div_o; i_off <= i_off + 1; st <= S_LCONV_ST; end
            end
            // ---- forward solve: acc = rhs[k] - sum L[k][k-i]*y[k-i] ----
            S_FWD_E: begin k <= 0; i_ <= 1; R_ra <= 0; st <= S_FWD_E1; end
            S_FWD_E1: begin st <= S_FWD_E2; end      // wait: R_rd = rhs[0]
            S_FWD_E2: begin acc <= R_rd; st <= S_FWD_L0; end
            S_FWD_L0: begin
                if (i_ <= HB && i_ <= k) begin
                    B_ra <= (k-i_)*(HB+1) + i_;
                    Y_ra <= k - i_;
                    st <= S_FWD_L1;
                end else begin
                    y[k] <= acc;
                    if (k + 1 >= N) st <= S_DDIV_ST;
                    else begin k <= k + 1; i_ <= 1; R_ra <= k + 1; st <= S_FWD_L2; end
                end
            end
            S_FWD_L1: begin st <= S_FWD_L4; end      // wait: B_rd/Y_rd valid
            S_FWD_L4: begin
                mul_a_r <= B_rd; mul_b_r <= Y_rd;
                st <= S_FWD_L3;
            end
            S_FWD_L3: begin
                add_sub <= 1;
                acc <= add_o;   // acc - L*y
                i_ <= i_ + 1;
                st <= S_FWD_L0;
            end
            S_FWD_L2: begin st <= S_FWD_L5; end      // wait: R_rd = rhs[k+1]
            S_FWD_L5: begin acc <= R_rd; st <= S_FWD_L0; end
            S_DDIV_ST: begin k <= 0; st <= S_DDIV_W; end
            S_DDIV_W: begin
                Y_ra <= k; B_ra <= k*(HB+1); st <= S_DDIV_W1;
            end
            S_DDIV_W1: begin st <= S_DDIV_W2; end    // wait: Y_rd/B_rd valid
            S_DDIV_W2: begin
                div_a <= Y_rd; div_b <= B_rd;
                div_start <= 1; st <= S_DDIV_ARM;
            end
            S_DDIV_ARM: st <= S_DDIV_D;
            S_DDIV_D: begin
                if (div_done) begin
                    y[k] <= div_o;
                    if (k + 1 >= N) st <= S_BACK_E; else begin k <= k + 1; st <= S_DDIV_W; end
                end
            end
            // ---- back solve: x[k] = y[k] - sum L[k+i][k]*x[k+i] ----
            S_BACK_E: begin k <= N - 1; i_ <= 1; Y_ra <= N - 1; st <= S_BACK_E1; end
            S_BACK_E1: begin st <= S_BACK_E2; end    // wait: Y_rd = y[N-1]
            S_BACK_E2: begin acc <= Y_rd; st <= S_BACK_L0; end
            S_BACK_L0: begin
                if (i_ <= HB && (k + i_) < N) begin
                    B_ra <= k*(HB+1) + i_;
                    X_ra <= k + i_;
                    st <= S_BACK_L1;
                end else begin
                    x[k] <= acc; zx[k] <= acc;
                    if (k == 0) begin zo <= 0; st <= S_ZOUT; end
                    else begin k <= k - 1; i_ <= 1; Y_ra <= k - 1; st <= S_BACK_L2; end
                end
            end
            S_BACK_L1: begin st <= S_BACK_L4; end    // wait: B_rd/X_rd valid
            S_BACK_L4: begin
                mul_a_r <= B_rd; mul_b_r <= X_rd;
                st <= S_BACK_L3;
            end
            S_BACK_L3: begin
                add_sub <= 1;
                acc <= add_o;   // acc - L*x
                i_ <= i_ + 1;
                st <= S_BACK_L0;
            end
            S_BACK_L2: begin st <= S_BACK_L5; end    // wait: Y_rd = y[k-1]
            S_BACK_L5: begin acc <= Y_rd; st <= S_BACK_L0; end
            S_ZOUT: begin
                zx_out <= zx[zo]; zx_valid <= 1;
                if (zo + 1 >= N) begin zo <= zo + 1; st <= S_DONE; end
                else zo <= zo + 1;
            end
            S_DONE: begin done <= 1; status <= 8'h01; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
