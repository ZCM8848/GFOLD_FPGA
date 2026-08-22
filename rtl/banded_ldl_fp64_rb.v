`timescale 1ns/1ps
// banded_ldl_fp64_rb: Stage-B banded LDL^T solver, IEEE-754 FP64, with RUNTIME
// BAND INPUT (Phase 2 — the refactorizable LDL). Same FSM/arithmetic as the
// validated banded_ldl_fp64_rt; the only change: the S band is STREAMED IN
// ((HB+1)*N words on band_valid) instead of loaded via $readmemh, so the
// adaptive-scaling / adaptive-TOF loop can re-factorize with a freshly built
// S band (Phase 3 s_build) every scale change / tof candidate.
// Protocol: pulse start, stream (HB+1)*N band words (band_valid, 1/cycle),
// then N rhs words (rhs_valid, 1/cycle). Solve is self-timed; then N zx words
// stream out (zx_valid, 1/cycle), then done pulses.
module banded_ldl_fp64_rb #(parameter N = 8, HB = 2) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    // runtime band input (streamed)
    input  wire [63:0] band_in,
    input  wire        band_valid,
    // runtime rhs input (streamed)
    input  wire [63:0] rhs_in,
    input  wire        rhs_valid,
    // zx output (streamed)
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

    // fp64 units
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

    localparam S_IDLE=0, S_BAND=1, S_RHS=2, S_SETUP=3, S_TDIV_ST=4, S_TDIV_ARM=5,
               S_TDIV_W=6, S_UPDATE=7, S_LCONV_ST=8, S_LCONV_ARM=9, S_LCONV_W=10,
               S_FWD_E=11, S_FWD_L=12, S_DDIV_ST=13, S_DDIV_W=14, S_DDIV_ARM=15,
               S_DDIV_D=16, S_BACK_E=17, S_BACK_L=18, S_ZOUT=19, S_DONE=20;
    reg [4:0] st;
    reg [15:0] k, i_off, j, i_, wp, zo;
    reg [63:0] d, t, acc;
    wire [15:0] r_cur = k + i_off;

    assign mul_a = (st==S_UPDATE) ? B[k*(HB+1) + (j-k)] :
                   (st==S_FWD_L)  ? B[(k-i_)*(HB+1) + i_] :
                   (st==S_BACK_L) ? B[k*(HB+1) + i_] : 64'h0;
    assign mul_b = (st==S_UPDATE) ? t :
                   (st==S_FWD_L)  ? y[k - i_] :
                   (st==S_BACK_L) ? x[k + i_] : 64'h0;
    assign add_a = (st==S_UPDATE) ? B[r_cur*(HB+1) + (j-r_cur)] :
                   (st==S_FWD_L)  ? acc :
                   (st==S_BACK_L) ? acc : 64'h0;
    assign add_b = (st==S_UPDATE) ? mul_o :
                   (st==S_FWD_L)  ? mul_o :
                   (st==S_BACK_L) ? mul_o : 64'h0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; zx_valid <= 0; status <= 0;
            div_start <= 0; add_sub <= 1;
            k <= 0; i_off <= 0; j <= 0; i_ <= 0; wp <= 0; zo <= 0;
            acc <= 0; t <= 0; d <= 0; zx_out <= 0;
        end else begin
            div_start <= 0; zx_valid <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_BAND; end
            end
            // ---- stream (HB+1)*N band words into B[] (sequential write) ----
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
                    if (wp + 1 >= N) begin k <= 0; st <= S_SETUP; end
                    else wp <= wp + 1;
                end
            end
            S_SETUP: begin d <= B[k*(HB+1)]; i_off <= 1; st <= S_TDIV_ST; end
            S_TDIV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    div_a <= B[k*(HB+1) + i_off]; div_b <= d;
                    div_start <= 1; st <= S_TDIV_ARM;
                end else begin i_off <= 1; st <= S_LCONV_ST; end
            end
            S_TDIV_ARM: st <= S_TDIV_W;
            S_TDIV_W: begin
                if (div_done) begin t <= div_o; j <= k + i_off; st <= S_UPDATE; end
            end
            S_UPDATE: begin
                add_sub <= 1;
                B[r_cur*(HB+1) + (j-r_cur)] <= add_o;
                if (j >= k + HB || j >= N - 1) begin i_off <= i_off + 1; st <= S_TDIV_ST; end
                else j <= j + 1;
            end
            S_LCONV_ST: begin
                if (i_off <= HB && (k + i_off) < N) begin
                    div_a <= B[k*(HB+1) + i_off]; div_b <= d;
                    div_start <= 1; st <= S_LCONV_ARM;
                end else begin
                    if (k + 1 >= N) st <= S_FWD_E; else begin k <= k + 1; st <= S_SETUP; end
                end
            end
            S_LCONV_ARM: st <= S_LCONV_W;
            S_LCONV_W: begin
                if (div_done) begin B[k*(HB+1) + i_off] <= div_o; i_off <= i_off + 1; st <= S_LCONV_ST; end
            end
            S_FWD_E: begin k <= 0; st <= S_FWD_L; i_ <= 1; acc <= rhs[0]; end
            S_FWD_L: begin
                if (i_ <= HB && i_ <= k) begin
                    add_sub <= 1;
                    acc <= add_o; i_ <= i_ + 1;
                end else begin
                    y[k] <= acc;
                    if (k + 1 >= N) st <= S_DDIV_ST;
                    else begin k <= k + 1; i_ <= 1; acc <= rhs[k + 1]; st <= S_FWD_L; end
                end
            end
            S_DDIV_ST: begin k <= 0; st <= S_DDIV_W; end
            S_DDIV_W: begin
                div_a <= y[k]; div_b <= B[k*(HB+1)];
                div_start <= 1; st <= S_DDIV_ARM;
            end
            S_DDIV_ARM: st <= S_DDIV_D;
            S_DDIV_D: begin
                if (div_done) begin
                    y[k] <= div_o;
                    if (k + 1 >= N) st <= S_BACK_E; else begin k <= k + 1; st <= S_DDIV_W; end
                end
            end
            S_BACK_E: begin k <= N - 1; st <= S_BACK_L; i_ <= 1; acc <= y[N - 1]; end
            S_BACK_L: begin
                if (i_ <= HB && (k + i_) < N) begin
                    add_sub <= 1;
                    acc <= add_o; i_ <= i_ + 1;
                end else begin
                    x[k] <= acc; zx[k] <= acc;
                    if (k == 0) begin zo <= 0; st <= S_ZOUT; end
                    else begin k <= k - 1; i_ <= 1; acc <= y[k - 1]; st <= S_BACK_L; end
                end
            end
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
