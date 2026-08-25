`timescale 1ns/1ps
// aa_gram: streamed Gram-matrix builder G = S'S + rreg*I and rhs = S'g.
// S (MEM*DIM) and garr (DIM) now live in the external SRAM (word interface
// via sram64_ctrl, same handshake as anderson/banded_ldl). G/rhs stay on-chip.
module aa_gram #(parameter DIM = 8, MEM = 10) (
    input  wire         clk, rst_n, start,
    input  wire         din_valid,
    input  wire [63:0]  s_in, g_in,
    input  wire [63:0]  rreg_in,
    output reg          done,
    output reg  [63:0]  g_out, r_out,
    output reg          o_valid,
    // ---- external SRAM word interface ----
    output reg          sram_req, sram_we,
    output reg  [17:0]  sram_waddr,
    output reg  [63:0]  sram_wdata,
    input  wire         sram_busy,
    input  wire [63:0]  sram_rdata
);
    // S/garr live in the ANDERSON SRAM layout (shared): S[p*MEM+col] row-major,
    // garr[p]. No local buffer - read them directly during G/rhs accumulation.
    localparam AA_S_BASE    = 4*DIM;     // anderson S_BASE
    localparam AA_GARR_BASE = 2*DIM;     // anderson GARR_BASE

    reg [63:0] G [0:MEM*MEM-1];
    reg [63:0] rhs [0:MEM-1];
    reg [31:0] wp;

    reg  [63:0] pa, pb, sa, sb; reg ssub;
    wire [63:0] po, so;
    fp64_mul  up(.a(pa), .b(pb), .o(po));
    fp64_add  us(.a(sa), .b(sb), .sub(ssub), .o(so));
    reg [63:0] gacc;

    reg [31:0] dj, dk, p, wq;

    // ---- SRAM access handshake (same as anderson) ----
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

    localparam S_IDLE=0, S_LOAD=1, S_LW=2,
               S_G1=3, S_GD1=4, S_GD1W=5, S_GD1X=6, S_GD1Y=7, S_GD1Z=8,
               S_GD2=9, S_GD3=10, S_GD4=11, S_GNEXT=12, S_GDIAG=13, S_GSTORE=14,
               S_R1=15, S_RD1=16, S_RD1W=17, S_RD1X=18, S_RD2=19, S_RD2W=20,
               S_RD2X=21, S_RD3=22, S_RD4=23, S_RSTORE=24,
               S_OUT0=25, S_OUT1=26, S_DONE=27;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; wp <= 0; pa <= 0; pb <= 0;
            sa <= 0; sb <= 0; ssub <= 0; gacc <= 0; dj <= 0; dk <= 0; p <= 0; wq <= 0;
        end else begin
            o_valid <= 0; sram_start <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin dj <= 0; dk <= 0; st <= S_G1; end
            end
            // ---- G[dj][dk] = sum_p S[dj][p]*S[dk][p] (+rreg if dj==dk), upper tri ----
            S_G1: begin gacc <= 64'h0; p <= 0; st <= S_GD1; end
            S_GD1: begin
                sram_waddr <= AA_S_BASE + p*MEM + dj; sram_we <= 0;
                sram_start <= 1; st <= S_GD1W;
            end
            S_GD1W: begin
                sram_start <= 0;
                if (sram_done) st <= S_GD1X;
            end
            S_GD1X: begin pa <= sram_rdata; st <= S_GD1Y; end
            S_GD1Y: begin
                sram_waddr <= AA_S_BASE + p*MEM + dk; sram_we <= 0;
                sram_start <= 1; st <= S_GD1Z;
            end
            S_GD1Z: begin
                sram_start <= 0;
                if (sram_done) st <= S_GD2;
            end
            S_GD2: begin pb <= sram_rdata; st <= S_GD3; end
            S_GD3: begin
                sa <= gacc; sb <= po; ssub <= 1'b0; st <= S_GD4;
            end
            S_GD4: begin
                gacc <= so;
                if (p + 1 < DIM) begin p <= p + 1; st <= S_GD1; end
                else st <= S_GNEXT;                 // gacc becomes final next cycle
            end
            S_GNEXT: begin
                if (dj == dk) begin sa <= gacc; sb <= rreg_in; ssub <= 1'b0; st <= S_GDIAG; end
                else begin G[dj*MEM+dk] <= gacc; st <= S_GSTORE; end
            end
            S_GDIAG: begin gacc <= so; st <= S_GSTORE; end
            S_GSTORE: begin
                G[dj*MEM+dk] <= gacc;
                if (dk + 1 < MEM) begin dk <= dk + 1; st <= S_G1; end
                else if (dj + 1 < MEM) begin dj <= dj + 1; dk <= 0; st <= S_G1; end
                else begin wq <= 0; st <= S_R1; end
            end
            // ---- rhs[wq] = sum_p S[wq][p]*garr[p] ----
            S_R1: begin gacc <= 64'h0; p <= 0; st <= S_RD1; end
            S_RD1: begin
                sram_waddr <= AA_S_BASE + p*MEM + wq; sram_we <= 0;
                sram_start <= 1; st <= S_RD1W;
            end
            S_RD1W: begin
                sram_start <= 0;
                if (sram_done) st <= S_RD1X;
            end
            S_RD1X: begin pa <= sram_rdata; st <= S_RD2; end
            S_RD2: begin
                sram_waddr <= AA_GARR_BASE + p; sram_we <= 0;
                sram_start <= 1; st <= S_RD2W;
            end
            S_RD2W: begin
                sram_start <= 0;
                if (sram_done) st <= S_RD2X;
            end
            S_RD2X: begin pb <= sram_rdata; st <= S_RD3; end
            S_RD3: begin
                sa <= gacc; sb <= po; ssub <= 1'b0; st <= S_RD4;
            end
            S_RD4: begin
                gacc <= so;
                if (p + 1 < DIM) begin p <= p + 1; st <= S_RD1; end
                else st <= S_RSTORE;                // gacc final next cycle
            end
            S_RSTORE: begin
                rhs[wq] <= gacc;
                if (wq + 1 < MEM) begin wq <= wq + 1; st <= S_R1; end
                else begin wq <= 0; st <= S_OUT0; end
            end
            // ---- stream G (MEM*MEM) then rhs (MEM) out ----
            S_OUT0: begin wq <= 0; st <= S_OUT1; end
            S_OUT1: begin
                if (wq < MEM*MEM) begin
                    g_out <= G[wq]; r_out <= 64'h0; o_valid <= 1;
                end else if (wq < MEM*MEM + MEM) begin
                    g_out <= 64'h0; r_out <= rhs[wq - MEM*MEM]; o_valid <= 1;
                end
                if (wq + 1 >= MEM*MEM + MEM) begin done <= 1; st <= S_DONE; end
                else wq <= wq + 1;
            end
            S_DONE: begin
                done <= 0;
                if (!start) st <= S_IDLE;
            end
            endcase
        end
    end
endmodule
