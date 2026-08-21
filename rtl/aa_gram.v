`timescale 1ns/1ps
// aa_gram: build Anderson's regularized Gram matrix + RHS from S and g.
//   G = S^T S + rreg*I   (MEM x MEM, symmetric; upper triangle computed, mirrored)
//   rhs = S^T g          (MEM)
// FP64 truncating, no div/rsqrt (pure dot products). Streamed:
//   load S (MEM*DIM words) then g (DIM words) with din_valid; then output
//   G (MEM*MEM row-major) + rhs (MEM) with o_valid.
// Dot-product pattern (3 cycles/term, latch-then-capture, from chol10):
//   S_x1: pa<=A(p);pb<=B(p) ; S_x2: sa<=gacc;sb<=po ; S_x3: gacc<=so.
module aa_gram #(parameter DIM = 32, parameter MEM = 10) (
    input  wire             clk, rst_n, start,
    input  wire [63:0]      s_in, g_in,
    input  wire [63:0]      rreg_in,
    input  wire             din_valid,
    output reg              done,
    output reg  [63:0]      g_out, r_out,
    output reg              o_valid
);
    reg [63:0] S [0:MEM*DIM-1];
    reg [63:0] garr [0:DIM-1];
    reg [63:0] G [0:MEM*MEM-1];
    reg [63:0] rhs [0:MEM-1];
    reg [31:0] wp;

    reg  [63:0] pa, pb, sa, sb; reg ssub;
    wire [63:0] po, so;
    fp64_mul  up(.a(pa), .b(pb), .o(po));
    fp64_add  us(.a(sa), .b(sb), .sub(ssub), .o(so));
    reg [63:0] gacc;

    reg [31:0] dj, dk, p, wq;

    localparam S_IDLE=0, S_LOAD=1,
               S_G1=2, S_GD1=3, S_GD2=4, S_GD3=5, S_GNEXT=6, S_GDIAG=7, S_GSTORE=8,
               S_R1=9, S_RD1=10, S_RD2=11, S_RD3=12, S_RSTORE=13,
               S_OUT0=14, S_OUT1=15, S_DONE=16;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; wp <= 0; pa <= 0; pb <= 0;
            sa <= 0; sb <= 0; ssub <= 0; gacc <= 0; dj <= 0; dk <= 0; p <= 0; wq <= 0;
        end else begin
            o_valid <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_LOAD; end
            end
            S_LOAD: if (din_valid) begin
                if (wp < MEM*DIM) S[wp] <= s_in;
                else garr[wp - MEM*DIM] <= g_in;
                if (wp + 1 >= MEM*DIM + DIM) begin dj <= 0; dk <= 0; st <= S_G1; end
                else wp <= wp + 1;
            end
            // ---- G[dj][dk] = sum_p S[dj][p]*S[dk][p] (+rreg if dj==dk), upper tri ----
            S_G1: begin gacc <= 64'h0; p <= 0; st <= S_GD1; end
            S_GD1: begin pa <= S[dj*DIM+p]; pb <= S[dk*DIM+p]; st <= S_GD2; end
            S_GD2: begin sa <= gacc; sb <= po; ssub <= 1'b0; st <= S_GD3; end
            S_GD3: begin
                gacc <= so;
                if (p + 1 < DIM) begin p <= p + 1; st <= S_GD1; end
                else st <= S_GNEXT;                 // gacc becomes final next cycle
            end
            S_GNEXT: begin
                if (dj == dk) begin sa <= gacc; sb <= rreg_in; ssub <= 1'b0; st <= S_GDIAG; end
                else st <= S_GSTORE;
            end
            S_GDIAG: begin st <= S_GSTORE; end        // so = gacc + rreg (diag)
            S_GSTORE: begin
                if (dj == dk) begin G[dj*MEM+dk] <= so; end   // diag: gacc + rreg
                else begin G[dj*MEM+dk] <= gacc; G[dk*MEM+dj] <= gacc; end
                if (dk + 1 < MEM) begin dk <= dk + 1; st <= S_G1; end
                else if (dj + 1 < MEM) begin dj <= dj + 1; dk <= dj + 1; st <= S_G1; end
                else begin dj <= 0; st <= S_R1; end          // all G done -> rhs
            end
            // ---- rhs[dj] = sum_p S[dj][p]*g[p] ----
            S_R1: begin gacc <= 64'h0; p <= 0; st <= S_RD1; end
            S_RD1: begin pa <= S[dj*DIM+p]; pb <= garr[p]; st <= S_RD2; end
            S_RD2: begin sa <= gacc; sb <= po; ssub <= 1'b0; st <= S_RD3; end
            S_RD3: begin
                gacc <= so;
                if (p + 1 < DIM) begin p <= p + 1; st <= S_RD1; end
                else st <= S_RSTORE;
            end
            S_RSTORE: begin
                rhs[dj] <= gacc;
                if (dj + 1 < MEM) begin dj <= dj + 1; st <= S_R1; end
                else begin wq <= 0; st <= S_OUT0; end
            end
            // ---- output G (row-major) then rhs ----
            S_OUT0: begin wq <= 0; st <= S_OUT1; end
            S_OUT1: begin
                if (wq < MEM*MEM) begin
                    g_out <= G[wq]; o_valid <= 1;
                    if (wq + 1 >= MEM*MEM) begin wq <= wq + 1; st <= S_OUT1; end
                    else wq <= wq + 1;
                end else begin
                    r_out <= rhs[wq - MEM*MEM]; o_valid <= 1;
                    if (wq + 1 >= MEM*MEM + MEM) begin wq <= wq + 1; st <= S_DONE; end
                    else wq <= wq + 1;
                end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
