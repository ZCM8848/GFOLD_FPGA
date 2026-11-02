`timescale 1ns/1ps
// chol10: dense Cholesky solve for Anderson's regularized normal equations.
//   Solves G gamma = rhs, G symmetric pos-def (MEM x MEM), via
//     Cholesky L L^T = G  ->  L z = rhs (fwd)  ->  L^T gamma = z (back).
// FP64 truncating. Streamed: load MEM*MEM G words (row-major) then MEM rhs words,
// then output MEM gamma words. Mirrors np.linalg.cholesky + two triangular solves.
// Dot-product pattern (3 cycles/term, latch-then-capture):
//   S_x1: pa<=A(q);pb<=B(q)   (issue mul)
//   S_x2: sa<=acc; sb<=po     (acc += A(q)B(q); po valid from prior pa/pb)
//   S_x3: acc<=so             (so = acc + product)
module chol10 #(parameter MEM = 10) (
    input  wire             clk, rst_n, start,
    input  wire [63:0]      g_in, r_in,
    input  wire             din_valid,
    output reg              done,
    output reg  [63:0]      gamma_out,
    output reg              o_valid
);
    reg [63:0] G [0:MEM*MEM-1];
    reg [63:0] L [0:MEM*MEM-1];
    reg [63:0] rhs [0:MEM-1];       // rhs then z (fwd overwrites in place)
    reg [63:0] gamma [0:MEM-1];
    reg [31:0] wp;                  // load write pointer

    // ---- shared arithmetic (latch-then-capture) ----
    reg  [63:0] pa, pb, sa, sb; reg ssub;
    wire [63:0] po, so;
    fp64_mul  up(.a(pa), .b(pb), .o(po));
    fp64_add  us(.a(sa), .b(sb), .sub(ssub), .o(so));
    reg rr_start; reg [63:0] rr_in; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt ur(.clk(clk), .start(rr_start), .x(rr_in), .done(rr_done), .o(rr_o));
    reg dv_start; reg [63:0] dva, dvb; wire dv_done; wire [63:0] dvo;
    fp64_div  ud(.clk(clk), .start(dv_start), .a(dva), .b(dvb), .done(dv_done), .o(dvo));
    reg [63:0] lacc;                // running sum accumulator

    reg [31:0] cj, ci, q, i;
    reg [31:0] wq;                  // output pointer

    localparam S_IDLE=0, S_LOAD=1,
               S_CD0=2, S_CD1=3, S_CD2=4, S_CD3=5, S_CDSQ=6, S_CDSQA=7, S_CDSQW=8, S_CDSQ2=9,
               S_OD0=10, S_OD1=11, S_OD2=12, S_OD3=13, S_ODDIV=14, S_ODARMA=15, S_ODDIVW=16,
               S_FW0=17, S_FW1=18, S_FW2=19, S_FW3=20, S_FW4=21, S_FWDIV=22, S_FWDARMA=23, S_FWDIVW=24,
               S_BK0=25, S_BK1=26, S_BK2=27, S_BK3=28, S_BK4=29, S_BKDIV=30, S_BKDARMA=31, S_BKDIVW=32,
               S_OUT0=33, S_OUT1=34, S_DONE=35;
    reg [5:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; wp <= 0; wq <= 0;
            rr_start <= 0; dv_start <= 0; pa <= 0; pb <= 0; sa <= 0; sb <= 0;
            ssub <= 0; lacc <= 0; cj <= 0; ci <= 0; q <= 0; i <= 0;
        end else begin
            rr_start <= 0; dv_start <= 0; o_valid <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_LOAD; end
            end
            S_LOAD: begin
                if (din_valid) begin
                    if (wp < MEM*MEM) G[wp] <= g_in;
                    else rhs[wp - MEM*MEM] <= r_in;
                    if (wp + 1 >= MEM*MEM + MEM) begin cj <= 0; st <= S_CD0; end
                    else wp <= wp + 1;
                end
            end
            // ================= Cholesky, column cj =================
            S_CD0: begin                       // diag: lacc = G[cj][cj] - sum_{q<cj} L[cj][q]^2
                lacc <= G[cj*MEM+cj]; q <= 0;
                if (cj == 0) st <= S_CDSQ; else st <= S_CD1;
            end
            S_CD1: begin pa <= L[cj*MEM+q]; pb <= L[cj*MEM+q]; st <= S_CD2; end
            S_CD2: begin sa <= lacc; sb <= po; ssub <= 1'b1; st <= S_CD3; end   // lacc -= L^2
            S_CD3: begin
                lacc <= so;
                if (q + 1 < cj) begin q <= q + 1; st <= S_CD1; end
                else st <= S_CDSQ;
            end
            S_CDSQ: begin rr_start <= 1; rr_in <= lacc; st <= S_CDSQA; end
            S_CDSQA: if (!rr_done) st <= S_CDSQW;   // wait stale rsqrt done to clear
            S_CDSQW: if (rr_done) begin pa <= lacc; pb <= rr_o; st <= S_CDSQ2; end  // sqrt
            S_CDSQ2: begin
                L[cj*MEM+cj] <= po;
                if (cj + 1 >= MEM) begin i <= 0; st <= S_FW0; end     // Cholesky done
                else begin ci <= cj + 1; st <= S_OD0; end
            end
            // off-diag for column cj
            S_OD0: begin                       // lacc = G[ci][cj] - sum_{q<cj} L[ci][q]*L[cj][q]
                lacc <= G[ci*MEM+cj]; q <= 0;
                if (cj == 0) st <= S_ODDIV; else st <= S_OD1;
            end
            S_OD1: begin pa <= L[ci*MEM+q]; pb <= L[cj*MEM+q]; st <= S_OD2; end
            S_OD2: begin sa <= lacc; sb <= po; ssub <= 1'b1; st <= S_OD3; end
            S_OD3: begin
                lacc <= so;
                if (q + 1 < cj) begin q <= q + 1; st <= S_OD1; end
                else st <= S_ODDIV;
            end
            S_ODDIV: begin dv_start <= 1; dva <= lacc; dvb <= L[cj*MEM+cj]; st <= S_ODARMA; end
            S_ODARMA: if (!dv_done) st <= S_ODDIVW;   // wait stale div done to clear
            S_ODDIVW: if (dv_done) begin
                L[ci*MEM+cj] <= dvo;
                if (ci + 1 >= MEM) begin cj <= cj + 1; st <= S_CD0; end
                else begin ci <= ci + 1; st <= S_OD0; end
            end
            // ================= forward solve: L z = rhs (z overwrites rhs) =================
            S_FW0: begin lacc <= rhs[i]; q <= 0; if (i == 0) st <= S_FWDIV; else st <= S_FW1; end
            S_FW1: begin pa <= L[i*MEM+q]; pb <= rhs[q]; st <= S_FW2; end   // z[q]=rhs[q] already solved
            S_FW2: begin sa <= lacc; sb <= po; ssub <= 1'b1; st <= S_FW3; end
            S_FW3: begin
                lacc <= so;
                if (q + 1 < i) begin q <= q + 1; st <= S_FW1; end
                else st <= S_FWDIV;
            end
            S_FWDIV: begin dv_start <= 1; dva <= lacc; dvb <= L[i*MEM+i]; st <= S_FWDARMA; end
            S_FWDARMA: if (!dv_done) st <= S_FWDIVW;
            S_FWDIVW: if (dv_done) begin
                rhs[i] <= dvo;
                if (i + 1 >= MEM) begin i <= MEM - 1; st <= S_BK0; end
                else begin i <= i + 1; st <= S_FW0; end
            end
            // ================= backward solve: L^T gamma = z =================
            S_BK0: begin lacc <= rhs[i]; q <= MEM - 1; if (i == MEM-1) st <= S_BKDIV; else st <= S_BK1; end
            S_BK1: begin pa <= L[q*MEM+i]; pb <= gamma[q]; st <= S_BK2; end
            S_BK2: begin sa <= lacc; sb <= po; ssub <= 1'b1; st <= S_BK3; end
            S_BK3: begin
                lacc <= so;
                if (q - 1 > i) begin q <= q - 1; st <= S_BK1; end
                else st <= S_BKDIV;
            end
            S_BKDIV: begin dv_start <= 1; dva <= lacc; dvb <= L[i*MEM+i]; st <= S_BKDARMA; end
            S_BKDARMA: if (!dv_done) st <= S_BKDIVW;
            S_BKDIVW: if (dv_done) begin
                gamma[i] <= dvo;
                if (i == 0) begin wq <= 0; st <= S_OUT0; end
                else begin i <= i - 1; st <= S_BK0; end
            end
            // ================= output =================
            S_OUT0: begin wq <= 0; st <= S_OUT1; end
            S_OUT1: begin
                gamma_out <= gamma[wq]; o_valid <= 1;
                if (wq + 1 >= MEM) begin wq <= wq + 1; st <= S_DONE; end
                else begin wq <= wq + 1; st <= S_OUT1; end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
