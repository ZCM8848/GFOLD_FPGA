`timescale 1ns/1ps
// spmv_fp64: sparse matrix-vector multiply by A (MxN, COO) in FP64, for the
// KKT-solve datapath (Phase 1). Computes (from zero accumulator):
//   transpose=0 : out[rows[k]] += Aval[k] * x[cols[k]]   (A x,  x len N, out len M)
//   transpose=1 : out[cols[k]] += Aval[k] * x[rows[k]]   (A^T x, x len M, out len N)
//
// RAM-PORT design (proj_dual_cone pattern — ModelSim 10.5b hangs on data-derived
// indices into internal unpacked arrays). x and out live in an EXTERNAL RAM that
// the module drives via a single port (sync write, async read):
//     x@[0,LMAX) , out@[LMAX,2*LMAX)   where LMAX = max(N,M)
// Protocol: pulse start, stream LENX x words (din_valid -> writes x to RAM), then
// NNZ scatter-accumulates, then LENO out words stream out. COO loaded from $readmemh.
module spmv_fp64 #(parameter N = 10, M = 20, NNZ = 41,
                   parameter transpose = 0,   // 0: A x ; 1: A^T x
                   parameter AROW_FILE = "../data/kkt/small/Arow.hex",
                   parameter ACOL_FILE = "../data/kkt/small/Acol.hex",
                   parameter AVAL_FILE = "../data/kkt/small/Aval.hex") (
    input  wire         clk, rst_n, start,
    input  wire [63:0]  x_in,
    input  wire         din_valid,
    // external RAM port (sync write, async read)
    output reg  [12:0]  ram_addr,
    output reg  [63:0]  ram_wdata,
    output reg          ram_we,
    input  wire [63:0]  ram_rdata,
    output reg  [63:0]  out_out,
    output reg          o_valid,
    output reg          done
);
    localparam LENX = transpose ? M : N;    // x length
    localparam LENO = transpose ? N : M;    // out length
    localparam LMAX = (N > M) ? N : M;      // max vector length
    localparam OFF  = LMAX;                 // out region offset in RAM
    localparam AW = (M > 2047) ? 12 : 11;   // COO index width

    reg [63:0]   Aval [0:NNZ-1];
    reg [AW-1:0] Arow [0:NNZ-1];
    reg [AW-1:0] Acol [0:NNZ-1];
    initial begin
`ifndef SYNTHESIS
        $readmemh(AROW_FILE, Arow);
`endif
`ifndef SYNTHESIS
        $readmemh(ACOL_FILE, Acol);
`endif
`ifndef SYNTHESIS
        $readmemh(AVAL_FILE, Aval);
`endif
    end

    reg [63:0] pa, pb, sa, sb, cur, xv, acc_new; reg ssub; wire [63:0] po, so;
    fp64_mul um(pa, pb, po);
    fp64_add us(sa, sb, ssub, so);

    reg [31:0] wp, kk, op;
    reg [AW-1:0] acc_r, src_r;

    localparam S_IDLE=0, S_CLR=1, S_X=2, S_A0=3, S_A1=4, S_A2=5, S_A3=6, S_A4=7,
               S_A5=8, S_A6=9, S_OUT0=10, S_OUT1=11, S_DONE=12;
    reg [3:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; ram_we <= 0; ssub <= 0;
            wp <= 0; kk <= 0; op <= 0; acc_r <= 0; src_r <= 0;
            pa <= 0; pb <= 0; sa <= 0; sb <= 0; cur <= 0; xv <= 0; acc_new <= 0;
            ram_addr <= 0; ram_wdata <= 0; out_out <= 0;
        end else begin
            o_valid <= 0; ram_we <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_X; end
            end
            // ---- stream LENX x words into RAM @wp (first, so caller can start
            //      streaming immediately after start) ----
            S_X: begin
                if (din_valid) begin
                    ram_addr <= wp; ram_wdata <= x_in; ram_we <= 1;
                    if (wp + 1 >= LENX) begin op <= 0; st <= S_CLR; end
                    else wp <= wp + 1;
                end
            end
            // ---- clear out[0..LENO-1] (writes RAM @OFF+op; disjoint from x) ----
            S_CLR: begin
                ram_addr <= OFF + op; ram_wdata <= 64'h0; ram_we <= 1;
                if (op + 1 >= LENO) begin kk <= 0; st <= S_A0; end
                else op <= op + 1;
            end
            // ---- out[acc_r] += Aval[kk]*x[src_r] ----
            S_A0: begin
                acc_r <= transpose ? Acol[kk] : Arow[kk];
                src_r <= transpose ? Arow[kk] : Acol[kk];
                st <= S_A1;
            end
            S_A1: begin ram_addr <= src_r; st <= S_A2; end          // read x[src_r]
            S_A2: begin
                xv <= ram_rdata; ram_addr <= OFF + acc_r; st <= S_A3;  // read out[acc_r]
            end
            S_A3: begin
                cur <= ram_rdata; pa <= Aval[kk]; pb <= xv; st <= S_A4;
            end
            S_A4: begin sa <= cur; sb <= po; ssub <= 1'b0; st <= S_A5; end
            S_A5: begin
                acc_new <= so; ram_addr <= OFF + acc_r; ram_wdata <= so; ram_we <= 1;
                st <= S_A6;
            end
            S_A6: begin
                if (kk + 1 >= NNZ) begin op <= 0; st <= S_OUT0; end
                else begin kk <= kk + 1; st <= S_A0; end
            end
            // ---- stream LENO out words (addr set, then async-read next cycle) ----
            S_OUT0: begin ram_addr <= OFF + op; st <= S_OUT1; end
            S_OUT1: begin
                out_out <= ram_rdata; o_valid <= 1;
                if (op + 1 >= LENO) begin op <= op + 1; st <= S_DONE; end
                else begin op <= op + 1; st <= S_OUT0; end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
