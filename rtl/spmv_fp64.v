`timescale 1ns/1ps
// spmv_fp64: sparse matrix-vector multiply by A (MxN, COO) in FP64, for the
// KKT-solve / adaptive-scale datapath. Computes (from zero accumulator):
//   transpose=0 : out[rows[k]] += Aval[k] * x[cols[k]]   (A x,  x len N, out len M)
//   transpose=1 : out[cols[k]] += Aval[k] * x[rows[k]]   (A^T x, x len M, out len N)
//
// RAM-PORT design (proj_dual_cone pattern — ModelSim 10.5b hangs on data-derived
// indices into internal unpacked arrays). x and out live in an EXTERNAL RAM that
// the module drives via a single port (sync write, async read):
//     x@[0,LMAX) , out@[LMAX,2*LMAX)   where LMAX = max(N,M)
//
// COO coefficients are read from EXTERNAL SRAM (sram64_ctrl 64-bit word
// interface), two words per entry:
//     word[2k]   = { Acol[k][15:0], Arow[k][15:0] }   (Arow in bits [15:0], Acol in [31:16])
//     word[2k+1] = Aval[k]
// stored at SRAM word base COO_BASE. This keeps the ~52KB COO out of on-chip M9K
// (which 5 duplicated internal COO ROMs would otherwise overrun on EP4CE115).
//
// Protocol: pulse start, stream LENX x words (din_valid -> writes x to RAM), then
// NNZ scatter-accumulates (COO streamed from SRAM), then LENO out words stream out.
module spmv_fp64 #(parameter N = 10, M = 20, NNZ = 41,
                   parameter transpose = 0,   // 0: A x ; 1: A^T x
                   parameter COO_BASE = 0) (  // SRAM word base of the packed COO
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
    output reg          done,
    // ---- external SRAM (COO coefficients, read-only) ----
    output reg          sram_req, sram_we,
    output reg  [17:0]  sram_waddr,
    output reg  [63:0]  sram_wdata,
    input  wire         sram_busy,
    input  wire [63:0]  sram_rdata
);
    localparam LENX = transpose ? M : N;    // x length
    localparam LENO = transpose ? N : M;    // out length
    localparam LMAX = (N > M) ? N : M;      // max vector length
    localparam OFF  = LMAX;                 // out region offset in RAM

    reg [63:0] pa, pb, sa, sb, cur, xv, acc_new, rowcol, aval; reg ssub;
    wire [63:0] po, so;
    fp64_mul um(pa, pb, po);
    fp64_add us(sa, sb, ssub, so);

    reg [31:0] wp, kk, op;
    reg [15:0] acc_r, src_r;

    // ---- SRAM access handshake (sub-FSM, banded_ldl_fp64_rb pattern) ----
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

    localparam S_IDLE=0, S_X=1, S_CLR=2,
               S_CA0=3, S_CA0W=4, S_CA1=5, S_CA1W=6,
               S_A1=7, S_A1W=8, S_A2=9, S_A2W=10, S_A3=11, S_A4=12,
               S_A5=13, S_A6=14, S_OUT0=15, S_OUT1W=16, S_OUT1=17, S_DONE=18;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0; o_valid <= 0; ram_we <= 0; ssub <= 0;
            wp <= 0; kk <= 0; op <= 0; acc_r <= 0; src_r <= 0;
            pa <= 0; pb <= 0; sa <= 0; sb <= 0; cur <= 0; xv <= 0; acc_new <= 0;
            rowcol <= 0; aval <= 0;
            ram_addr <= 0; ram_wdata <= 0; out_out <= 0;
            sram_start <= 0; sram_we <= 0; sram_waddr <= 0; sram_wdata <= 0;
        end else begin
            o_valid <= 0; ram_we <= 0; sram_start <= 0; sram_we <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_X; end
            end
            // ---- stream LENX x words into RAM @wp ----
            S_X: begin
                if (din_valid) begin
                    ram_addr <= wp; ram_wdata <= x_in; ram_we <= 1;
                    if (wp + 1 >= LENX) begin op <= 0; st <= S_CLR; end
                    else wp <= wp + 1;
                end
            end
            // ---- clear out[0..LENO-1] ----
            S_CLR: begin
                ram_addr <= OFF + op; ram_wdata <= 64'h0; ram_we <= 1;
                if (op + 1 >= LENO) begin kk <= 0; st <= S_CA0; end
                else op <= op + 1;
            end
            // ---- read word[2k] = {Acol, Arow} from SRAM ----
            S_CA0: begin
                sram_waddr <= COO_BASE + (kk << 1); sram_we <= 0; sram_start <= 1;
                st <= S_CA0W;
            end
            S_CA0W: begin
                if (sram_done) begin rowcol <= sram_rdata; st <= S_CA1; end
            end
            // ---- read word[2k+1] = Aval from SRAM ----
            S_CA1: begin
                sram_waddr <= COO_BASE + (kk << 1) + 1; sram_we <= 0; sram_start <= 1;
                st <= S_CA1W;
            end
            S_CA1W: begin
                if (sram_done) begin
                    aval <= sram_rdata;
                    // Arow = rowcol[15:0], Acol = rowcol[31:16]
                    if (transpose) begin acc_r <= rowcol[31:16]; src_r <= rowcol[15:0]; end
                    else           begin acc_r <= rowcol[15:0];  src_r <= rowcol[31:16]; end
                    st <= S_A1;
                end
            end
            // ---- out[acc_r] += aval * x[src_r] ----
            S_A1: begin ram_addr <= src_r; st <= S_A1W; end          // read x[src_r]
            S_A1W: begin st <= S_A2; end                              // wait 1 cyc for sync RAM
            S_A2: begin
                xv <= ram_rdata; ram_addr <= OFF + acc_r; st <= S_A2W;  // read out[acc_r]
            end
            S_A2W: begin st <= S_A3; end
            S_A3: begin
                cur <= ram_rdata; pa <= aval; pb <= xv; st <= S_A4;
            end
            S_A4: begin sa <= cur; sb <= po; ssub <= 1'b0; st <= S_A5; end
            S_A5: begin
                acc_new <= so; ram_addr <= OFF + acc_r; ram_wdata <= so; ram_we <= 1;
                st <= S_A6;
            end
            S_A6: begin
                if (kk + 1 >= NNZ) begin op <= 0; st <= S_OUT0; end
                else begin kk <= kk + 1; st <= S_CA0; end
            end
            // ---- stream LENO out words ----
            S_OUT0: begin ram_addr <= OFF + op; st <= S_OUT1W; end
            S_OUT1W: begin st <= S_OUT1; end
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
