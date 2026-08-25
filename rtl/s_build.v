`timescale 1ns/1ps
// s_build: runtime construction of the Schur band S = rho_x*I + A^T*D_y*A
// (Phase 3 — the "hardware assembler"). Output band B[i][k] = S[k+i][k],
// (HB+1)*N words, stored in shared RAM at [M, M+(HB+1)*N); D_y at [0, M).
//
// Algorithm: one pass over the COO (row-major: entries sorted by ROW, but NOT
// necessarily by column within a row — the real G-FOLD dynamics rows have
// unsorted columns, e.g. [0,3,11,6,17]). Per row r, buffer its entries
// (<= MAXROW), then for every pair (a<=b) accumulate into the lower triangle:
//   c_lo = min(col[a],col[b]); dc = |col[b]-col[a]|
//   B[dc][c_lo] += val[a] * D_y[r] * val[b]   (for dc <= HB)
// (val[a]*val[b] is symmetric, so buffer order is irrelevant). The diagonal
// comes from pairs with a==b (D_y*v^2). rho_x is added to the diagonal in a
// separate init pass. Sum order differs from numpy's -> the oracle comparison
// tolerance accounts for ~n*ulp rounding differences.
//
// RAM-port design: D_y is read by ROW INDEX (a data-derived index from Arow)
// -> it must live in RAM, NOT an internal array (ModelSim 10.5b hangs on
// data-derived indices into internal unpacked arrays). A COO arrays are
// internal (sequential kk indexing, aa_gram pattern). The band accumulate is
// read-modify-write into RAM.
// Protocol: pulse start -> done. Band ready in RAM[M ... M+(HB+1)*N).
module s_build #(
    parameter N = 10, M = 20, NNZ = 41, HB = 4, MAXROW = 4,
    parameter RAM_AW = 15,
    parameter BAND_OFFSET = 0,   // band output base in the shared RAM
    parameter DY_OFFSET = 0,     // D_y base (else cur_row = DY_OFFSET + row)
    parameter AROW_FILE = "../data/kkt/small/Arow.hex",
    parameter ACOL_FILE = "../data/kkt/small/Acol.hex",
    parameter AVAL_FILE = "../data/kkt/small/Aval.hex"
)(
    input  wire          clk, rst_n, start,
    // external RAM port (sync write, async read)
    output reg  [RAM_AW-1:0] ram_addr,
    output reg  [63:0]   ram_wdata,
    output reg           ram_we,
    input  wire [63:0]   ram_rdata,
    output reg           done
);
    localparam RHO_X = 64'h3eb0c6f7a0b5ed8d;   // 1e-6 (bit-exact double)

    // ---- A COO (internal, sequential kk index — safe) ----
    reg [63:0]   Aval [0:NNZ-1];
    reg [15:0]   Arow [0:NNZ-1];
    reg [15:0]   Acol [0:NNZ-1];
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

    // ---- per-row entry buffer (full problem: max 5 entries/row) ----
    reg [15:0] cols [0:MAXROW-1];
    reg [63:0] vals [0:MAXROW-1];

    // ---- FSM regs (declared BEFORE the fp64 instances that reference them —
    //      an undeclared identifier in a port connection becomes a 1-bit wire) ----
    reg [15:0] kk, wp, cur_row, pa_col, dc_r;
    reg [3:0] nrow, pa, pb;
    reg [63:0] dy_row, band_r, acc_r, t1_r;
    reg last_row_flag;

    // ---- FP64 units (combinational) ----
    wire [63:0] po_m1, po_m2, so_acc;
    fp64_mul um1(vals[pa], vals[pb], po_m1);
    fp64_mul um2(t1_r, dy_row, po_m2);
    fp64_add uacc(band_r, po_m2, 1'b0, so_acc);
    // pair column bounds — min/max so unsorted row buffers still hit the
    // lower triangle: c_lo = min(cols), dc = |cols[pb]-cols[pa]|
    wire [15:0] c_lo_w = (cols[pa] < cols[pb]) ? cols[pa] : cols[pb];
    wire [15:0] c_hi_w = (cols[pa] < cols[pb]) ? cols[pb] : cols[pa];
    wire [15:0] dc_w = c_hi_w - c_lo_w;

    localparam S_IDLE=0, S_CLEAR=1, S_DIAG=2, S_ROWINIT=3, S_LOAD=4,
               S_ROWEND=5, S_ROWEND_W=6, S_ROWEND_DY=7, S_PAIR=8, S_PAIR_BAND=9,
               S_PAIR_BAND_W=10, S_PAIR_M2ACC=11, S_PAIR_W=12, S_PAIRADV=13, S_ROWDONE=14, S_DONE=15;
    reg [3:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0;
            ram_addr <= 0; ram_wdata <= 0; ram_we <= 0;
            kk <= 0; wp <= 0; cur_row <= 0; pa_col <= 0; dc_r <= 0;
            nrow <= 0; pa <= 0; pb <= 0;
            dy_row <= 0; band_r <= 0; acc_r <= 0; t1_r <= 0;
            last_row_flag <= 0;
        end else begin
            ram_we <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_CLEAR; end
            end
            // ---- clear whole band region to 0 (1 write/cycle) ----
            S_CLEAR: begin
                ram_addr <= BAND_OFFSET + wp; ram_wdata <= 64'h0; ram_we <= 1;
                if (wp + 1 >= (HB+1)*N) begin wp <= 0; st <= S_DIAG; end
                else wp <= wp + 1;
            end
            // ---- init diagonal: B[0][k] = rho_x ----
            S_DIAG: begin
                ram_addr <= BAND_OFFSET + wp*(HB+1); ram_wdata <= RHO_X; ram_we <= 1;
                if (wp + 1 >= N) begin kk <= 0; st <= S_ROWINIT; end
                else wp <= wp + 1;
            end
            // ---- prime the COO scan ----
            S_ROWINIT: begin
                nrow <= 0; last_row_flag <= 0;
                if (NNZ == 0) st <= S_DONE;
                else begin cur_row <= Arow[0]; st <= S_LOAD; end
            end
            // ---- accumulate entries of the current row into the buffer ----
            S_LOAD: begin
                if (kk >= NNZ) begin last_row_flag <= 1; st <= S_ROWEND; end
                else if (nrow > 0 && Arow[kk] != cur_row) st <= S_ROWEND;
                else begin
                    if (nrow == 0) cur_row <= Arow[kk];
                    cols[nrow] <= Acol[kk];
                    vals[nrow] <= Aval[kk];
                    nrow <= nrow + 1;
                    kk <= kk + 1;
                end
            end
            // ---- row complete: read D_y[cur_row] (RAM, data-derived row idx) ----
            S_ROWEND: begin
                ram_addr <= DY_OFFSET + cur_row; pa <= 0; pb <= 0; st <= S_ROWEND_W;
            end
            S_ROWEND_W: begin
                st <= S_ROWEND_DY;   // wait 1 cyc for sync RAM read
            end
            S_ROWEND_DY: begin
                dy_row <= ram_rdata; st <= S_PAIR;
            end
            // ---- pair (pa,pb): skip if dc out of [0,HB]; else latch + read band ----
            S_PAIR: begin
                if (dc_w > HB) st <= S_PAIRADV;
                else begin
                    pa_col <= c_lo_w; dc_r <= dc_w;
                    ram_addr <= BAND_OFFSET + c_lo_w*(HB+1) + dc_w;
                    st <= S_PAIR_BAND_W;
                end
            end
            S_PAIR_BAND_W: begin
                st <= S_PAIR_BAND;   // wait 1 cyc for sync RAM read
            end
            S_PAIR_BAND: begin
                band_r <= ram_rdata; t1_r <= po_m1; st <= S_PAIR_M2ACC;
            end
            S_PAIR_M2ACC: begin
                acc_r <= so_acc; st <= S_PAIR_W;
            end
            S_PAIR_W: begin
                ram_addr <= BAND_OFFSET + pa_col*(HB+1) + dc_r;
                ram_wdata <= acc_r; ram_we <= 1;
                st <= S_PAIRADV;
            end
            S_PAIRADV: begin
                if (pb + 1 >= nrow) begin
                    if (pa + 1 >= nrow) st <= S_ROWDONE;
                    else begin pa <= pa + 1; pb <= pa + 1; st <= S_PAIR; end
                end else begin pb <= pb + 1; st <= S_PAIR; end
            end
            S_ROWDONE: begin
                if (last_row_flag) st <= S_DONE;
                else begin nrow <= 0; st <= S_LOAD; end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
