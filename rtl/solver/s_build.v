`timescale 1ns/1ps
// s_build: runtime construction of the Schur band S = rho_x*I + A^T*D_y*A
// (Phase 3 — the "hardware assembler"). Output band B[i][k] = S[k+i][k],
// (HB+1)*N words, written to EXTERNAL SRAM at BAND_SRAM_BASE.
//
// Algorithm: one pass over the COO (row-major: entries sorted by ROW, but NOT
// necessarily by column within a row). Per row r, buffer its entries
// (<= MAXROW), then for every pair (a<=b) accumulate into the lower triangle:
//   c_lo = min(col[a],col[b]); dc = |col[b]-col[a]|
//   B[dc][c_lo] += val[a] * D_y[r] * val[b]   (for dc <= HB)
// The diagonal comes from pairs with a==b (D_y*v^2). rho_x is added to the
// diagonal in a separate init pass.
//
// SRAM-port design: the COO is read from external SRAM (2 x 64-bit words per
// entry: word[2k]={Acol,Arow}, word[2k+1]=Aval at COO_BASE) and the band is
// written to external SRAM (BAND_SRAM_BASE), keeping both out of on-chip M9K.
// D_y is read from the shared RAM by row index (data-derived index) -> it stays
// in the caller's RAM, not an internal array.
// Protocol: pulse start -> done. Band ready in SRAM[BAND_SRAM_BASE ... +(HB+1)*N).
module s_build #(
    parameter N = 10, M = 20, NNZ = 41, HB = 4, MAXROW = 4,
    parameter RAM_AW = 15,
    parameter DY_OFFSET = 0,     // D_y base in the shared RAM (cur_row = DY_OFFSET + row)
    parameter COO_BASE = 0,      // SRAM word base of the packed COO
    parameter BAND_SRAM_BASE = 0 // SRAM word base of the band output
)(
    input  wire          clk, rst_n, start,
    // external RAM port (sync write, async read) — D_y reads only
    output reg  [RAM_AW-1:0] ram_addr,
    output reg  [63:0]   ram_wdata,
    output reg           ram_we,
    input  wire [63:0]   ram_rdata,
    output reg           done,
    // ---- external SRAM (COO read + band write via shared word interface) ----
    output reg           sram_req, sram_we,
    output reg  [17:0]   sram_waddr,
    output reg  [63:0]   sram_wdata,
    input  wire          sram_busy,
    input  wire [63:0]   sram_rdata
);
    localparam RHO_X = 64'h3eb0c6f7a0b5ed8d;   // 1e-6 (bit-exact double)

    // ---- per-row entry buffer (full problem: max 6 entries/row) ----
    reg [15:0] cols [0:MAXROW-1];
    reg [63:0] vals [0:MAXROW-1];

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

    // ---- FSM regs ----
    reg [15:0] kk, wp, cur_row, pa_col, dc_r;
    reg [63:0] rowcol;   // SRAM word {Acol[31:16], Arow[15:0]}
    reg [3:0] nrow, pa, pb;
    reg [63:0] dy_row, band_r, acc_r, t1_r, po_m2_r;
    reg last_row_flag;

    // ---- FP64 units (combinational) ----
    wire [63:0] po_m1, po_m2, so_acc;
    fp64_mul um1(vals[pa], vals[pb], po_m1);
    fp64_mul um2(t1_r, dy_row, po_m2);
    fp64_add uacc(band_r, po_m2_r, 1'b0, so_acc);
    wire [15:0] c_lo_w = (cols[pa] < cols[pb]) ? cols[pa] : cols[pb];
    wire [15:0] c_hi_w = (cols[pa] < cols[pb]) ? cols[pb] : cols[pa];
    wire [15:0] dc_w = c_hi_w - c_lo_w;

    localparam S_IDLE=0, S_CLEAR=1, S_CLEARW=2, S_DIAG=3, S_DIAGW=4,
               S_ROWINIT=5, S_ROWINITW=6, S_ROWINIT2=7,
               S_LOAD=8, S_LOADW=9, S_LOADW2=10,
               S_ROWEND=11, S_ROWEND_W=12, S_ROWEND_DY=13,
               S_PAIR=14, S_PAIR_BAND=15, S_PAIR_BAND_W=16,
                S_PAIR_M2ACC=17, S_PAIR_M2ACCr=24, S_PAIR_W=18, S_PAIR_WW=19,
               S_PAIRADV=20, S_ROWDONE=21, S_NEXTROW=22, S_DONE=23;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 0;
            ram_addr <= 0; ram_wdata <= 0; ram_we <= 0;
            sram_start <= 0; sram_we <= 0; sram_waddr <= 0; sram_wdata <= 0;
            kk <= 0; wp <= 0; cur_row <= 0; pa_col <= 0; dc_r <= 0; rowcol <= 0;
            nrow <= 0; pa <= 0; pb <= 0;
            dy_row <= 0; band_r <= 0; acc_r <= 0; t1_r <= 0; po_m2_r <= 0;
            last_row_flag <= 0;
        end else begin
            ram_we <= 0; sram_start <= 0;
            case (st)
            S_IDLE: begin
                done <= 0;
                if (start) begin wp <= 0; st <= S_CLEAR; end
            end
            // ---- clear whole band region to 0 in SRAM ----
            S_CLEAR: begin
                sram_waddr <= BAND_SRAM_BASE + wp; sram_wdata <= 64'h0; sram_we <= 1; sram_start <= 1;
                st <= S_CLEARW;
            end
            S_CLEARW: begin
                if (sram_done) begin
                    if (wp + 1 >= (HB+1)*N) begin wp <= 0; st <= S_DIAG; end
                    else begin wp <= wp + 1; st <= S_CLEAR; end
                end
            end
            // ---- init diagonal: B[0][k] = rho_x ----
            S_DIAG: begin
                sram_waddr <= BAND_SRAM_BASE + wp*(HB+1); sram_wdata <= RHO_X; sram_we <= 1; sram_start <= 1;
                st <= S_DIAGW;
            end
            S_DIAGW: begin
                if (sram_done) begin
                    if (wp + 1 >= N) begin
                        kk <= 0; nrow <= 0; last_row_flag <= 0;
                        if (NNZ == 0) st <= S_DONE;
                        else begin
                            sram_waddr <= COO_BASE; sram_we <= 0; sram_start <= 1;   // word[0]
                            st <= S_ROWINITW;
                        end
                    end else begin wp <= wp + 1; st <= S_DIAG; end
                end
            end
            // ---- prime the COO scan: read word[0] (Arow+Acol) then word[1] (Aval) ----
            S_ROWINITW: begin
                if (sram_done) begin
                    rowcol <= sram_rdata; cur_row <= sram_rdata[15:0];
                    sram_waddr <= COO_BASE + 1; sram_we <= 0; sram_start <= 1;
                    st <= S_ROWINIT2;
                end
            end
            S_ROWINIT2: begin
                if (sram_done) begin
                    cols[0] <= rowcol[31:16]; vals[0] <= sram_rdata;
                    nrow <= 1; kk <= 1; st <= S_LOAD;
                end
            end
            // ---- scan next entry's Arow/Acol ----
            S_LOAD: begin
                if (kk >= NNZ) begin last_row_flag <= 1; st <= S_ROWEND; end
                else begin
                    sram_waddr <= COO_BASE + (kk << 1); sram_we <= 0; sram_start <= 1;
                    st <= S_LOADW;
                end
            end
            S_LOADW: begin
                if (sram_done) begin
                    rowcol <= sram_rdata;
                    if (nrow > 0 && sram_rdata[15:0] != cur_row) begin
                        st <= S_ROWEND;   // row boundary: rowcol holds next row's first entry
                    end else begin
                        sram_waddr <= COO_BASE + (kk << 1) + 1; sram_we <= 0; sram_start <= 1;
                        st <= S_LOADW2;
                    end
                end
            end
            S_LOADW2: begin
                if (sram_done) begin
                    cols[nrow] <= rowcol[31:16]; vals[nrow] <= sram_rdata;
                    nrow <= nrow + 1; kk <= kk + 1; st <= S_LOAD;
                end
            end
            // ---- row complete: read D_y[cur_row] (shared RAM) ----
            S_ROWEND: begin
                ram_addr <= DY_OFFSET + cur_row; pa <= 0; pb <= 0; st <= S_ROWEND_W;
            end
            S_ROWEND_W: begin st <= S_ROWEND_DY; end
            S_ROWEND_DY: begin dy_row <= ram_rdata; st <= S_PAIR; end
            // ---- pair (pa,pb): read-modify-write band[dc][c_lo] in SRAM ----
            S_PAIR: begin
                if (dc_w > HB) st <= S_PAIRADV;
                else begin
                    pa_col <= c_lo_w; dc_r <= dc_w;
                    sram_waddr <= BAND_SRAM_BASE + c_lo_w*(HB+1) + dc_w; sram_we <= 0; sram_start <= 1;
                    st <= S_PAIR_BAND_W;
                end
            end
            S_PAIR_BAND_W: begin
                if (sram_done) begin band_r <= sram_rdata; t1_r <= po_m1; st <= S_PAIR_M2ACC; end
            end
            S_PAIR_M2ACC: begin po_m2_r <= po_m2; st <= S_PAIR_M2ACCr; end
            S_PAIR_M2ACCr: begin acc_r <= so_acc; st <= S_PAIR_W; end
            S_PAIR_W: begin
                sram_waddr <= BAND_SRAM_BASE + pa_col*(HB+1) + dc_r; sram_wdata <= acc_r; sram_we <= 1; sram_start <= 1;
                st <= S_PAIR_WW;
            end
            S_PAIR_WW: begin
                if (sram_done) st <= S_PAIRADV;
            end
            S_PAIRADV: begin
                if (pb + 1 >= nrow) begin
                    if (pa + 1 >= nrow) st <= S_ROWDONE;
                    else begin pa <= pa + 1; pb <= pa + 1; st <= S_PAIR; end
                end else begin pb <= pb + 1; st <= S_PAIR; end
            end
            S_ROWDONE: begin
                if (last_row_flag) st <= S_DONE;
                else begin
                    // next row's first entry is cached in rowcol; read its Aval
                    cur_row <= rowcol[15:0];
                    sram_waddr <= COO_BASE + (kk << 1) + 1; sram_we <= 0; sram_start <= 1;
                    st <= S_NEXTROW;
                end
            end
            S_NEXTROW: begin
                if (sram_done) begin
                    cols[0] <= rowcol[31:16]; vals[0] <= sram_rdata;
                    nrow <= 1; kk <= kk + 1; st <= S_LOAD;
                end
            end
            S_DONE: begin done <= 1; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
