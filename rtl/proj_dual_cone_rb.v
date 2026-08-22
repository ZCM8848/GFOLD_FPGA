`timescale 1ns/1ps
// proj_dual_cone_rb: R_y-weighted Moreau projection of the dual block
// u[n:l-1] in the REORDERED (node-major) frame (Phase 4b). Same math as the
// validated proj_dual_cone.v (zero->pass, nonneg->max(x,0), SOC->proj_soc(-x)+x),
// but the cone layout is the node-interleaved one:
//     [Z0 zero rows]  then per node: [NP nonneg, 2x dim4 SOC, 1x dim3 SOC]
//     with ZP zero rows between nodes; tail: [TAIL_NN nonneg, TAIL_Z zero].
// Default = G-FOLD reordered layout (100 nodes, DE2-115 problem).
// In-place over a single-port RAM (sync write, async read). Zero rows are
// skipped (no RAM traffic).
// NOTE: all 64-bit data use FLATTENED buses + variable part-select (the
// ModelSim 10.5b unpacked-array-indexing pitfall).
module proj_dual_cone_rb #(parameter M = 2107,
                           parameter NODE = 100,      // number of nodes
                           parameter Z0 = 14,         // leading zero block
                           parameter ZP = 7,          // zero rows between nodes
                           parameter NP = 3,          // nonneg rows per node
                           parameter NS4 = 2, DIM4 = 4,   // dim-4 SOC per node
                           parameter NS3 = 1, DIM3 = 3,   // dim-3 SOC per node
                           parameter TAIL_NN = 1,     // trailing nonneg (dry mass)
                           parameter TAIL_Z = 6) (    // trailing zero (final)
    input  wire             clk, rst_n, start,
    // RAM port
    output reg  [11:0]      addr,
    output reg  [63:0]      wdata,
    output reg              we,
    input  wire [63:0]      rdata,
    output reg              done
);
    localparam NSOC = NS4 + NS3;     // soc blocks per node

    // ---- proj_soc #(DIM=4) instance (dim3 handled by zero-padding comp 3) ----
    reg  ps_start;
    wire [255:0] ps_in;
    wire ps_done;
    wire [255:0] ps_out;
    proj_soc #(.DIM(4)) usoc(.clk(clk), .rst_n(rst_n), .start(ps_start),
                             .v(ps_in), .done(ps_done), .o(ps_out));
    reg [255:0] xblk_bus;          // current SOC block (flattened 4x64)
    reg [1:0]  j;                  // within-block index (0..3)
    wire [63:0] xb0 = xblk_bus[63:0],  xb1 = xblk_bus[127:64],
                xb2 = xblk_bus[191:128], xb3 = xblk_bus[255:192];
    assign ps_in = {{xb3[63]^1'b1, xb3[62:0]}, {xb2[63]^1'b1, xb2[62:0]},
                    {xb1[63]^1'b1, xb1[62:0]}, {xb0[63]^1'b1, xb0[62:0]}};
    wire [63:0] po_mux  = ps_out[j*64 +: 64];
    wire [63:0] xblk_mux = xblk_bus[j*64 +: 64];
    wire [63:0] oadd_w;
    fp64_add uadd(po_mux, xblk_mux, 1'b0, oadd_w);

    reg [31:0] row;       // row pointer
    reg [31:0] b;         // SOC block index within node
    reg [31:0] nn;        // nonneg counter within node
    reg [31:0] node;      // node counter
    reg [63:0] maxv;
    reg is_dim3;

    localparam S0=0, S_NN=1, S_NRD=2, S_NW=3, S_NWR=4,
               S_SRD=5, S_SR=6, S_PROJ=7, S_PW=8, S_PW2=9,
               S_OUT=10, S_OW=11, S_TAIL=12, S_TNRD=13, S_TNW=14, S_TNWR=15,
               S_TAILZ=16, S_DONE=17;
    reg [4:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; we <= 0; ps_start <= 0;
            row <= 0; b <= 0; nn <= 0; node <= 0;
            j <= 0; xblk_bus <= 0; maxv <= 0; is_dim3 <= 0;
        end else begin
            ps_start <= 0;
            case (st)
            S0: begin
                done <= 0; we <= 0;
                if (start) begin node <= 0; nn <= 0; row <= Z0; st <= S_NN; end
            end
            // ---- nonneg segment of current node (NP rows): max(x,0) ----
            S_NN: begin
                if (nn >= NP) begin b <= 0; st <= S_SRD; end
                else begin addr <= row; st <= S_NRD; end
            end
            S_NRD: begin
                maxv <= (rdata[63] ? 64'h0 : rdata);
                st <= S_NW;
            end
            S_NW: begin
                we <= 1; addr <= row; wdata <= maxv; st <= S_NWR;
            end
            S_NWR: begin
                we <= 0; row <= row + 1; nn <= nn + 1; st <= S_NN;
            end
            // ---- SOC segment of current node (NSOC blocks) ----
            S_SRD: begin
                is_dim3 <= (b >= NS4);
                j <= 0; addr <= row; st <= S_SR;
            end
            S_SR: begin
                xblk_bus[j*64 +: 64] <= rdata;
                if (j >= ((b < NS4) ? DIM4-1 : DIM3-1)) begin
                    if (b >= NS4) xblk_bus[255:192] <= 64'b0;   // pad dim3
                    st <= S_PROJ;
                end else begin
                    j <= j + 1; addr <= addr + 1; st <= S_SR;
                end
            end
            S_PROJ: begin
                ps_start <= 1; st <= S_PW;
            end
            S_PW: st <= S_PW2;
            S_PW2: if (ps_done) begin
                we <= 0; j <= 0; st <= S_OUT;
            end
            S_OUT: begin
                we <= 1; addr <= row + j; wdata <= oadd_w;
                if (j >= ((b < NS4) ? DIM4-1 : DIM3-1)) st <= S_OW;
                else begin j <= j + 1; st <= S_OUT; end
            end
            S_OW: begin
                we <= 0;
                if (b + 1 >= NSOC) begin
                    if (node + 1 >= NODE) begin
                        // last node done -> tail (no ZP after)
                        row <= row + ((b < NS4) ? DIM4 : DIM3);
                        nn <= 0; st <= S_TAIL;
                    end else if (node + 2 >= NODE) begin
                        // node NODE-2 done -> last node: NO ZP (dyn(NODE-1)
                        // does not exist; there are only NODE-1 transitions)
                        row <= row + ((b < NS4) ? DIM4 : DIM3);
                        node <= node + 1; nn <= 0; st <= S_NN;
                    end else begin
                        row <= row + ((b < NS4) ? DIM4 : DIM3) + ZP;
                        node <= node + 1; nn <= 0; st <= S_NN;
                    end
                end else begin
                    row <= row + ((b < NS4) ? DIM4 : DIM3);
                    b <= b + 1; st <= S_SRD;
                end
            end
            // ---- tail: TAIL_NN nonneg rows (dry mass), then TAIL_Z zero ----
            S_TAIL: begin
                if (nn >= TAIL_NN) begin row <= row + TAIL_Z; st <= S_TAILZ; end
                else begin addr <= row; st <= S_TNRD; end
            end
            S_TNRD: begin maxv <= (rdata[63] ? 64'h0 : rdata); st <= S_TNW; end
            S_TNW: begin we <= 1; addr <= row; wdata <= maxv; st <= S_TNWR; end
            S_TNWR: begin we <= 0; row <= row + 1; nn <= nn + 1; st <= S_TAIL; end
            S_TAILZ: begin st <= S_DONE; end
            S_DONE: begin done <= 1; st <= S0; end
            default: st <= S0;
            endcase
        end
    end
endmodule
