`timescale 1ns/1ps
// proj_dual_cone: R_y-weighted Moreau projection of the dual block u[n:l-1]
// (SCS proj_dual_cone_r), FP64, in-place over a RAM of M rows.
//   proj_dual_cone_r(x) = proj_cone(-r_y*x)/r_y + x
// With r_y scalar-constant per block this collapses to:
//     zero cone rows    -> x (pass-through)
//     nonneg cone rows  -> max(x,0)
//     SOC blocks        -> proj_soc(-x_block) + x_block   (proj_soc homogeneous)
// Cone layout (parameterized, DE2-115 G-FOLD defaults):
//     rows 0..Z-1              : zero  cone   (pass)
//     rows NONNEG..NONNEG+N-1  : nonneg cone  (max)
//     rows SOC_START..         : N4 soc cones dim DIM4, then N3 cones dim DIM3
// Single-port RAM interface: set addr, read rdata (1 cycle later), assert we+addr
// +wdata to write. Zero rows are left untouched (no RAM traffic).
// NOTE: all 64-bit data use FLATTENED buses + variable part-select (ps_out[j*64+:64],
// xblk_bus[j*64+:64]) -- ModelSim 10.5b returns garbage for variable-indexed reads
// of unpacked arrays, so no unpacked arrays are indexed by a reg here.
module proj_dual_cone #(parameter M = 2107,
                        parameter Z = 706,          // zero cone size
                        parameter NONNEG = 706,     // nonneg start row
                        parameter N = 301,          // nonneg cone size
                        parameter SOC_START = 1007, // first SOC row
                        parameter N4 = 200, DIM4 = 4,
                        parameter N3 = 100, DIM3 = 3) (
    input  wire             clk, rst_n, start,
    // RAM port
    output reg  [11:0]      addr,
    output reg  [63:0]      wdata,
    output reg              we,
    input  wire [63:0]      rdata,
    output reg              done
);
    localparam NBLK = N4 + N3;

    // ---- proj_soc #(DIM=4) instance ----
    reg  ps_start;
    wire [255:0] ps_in;
    wire ps_done;
    wire [255:0] ps_out;
    proj_soc #(.DIM(4)) usoc(.clk(clk), .rst_n(rst_n), .start(ps_start),
                             .v(ps_in), .done(ps_done), .o(ps_out));
    reg [255:0] xblk_bus;          // current SOC block (flattened 4x64)
    reg [1:0]  j;                  // within-block index (0..3)
    // negated block fed to proj_soc (fixed-index slices, reliable)
    wire [63:0] xb0 = xblk_bus[63:0],  xb1 = xblk_bus[127:64],
                xb2 = xblk_bus[191:128], xb3 = xblk_bus[255:192];
    assign ps_in = {{xb3[63]^1'b1, xb3[62:0]}, {xb2[63]^1'b1, xb2[62:0]},
                    {xb1[63]^1'b1, xb1[62:0]}, {xb0[63]^1'b1, xb0[62:0]}};
    // out = ps_out[j] + xblk_bus[j]  (variable part-select, reliable)
    wire [63:0] po_mux  = ps_out[j*64 +: 64];
    wire [63:0] xblk_mux = xblk_bus[j*64 +: 64];
    wire [63:0] oadd_w;
    fp64_add uadd(po_mux, xblk_mux, 1'b0, oadd_w);

    reg [31:0] row;       // row pointer
    reg [31:0] b;         // SOC block index
    reg [31:0] s;         // SOC block start row

    localparam S0=0, S_N=1, S_NRD=2, S_NW=3, S_NWR=4,
               S_SOC=5, S_SRD=6, S_SR=7, S_PROJ=8, S_PW=9, S_PW2=10,
               S_OUT=11, S_OW=12, S_DONE=13;
    reg [3:0] st;
    reg [63:0] maxv;
    reg is_dim3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; we <= 0; ps_start <= 0; row <= 0; b <= 0; s <= 0;
            j <= 0; xblk_bus <= 0; is_dim3 <= 0;
        end else begin
            ps_start <= 0;
            case (st)
            S0: begin
                done <= 0; we <= 0;
                if (start) begin row <= NONNEG; st <= S_N; end
            end
            S_N: begin
                if (row >= SOC_START) begin b <= 0; s <= SOC_START; st <= S_SRD; end
                else begin addr <= row; st <= S_NRD; end
            end
            S_NRD: begin                       // read x at addr (row)
                maxv <= (rdata[63] ? 64'h0 : rdata);   // max(x,0)
                st <= S_NW;
            end
            S_NW: begin                        // write maxv
                we <= 1; addr <= row; wdata <= maxv; st <= S_NWR;
            end
            S_NWR: begin
                we <= 0; row <= row + 1; st <= S_N;
            end
            S_SRD: begin                       // init block read
                is_dim3 <= (b >= N4);
                j <= 0; addr <= s; st <= S_SR;
            end
            S_SR: begin                        // latch rdata -> xblk_bus[j], advance
                xblk_bus[j*64 +: 64] <= rdata;
                if (j >= ((b < N4) ? DIM4-1 : DIM3-1)) begin
                    if (b >= N4) xblk_bus[255:192] <= 64'b0;   // pad 4th comp (index 3) for dim3
                    st <= S_PROJ;
                end else begin
                    j <= j + 1; addr <= addr + 1; st <= S_SR;
                end
            end
            S_PROJ: begin                      // start proj_soc on negated block
                ps_start <= 1; st <= S_PW;
            end
            S_PW: st <= S_PW2;
            S_PW2: if (ps_done) begin
                we <= 0; j <= 0; st <= S_OUT;
            end
            S_OUT: begin                       // write oadd_w to row s+j
                we <= 1; addr <= s + j; wdata <= oadd_w;
                if (j >= ((b < N4) ? DIM4-1 : DIM3-1)) st <= S_OW;
                else begin j <= j + 1; st <= S_OUT; end
            end
            S_OW: begin
                we <= 0;
                if (b + 1 >= NBLK) begin done <= 1; st <= S_DONE; end
                else begin b <= b + 1; s <= s + ((b < N4) ? DIM4 : DIM3); st <= S_SRD; end
            end
            S_DONE: begin done <= 0; st <= S0; end
            default: st <= S0;
            endcase
        end
    end
endmodule
