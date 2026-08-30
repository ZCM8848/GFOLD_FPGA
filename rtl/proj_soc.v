`timescale 1ns/1ps
// SOC cone projection proj_soc(v), v=[t; x1..x_{DIM-1}], FP64, FLATTENED ports.
//   if ||x|| <= t : out = v ; if ||x|| <= -t : out = 0
//   else: lam=(||x||+t)/2 ; out=[lam; (lam/||x||)*x]   (1/||x|| = rsqrt(nx2))
// nx2 = sum x_i^2 via combinational fp64_mul tree + fp64_add tree (DIM small).
// rsqrt + lam sequential; projected components computed one-per-cycle (S_OUT).
module proj_soc #(parameter DIM = 4) (
    input  wire             clk, rst_n, start,
    input  wire [DIM*64-1:0] v,
    output reg              done,
    output reg  [DIM*64-1:0] o
);
    wire [63:0] ve [0:DIM-1];
    genvar g_unp;
    generate for (g_unp = 0; g_unp < DIM; g_unp = g_unp + 1) begin : unpack
        assign ve[g_unp] = v[(g_unp+1)*64-1 -: 64];
    end endgenerate

    // ---- combinational sum-of-squares tree: nx2 = sum ve[1..DIM-1]^2 ----
    wire [63:0] vsq [1:DIM-1];
    genvar g_sq;
    generate for (g_sq = 1; g_sq < DIM; g_sq = g_sq + 1) begin : sq
        fp64_mul usq(ve[g_sq], ve[g_sq], vsq[g_sq]);
    end endgenerate
    wire [63:0] nx2;
    genvar g_at;
    generate if (DIM >= 3) begin : at
        wire [63:0] t0 [0:DIM-2];
        assign t0[0] = vsq[1];
        for (g_at = 2; g_at < DIM; g_at = g_at + 1) begin : aa
            fp64_add ua(t0[g_at-2], vsq[g_at], 1'b0, t0[g_at-1]);
        end
        assign nx2 = t0[DIM-2];
    end else assign nx2 = vsq[1];
    endgenerate

    // ---- rsqrt ----
    reg rr_start; reg [63:0] r_acc; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt ur(clk, rr_start, r_acc, rr_done, rr_o);

    // ---- shared combinational mul / add / cmp ----
    reg [63:0] pa, pb, sa, sb; reg ssub; wire [63:0] po, so;
    fp64_mul up(pa, pb, po);
    fp64_add us(sa, sb, ssub, so);
    reg [63:0] ca, cb; wire c_le;
    fp64_cmp uc(ca, cb, , c_le, , , );

    reg [63:0] ratio, lam, nx, acc, t;
    reg [63:0] vsq3_r, s12_r;   // pipelined sum-of-squares (meet 50MHz)

    localparam S0=0, S_SQ2=1, S_SQ3=2, S_LATCH=3, S_RSQ=4, S_RSQW=5, S_NX=6, S_NXW=7, S_CMP=8,
               S_CMP2=9, S_LAM=10, S_LAMW=11, S_RAT=12, S_RATW=13, S_OUTA=14, S_OUTB=15, S_DONE=16;
    reg [4:0] st;
    reg [1:0] branch;   // 0=inside(v), 1=outside(0), 2=project
    reg [3:0] j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; rr_start <= 0; ssub <= 0; branch <= 0; j <= 0;
            vsq3_r <= 0; s12_r <= 0;
        end else begin
            rr_start <= 0; ssub <= 0;
            case (st)
            S0: begin
                done <= 0;
                if (start) begin
                    sa <= vsq[1]; sb <= vsq[2]; ssub <= 1'b0;   // add: vsq1+vsq2
                    vsq3_r <= vsq[3]; t <= ve[0]; st <= S_SQ2;
                end
            end
            S_SQ2: begin
                s12_r <= so; sa <= so; sb <= vsq3_r; ssub <= 1'b0; st <= S_SQ3;   // add: (vsq1+vsq2)+vsq3
            end
            S_SQ3: begin acc <= so; st <= S_LATCH; end
            S_LATCH: begin r_acc <= acc; rr_start <= 1; st <= S_RSQ; end
            S_RSQ: st <= S_RSQW;
            S_RSQW: if (rr_done) st <= S_NX;
            S_NX: begin pa <= acc; pb <= rr_o; st <= S_NXW; end   // nx = acc*rsqrt(acc)
            S_NXW: begin
                nx <= po; ca <= po; cb <= t; st <= S_CMP;        // compare nx vs t
            end
            S_CMP: begin
                if (c_le) begin branch <= 2'd0; st <= S_DONE; end   // nx<=t inside
                else begin ca <= nx; cb <= {t[63]^1'b1, t[62:0]}; st <= S_CMP2; end
            end
            S_CMP2: begin
                if (c_le) begin branch <= 2'd1; st <= S_DONE; end   // nx<=-t outside
                else begin branch <= 2'd2; st <= S_LAM; end         // project
            end
            S_LAM: begin sa <= nx; sb <= t; ssub <= 0; st <= S_LAMW; end   // nx+t
            S_LAMW: begin pa <= so; pb <= 64'h3FE0000000000000; st <= S_RAT; end // *0.5
            S_RAT: begin lam <= po; pa <= po; pb <= rr_o; st <= S_RATW; end // lam*rsqrt(acc)
            S_RATW: begin ratio <= po; j <= DIM - 1; st <= S_OUTA; end
            S_OUTA: begin pa <= ratio; pb <= ve[j]; st <= S_OUTB; end  // set mul inputs
            S_OUTB: begin
                // o[j] = po = ratio*ve[j] (from prior cycle); o[0] = lam
                o[j*64 +: 64] <= po;
                if (j > 1) begin j <= j - 1; st <= S_OUTA; end
                else begin o[0*64 +: 64] <= lam; st <= S_DONE; end
            end
            S_DONE: begin
                if (branch == 2'd0) o <= v;                      // inside: copy input
                else if (branch == 2'd1) o <= {DIM*64{1'b0}};    // outside: zero
                done <= 1; st <= S0;
            end
            default: st <= S0;
            endcase
        end
    end
endmodule
