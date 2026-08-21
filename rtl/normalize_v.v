`timescale 1ns/1ps
// normalize_v: v_out[i] = v_in[i] * (sqrtN / ||v||_2)  for i in [0,N).
// 1/||v|| = rsqrt(sum v^2) via fp64_rsqrt. sum-of-squares = 3-cycle MAC loop,
// then rsqrt, then one-per-cycle scale+output. sqrtN (=sqrt(N)) is a control input.
module normalize_v #(parameter N = 8) (
    input  wire             clk, rst_n, start,
    input  wire [N*64-1:0]  v_in,
    input  wire [63:0]      sqrtN,
    output reg              done,
    output reg  [N*64-1:0]  v_out
);
    wire [63:0] vi [0:N-1];
    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : unp
        assign vi[g] = v_in[(g+1)*64-1 -: 64];
    end endgenerate

    reg [63:0] pa, pb, sa, sb; reg ssub; wire [63:0] po, so;
    fp64_mul up(pa, pb, po);
    fp64_add us(sa, sb, ssub, so);
    reg rr_start; reg [63:0] r_acc; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt ur(clk, rr_start, r_acc, rr_done, rr_o);

    reg [15:0] i;
    reg [63:0] acc, scale;

    localparam S0=0, S_ACC_SET=1, S_ACC_ADD=2, S_ACC_LATCH=3, S_RSQ=4, S_RSQW=5,
               S_SCALE=6, S_SCALEW=7, S_OUT=8, S_OUTW=9, S_DONE=10;
    reg [3:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S0; done <= 0; rr_start <= 0; ssub <= 0; i <= 0; acc <= 0;
        end else begin
            rr_start <= 0; ssub <= 0;
            case (st)
            S0: begin
                done <= 0;
                if (start) begin i <= 0; acc <= 0; st <= S_ACC_SET; end
            end
            S_ACC_SET: begin pa <= vi[i]; pb <= vi[i]; st <= S_ACC_ADD; end
            S_ACC_ADD: begin sa <= acc; sb <= po; st <= S_ACC_LATCH; end
            S_ACC_LATCH: begin
                acc <= so;
                if (i + 1 < N) begin i <= i + 1; st <= S_ACC_SET; end
                else begin r_acc <= so; rr_start <= 1; st <= S_RSQ; end
            end
            S_RSQ: st <= S_RSQW;
            S_RSQW: if (rr_done) st <= S_SCALE;
            S_SCALE: begin pa <= sqrtN; pb <= rr_o; st <= S_SCALEW; end
            S_SCALEW: begin scale <= po; i <= 0; st <= S_OUT; end
            S_OUT: begin pa <= scale; pb <= vi[i]; st <= S_OUTW; end
            S_OUTW: begin
                v_out[(i+1)*64-1 -: 64] <= po;
                if (i + 1 >= N) st <= S_DONE;
                else begin i <= i + 1; st <= S_OUT; end
            end
            S_DONE: begin done <= 1; st <= S0; end
            default: st <= S0;
            endcase
        end
    end
endmodule
