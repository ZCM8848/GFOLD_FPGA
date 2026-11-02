// proj_soc_pool.v — SOC cone projection, pool version.
// sum-of-squares 改为顺序（3 mul → 3 拍），tail 用 1 mul + 1 add。
// rsqrt 保持本地实例（后续与 root_plus 共享）。
`timescale 1ns/1ps
module proj_soc_pool #(parameter DIM = 4) (
    input  wire             clk, rst_n, start,
    input  wire [DIM*64-1:0] v,
    output reg              done,
    output reg  [DIM*64-1:0] o,
    // Pool mul channels (3 for sum-of-squares, 1 for tail = share mul0)
    output reg [63:0] mul0_a, mul0_b, output reg [63:0] mul1_a, mul1_b,
    output reg [63:0] mul2_a, mul2_b,
    input  wire [63:0] mul0_o, mul1_o, mul2_o,
    // Pool add channel
    output reg [63:0] add0_a, add0_b, output reg add0_sub,
    input  wire [63:0] add0_o
);
    wire [63:0] ve [0:DIM-1];
    genvar g; generate for (g = 0; g < DIM; g = g + 1) begin : unp
        assign ve[g] = v[(g+1)*64-1 -: 64];
    end endgenerate

    // rsqrt (local, 8 DSP — TODO: share with root_plus)
    reg  rr_start; reg [63:0] r_acc; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt ur(.clk(clk), .start(rr_start), .x(r_acc), .done(rr_done), .o(rr_o));
    // cmp (LUT only)
    reg [63:0] ca, cb; wire c_le;
    fp64_cmp uc(.a(ca), .b(cb), .al(), .ale(c_le), .ag(), .age(), .ae());

    reg [63:0] vsq1, vsq2, vsq3, s12, acc, t;
    reg [63:0] nx, lam, ratio, nx_plus_t;
    reg [4:0] st; reg [1:0] branch; reg [3:0] j;

    localparam S0=0, S_SQ1=1, S_SQ1W=2, S_SQ2=3, S_SQ2W=4, S_SQ3=5, S_SQ3W=6,
               S_LATCH=7, S_RSQ=8, S_RSQW=9, S_NX=10, S_NXW=11,
               S_CMP=12, S_CMP2=13, S_LAM=14, S_LAMW=15, S_RAT=16, S_RATW=17,
               S_OUTA=18, S_OUTB=19, S_DONE=20;

    // 组合：根据状态驱动 pool 操作数
    always @(*) begin
        mul0_a=0; mul0_b=0; mul1_a=0; mul1_b=0; mul2_a=0; mul2_b=0;
        add0_a=0; add0_b=0; add0_sub=0; rr_start=0; r_acc=0;
        case (st)
        S_SQ1: begin
            mul0_a=ve[1]; mul0_b=ve[1];  // vsq1 = ve[1]^2
            mul1_a=ve[2]; mul1_b=ve[2];  // vsq2 = ve[2]^2
            if (DIM >= 4) begin mul2_a=ve[3]; mul2_b=ve[3]; end // vsq3 = ve[3]^2
        end
        S_SQ2: begin
            add0_a=vsq1; add0_b=vsq2; add0_sub=0;  // s12 = vsq1+vsq2
        end
        S_SQ3: begin
            add0_a=s12; add0_b=vsq3; add0_sub=0;   // acc = s12+vsq3
        end
        S_LATCH: begin rr_start=1; r_acc=acc; end
        S_NX: begin
            mul0_a=acc; mul0_b=rr_o;               // nx = acc*rsqrt(acc)
        end
        S_LAM: begin
            add0_a=nx; add0_b=t; add0_sub=0;       // nx+t
        end
        S_LAMW: begin
            mul0_a=nx_plus_t; mul0_b=64'h3FE0000000000000; // *0.5
        end
        S_RAT: begin
            mul0_a=lam; mul0_b=rr_o;            // lam*rsqrt(acc)
        end
        S_OUTA: begin
            mul0_a=ratio; mul0_b=ve[j];            // ratio*ve[j]
        end
        default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S0; done<=0; rr_start<=0; branch<=0; j<=0;
            vsq1<=0; vsq2<=0; vsq3<=0; s12<=0; acc<=0; t<=0;
            nx<=0; lam<=0; ratio<=0; nx_plus_t<=0;
        end else begin
            rr_start<=0;
            case (st)
            S0: begin
                done<=0;
                if (start) begin t<=ve[0]; st<=S_SQ1; end
            end
            S_SQ1: begin st<=S_SQ1W; end
            S_SQ1W: begin
                vsq1<=mul0_o; vsq2<=mul1_o;
                if (DIM>=4) vsq3<=mul2_o; else vsq3<=0;
                st<=S_SQ2;
            end
            S_SQ2: begin s12<=add0_o; st<=S_SQ2W; end
            S_SQ2W: begin st<=S_SQ3; end
            S_SQ3: begin acc<=add0_o; st<=S_SQ3W; end
            S_SQ3W: begin st<=S_LATCH; end
            S_LATCH: begin st<=S_RSQ; end
            S_RSQ: st<=S_RSQW;
            S_RSQW: if (rr_done) st<=S_NX;
            S_NX: begin nx<=mul0_o; ca<=mul0_o; cb<=t; st<=S_NXW; end
            S_NXW: begin st<=S_CMP; end
            S_CMP: begin
                if (c_le) begin branch<=0; st<=S_DONE; end
                else begin ca<=nx; cb<={t[63]^1'b1,t[62:0]}; st<=S_CMP2; end
            end
            S_CMP2: begin
                if (c_le) begin branch<=1; st<=S_DONE; end
                else begin branch<=2; st<=S_LAM; end
            end
            S_LAM: begin nx_plus_t<=add0_o; st<=S_LAMW; end
            S_LAMW: begin lam<=mul0_o; st<=S_RAT; end
            S_RAT: begin ratio<=mul0_o; j<=DIM-1; st<=S_RATW; end
            S_RATW: begin st<=S_OUTA; end
            S_OUTA: begin st<=S_OUTB; end
            S_OUTB: begin
                o[j*64 +: 64]<=mul0_o;
                if (j>1) begin j<=j-1; st<=S_OUTA; end
                else begin o[0*64 +: 64]<=lam; st<=S_DONE; end
            end
            S_DONE: begin
                if (branch==0) o<=v;
                else if (branch==1) o<={DIM*64{1'b0}};
                done<=1; st<=S0;
            end
            default: st<=S0;
            endcase
        end
    end
endmodule
