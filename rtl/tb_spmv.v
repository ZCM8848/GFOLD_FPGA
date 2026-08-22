`timescale 1ns/1ps
// TB for spmv_fp64 (small). Two instances with separate RAMs: transpose=0 (A x,
// stream vx len N) and transpose=1 (A^T x, stream vy len M). Print CASE 0 / CASE 1.
module tb_spmv;
    parameter N = 10, M = 20, NNZ = 41;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    // u0: transpose=0
    reg start0 = 0, dv0 = 0; reg [63:0] x0;
    wire [63:0] o0; wire ov0, dn0;
    wire [12:0] a0; wire [63:0] w0; wire we0; wire [63:0] r0;
    spmv_fp64 #(.N(N),.M(M),.NNZ(NNZ),.transpose(0)) u0(
        .clk(clk),.rst_n(rst_n),.start(start0),.x_in(x0),.din_valid(dv0),
        .ram_addr(a0),.ram_wdata(w0),.ram_we(we0),.ram_rdata(r0),
        .out_out(o0),.o_valid(ov0),.done(dn0));
    reg [63:0] mem0 [0:4095];
    always @(posedge clk) if (we0) mem0[a0] <= w0;
    assign r0 = mem0[a0];
    // u1: transpose=1
    reg start1 = 0, dv1 = 0; reg [63:0] x1;
    wire [63:0] o1; wire ov1, dn1;
    wire [12:0] a1; wire [63:0] w1; wire we1; wire [63:0] r1;
    spmv_fp64 #(.N(N),.M(M),.NNZ(NNZ),.transpose(1)) u1(
        .clk(clk),.rst_n(rst_n),.start(start1),.x_in(x1),.din_valid(dv1),
        .ram_addr(a1),.ram_wdata(w1),.ram_we(we1),.ram_rdata(r1),
        .out_out(o1),.o_valid(ov1),.done(dn1));
    reg [63:0] mem1 [0:4095];
    always @(posedge clk) if (we1) mem1[a1] <= w1;
    assign r1 = mem1[a1];

    reg [63:0] vx [0:31], vy [0:31], got0 [0:2047], got1 [0:2047];
    integer k, cnt;
    reg watchdog = 0;
    always begin #200000; if (!watchdog) begin $display("TIMEOUT: sim stuck"); $finish; end end
    initial begin
        $readmemh("../data/kkt/small/vx.hex", vx);
        $readmemh("../data/kkt/small/vy.hex", vy);
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        // u0: A x
        @(negedge clk); start0 = 1; dv0 = 0;
        @(negedge clk); start0 = 0;
        for (k = 0; k < N; k = k + 1) begin @(negedge clk); x0 = vx[k]; dv0 = 1; end
        @(negedge clk); dv0 = 0;
        cnt = 0; while (cnt < M) begin @(posedge clk); if (ov0) begin got0[cnt] = o0; cnt = cnt + 1; end end
        while (!dn0) @(posedge clk);
        $write("CASE 0:");
        for (k = 0; k < M; k = k + 1) $write(" %h", got0[k]);
        $write("\n");
        // u1: A^T x
        @(negedge clk); start1 = 1; dv1 = 0;
        @(negedge clk); start1 = 0;
        for (k = 0; k < M; k = k + 1) begin @(negedge clk); x1 = vy[k]; dv1 = 1; end
        @(negedge clk); dv1 = 0;
        cnt = 0; while (cnt < N) begin @(posedge clk); if (ov1) begin got1[cnt] = o1; cnt = cnt + 1; end end
        while (!dn1) @(posedge clk);
        $write("CASE 1:");
        for (k = 0; k < N; k = k + 1) $write(" %h", got1[k]);
        $write("\n");
        $display("ALL DONE"); $finish;
    end
endmodule
