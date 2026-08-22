`timescale 1ns/1ps
// smoke: single spmv instance (transpose=0, A x), RAM provided by tb (proj_dual_cone
// pattern: sync write, async read).
module tb_spmv1;
    parameter N = 10, M = 20, NNZ = 41;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, dv = 0; reg [63:0] xin;
    wire [63:0] o; wire ov, dn;
    wire [12:0] ram_addr; wire [63:0] ram_wdata; wire ram_we; wire [63:0] ram_rdata;
    spmv_fp64 #(.N(N),.M(M),.NNZ(NNZ),.transpose(0)) u0(
        .clk(clk),.rst_n(rst_n),.start(start),.x_in(xin),.din_valid(dv),
        .ram_addr(ram_addr),.ram_wdata(ram_wdata),.ram_we(ram_we),.ram_rdata(ram_rdata),
        .out_out(o),.o_valid(ov),.done(dn));
    reg [63:0] mem [0:4095];
    always @(posedge clk) if (ram_we) mem[ram_addr] <= ram_wdata;
    assign ram_rdata = mem[ram_addr];

    reg [63:0] vx [0:31], got [0:31];
    integer k, cnt;
    reg watchdog_done = 0;
    always begin #100000; if (!watchdog_done) begin $display("TIMEOUT: sim stuck"); $finish; end end
    initial begin
        $readmemh("../data/kkt/small/vx.hex", vx);
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        @(negedge clk); start = 1; dv = 0;
        @(negedge clk); start = 0;
        for (k = 0; k < N; k = k + 1) begin @(negedge clk); xin = vx[k]; dv = 1; end
        @(negedge clk); dv = 0;
        cnt = 0; while (cnt < M) begin @(posedge clk); if (ov) begin got[cnt] = o; cnt = cnt + 1; end end
        while (!dn) @(posedge clk);
        $write("CASE 0:");
        for (k = 0; k < M; k = k + 1) $write(" %h", got[k]);
        $write("\n");
        $display("ALL DONE"); $finish;
    end
endmodule
