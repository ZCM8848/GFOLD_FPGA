`timescale 1ns/1ps
// tb_solver_core: load the small2 boot image into the image RAM, pulse start,
// capture the VS1/VS2 words written to vs_ram.
module tb_solver_core;
    parameter N = 10, M = 49, HB = 4, L = 60;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;   // ~25 MHz solver clock

    reg start = 0;
    wire done;
    wire [9:0] img_addr;
    reg  [63:0] img_rdata;
    wire vs_we;
    wire [7:0] vs_waddr;
    wire [63:0] vs_wdata;

    reg [63:0] img_mem [0:1023];
    always @(posedge clk) img_rdata <= img_mem[img_addr];

    solver_core #(.N(N), .M(M), .NNZ(147), .HB(HB), .L(L), .LM1(L-1),
                  .RAM_AW(10), .SRAM_AW(12), .NODE(2),
                  .RY_FILE("../data/kkt/small2/r_y.hex"),
                  .DY_FILE("../data/kkt/small2/Dy.hex")) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .done(done),
        .img_addr(img_addr), .img_rdata(img_rdata),
        .vs_we(vs_we), .vs_waddr(vs_waddr), .vs_wdata(vs_wdata));

    reg [63:0] vs [0:255];
    integer i;
    always @(posedge clk) if (vs_we) vs[vs_waddr] <= vs_wdata;

    initial begin
        for (i = 0; i < 1024; i = i + 1) img_mem[i] = 64'h0;
        $readmemh("../data/kkt/small2/flash_image.txt", img_mem);
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        repeat (4) @(negedge clk);
        start = 1; @(negedge clk); start = 0;
        while (!done) @(posedge clk);
        @(negedge clk);
        $write("VS1:");
        for (i = 0; i < L; i = i + 1) $write(" %h", vs[i]);
        $write("\n");
        $write("VS2:");
        for (i = 0; i < L; i = i + 1) $write(" %h", vs[L + i]);
        $write("\n");
        $display("CORE DONE");
        $finish;
    end
    reg watchdog = 0;
    always begin #(64'd500000000); if (!watchdog) begin $display("TIMEOUT cst=%0d st=%0d", dut.cst, dut.dut.st); $finish; end end
endmodule
