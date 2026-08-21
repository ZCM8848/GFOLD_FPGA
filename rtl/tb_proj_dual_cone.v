`timescale 1ns/1ps
// TB for proj_dual_cone. Loads init hex into a RAM, runs the DUT in-place, then
// dumps the RAM to an out hex for authoritative Python comparison.
module tb_proj_dual_cone;
    // ---- config (override via -g) ----
    parameter M = 2107;
    parameter Z = 706, NONNEG = 706, N = 301, SOC_START = 1007,
              N4 = 200, DIM4 = 4, N3 = 100, DIM3 = 3;
    parameter INIT_FILE = "data/pdc_full_init.hex";
    parameter OUT_FILE  = "data/pdc_full_out.hex";

    reg clk=0, rst_n=0;
    always #5 clk = ~clk;

    wire [11:0] addr; wire [63:0] wdata; wire we; wire [63:0] rdata;
    wire done;
    reg start=0;

    proj_dual_cone #(.M(M), .Z(Z), .NONNEG(NONNEG), .N(N), .SOC_START(SOC_START),
                     .N4(N4), .DIM4(DIM4), .N3(N3), .DIM3(DIM3)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .addr(addr), .wdata(wdata), .we(we), .rdata(rdata), .done(done));

    // ---- RAM (sync write, async read) ----
    reg [63:0] mem [0:4095];
    always @(posedge clk) if (we) mem[addr] <= wdata;
    assign rdata = mem[addr];

    integer i, fd;

    initial begin
        // load init
        $readmemh(INIT_FILE, mem);
        $display("loaded %s (%0d rows)", INIT_FILE, M);
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        start = 1;
        @(negedge clk);
        start = 0;
        while (!done) @(posedge clk);
        $display("DONE");
        // dump out
        fd = $fopen(OUT_FILE, "w");
        for (i = 0; i < M; i = i + 1) $fwrite(fd, "%016h\n", mem[i]);
        $fclose(fd);
        $display("wrote %s", OUT_FILE);
        $finish;
    end
endmodule
