`timescale 1ns/1ps
// TB for proj_dual_cone_rb (reordered node-major layout). Loads pdc_in.hex
// into a RAM, runs the DUT in-place, dumps the RAM to pdc_out_dut.hex.
// compare with software/check_pdc_rb.py vs pdc_out.hex (software oracle).
module tb_proj_dual_cone_rb;
    parameter M = 2107;
    parameter NODE = 100, Z0 = 14, ZP = 7, NP = 3,
              NS4 = 2, DIM4 = 4, NS3 = 1, DIM3 = 3,
              TAIL_NN = 1, TAIL_Z = 6;
    parameter INIT_FILE = "../data/kkt/full/pdc_in.hex";
    parameter OUT_FILE  = "pdc_rb_out.hex";

    reg clk=0, rst_n=0;
    always #5 clk = ~clk;

    wire [11:0] addr; wire [63:0] wdata; wire we; wire [63:0] rdata;
    wire done;
    reg start=0;

    proj_dual_cone_rb #(.M(M), .NODE(NODE), .Z0(Z0), .ZP(ZP), .NP(NP),
                        .NS4(NS4), .DIM4(DIM4), .NS3(NS3), .DIM3(DIM3),
                        .TAIL_NN(TAIL_NN), .TAIL_Z(TAIL_Z)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .addr(addr), .wdata(wdata), .we(we), .rdata(rdata), .done(done));

    reg [63:0] mem [0:4095];
    always @(posedge clk) if (we) mem[addr] <= wdata;
    assign rdata = mem[addr];

    integer i, fd;
    initial begin
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
        fd = $fopen(OUT_FILE, "w");
        for (i = 0; i < M; i = i + 1) $fwrite(fd, "%016h\n", mem[i]);
        $fclose(fd);
        $display("wrote %s", OUT_FILE);
        $finish;
    end
endmodule
