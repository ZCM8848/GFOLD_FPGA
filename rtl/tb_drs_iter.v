`timescale 1ns/1ps
// TB for drs_iter: two iterations on the FULL reordered problem.
//   iter 0 (refactor=1): v0 -> v1  (FEAS: no normalize, u_t[l-1]=1, u[l-1]=1)
//   iter 1 (refactor=0): v1 -> v2  (normalize + root_plus + max)
// State RAM preloaded: v0 -> v, g.hex -> g, diag_r.hex -> diag_r.
// Dumps "V1:" / "V2:" lines for check_drs.py vs drs_reference v1/v2 hex.
module tb_drs_iter;
    parameter N = 1100, M = 2107, NNZ = 4783, HB = 17;
    parameter L = N + M + 1, LM1 = L - 1;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, refactor = 0, band_valid = 0;
    reg [63:0] band_in;
    reg [15:0] iter = 0;
    wire [14:0] ram_addr; wire [63:0] ram_wdata; wire ram_we; wire [63:0] ram_rdata;
    wire [13:0] kkt_addr; wire [63:0] kkt_wdata; wire kkt_we; wire [63:0] kkt_rdata;
    wire done;
    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done));
    reg [63:0] smem [0:32767];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    assign ram_rdata = smem[ram_addr];
    reg [63:0] kmem [0:16383];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;
    assign kkt_rdata = kmem[kkt_addr];

    reg [63:0] v0 [0:4095], g [0:4095], dr [0:4095], band [0:32767];
    integer k;
    reg watchdog = 0;
    always begin #300000000; if (!watchdog) begin $display("TIMEOUT: sim stuck"); $finish; end end

    task run_iter;
        input [15:0] it;
        input rf;
        begin
            @(negedge clk); start = 1; refactor = rf; iter = it; band_valid = 0;
            @(negedge clk); start = 0;
            if (rf) begin
                // wait until drs_iter is actually in S_KSBAND (after normalize
                // on iter>=1) before streaming — the stream is handshakeless.
                while (dut.st !== dut.S_KSBAND) @(posedge clk);
                for (k = 0; k < (HB+1)*N; k = k + 1) begin @(negedge clk); band_in = band[k]; band_valid = 1; end
                @(negedge clk); band_valid = 0;
            end
            while (!done) @(posedge clk);
        end
    endtask

    initial begin
        $readmemh("../data/kkt/full/v0.hex", v0);
        $readmemh("../data/kkt/full/g.hex", g);
        $readmemh("../data/kkt/full/diag_r.hex", dr);
        $readmemh("../data/kkt/full/band_r.hex", band);
        for (k = 0; k < L; k = k + 1) smem[k] = v0[k];
        for (k = 0; k < LM1; k = k + 1) smem[4*L + k] = g[k];
        for (k = 0; k < L; k = k + 1) smem[4*L + LM1 + k] = dr[k];
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        run_iter(0, 1);
        $display("ITER 0 done (refactor=1)");
        $write("V1:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[k]);
        $write("\n");
        $write("UT1:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[L + k]);
        $write("\n");
        $write("U1:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[2*L + k]);
        $write("\n");
        $write("RSK1:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[3 * L + k]);
        $write("\n");
        run_iter(1, 0);
        $display("ITER 1 done (refactor=0: reuse L/D)");
        $write("V2:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[k]);
        $write("\n");
        $write("UT2:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[L + k]);
        $write("\n");
        $write("U2:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[2 * L + k]);
        $write("\n");
        $write("RSK2:");
        for (k = 0; k < L; k = k + 1) $write(" %h", smem[3 * L + k]);
        $write("\n");
        $display("ALL DONE");
        $finish;
    end
endmodule
