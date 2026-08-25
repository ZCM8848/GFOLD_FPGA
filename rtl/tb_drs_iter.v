`timescale 1ns/1ps
// TB for drs_iter: two iterations on the FULL reordered problem.
//   iter 0 (refactor=1): v0 -> v1  (FEAS: no normalize, u_t[l-1]=1, u[l-1]=1)
//   iter 1 (refactor=0): v1 -> v2  (normalize + root_plus + max)
// State RAM preloaded: v0 -> v, g.hex -> g, diag_r.hex -> diag_r.
// Dumps "V1:" / "V2:" lines for check_drs.py vs drs_reference v1/v2 hex.
module tb_drs_iter;
    parameter N = 1100, M = 2107, NNZ = 4783, HB = 17;
    parameter L = N + M + 1, LM1 = L - 1;
    parameter integer ITERS = 401;   // total iterations to run (Verilator sweep)
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, refactor = 0, band_valid = 0;
    reg [63:0] band_in;
    reg [15:0] iter = 0;
    reg scale_valid = 0; reg [63:0] scale_r = 64'h3FF0000000000000;
    wire [16:0] ram_addr; wire [63:0] ram_wdata; wire ram_we; wire [63:0] ram_rdata;
    wire [13:0] kkt_addr; wire [63:0] kkt_wdata; wire kkt_we; wire [63:0] kkt_rdata;
    wire done;
    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done));
    reg [63:0] smem [0:131071];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    assign ram_rdata = smem[ram_addr];
    reg [63:0] kmem [0:32767];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;
    assign kkt_rdata = kmem[kkt_addr];

    reg [63:0] v0 [0:4095], g [0:4095], dr [0:4095], band [0:32767];
    reg [63:0] cr [0:2047], nb [0:4095], zmask [0:4095];
    integer k, i;
    reg watchdog = 0;
    always begin #300000000000; if (!watchdog) begin $display("TIMEOUT: sim stuck"); $finish; end end

    task run_iter;
        input [15:0] it;
        input rf;
        integer kk;
        begin
            @(negedge clk); start = 1; refactor = rf; iter = it; band_valid = 0;
            @(negedge clk); start = 0;
            if (rf) begin
                // wait until drs_iter is actually in S_KSBAND (after normalize
                // on iter>=1) before streaming — the stream is handshakeless.
                while (dut.st !== dut.S_KSBAND) @(posedge clk);
                for (kk = 0; kk < (HB+1)*N; kk = kk + 1) begin @(negedge clk); band_in = band[kk]; band_valid = 1; end
                @(negedge clk); band_valid = 0;
            end
            while (!done) @(posedge clk);
        end
    endtask

    // ---- spmv debug probe (residual computation) ----
    always @(posedge clk) begin
        if (dut.u_saty.st != 0 || dut.u_sax.st != 0) begin end
    end

    // ---- scale-chain probe: remap div + rinv at iter25 (numeric states) ----
    reg prev_vr0 = 0;
    always @(posedge clk) begin
        if (dut.st == 128 && !prev_vr0)
            $display("ST128 iter=%0d i=%0d rinv=%h scale=%h", dut.iter, dut.i, dut.rinv, dut.scale_cur);
        if (dut.st == 111)
            $display("ST111 iter=%0d rinv=%h scale=%h", dut.iter, dut.rinv, dut.scale_cur);
        prev_vr0 <= (dut.st == 128);
    end

    initial begin
        $readmemh("../data/kkt/full/v0.hex", v0);
        $readmemh("../data/kkt/full/g.hex", g);
        $readmemh("../data/kkt/full/diag_r.hex", dr);
        $readmemh("../data/kkt/full/band_r.hex", band);
        $readmemh("../data/kkt/full/c_r.hex", cr);
        $readmemh("../data/kkt/full/nb_r.hex", nb);
        $readmemh("../data/kkt/full/zmask.hex", zmask);
        for (k = 0; k < L; k = k + 1) smem[k] = v0[k];
        for (k = 0; k < LM1; k = k + 1) smem[4*L + k] = g[k];
        for (k = 0; k < L; k = k + 1) smem[4*L + LM1 + k] = dr[k];
        for (k = 0; k < N; k = k + 1) smem[dut.CB_BASE + k] = cr[k];
        for (k = 0; k < M; k = k + 1) smem[dut.CB_BASE + N + k] = nb[k];
        for (k = 0; k < M; k = k + 1) smem[dut.ZMASK_BASE + k] = zmask[k];
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        // wait for zmask boot load (S_INIT0-2) to finish before the first start
        while (dut.st !== 0) @(posedge clk);
        // generalized: run ITERS iterations, dump every 25 (scale-check iters) + last 3
        for (k = 0; k < ITERS; k = k + 1) begin
            run_iter(k, (k == 0) ? 1 : 0);
            if (k % 25 == 0 || k >= ITERS - 3) begin
                $write("VS%0d:", k + 1);
                for (i = 0; i < L; i = i + 1) $write(" %h", smem[i]);
                $write("\n");
            end
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
