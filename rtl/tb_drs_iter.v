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
    wire [16:0] ram_addr; wire [63:0] ram_wdata; wire ram_we;
    wire [13:0] kkt_addr; wire [63:0] kkt_wdata; wire kkt_we;
    wire done;
    wire band_ready;
    // ---- external SRAM (behavioral model) ----
    wire [19:0] sram_addr; wire [15:0] sram_dq;
    wire sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
    reg [15:0] sram_mem [0:1048575];
    assign sram_dq = (!sram_oe_n && sram_addr < 1048576) ? sram_mem[sram_addr] : 16'hz;
    always @(posedge clk) if (!sram_we_n && sram_addr < 1048576) sram_mem[sram_addr] <= sram_dq;
    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done),
        .band_ready(band_ready),
        .SRAM_ADDR(sram_addr), .SRAM_DQ(sram_dq), .SRAM_CE_N(sram_ce_n),
        .SRAM_OE_N(sram_oe_n), .SRAM_WE_N(sram_we_n), .SRAM_UB_N(sram_ub_n),
        .SRAM_LB_N(sram_lb_n));
    reg [63:0] smem [0:131071];
    // sync read (match top-level for M9K inference; comb read = 276003)
    reg [63:0] ram_rdata;
    always @(posedge clk) ram_rdata <= smem[ram_addr];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    reg [63:0] kmem [0:32767];
    reg [63:0] kkt_rdata;
    always @(posedge clk) kkt_rdata <= kmem[kkt_addr];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;

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
                // on iter>=1) before streaming; LDL now consumes 1 word/~6 cyc
                // so wait for band_ready (back-pressure) per word.
                while (dut.st !== dut.S_KSBAND) @(posedge clk);
                for (kk = 0; kk < (HB+1)*N; kk = kk + 1) begin
                    while (!band_ready) @(negedge clk);
                    @(negedge clk); band_in = band[kk]; band_valid = 1;
                    while (band_ready) @(negedge clk);   // wait until LDL sampled (ready falls)
                    @(negedge clk); band_valid = 0;
                end
                @(negedge clk); band_valid = 0;
            end
            while (!done) @(posedge clk);
        end
    endtask

    // ---- spmv debug probe (residual computation) ----
    always @(posedge clk) begin
        if (dut.u_saty.st != 0 || dut.u_sax.st != 0) begin end
    end

    // ---- sync-read porting debug: print read-state transitions in iter1 (st window) ----
    reg [8:0] prev_st_d = 999;
    always @(posedge clk) begin
        if (dut.st !== prev_st_d && dut.st >= 0 && dut.st <= 300) begin
            if ($time >= 95000000000 && $time <= 104000000000) begin  // iter1: 91ms..104ms
                $display("DST t=%0d st=%0d", $time, dut.st);
            end
            prev_st_d <= dut.st;
        end
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
            $display("ITER %0d done at %0t", k, $time);
            $fflush();
            if (k % 25 == 0 || k >= ITERS - 3) begin
                $write("VS%0d:", k + 1);
                for (i = 0; i < L; i = i + 1) $write(" %h", smem[i]);
                $write("\n");
                $fflush();   // flush stdout so progress is visible in real time
            end
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
