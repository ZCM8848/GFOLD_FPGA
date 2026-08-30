`timescale 1ns/1ps
// TB for drs_iter: boot (Flash -> SRAM/smem, compute band/g/diag_r) + N iters.
//   Flash behavior model preloaded from ../data/flash/flash_image.txt.
//   After boot, band/g/diag_r are compared against band_r.hex/g.hex/diag_r.hex.
//   Then ITERS DRS iterations run, dumping VS<i> lines for the oracle check.
module tb_drs_iter;
    parameter N = 1100, M = 2107, NNZ = 4783, HB = 17;
    parameter L = N + M + 1, LM1 = L - 1;
    parameter integer ITERS = 401;
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
    // ---- state RAM (sync read, mirror top-level for M9K inference) ----
    reg [63:0] smem [0:131071];
    reg [63:0] ram_rdata;
    reg [63:0] kmem [0:32767];
    reg [63:0] kkt_rdata;
    // ---- external SRAM (behavioral model) ----
    wire [19:0] sram_addr; wire [15:0] sram_dq;
    wire sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
    reg [15:0] sram_mem [0:1048575];
    assign sram_dq = (!sram_oe_n && sram_addr < 1048576) ? sram_mem[sram_addr] : 16'hz;
    always @(posedge clk) if (!sram_we_n && sram_addr < 1048576) sram_mem[sram_addr] <= sram_dq;
    // ---- CFI Flash (behavioral model, 8-bit) ----
    wire [22:0] fl_addr; wire [7:0] fl_dq;
    wire fl_ce_n, fl_oe_n, fl_we_n, fl_reset_n, fl_wp_n;
    reg fl_ry = 1'b1;
    reg [7:0] fl_mem [0:303095];   // 37887 words * 8 bytes
    assign fl_dq = (!fl_ce_n && !fl_oe_n && fl_addr < 303096) ? fl_mem[fl_addr] : 8'hz;

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
        .SRAM_LB_N(sram_lb_n),
        .FL_ADDR(fl_addr), .FL_DQ(fl_dq), .FL_CE_N(fl_ce_n), .FL_OE_N(fl_oe_n),
        .FL_WE_N(fl_we_n), .FL_RESET_N(fl_reset_n), .FL_WP_N(fl_wp_n), .FL_RY(fl_ry));
    always @(posedge clk) ram_rdata <= smem[ram_addr];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;
    always @(posedge clk) kkt_rdata <= kmem[kkt_addr];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;

    // ---- reference data for boot verification ----
    reg [63:0] g_ref [0:4095], dr_ref [0:4095], zmask_ref [0:4095], dy_ref [0:4095], c_ref [0:2047], nb_ref [0:4095];
    reg [63:0] coo_ref [0:9565];
    reg [63:0] flash_img [0:37886];
    integer k, i;
    integer boot_cnt = 0;
    always @(posedge clk) if (dut.st == dut.S_BOOT_FW && dut.flash_busy_p && !dut.flash_busy) boot_cnt = boot_cnt + 1;
    reg watchdog = 0;
    always begin #(64'd300000000000); if (!watchdog) begin $display("TIMEOUT: sim stuck at st=%0d", dut.st); $finish; end end

    task run_iter;
        input [15:0] it;
        input rf;
        begin
            @(negedge clk); start = 1; refactor = rf; iter = it; band_valid = 0;
            @(negedge clk); start = 0;
            while (!done) @(posedge clk);
        end
    endtask

    // ---- boot verification: compare computed band/g/diag_r vs reference ----
    task check_boot;
        integer nbad;
        begin
            // NOTE: band in SRAM is overwritten in-place by LDL factorize during the
            // g recompute, so it is not compared here (s_build itself is checked by
            // tb_sbuild_sram). The g recompute result is checked below with tolerance.
            nbad = 0;
            for (k = 0; k < 2*NNZ; k = k + 1) begin
                if ({sram_mem[4*k+3], sram_mem[4*k+2], sram_mem[4*k+1], sram_mem[4*k+0]} !== coo_ref[k]) begin
                    if (nbad < 5) $display("  coo[%0d]: got %h exp %h", k, {sram_mem[4*k+3], sram_mem[4*k+2], sram_mem[4*k+1], sram_mem[4*k+0]}, coo_ref[k]);
                    nbad = nbad + 1;
                end
            end
            $display("BOOT coo: bad=%0d/%0d", nbad, 2*NNZ);
            nbad = 0;
            for (k = 0; k < M; k = k + 1) begin
                if (smem[dut.DY_BASE + k] !== dy_ref[k]) begin
                    if (nbad < 5) $display("  Dy[%0d]: got %h exp %h", k, smem[dut.DY_BASE + k], dy_ref[k]);
                    nbad = nbad + 1;
                end
            end
            $display("BOOT Dy: bad=%0d/%0d", nbad, M);
            nbad = 0;
            for (k = 0; k < N; k = k + 1) if (smem[dut.CB_BASE + k] !== c_ref[k]) nbad = nbad + 1;
            $display("BOOT c: bad=%0d/%0d", nbad, N);
            nbad = 0;
            for (k = 0; k < M; k = k + 1) if (smem[dut.CB_BASE + N + k] !== nb_ref[k]) nbad = nbad + 1;
            $display("BOOT nb: bad=%0d/%0d", nbad, M);
            $display("BOOT flash words read: %0d (expect 37887)", boot_cnt);
            nbad = 0;
            for (k = 0; k < LM1; k = k + 1) if (smem[dut.G_BASE + k] !== g_ref[k]) nbad = nbad + 1;
            $display("BOOT g: bad=%0d/%0d", nbad, LM1);
            nbad = 0;
            for (k = 0; k < M; k = k + 1) begin
                if (dut.zmask_bits[k] !== (zmask_ref[k] != 64'h0)) begin
                    if (nbad < 5) $display("  zmask_bits[%0d]: got %b smem=%h exp %h", k, dut.zmask_bits[k], smem[dut.ZMASK_BASE + k], zmask_ref[k]);
                    nbad = nbad + 1;
                end
            end
            $display("BOOT zmask_bits: bad=%0d/%0d", nbad, M);
            nbad = 0;
            for (k = 0; k < L; k = k + 1) begin
                if (smem[dut.DR_BASE + k] !== dr_ref[k]) begin
                    if (nbad < 5) $display("  diag_r[%0d]: got %h exp %h", k, smem[dut.DR_BASE + k], dr_ref[k]);
                    nbad = nbad + 1;
                end
            end
            $display("BOOT diag_r: bad=%0d/%0d", nbad, L);
        end
    endtask

    initial begin
        $readmemh("../data/kkt/full/g.hex", g_ref);
        $readmemh("../data/kkt/full/diag_r.hex", dr_ref);
        $readmemh("../data/kkt/full/zmask.hex", zmask_ref);
        $readmemh("../data/kkt/full/coo_sram.hex", coo_ref);
        $readmemh("../data/kkt/full/Dy_r.hex", dy_ref);
        $readmemh("../data/kkt/full/c_r.hex", c_ref);
        $readmemh("../data/kkt/full/nb_r.hex", nb_ref);
        // ---- Flash image (bytes from 64-bit words) ----
        $readmemh("../data/flash/flash_image.txt", flash_img);
        for (k = 0; k < 37887; k = k + 1) begin
            fl_mem[8*k+0] = flash_img[k][7:0];
            fl_mem[8*k+1] = flash_img[k][15:8];
            fl_mem[8*k+2] = flash_img[k][23:16];
            fl_mem[8*k+3] = flash_img[k][31:24];
            fl_mem[8*k+4] = flash_img[k][39:32];
            fl_mem[8*k+5] = flash_img[k][47:40];
            fl_mem[8*k+6] = flash_img[k][55:48];
            fl_mem[8*k+7] = flash_img[k][63:56];
        end
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        // wait for boot to finish (Flash load + compute + S_INIT -> S_IDLE)
        while (dut.st !== 0) @(posedge clk);
        $display("BOOT DONE at %0t", $time);
        check_boot;
        // run ITERS iterations
        for (k = 0; k < ITERS; k = k + 1) begin
            run_iter(k, (k == 0) ? 1 : 0);
            $display("ITER %0d done at %0t", k, $time);
            $fflush();
            if (k % 25 == 0 || k >= ITERS - 3) begin
                $write("VS%0d:", k + 1);
                for (i = 0; i < L; i = i + 1) $write(" %h", smem[i]);
                $write("\n");
                $fflush();
            end
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
