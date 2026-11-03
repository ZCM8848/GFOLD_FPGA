`timescale 1ns/1ps
// tb_drs_small: 2 DRS iterations on the small on-chip instance (N=10, M=63,
// HB=4, L=74), same cone layout as the real G-FOLD problem. Uses the CFI
// flash model with rtl/data/kkt/small2/flash_image.txt.
module tb_drs_small;
    parameter N = 10, M = 49, NNZ = 147, HB = 4;
    parameter L = N + M + 1, LM1 = L - 1;
    parameter integer ITERS = 2;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg start = 0, refactor = 0, band_valid = 0;
    reg [63:0] band_in;
    reg [15:0] iter = 0;
    reg scale_valid = 0; reg [63:0] scale_r = 64'h3FF0000000000000;
    wire [9:0] ram_addr; wire [63:0] ram_wdata; wire ram_we;
    wire [13:0] kkt_addr; wire [63:0] kkt_wdata; wire kkt_we;
    wire done;
    wire band_ready;
    reg [63:0] smem [0:1023];
    reg [63:0] ram_rdata;
    reg [63:0] kmem [0:255];
    reg [63:0] kkt_rdata;
    wire [19:0] sram_addr; wire [15:0] sram_dq;
    wire sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
    reg [15:0] sram_mem [0:16383];
    assign sram_dq = (!sram_oe_n && sram_addr < 16384) ? sram_mem[sram_addr] : 16'hz;
    always @(posedge clk) if (!sram_we_n && sram_addr < 16384) sram_mem[sram_addr] <= sram_dq;
    wire [22:0] fl_addr; wire [7:0] fl_dq;
    wire fl_ce_n, fl_oe_n, fl_we_n, fl_reset_n, fl_wp_n;
    reg fl_ry = 1'b1;
    reg [7:0] fl_mem [0:8*1024-1];
    assign fl_dq = (!fl_ce_n && !fl_oe_n && fl_addr < 8*1024) ? fl_mem[fl_addr] : 8'hz;

    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB), .L(L), .LM1(LM1), .RAM_AW(10),
               .SRAM_AW(12), .NODE(2), .SQRT_L(64'h401efbdeb14f4eda), .RY_FILE("../data/kkt/small2/r_y.hex"), .DY_FILE("../data/kkt/small2/Dy.hex")) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done), .band_ready(band_ready),
        .SRAM_ADDR(sram_addr), .SRAM_DQ(sram_dq), .SRAM_CE_N(sram_ce_n),
        .SRAM_OE_N(sram_oe_n), .SRAM_WE_N(sram_we_n), .SRAM_UB_N(sram_ub_n), .SRAM_LB_N(sram_lb_n),
        .FL_ADDR(fl_addr), .FL_DQ(fl_dq), .FL_CE_N(fl_ce_n), .FL_OE_N(fl_oe_n),
        .FL_WE_N(fl_we_n), .FL_RESET_N(fl_reset_n), .FL_WP_N(fl_wp_n), .FL_RY(fl_ry),
        .boot_req(), .boot_waddr(), .boot_rdata(64'h0), .boot_busy(1'b0), .boot_ready(1'b0));
    always @(posedge clk) ram_rdata <= smem[ram_addr[9:0]];
    always @(posedge clk) if (ram_we) smem[ram_addr[9:0]] <= ram_wdata;
    always @(posedge clk) kkt_rdata <= kmem[kkt_addr[7:0]];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr[7:0]] <= kkt_wdata;

    reg [63:0] flash_img [0:1023];
    integer k, i, fd;
    task run_iter;
        input [15:0] it;
        input rf;
        begin
            @(negedge clk); start = 1; refactor = rf; iter = it; band_valid = 0;
            @(negedge clk); start = 0;
            while (!done) @(posedge clk);
        end
    endtask

    initial begin
        for (k = 0; k < 1024; k = k + 1) begin flash_img[k] = 0; fl_mem[8*k+0]=0; fl_mem[8*k+1]=0; fl_mem[8*k+2]=0; fl_mem[8*k+3]=0; fl_mem[8*k+4]=0; fl_mem[8*k+5]=0; fl_mem[8*k+6]=0; fl_mem[8*k+7]=0; end
        $readmemh("../data/kkt/small2/flash_image.txt", flash_img);
        for (k = 0; k < 1024; k = k + 1) begin
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
        while (dut.st !== 0) @(posedge clk);
        $display("BOOT DONE at %0t", $time);
        for (k = 0; k < ITERS; k = k + 1) begin
            run_iter(k, (k == 0) ? 1 : 0);
            $display("ITER %0d done at %0t", k, $time);
            $write("VS%0d:", k + 1);
            for (i = 0; i < L; i = i + 1) $write(" %h", smem[i]);
            $write("\n");
        end
        $display("ALL DONE");
        $finish;
    end
    reg watchdog = 0;
    always begin #(64'd1000000000); if (!watchdog) begin $display("TIMEOUT st=%0d", dut.st); $finish; end end
endmodule
