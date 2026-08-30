`timescale 1ns/1ps
module tb_sbuild_sram;
    parameter N = 1100, M = 2107, NNZ = 4783, HB = 17, MAXROW = 6;
    localparam BAND_SRAM_BASE = 109072;
    reg clk = 0, rst_n = 0, start = 0;
    always #5 clk = ~clk;
    wire [16:0] ram_addr; wire [63:0] ram_wdata; wire ram_we;
    wire sram_req, sram_we; wire [17:0] sram_waddr; wire [63:0] sram_wdata;
    wire sram_busy; wire [63:0] sram_rdata;
    wire done;

    reg [63:0] smem [0:47568];
    reg [63:0] ram_rdata;
    always @(posedge clk) ram_rdata <= smem[ram_addr];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;

    wire [19:0] sram_addr; wire [15:0] sram_dq;
    wire sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
    reg [15:0] sram_mem [0:1048575];
    assign sram_dq = (!sram_oe_n && sram_addr < 1048576) ? sram_mem[sram_addr] : 16'hz;
    always @(posedge clk) if (!sram_we_n && sram_addr < 1048576) sram_mem[sram_addr] <= sram_dq;

    s_build #(.N(N), .M(M), .NNZ(NNZ), .HB(HB), .MAXROW(MAXROW), .RAM_AW(17),
              .DY_OFFSET(0), .COO_BASE(0), .BAND_SRAM_BASE(BAND_SRAM_BASE)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .sram_req(sram_req), .sram_we(sram_we), .sram_waddr(sram_waddr),
        .sram_wdata(sram_wdata), .sram_busy(sram_busy), .sram_rdata(sram_rdata),
        .done(done));

    sram64_ctrl #(.AW(18)) u_sram(.clk(clk), .rst_n(rst_n), .req(sram_req), .we(sram_we),
        .waddr(sram_waddr), .wdata(sram_wdata), .busy(sram_busy), .rdata(sram_rdata),
        .SRAM_ADDR(sram_addr), .SRAM_DQ(sram_dq), .SRAM_CE_N(sram_ce_n),
        .SRAM_OE_N(sram_oe_n), .SRAM_WE_N(sram_we_n), .SRAM_UB_N(sram_ub_n),
        .SRAM_LB_N(sram_lb_n));

    reg [63:0] coo_word [0:2*NNZ-1];
    reg [63:0] dy [0:M-1];
    reg [63:0] band_ref [0:32767];
    integer k, nbad;
    reg watchdog = 0;
    always begin #(64'd100000000); if (!watchdog) begin $display("TIMEOUT st=%0d", dut.st); $finish; end end

    initial begin
        $readmemh("../data/kkt/full/coo_sram.hex", coo_word);
        for (k = 0; k < 2*NNZ; k = k + 1) begin
            sram_mem[4*k+0] = coo_word[k][15:0];
            sram_mem[4*k+1] = coo_word[k][31:16];
            sram_mem[4*k+2] = coo_word[k][47:32];
            sram_mem[4*k+3] = coo_word[k][63:48];
        end
        $readmemh("../data/kkt/full/Dy_r.hex", dy);
        for (k = 0; k < M; k = k + 1) smem[k] = dy[k];
        $readmemh("../data/kkt/full/band_r.hex", band_ref);
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        repeat (2) @(negedge clk);
        start = 1; @(negedge clk); start = 0;
        while (!done) @(posedge clk);
        watchdog = 1;
        nbad = 0;
        for (k = 0; k < (HB+1)*N; k = k + 1) begin
            if ({sram_mem[4*(BAND_SRAM_BASE+k)+3], sram_mem[4*(BAND_SRAM_BASE+k)+2],
                 sram_mem[4*(BAND_SRAM_BASE+k)+1], sram_mem[4*(BAND_SRAM_BASE+k)+0]} !== band_ref[k]) begin
                if (nbad < 5) $display("  band[%0d]: got %h exp %h", k,
                    {sram_mem[4*(BAND_SRAM_BASE+k)+3], sram_mem[4*(BAND_SRAM_BASE+k)+2],
                     sram_mem[4*(BAND_SRAM_BASE+k)+1], sram_mem[4*(BAND_SRAM_BASE+k)+0]}, band_ref[k]);
                nbad = nbad + 1;
            end
        end
        $display("SB DONE band: bad=%0d/%0d", nbad, (HB+1)*N);
        $write("SBAND:");
        for (k = 0; k < (HB+1)*N; k = k + 1) $write(" %h",
            {sram_mem[4*(BAND_SRAM_BASE+k)+3], sram_mem[4*(BAND_SRAM_BASE+k)+2],
             sram_mem[4*(BAND_SRAM_BASE+k)+1], sram_mem[4*(BAND_SRAM_BASE+k)+0]});
        $write("\n");
        $finish;
    end
endmodule
