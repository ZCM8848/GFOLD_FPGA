`timescale 1ns/1ps
// TB for s_build: preload D_y into RAM[0..M), run, dump the built band
// RAM[M .. M+(HB+1)*N) as one "COL k:" line per column for check_sbuild.py.
// WHICH=0 -> small (n10/m20/hb4), WHICH=1 -> full (n1100/m2107/hb17).
// Literal filenames via case(WHICH) — $sformatf hangs ModelSim 10.5b.
module tb_s_build;
    parameter N = 10, M = 20, NNZ = 41, HB = 4, MAXROW = 4, WHICH = 0;
    parameter AROW_FILE = "../data/kkt/small/Arow.hex";
    parameter ACOL_FILE = "../data/kkt/small/Acol.hex";
    parameter AVAL_FILE = "../data/kkt/small/Aval.hex";
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0;
    wire [14:0] ram_addr; wire [63:0] ram_wdata; wire ram_we; wire [63:0] ram_rdata;
    wire done;
    s_build #(.N(N),.M(M),.NNZ(NNZ),.HB(HB),.MAXROW(MAXROW),
              .BAND_OFFSET(M),.DY_OFFSET(0),
              .AROW_FILE(AROW_FILE),.ACOL_FILE(ACOL_FILE),.AVAL_FILE(AVAL_FILE)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),
        .ram_addr(ram_addr),.ram_wdata(ram_wdata),.ram_we(ram_we),.ram_rdata(ram_rdata),
        .done(done));
    reg [63:0] mem [0:32767];
    always @(posedge clk) if (ram_we) mem[ram_addr] <= ram_wdata;
    assign ram_rdata = mem[ram_addr];

    reg [63:0] Dy [0:4095];
    integer k, i;
    reg watchdog = 0;
    always begin #50000000; if (!watchdog) begin $display("TIMEOUT: sim stuck"); $finish; end end
    initial begin
        case (WHICH)
            0: $readmemh("../data/kkt/small/Dy.hex", Dy);
            1: $readmemh("../data/kkt/full/Dy.hex", Dy);
        endcase
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        for (k = 0; k < M; k = k + 1) mem[k] = Dy[k];   // preload D_y (tb-side)
        @(negedge clk); start = 1;
        @(negedge clk); start = 0;
        while (!done) @(posedge clk);
        // dump band: one line per column k: "COL k:" + (HB+1) words
        for (k = 0; k < N; k = k + 1) begin
            $write("COL %0d:", k);
            for (i = 0; i <= HB; i = i + 1) $write(" %h", mem[M + k*(HB+1) + i]);
            $write("\n");
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
