`timescale 1ns/1ps
// TB for kkt_solve: stream band ((HB+1)*N) + vx (N) + vy (M), capture
// zx (N) + zy (M) output stream, print "CASE 0: ..." for check_kkt.py.
// REF=1 additionally runs a second solve with refactor=0 (no band stream,
// reuses L/D) and reports bit-mismatches vs run 1 — must be zero.
// WHICH=0 -> small (n10/m20/hb4), WHICH=1 -> full (n1100/m2107/hb17).
// Literal filenames via case(WHICH) — $sformatf hangs ModelSim 10.5b.
module tb_kkt_solve;
    parameter N = 10, M = 20, NNZ = 41, HB = 4, WHICH = 0, REF = 1;
    parameter AROW_FILE = "../data/kkt/small/Arow.hex";
    parameter ACOL_FILE = "../data/kkt/small/Acol.hex";
    parameter AVAL_FILE = "../data/kkt/small/Aval.hex";
    parameter RY_FILE   = "../data/kkt/small/r_y.hex";
    parameter DY_FILE   = "../data/kkt/small/Dy.hex";
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, rf = 1, bv = 0, dv = 0; reg [63:0] band_in, xin;
    wire [63:0] z_out; wire o_valid, done;
    wire [13:0] ram_addr; wire [63:0] ram_wdata; wire ram_we; wire [63:0] ram_rdata;
    kkt_solve #(.N(N),.M(M),.NNZ(NNZ),.HB(HB),
                .AROW_FILE(AROW_FILE),.ACOL_FILE(ACOL_FILE),.AVAL_FILE(AVAL_FILE),
                .RY_FILE(RY_FILE),.DY_FILE(DY_FILE)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),
        .band_in(band_in),.band_valid(bv),.refactor(rf),
        .x_in(xin),.din_valid(dv),
        .ram_addr(ram_addr),.ram_wdata(ram_wdata),.ram_we(ram_we),.ram_rdata(ram_rdata),
        .z_out(z_out),.o_valid(o_valid),.done(done));
    reg [63:0] mem [0:16383];
    always @(posedge clk) if (ram_we) mem[ram_addr] <= ram_wdata;
    assign ram_rdata = mem[ram_addr];

    reg [63:0] band [0:32767];
    reg [63:0] vx [0:4095], vy [0:4095], got1 [0:4095], got2 [0:4095];
    integer k, cnt, nbad;
    reg watchdog = 0;
    always begin #200000000; if (!watchdog) begin $display("TIMEOUT: sim stuck"); $finish; end end

    task run_solve;
        input use_refactor;
        begin
            @(negedge clk); start = 1; rf = use_refactor; bv = 0; dv = 0;
            @(negedge clk); start = 0;
            if (use_refactor) begin
                for (k = 0; k < (HB+1)*N; k = k + 1) begin @(negedge clk); band_in = band[k]; bv = 1; end
                @(negedge clk); bv = 0;
            end
            for (k = 0; k < N; k = k + 1) begin @(negedge clk); xin = vx[k]; dv = 1; end
            for (k = 0; k < M; k = k + 1) begin @(negedge clk); xin = vy[k]; dv = 1; end
            @(negedge clk); dv = 0;
            cnt = 0;
            while (cnt < N + M) begin
                @(posedge clk);
                if (o_valid) begin
                    if (use_refactor == 1) got1[cnt] = z_out; else got2[cnt] = z_out;
                    cnt = cnt + 1;
                end
            end
            while (!done) @(posedge clk);
        end
    endtask

    initial begin
        case (WHICH)
            0: begin
                $readmemh("../data/kkt/small/band_f64.hex", band);
                $readmemh("../data/kkt/small/vx.hex", vx);
                $readmemh("../data/kkt/small/vy.hex", vy);
            end
            1: begin
                $readmemh("../data/kkt/full/band_f64.hex", band);
                $readmemh("../data/kkt/full/vx.hex", vx);
                $readmemh("../data/kkt/full/vy.hex", vy);
            end
        endcase
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        run_solve(1);
        $write("CASE 0:");
        for (k = 0; k < N + M; k = k + 1) $write(" %h", got1[k]);
        $write("\n");
        if (REF) begin
            run_solve(0);
            nbad = 0;
            for (k = 0; k < N + M; k = k + 1)
                if (got1[k] != got2[k]) nbad = nbad + 1;
            $display("REFACTOR-0 vs REFACTOR-1: %0d bit mismatches of %0d", nbad, N + M);
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
