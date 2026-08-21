`timescale 1ns/1ps
// TB for chol10: load G (M*M) + rhs (M), capture gamma, dump for Python compare.
module tb_chol10;
    parameter MEM = 10;
    parameter N = 12;
    reg clk=0, rst_n=0;
    always #5 clk = ~clk;
    reg start=0, din_valid=0;
    reg [63:0] g_in, r_in;
    wire done; wire [63:0] gamma_out; wire o_valid;
    chol10 #(.MEM(MEM)) dut(.clk(clk),.rst_n(rst_n),.start(start),
        .g_in(g_in),.r_in(r_in),.din_valid(din_valid),
        .done(done),.gamma_out(gamma_out),.o_valid(o_valid));

    integer c, i, j, fd;
    reg [63:0] Gbuf [0:99], rhsbuf [0:9];
    reg [63:0] got [0:9];
    reg [63:0] mem [0:199];
    integer n;

    task read_case(input integer ci);
        begin
            // read G (M*M) then rhs (M) from chol_<ci>.hex via $readmemh (reliable)
            $readmemh($sformatf("data/chol_%0d.hex", ci), mem);
            for (n = 0; n < MEM*MEM; n = n + 1) Gbuf[n] = mem[n];
            for (n = 0; n < MEM; n = n + 1) rhsbuf[n] = mem[MEM*MEM + n];
        end
    endtask

    task run_case(input integer ci);
        integer k;
        begin
            read_case(ci);
            // drive load: stream G then rhs. First word set on the SAME negedge as
            // din_valid so the DUT's first posedge sample sees it (no off-by-one).
            @(negedge clk); start = 1; din_valid = 0; g_in = 0; r_in = 0;
            @(negedge clk); start = 0; din_valid = 1; g_in = Gbuf[0]; r_in = 0;
            for (k = 1; k < MEM*MEM; k = k + 1) begin
                @(negedge clk); g_in = Gbuf[k];
            end
            for (k = 0; k < MEM; k = k + 1) begin
                @(negedge clk); r_in = rhsbuf[k];
            end
            // deassert valid, capture MEM gamma outputs on o_valid
            @(negedge clk); din_valid = 0;
            k = 0;
            while (k < MEM) begin
                @(posedge clk);
                if (o_valid) begin got[k] = gamma_out; k = k + 1; end
            end
            // report
            $write("CASE %0d:", ci);
            for (k = 0; k < MEM; k = k + 1) $write(" %h", got[k]);
            $write("\n");
        end
    endtask

    initial begin
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        for (c = 0; c < N; c = c + 1) run_case(c);
        $display("ALL DONE");
        $finish;
    end
endmodule
