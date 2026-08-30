`timescale 1ns/1ps
// Direct numeric check of the sequential fp64_exp / fp64_log against Python.
// Drives a handful of inputs through each unit and dumps 16-hex results.
module tb_transc;
    reg clk = 0;
    always #5 clk = ~clk;

    reg  start_e, start_l;
    reg  [63:0] x_e, x_l;
    wire de, dl;
    wire [63:0] oe, ol;
    fp64_exp ue(clk, start_e, x_e, de, oe);
    fp64_log ul(clk, start_l, x_l, dl, ol);

    reg [63:0] exp_in [0:7];
    reg [63:0] log_in [0:7];
    integer k;
    reg watchdog = 0;
    always begin #(64'd100000000); if (!watchdog) begin $display("TIMEOUT"); $finish; end end

    task run_exp;
        input [63:0] x;
        begin
            @(negedge clk); start_e = 1; x_e = x;
            @(negedge clk); start_e = 0;
            while (!de) @(posedge clk);
            $display("EXP %016X -> %016X", x, oe);
        end
    endtask
    task run_log;
        input [63:0] x;
        begin
            @(negedge clk); start_l = 1; x_l = x;
            @(negedge clk); start_l = 0;
            while (!dl) @(posedge clk);
            $display("LOG %016X -> %016X", x, ol);
        end
    endtask

    initial begin
        start_e = 0; start_l = 0; x_e = 0; x_l = 0;
        $readmemh("../data/kkt/full/transc_exp_in.hex", exp_in);
        $readmemh("../data/kkt/full/transc_log_in.hex", log_in);
        repeat (4) @(negedge clk);
        for (k = 0; k < 8; k = k + 1) run_exp(exp_in[k]);
        for (k = 0; k < 8; k = k + 1) run_log(log_in[k]);
        watchdog = 1;
        $display("DONE");
        $finish;
    end
endmodule
