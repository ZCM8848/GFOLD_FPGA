`timescale 1ns/1ps
// TB for anderson orchestrator: for each apply call, pulse start, stream DIM
// (x,f) pairs on the rdy/din_valid handshake, capture DIM accelerated f_out on
// o_valid, print "CASE c: ..." for the Python verifier.
// Input vectors from software/gen_anderson.py -> rtl/data/aa_<c>.hex (DIM rows
// "x f fout"). Literal filenames (case) — $sformatf hangs ModelSim 10.5b.
module tb_anderson;
    parameter DIM = 8, MEM = 10, NCALL = 25;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg start = 0, din_valid = 0;
    reg [63:0] x_in, f_in;
    wire rdy; wire [63:0] f_out; wire o_valid, done;
    anderson #(.DIM(DIM), .MEM(MEM)) dut(.clk(clk),.rst_n(rst_n),.start(start),
        .x_in(x_in),.f_in(f_in),.din_valid(din_valid),.rdy(rdy),
        .f_out(f_out),.o_valid(o_valid),.done(done));

    reg [63:0] callmem [0:127];        // DIM*3 <= 96
    reg [63:0] got [0:31];             // DIM <= 32
    integer c, i, k, fd;
    reg [63:0] ex;

    task read_call(input integer ci);
        begin
            case (ci)
                 0: $readmemh("data/aa_0.hex",  callmem);
                 1: $readmemh("data/aa_1.hex",  callmem);
                 2: $readmemh("data/aa_2.hex",  callmem);
                 3: $readmemh("data/aa_3.hex",  callmem);
                 4: $readmemh("data/aa_4.hex",  callmem);
                 5: $readmemh("data/aa_5.hex",  callmem);
                 6: $readmemh("data/aa_6.hex",  callmem);
                 7: $readmemh("data/aa_7.hex",  callmem);
                 8: $readmemh("data/aa_8.hex",  callmem);
                 9: $readmemh("data/aa_9.hex",  callmem);
                10: $readmemh("data/aa_10.hex", callmem);
                11: $readmemh("data/aa_11.hex", callmem);
                12: $readmemh("data/aa_12.hex", callmem);
                13: $readmemh("data/aa_13.hex", callmem);
                14: $readmemh("data/aa_14.hex", callmem);
                15: $readmemh("data/aa_15.hex", callmem);
                16: $readmemh("data/aa_16.hex", callmem);
                17: $readmemh("data/aa_17.hex", callmem);
                18: $readmemh("data/aa_18.hex", callmem);
                19: $readmemh("data/aa_19.hex", callmem);
                20: $readmemh("data/aa_20.hex", callmem);
                21: $readmemh("data/aa_21.hex", callmem);
                22: $readmemh("data/aa_22.hex", callmem);
                23: $readmemh("data/aa_23.hex", callmem);
                24: $readmemh("data/aa_24.hex", callmem);
                default: $readmemh("data/aa_0.hex", callmem);
            endcase
        end
    endtask

    task run_call(input integer ci);
        integer k2;
        begin
            read_call(ci);
            @(negedge clk); start = 1;
            @(negedge clk); start = 0;
            for (i = 0; i < DIM; i = i + 1) begin
                while (!rdy) @(negedge clk);
                @(negedge clk); x_in = callmem[i*3]; f_in = callmem[i*3+1]; din_valid = 1;
                @(negedge clk); din_valid = 0;
            end
            k2 = 0;
            while (k2 < DIM) begin
                @(posedge clk);
                if (o_valid) begin got[k2] = f_out; k2 = k2 + 1; end
            end
            while (!done) @(posedge clk);
            $write("CASE %0d:", ci);
            for (k2 = 0; k2 < DIM; k2 = k2 + 1) $write(" %h", got[k2]);
            $write("\n");
        end
    endtask

    initial begin
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        for (c = 0; c < NCALL; c = c + 1) run_call(c);
        $display("ALL DONE");
        $finish;
    end
endmodule
