`timescale 1ns/1ps
// TB for aa_gram (inline, no task, modest arrays — ModelSim 10.5b hangs on the
// task+large-mem version). Load S (MEM*DIM) + g (DIM) + rreg, capture G+rhs.
module tb_aa_gram;
    parameter DIM = 8, MEM = 10, N = 6;
    reg clk=0, rst_n=0, start=0, din_valid=0;
    reg [63:0] s_in, g_in, rreg_in;
    wire done; wire [63:0] g_out, r_out; wire o_valid;
    always #5 clk = ~clk;
    aa_gram #(.DIM(DIM), .MEM(MEM)) dut(.clk(clk),.rst_n(rst_n),.start(start),
        .s_in(s_in),.g_in(g_in),.rreg_in(rreg_in),.din_valid(din_valid),
        .done(done),.g_out(g_out),.r_out(r_out),.o_valid(o_valid));

    reg [63:0] mem [0:399];           // MEM*DIM+DIM+1 = 353
    integer c, k, cnt, fd;
    reg [63:0] got [0:119];           // MEM*MEM+MEM = 110

    initial begin
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        for (c = 0; c < N; c = c + 1) begin
            case (c)   // literal filenames — $readmemh($sformatf(...)) hangs ModelSim 10.5b
                0: $readmemh("data/ag_0.hex", mem);
                1: $readmemh("data/ag_1.hex", mem);
                2: $readmemh("data/ag_2.hex", mem);
                3: $readmemh("data/ag_3.hex", mem);
                4: $readmemh("data/ag_4.hex", mem);
                5: $readmemh("data/ag_5.hex", mem);
            endcase
            rreg_in = mem[MEM*DIM + DIM];
            @(negedge clk); start = 1; din_valid = 0;
            @(negedge clk); start = 0; din_valid = 1; s_in = mem[0];
            for (k = 1; k < MEM*DIM; k = k + 1) begin @(negedge clk); s_in = mem[k]; end
            for (k = 0; k < DIM; k = k + 1) begin @(negedge clk); g_in = mem[MEM*DIM + k]; end
            @(negedge clk); din_valid = 0;
            cnt = 0;
            while (cnt < MEM*MEM + MEM) begin
                @(posedge clk);
                if (o_valid) begin
                    got[cnt] = (cnt < MEM*MEM) ? g_out : r_out;
                    cnt = cnt + 1;
                end
            end
            // dump
            case (c)   // literal filenames ($sformatf in fopen also risky)
                0: fd = $fopen("data/ag_0_out.hex", "w");
                1: fd = $fopen("data/ag_1_out.hex", "w");
                2: fd = $fopen("data/ag_2_out.hex", "w");
                3: fd = $fopen("data/ag_3_out.hex", "w");
                4: fd = $fopen("data/ag_4_out.hex", "w");
                5: fd = $fopen("data/ag_5_out.hex", "w");
            endcase
            for (k = 0; k < MEM*MEM + MEM; k = k + 1) $fwrite(fd, "%016h\n", got[k]);
            $fclose(fd);
        end
        $display("ALL DONE");
        $finish;
    end
endmodule
