`timescale 1ns/1ps
// TB for root_plus. Loads data/rp_<i>.hex (L lines of r g p mu, then 1 line eta tau),
// streams r,g,p,mu to DUT, captures tau, compares with expected (printed as hex for
// authoritative Python decode).

module tb_root_plus;
    parameter L = 8;
    reg clk=0, rst_n=0;
    always #5 clk = ~clk;

    reg [63:0] r_in, g_in, p_in, mu_in;
    reg din_valid, start;
    reg [63:0] eta;
    wire done;
    wire [63:0] tau;
    root_plus #(.L(L)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .r_in(r_in), .g_in(g_in), .p_in(p_in), .mu_in(mu_in),
        .din_valid(din_valid), .eta(eta),
        .done(done), .tau(tau)
    );

    reg [63:0] rm[0:255], gm[0:255], pm[0:255], mum[0:255];
    integer ncases, i, j, fd;
    reg [63:0] exp_tau, got;
    reg [7:0] expc[0:15];
    reg [7:0] lin[0:127];
    integer nchars, pos, lp;

    task readhexline(input integer f, output reg [63:0] a, b, c, d);
        reg [7:0] ch; integer n; reg [3:0] h; integer k; reg [63:0] v;
        begin
            n = 0; v = 0;
            // read 4 whitespace-separated 16-hex groups
            for (k = 0; k < 4; k = k + 1) begin
                v = 0;
                repeat (16) begin
                    ch = $fgetc(f);
                    if (ch >= "0" && ch <= "9") h = ch - "0";
                    else if (ch >= "a" && ch <= "f") h = ch - "a" + 10;
                    else h = 0;
                    v = (v << 4) | h;
                end
                $fgetc(f); // space or newline
                case (k) 0: a = v; 1: b = v; 2: c = v; 3: d = v; endcase
            end
        end
    endtask

    task read_eta_line(input integer f, output reg [63:0] eta_, output reg [63:0] expt);
        reg [7:0] ch; reg [3:0] h; reg [63:0] v; integer k, g;
        begin
            v = 0;
            for (g = 0; g < 2; g = g + 1) begin
                v = 0;
                repeat (16) begin
                    ch = $fgetc(f);
                    if (ch >= "0" && ch <= "9") h = ch - "0";
                    else if (ch >= "a" && ch <= "f") h = ch - "a" + 10;
                    else h = 0;
                    v = (v << 4) | h;
                end
                $fgetc(f);
                if (g == 0) eta_ = v; else expt = v;
            end
        end
    endtask

    task run_case(input integer ci);
        integer k;
        reg [63:0] e_;
        begin
            fd = $fopen($sformatf("data/rp_%0d.hex", ci), "r");
            for (k = 0; k < L; k = k + 1) begin
                readhexline(fd, rm[k], gm[k], pm[k], mum[k]);
            end
            read_eta_line(fd, e_, exp_tau);
            $fclose(fd);
            // drive DUT
            @(negedge clk);
            start = 1; din_valid = 0; r_in = 0; g_in = 0; p_in = 0; mu_in = 0;
            eta = e_;
            @(negedge clk);
            start = 0;
            for (k = 0; k < L; k = k + 1) begin
                @(negedge clk);
                r_in = rm[k]; g_in = gm[k]; p_in = pm[k]; mu_in = mum[k];
                din_valid = 1;
            end
            // deassert valid, wait for done
            @(negedge clk);
            din_valid = 0;
            while (!done) @(posedge clk);
            got = tau;
            // report hex
            $display("CASE %0d: exp=%h got=%h cnt=%0d gg=%h st=%0d divo=%h %s",
                     ci, exp_tau, got, dut.cnt, dut.gg, dut.st, dut.divo,
                     (got===exp_tau ? "MATCH" : "DIFF"));
            @(negedge clk);
        end
    endtask

    initial begin
        ncases = 24;   // must match gen_root_plus.py default
        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        for (i = 0; i < ncases; i = i + 1) run_case(i);
        $display("ALL DONE");
        $finish;
    end
endmodule
