`timescale 1ns/1ps
// tb_proj_soc_pool: drive the pool-based proj_soc with the same 5 cases as
// the master tb_proj_soc, printing hex outputs for comparison.
module tb_proj_soc_pool;
    parameter DIM = 4;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, start = 0;
    reg [DIM*64-1:0] v;
    wire done; wire [DIM*64-1:0] o;

    // global pool
    reg  [63:0] m0a,m0b,m1a,m1b,m2a,m2b,m3a,m3b;
    wire [63:0] m0o,m1o,m2o,m3o;
    reg  [63:0] a0a,a0b,a1a,a1b,a2a,a2b,a3a,a3b,a4a,a4b;
    reg  a0s,a1s,a2s,a3s,a4s;
    wire [63:0] a0o,a1o,a2o,a3o,a4o;
    reg  ds; reg [63:0] da,db; wire dd; wire [63:0] d_o;
    fp64_pool pool(.clk(clk),.rst_n(rst_n),
        .mul0_a(m0a),.mul0_b(m0b),.mul1_a(m1a),.mul1_b(m1b),
        .mul2_a(m2a),.mul2_b(m2b),.mul3_a(m3a),.mul3_b(m3b),
        .mul0_o(m0o),.mul1_o(m1o),.mul2_o(m2o),.mul3_o(m3o),
        .add0_a(a0a),.add0_b(a0b),.add0_sub(a0s),
        .add1_a(a1a),.add1_b(a1b),.add1_sub(a1s),
        .add2_a(a2a),.add2_b(a2b),.add2_sub(a2s),
        .add3_a(a3a),.add3_b(a3b),.add3_sub(a3s),
        .add4_a(a4a),.add4_b(a4b),.add4_sub(a4s),
        .add0_o(a0o),.add1_o(a1o),.add2_o(a2o),.add3_o(a3o),.add4_o(a4o),
        .div_start(ds),.div_a(da),.div_b(db),.div_done(dd),.div_o(d_o));
    // shared rsqrt
    wire rr_start; wire [63:0] rr_in; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt urs(.clk(clk),.start(rr_start),.x(rr_in),.done(rr_done),.o(rr_o));

    wire [63:0] u_m0a,u_m0b,u_m1a,u_m1b,u_m2a,u_m2b;
    wire [63:0] u_a0a,u_a0b; wire u_a0s;
    proj_soc_pool #(.DIM(DIM)) dut(.clk(clk), .rst_n(rst_n), .start(start),
        .v(v), .done(done), .o(o),
        .mul0_a(u_m0a), .mul0_b(u_m0b), .mul1_a(u_m1a), .mul1_b(u_m1b),
        .mul2_a(u_m2a), .mul2_b(u_m2b),
        .mul0_o(m0o), .mul1_o(m1o), .mul2_o(m2o),
        .add0_a(u_a0a), .add0_b(u_a0b), .add0_sub(u_a0s), .add0_o(a0o),
        .rr_start(rr_start), .rr_in(rr_in), .rr_done(rr_done), .rr_o(rr_o));

    // mux pool to the DUT (only consumer)
    always @(*) begin
        m0a=u_m0a; m0b=u_m0b; m1a=u_m1a; m1b=u_m1b; m2a=u_m2a; m2b=u_m2b;
        m3a=0; m3b=0;
        a0a=u_a0a; a0b=u_a0b; a0s=u_a0s;
        a1a=0;a1b=0;a1s=0;a2a=0;a2b=0;a2s=0;a3a=0;a3b=0;a3s=0;a4a=0;a4b=0;a4s=0;
        ds=0;da=0;db=0;
    end

    task setv(input [63:0] a, b, c, d);
        begin v[0*64+:64]=a; v[1*64+:64]=b; v[2*64+:64]=c; v[3*64+:64]=d; end
    endtask
    task runcase(input [127:0] name);
        begin
            while (dut.st != 0) @(posedge clk);
            @(posedge clk);
            start=1; @(posedge clk); #1 start=0;
            wait(done); #1;
            $display("%0s: %h %h %h %h", name, o[0*64+:64], o[1*64+:64], o[2*64+:64], o[3*64+:64]);
            @(posedge clk);
            // let done drop before the next case
            while (done) @(posedge clk);
        end
    endtask
    initial begin
        #20 rst_n=1;
        setv(64'h4024000000000000,64'h3FF0000000000000,64'h4000000000000000,64'h4008000000000000); runcase("inside");
        setv(64'hBFF0000000000000,64'h3FE0000000000000,64'h3FE0000000000000,64'h3FE0000000000000); runcase("outside");
        setv(64'h3FF0000000000000,64'h4000000000000000,64'h4008000000000000,64'h4010000000000000); runcase("project");
        setv(64'h4014000000000000,64'h4008000000000000,64'h4010000000000000,64'h0); runcase("boundary");
        setv(64'hC024000000000000,64'h4000000000000000,64'h4008000000000000,64'h4010000000000000); runcase("outside2");
        $finish;
    end
endmodule
