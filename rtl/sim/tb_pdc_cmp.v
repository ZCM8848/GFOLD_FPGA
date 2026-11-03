`timescale 1ns/1ps
// tb_pdc_cmp: run master proj_dual_cone_rb and the pool version on identical
// RAM contents and compare bit-exactly.
module tb_pdc_cmp;
    parameter M = 2107;
    parameter NODE = 100, Z0 = 14, ZP = 7, NP = 3,
              NS4 = 2, DIM4 = 4, NS3 = 1, DIM3 = 3,
              TAIL_NN = 1, TAIL_Z = 6;
    reg clk=0, rst_n=0; always #5 clk=~clk;

    // master
    wire [11:0] m_addr; wire [63:0] m_wdata; wire m_we; wire [63:0] m_rdata;
    wire m_done; reg m_start=0;
    reg [63:0] mmem [0:4095];
    always @(posedge clk) if (m_we) mmem[m_addr] <= m_wdata;
    assign m_rdata = mmem[m_addr];
    proj_dual_cone_rb_m #(.M(M), .NODE(NODE), .Z0(Z0), .ZP(ZP), .NP(NP),
        .NS4(NS4), .DIM4(DIM4), .NS3(NS3), .DIM3(DIM3),
        .TAIL_NN(TAIL_NN), .TAIL_Z(TAIL_Z)) mdut(
        .clk(clk), .rst_n(rst_n), .start(m_start),
        .addr(m_addr), .wdata(m_wdata), .we(m_we), .rdata(m_rdata), .done(m_done));

    // pool
    reg  [63:0] m0a,m0b,m1a,m1b,m2a,m2b,m3a,m3b;
    wire [63:0] m0o,m1o,m2o,m3o;
    reg  [63:0] a0a,a0b,a1a,a1b,a2a,a2b,a3a,a3b,a4a,a4b;
    reg  a0s,a1s,a2s,a3s,a4s;
    wire [63:0] a0o,a1o,a2o,a3o,a4o;
    reg ds; reg [63:0] da,db; wire dd; wire [63:0] d_o;
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
    wire rr_start; wire [63:0] rr_in; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt urs(.clk(clk),.start(rr_start),.x(rr_in),.done(rr_done),.o(rr_o));

    wire [11:0] p_addr; wire [63:0] p_wdata; wire p_we; wire [63:0] p_rdata;
    wire p_done; reg p_start=0;
    reg [63:0] pmem [0:4095];
    always @(posedge clk) if (p_we) pmem[p_addr] <= p_wdata;
    assign p_rdata = pmem[p_addr];
    wire [63:0] u_m0a,u_m0b,u_m1a,u_m1b,u_m2a,u_m2b;
    wire [63:0] u_a0a,u_a0b; wire u_a0s;
    proj_dual_cone_rb #(.M(M), .NODE(NODE), .Z0(Z0), .ZP(ZP), .NP(NP),
        .NS4(NS4), .DIM4(DIM4), .NS3(NS3), .DIM3(DIM3),
        .TAIL_NN(TAIL_NN), .TAIL_Z(TAIL_Z)) pdut(
        .clk(clk), .rst_n(rst_n), .start(p_start),
        .addr(p_addr), .wdata(p_wdata), .we(p_we), .rdata(p_rdata), .done(p_done),
        .add0_a(u_a0a), .add0_b(u_a0b), .add0_sub(u_a0s), .add0_o(a0o),
        .mul0_a(u_m0a), .mul0_b(u_m0b), .mul1_a(u_m1a), .mul1_b(u_m1b),
        .mul2_a(u_m2a), .mul2_b(u_m2b),
        .mul0_o(m0o), .mul1_o(m1o), .mul2_o(m2o),
        .rr_start(rr_start), .rr_in(rr_in), .rr_done(rr_done), .rr_o(rr_o));
    always @(*) begin
        m0a=u_m0a; m0b=u_m0b; m1a=u_m1a; m1b=u_m1b; m2a=u_m2a; m2b=u_m2b;
        m3a=0; m3b=0;
        a0a=u_a0a; a0b=u_a0b; a0s=u_a0s;
        a1a=0;a1b=0;a1s=0;a2a=0;a2b=0;a2s=0;a3a=0;a3b=0;a3s=0;a4a=0;a4b=0;a4s=0;
        ds=0;da=0;db=0;
    end

    integer i,nbad;
    initial begin
        $readmemh("../data/kkt/full/pdc_test_in.hex", mmem);
        for (i=0;i<M;i=i+1) pmem[i]=mmem[i];
        rst_n=0; repeat(4) @(negedge clk); rst_n=1;
        @(negedge clk); m_start=1; p_start=1;
        @(negedge clk); m_start=0; p_start=0;
        while (!(m_done && p_done)) @(posedge clk);
        nbad=0;
        for (i=0;i<M;i=i+1) if (mmem[i]!==pmem[i]) begin
            if (nbad<10) $display("  [%0d] master=%h pool=%h", i, mmem[i], pmem[i]);
            nbad=nbad+1;
        end
        $display("PDC CMP: %0d mismatches of %0d", nbad, M);
        $finish;
    end
endmodule
