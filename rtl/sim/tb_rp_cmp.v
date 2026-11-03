`timescale 1ns/1ps
// tb_rp_cmp: feed the exact iter-1 root_plus stream captured from the DRS sim
// to the master root_plus and the pool root_plus, compare tau bit-exactly.
module tb_rp_cmp;
    localparam L = 3207;
    reg clk=0, rst_n=0; always #5 clk=~clk;
    reg m_seen=0,p_seen=0;
    always @(posedge clk) begin if (m_done) m_seen<=1; if (p_done) p_seen<=1; end

    reg [63:0] w [0:4*L-1];
    reg [63:0] eta = 64'h3f59e3b600b0559d;  // captured v[l-1]

    // ---- master ----
    reg m_start=0, m_dv=0; reg [63:0] m_r,m_g,m_p,m_mu;
    wire m_done; wire [63:0] m_tau;
    root_plus_m #(.L(L)) mdut(.clk(clk), .rst_n(rst_n), .start(m_start),
        .r_in(m_r), .g_in(m_g), .p_in(m_p), .mu_in(m_mu), .din_valid(m_dv),
        .eta(eta), .done(m_done), .tau(m_tau));

    // ---- pool ----
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
    wire rr_start; wire [63:0] rr_in; wire rr_done; wire [63:0] rr_o;
    fp64_rsqrt urs(.clk(clk),.start(rr_start),.x(rr_in),.done(rr_done),.o(rr_o));

    reg p_start=0, p_dv=0; reg [63:0] p_r,p_g,p_p,p_mu;
    wire p_done; wire [63:0] p_tau;
    wire [63:0] u_m0a,u_m0b,u_m1a,u_m1b,u_m2a,u_m2b,u_m3a,u_m3b;
    wire [63:0] u_a0a,u_a0b,u_a1a,u_a1b,u_a2a,u_a2b,u_a3a,u_a3b,u_a4a,u_a4b;
    wire u_a0s,u_a1s,u_a2s,u_a3s,u_a4s;
    wire u_ds; wire [63:0] u_da,u_db;
    root_plus #(.L(L)) pdut(.clk(clk), .rst_n(rst_n), .start(p_start),
        .r_in(p_r), .g_in(p_g), .p_in(p_p), .mu_in(p_mu), .din_valid(p_dv),
        .eta(eta), .done(p_done), .tau(p_tau),
        .mul0_a(u_m0a), .mul0_b(u_m0b), .mul1_a(u_m1a), .mul1_b(u_m1b),
        .mul2_a(u_m2a), .mul2_b(u_m2b), .mul3_a(u_m3a), .mul3_b(u_m3b),
        .mul0_o(m0o), .mul1_o(m1o), .mul2_o(m2o), .mul3_o(m3o),
        .add0_a(u_a0a), .add0_b(u_a0b), .add0_sub(u_a0s),
        .add1_a(u_a1a), .add1_b(u_a1b), .add1_sub(u_a1s),
        .add2_a(u_a2a), .add2_b(u_a2b), .add2_sub(u_a2s),
        .add3_a(u_a3a), .add3_b(u_a3b), .add3_sub(u_a3s),
        .add4_a(u_a4a), .add4_b(u_a4b), .add4_sub(u_a4s),
        .add0_o(a0o), .add1_o(a1o), .add2_o(a2o), .add3_o(a3o), .add4_o(a4o),
        .div_start(u_ds), .div_a(u_da), .div_b(u_db),
        .div_done(dd), .div_o(d_o),
        .rr_start(rr_start), .rr_in(rr_in), .rr_done(rr_done), .rr_o(rr_o));
    always @(*) begin
        m0a=u_m0a; m0b=u_m0b; m1a=u_m1a; m1b=u_m1b;
        m2a=u_m2a; m2b=u_m2b; m3a=u_m3a; m3b=u_m3b;
        a0a=u_a0a; a0b=u_a0b; a0s=u_a0s;
        a1a=u_a1a; a1b=u_a1b; a1s=u_a1s;
        a2a=u_a2a; a2b=u_a2b; a2s=u_a2s;
        a3a=u_a3a; a3b=u_a3b; a3s=u_a3s;
        a4a=u_a4a; a4b=u_a4b; a4s=u_a4s;
        ds=u_ds; da=u_da; db=u_db;
    end

    integer i, cyc;
    initial begin
        $readmemh("rp_stream.hex", w);
        rst_n=0; repeat(4) @(negedge clk); rst_n=1;
        @(negedge clk); m_start=1; p_start=1;
        @(negedge clk); m_start=0; p_start=0;
        // master: one element per cycle, back-to-back
        m_dv=1;
        for (i=0;i<L;i=i+1) begin
            m_r=w[4*i]; m_g=w[4*i+1]; m_p=w[4*i+2]; m_mu=w[4*i+3];
            @(negedge clk);
        end
        m_dv=0;
        // pool: pulse one element every 6 cycles
        for (i=0;i<L;i=i+1) begin
            p_r=w[4*i]; p_g=w[4*i+1]; p_p=w[4*i+2]; p_mu=w[4*i+3];
            p_dv=1; @(negedge clk); p_dv=0;
            repeat(5) @(negedge clk);
        end
        while (!(m_seen && p_seen)) @(posedge clk);
        $display("MASTER tau=%h", m_tau);
        $display("POOL   tau=%h", p_tau);
        $display("MATCH=%s", (m_tau===p_tau) ? "YES" : "NO");
        $finish;
    end
endmodule
