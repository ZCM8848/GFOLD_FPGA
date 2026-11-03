`timescale 1ns/1ps
// tb_dump_tx: capture one DUMP_DATA frame and write the raw GMII bytes,
// so the Python checker can verify the pipelined FCS.
module tb_dump_tx;
    reg clk = 0, rst_n = 0;
    always #4 clk = ~clk;      // 125 MHz

    reg         req = 0;
    reg  [47:0] dmac = 48'h02_00_00_00_00_64;
    reg  [31:0] dip  = 32'hC0A8_0164;   // 192.168.1.100
    reg  [15:0] dport = 16'd4661;
    reg  [9:0]  dstart = 0;
    reg  [3:0]  dnwords = 4'd10;

    wire [9:0]  raddr;
    reg  [63:0] rdata;
    wire        busy, done;
    wire [7:0]  txd;
    wire        txen;

    reg [63:0] mem [0:15];
    always @(posedge clk) rdata <= mem[raddr];

    dump_tx dut(
        .clk(clk), .rst_n(rst_n), .req(req),
        .dst_mac(dmac), .dst_ip(dip), .dst_port(dport),
        .dump_start(dstart), .dump_nwords(dnwords),
        .ram_raddr(raddr), .ram_rdata(rdata),
        .busy(busy), .done(done),
        .gmii_txd(txd), .gmii_tx_en(txen));

    integer i, n = 0;
    reg [7:0] frame [0:255];
    always @(posedge clk) if (txen) begin frame[n] <= txd; n <= n + 1; end

    integer fd;
    initial begin
        for (i = 0; i < 10; i = i + 1) mem[i] = 64'h0123456789ABCDEF + i;
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        @(negedge clk); req = 1; @(negedge clk); req = 0;
        while (!done) @(posedge clk);
        @(posedge clk);
        fd = $fopen("dump_frame.hex", "w");
        for (i = 0; i < n; i = i + 1) $fwrite(fd, "%02X\n", frame[i]);
        $fclose(fd);
        $display("CAPTURED %0d bytes", n);
        $finish;
    end
    reg watchdog = 0;
    always begin #(64'd1000000); if (!watchdog) begin $display("TIMEOUT st=%0d", dut.st); $finish; end end
endmodule
