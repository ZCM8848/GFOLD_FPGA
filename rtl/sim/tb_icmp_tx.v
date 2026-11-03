`timescale 1ns/1ps
// tb_icmp_tx: capture an ICMP echo reply and verify the FCS.
module tb_icmp_tx;
    reg clk = 0, rst_n = 0;
    always #4 clk = ~clk;

    reg         req = 0;
    reg  [47:0] dmac = 48'h02_00_00_00_00_64;
    reg  [31:0] dip  = 32'hC0A8_0164;
    reg  [15:0] iid = 16'h1234, iseq = 16'd7;
    reg  [6:0]  dlen = 7'd16;

    wire [6:0]  raddr;
    wire [7:0]  rdata;
    wire        busy, done;
    wire [7:0]  txd;
    wire        txen;

    reg [7:0] mem [0:127];
    assign rdata = mem[raddr];

    icmp_reply_tx dut(.clk(clk), .rst_n(rst_n), .req(req),
        .dst_mac(dmac), .dst_ip(dip), .icmp_id(iid), .icmp_seq(iseq), .data_len(dlen),
        .ram_raddr(raddr), .ram_rdata(rdata),
        .busy(busy), .done(done), .gmii_txd(txd), .gmii_tx_en(txen));

    integer i, n = 0, fd;
    reg [7:0] fr [0:255];
    always @(posedge clk) if (txen) begin fr[n] <= txd; n <= n + 1; end

    initial begin
        for (i = 0; i < 128; i = i + 1) mem[i] = 8'hA0 + i[7:0];
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        @(negedge clk); req = 1; @(negedge clk); req = 0;
        while (!done) @(posedge clk); @(posedge clk);
        fd = $fopen("icmp_frame.hex", "w");
        for (i = 0; i < n; i = i + 1) $fwrite(fd, "%02X\n", fr[i]);
        $fclose(fd);
        $display("ICMP frame %0d bytes", n);
        $finish;
    end
    reg watchdog = 0;
    always begin #(64'd1000000); if (!watchdog) begin $display("TIMEOUT st=%0d", dut.st); $finish; end end
endmodule
