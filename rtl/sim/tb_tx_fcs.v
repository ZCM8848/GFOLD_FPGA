`timescale 1ns/1ps
// tb_tx_fcs: capture UDP ACK and ARP reply frames; the Python checker verifies
// the Ethernet FCS after the registered-CRC-input fix.
module tb_tx_fcs;
    reg clk = 0, rst_n = 0;
    always #4 clk = ~clk;

    reg         ureq = 0, areq = 0;
    reg  [47:0] dmac = 48'h02_00_00_00_00_64;
    reg  [31:0] dip  = 32'hC0A8_0164;
    reg  [15:0] dport = 16'd4661;
    reg  [7:0]  atype = 8'h02;
    reg  [15:0] aseq = 16'd1;
    reg  [31:0] pw0 = 32'h11223344, pw1 = 32'h55667788;

    wire ubusy, udone, abusy, adone;
    wire [7:0] utxd, atxd;
    wire utxen, atxen;

    udp_reply_tx u_udp(.clk(clk), .rst_n(rst_n), .req(ureq),
        .dst_mac(dmac), .dst_ip(dip), .dst_port(dport),
        .ack_type(atype), .ack_seq(aseq), .payload_w0(pw0), .payload_w1(pw1),
        .busy(ubusy), .done(udone), .gmii_txd(utxd), .gmii_tx_en(utxen));

    arp_reply_tx u_arp(.clk(clk), .rst_n(rst_n), .req(areq),
        .dst_mac(dmac), .dst_ip(dip),
        .busy(abusy), .done(adone), .gmii_txd(atxd), .gmii_tx_en(atxen));

    integer un = 0, an = 0, i, fd;
    reg [7:0] uf [0:127], af [0:127];
    always @(posedge clk) if (utxen) begin uf[un] <= utxd; un <= un + 1; end
    always @(posedge clk) if (atxen) begin af[an] <= atxd; an <= an + 1; end

    initial begin
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1;
        @(negedge clk); ureq = 1; @(negedge clk); ureq = 0;
        while (!udone) @(posedge clk); @(posedge clk);
        fd = $fopen("udp_frame.hex", "w");
        for (i = 0; i < un; i = i + 1) $fwrite(fd, "%02X\n", uf[i]);
        $fclose(fd); $display("UDP frame %0d bytes", un);

        @(negedge clk); areq = 1; @(negedge clk); areq = 0;
        while (!adone) @(posedge clk); @(posedge clk);
        fd = $fopen("arp_frame.hex", "w");
        for (i = 0; i < an; i = i + 1) $fwrite(fd, "%02X\n", af[i]);
        $fclose(fd); $display("ARP frame %0d bytes", an);
        $finish;
    end
    reg watchdog = 0;
    always begin #(64'd1000000); if (!watchdog) begin $display("TIMEOUT"); $finish; end end
endmodule
