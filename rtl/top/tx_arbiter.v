// tx_arbiter.v — ARP / ICMP / UDP / DUMP 四路仲裁（clk_125 域）
// 优先级 ARP > ICMP > UDP > DUMP
`timescale 1ns/1ps
module tx_arbiter (
    input  wire clk, input wire rst_n,
    input  wire arp_req, icmp_req, udp_req, dump_req,
    output reg  arp_start, icmp_start, udp_start, dump_start,
    input  wire arp_busy, icmp_busy, udp_busy, dump_busy,
    output wire tx_busy
);
    localparam S_IDLE=3'd0, S_ARP=3'd1, S_ICMP=3'd2, S_UDP=3'd3, S_DUMP=3'd4;
    reg [2:0] st; reg arp_pend, icmp_pend, udp_pend, dump_pend;
    assign tx_busy = (st != S_IDLE);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_IDLE; arp_pend<=0; icmp_pend<=0; udp_pend<=0; dump_pend<=0;
            arp_start<=0; icmp_start<=0; udp_start<=0; dump_start<=0;
        end else begin
            arp_start<=0; icmp_start<=0; udp_start<=0; dump_start<=0;
            if (arp_req) arp_pend<=1; if (icmp_req) icmp_pend<=1;
            if (udp_req) udp_pend<=1; if (dump_req) dump_pend<=1;
            case (st)
                S_IDLE: begin
                    if (arp_pend) begin arp_start<=1; arp_pend<=0; st<=S_ARP; end
                    else if (icmp_pend) begin icmp_start<=1; icmp_pend<=0; st<=S_ICMP; end
                    else if (udp_pend) begin udp_start<=1; udp_pend<=0; st<=S_UDP; end
                    else if (dump_pend) begin dump_start<=1; dump_pend<=0; st<=S_DUMP; end
                end
                S_ARP: if (!arp_busy && !arp_start) st<=S_IDLE;
                S_ICMP: if (!icmp_busy && !icmp_start) st<=S_IDLE;
                S_UDP: if (!udp_busy && !udp_start) st<=S_IDLE;
                S_DUMP: if (!dump_busy && !dump_start) st<=S_IDLE;
                default: st<=S_IDLE;
            endcase
        end
    end
endmodule
