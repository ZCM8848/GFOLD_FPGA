// tx_arbiter.v — ARP / ICMP / UDP 发送仲裁（clk_125 域）
// 优先级 ARP > ICMP > UDP。请求脉冲锁存为 pending，发送完成后处理下一个。
`timescale 1ns/1ps

module tx_arbiter (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        arp_req,
    input  wire        icmp_req,
    input  wire        udp_req,

    output reg         arp_start,
    output reg         icmp_start,
    output reg         udp_start,

    input  wire        arp_busy,
    input  wire        icmp_busy,
    input  wire        udp_busy,

    output wire        tx_busy
);
    localparam S_IDLE = 2'd0,
               S_ARP  = 2'd1,
               S_ICMP = 2'd2,
               S_UDP  = 2'd3;

    reg [1:0]  st;
    reg        arp_pend, icmp_pend, udp_pend;

    assign tx_busy = (st != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            arp_pend   <= 1'b0;
            icmp_pend  <= 1'b0;
            udp_pend   <= 1'b0;
            arp_start  <= 1'b0;
            icmp_start <= 1'b0;
            udp_start  <= 1'b0;
        end else begin
            arp_start  <= 1'b0;
            icmp_start <= 1'b0;
            udp_start  <= 1'b0;

            if (arp_req)   arp_pend  <= 1'b1;
            if (icmp_req)  icmp_pend <= 1'b1;
            if (udp_req)   udp_pend  <= 1'b1;

            case (st)
                S_IDLE: begin
                    if (arp_pend) begin
                        arp_start  <= 1'b1;
                        arp_pend   <= 1'b0;
                        st         <= S_ARP;
                    end else if (icmp_pend) begin
                        icmp_start <= 1'b1;
                        icmp_pend  <= 1'b0;
                        st         <= S_ICMP;
                    end else if (udp_pend) begin
                        udp_start  <= 1'b1;
                        udp_pend   <= 1'b0;
                        st         <= S_UDP;
                    end
                end

                S_ARP: begin
                    if (!arp_busy && !arp_start) st <= S_IDLE;
                end

                S_ICMP: begin
                    if (!icmp_busy && !icmp_start) st <= S_IDLE;
                end

                S_UDP: begin
                    if (!udp_busy && !udp_start) st <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
