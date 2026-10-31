// rx_proto.v — RX 协议解析器（RX_CLK 域）
// 解析 eth_rx_mac 字节流，检测 ARP request 与 ICMP echo request。
//
// 帧字节索引（frame_idx，0 = DA）：
//   6..11  Ethernet SA（请求方 MAC）
//   12..13 ethertype（0x0806=ARP / 0x0800=IPv4）
//   16..17 IP total length
//   23     IP protocol（0x01=ICMP）
//   26..29 IP src IP
//   30..33 IP dst IP
//   28..29 ARP opcode（0x0001=request）
//   34     ICMP type（0x08=echo request）
//   36..39 ARP sender IP
//   38..39 ICMP identifier
//   40..41 ICMP sequence
//   42+    ICMP data → 写入 dc_ram
//   46..49 ARP target IP
//
// 注意：idx 28-29 在 ARP 帧里是 opcode，在 IPv4 帧里是 IP src IP 低两字节。
// 两者 ethertype 互斥，寄存器并行锁存但只有对应帧的检测条件会成立。
//
`timescale 1ns/1ps

module rx_proto #(
    parameter [31:0] LOCAL_IP = 32'hC0A8_01C8
)(
    input  wire        clk,            // RX_CLK
    input  wire        rst_n,

    input  wire        frame_start,
    input  wire [7:0]  frame_data,
    input  wire [10:0] frame_idx,
    input  wire        frame_byte_v,
    input  wire        frame_end,
    input  wire [10:0] frame_len,
    input  wire        frame_err,

    // ARP request 检出（frame_end 拍脉冲）
    output reg         arp_req,
    output reg  [47:0] arp_sender_mac,
    output reg  [31:0] arp_sender_ip,

    // ICMP echo request 检出（frame_end 拍脉冲）
    output reg         icmp_req,
    output reg  [47:0] icmp_sender_mac,
    output reg  [31:0] icmp_sender_ip,
    output reg  [15:0] icmp_id,
    output reg  [15:0] icmp_seq,
    output reg  [6:0]  icmp_data_len,

    // data RAM 写口（组合输出，与 frame_byte_v 同拍）
    output wire        ram_wen,
    output wire [6:0]  ram_waddr,
    output wire [7:0]  ram_wdata
);
    // ---- 逐帧锁存 ----
    reg [47:0] sa_mac;
    reg [7:0]  et_hi;
    reg        is_arp, is_ip;
    reg [7:0]  arp_op_hi, arp_op_lo;
    reg [31:0] arp_sip_r, arp_tip_r;
    reg [7:0]  ip_proto_r;
    reg [15:0] ip_total_r;
    reg [31:0] ip_src_r, ip_dst_r;
    reg [7:0]  icmp_type_r;
    reg [15:0] icmp_id_r, icmp_seq_r;

    // RAM 写口
    assign ram_wen   = frame_byte_v && (frame_idx >= 11'd42) && (frame_idx < 11'd170);
    assign ram_waddr = frame_idx[6:0] - 7'd42;
    assign ram_wdata = frame_data;

    // 检测条件
    wire arp_match   = is_arp && (arp_op_hi == 8'h00) && (arp_op_lo == 8'h01)
                       && (arp_tip_r == LOCAL_IP);
    wire icmp_match  = is_ip && (ip_proto_r == 8'h01) && (icmp_type_r == 8'h08)
                       && (ip_dst_r == LOCAL_IP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arp_req <= 1'b0;  icmp_req <= 1'b0;
            sa_mac <= 48'd0;  et_hi <= 8'd0;
            is_arp <= 1'b0;   is_ip <= 1'b0;
            arp_op_hi <= 8'd0; arp_op_lo <= 8'd0;
            arp_sip_r <= 32'd0; arp_tip_r <= 32'd0;
            ip_proto_r <= 8'd0; ip_total_r <= 16'd0;
            ip_src_r <= 32'd0; ip_dst_r <= 32'd0;
            icmp_type_r <= 8'd0; icmp_id_r <= 16'd0; icmp_seq_r <= 16'd0;
            arp_sender_mac <= 48'd0; arp_sender_ip <= 32'd0;
            icmp_sender_mac <= 48'd0; icmp_sender_ip <= 32'd0;
            icmp_id <= 16'd0; icmp_seq <= 16'd0; icmp_data_len <= 7'd0;
        end else begin
            arp_req  <= 1'b0;
            icmp_req <= 1'b0;

            if (frame_start) begin
                is_arp <= 1'b0;  is_ip <= 1'b0;
                arp_op_hi <= 8'd0; arp_op_lo <= 8'd0;
                ip_proto_r <= 8'd0; icmp_type_r <= 8'd0;
            end

            if (frame_byte_v) begin
                case (frame_idx)
                    // Ethernet SA（idx 6-11）
                    11'd6 : sa_mac[47:40] <= frame_data;
                    11'd7 : sa_mac[39:32] <= frame_data;
                    11'd8 : sa_mac[31:24] <= frame_data;
                    11'd9 : sa_mac[23:16] <= frame_data;
                    11'd10: sa_mac[15:8]  <= frame_data;
                    11'd11: sa_mac[7:0]   <= frame_data;
                    // ethertype
                    11'd12: et_hi <= frame_data;
                    11'd13: begin
                        if (et_hi == 8'h08 && frame_data == 8'h06) is_arp <= 1'b1;
                        if (et_hi == 8'h08 && frame_data == 8'h00) is_ip <= 1'b1;
                    end
                    // IP total length（idx 16-17）
                    11'd16: ip_total_r[15:8] <= frame_data;
                    11'd17: ip_total_r[7:0]  <= frame_data;
                    // ARP opcode（idx 28-29）/ IP src IP 低两字节
                    11'd28: begin arp_op_hi <= frame_data; ip_src_r[15:8] <= frame_data; end
                    11'd29: begin arp_op_lo <= frame_data; ip_src_r[7:0]  <= frame_data; end
                    // IP protocol（idx 23）
                    11'd23: ip_proto_r <= frame_data;
                    // IP src IP 高两字节（idx 26-27）
                    11'd26: ip_src_r[31:24] <= frame_data;
                    11'd27: ip_src_r[23:16] <= frame_data;
                    // IP dst IP（idx 30-33）
                    11'd30: ip_dst_r[31:24] <= frame_data;
                    11'd31: ip_dst_r[23:16] <= frame_data;
                    11'd32: ip_dst_r[15:8]  <= frame_data;
                    11'd33: ip_dst_r[7:0]   <= frame_data;
                    // ARP sender IP（idx 36-39）/ ICMP id（idx 38-39）
                    11'd36: arp_sip_r[31:24] <= frame_data;
                    11'd37: arp_sip_r[23:16] <= frame_data;
                    11'd38: begin arp_sip_r[15:8] <= frame_data; icmp_id_r[15:8] <= frame_data; end
                    11'd39: begin arp_sip_r[7:0]  <= frame_data; icmp_id_r[7:0]  <= frame_data; end
                    // ARP target IP（idx 46-49）
                    11'd46: arp_tip_r[31:24] <= frame_data;
                    11'd47: arp_tip_r[23:16] <= frame_data;
                    11'd48: arp_tip_r[15:8]  <= frame_data;
                    11'd49: arp_tip_r[7:0]   <= frame_data;
                    // ICMP type（idx 34）
                    11'd34: icmp_type_r <= frame_data;
                    // ICMP seq（idx 40-41）
                    11'd40: icmp_seq_r[15:8] <= frame_data;
                    11'd41: icmp_seq_r[7:0]  <= frame_data;
                    default: ;
                endcase
            end

            // frame_end 时判定
            if (frame_end && !frame_err) begin
                if (arp_match) begin
                    arp_req          <= 1'b1;
                    arp_sender_mac   <= sa_mac;
                    arp_sender_ip    <= arp_sip_r;
                end
                if (icmp_match) begin
                    icmp_req         <= 1'b1;
                    icmp_sender_mac  <= sa_mac;
                    icmp_sender_ip   <= ip_src_r;
                    icmp_id          <= icmp_id_r;
                    icmp_seq         <= icmp_seq_r;
                    // data_len = ip_total - 28（IP header 20 + ICMP header 8）
                    icmp_data_len    <= (ip_total_r >= 16'd28) ? (ip_total_r[6:0] - 7'd28) : 7'd0;
                end
            end
        end
    end
endmodule
