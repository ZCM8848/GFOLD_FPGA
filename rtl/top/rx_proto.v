// rx_proto.v — RX 协议解析器（RX_CLK 域）
// 解析 eth_rx_mac 字节流，检测 ARP request、ICMP echo request、UDP 命令。
//
// 帧字节索引（frame_idx，0 = DA，不含前导码/SFD）：
//   6..11  Ethernet SA（请求方 MAC）
//   12..13 ethertype（0x0806=ARP / 0x0800=IPv4）
//   16..17 IP total length
//   20..21 ARP opcode
//   23     IP protocol（0x01=ICMP / 0x11=UDP）
//   26..33 IP src/dst IP
//   28..31 ARP sender IP
//   34     ICMP type / UDP src port hi
//   35     UDP src port lo
//   36..37 UDP dst port
//   38..41 ARP target IP / ICMP id+seq
//   42..49 应用头（magic / type / flags / seq / nwords）
//   42+    UDP/ICMP data → 写入 dc_ram
//
`timescale 1ns/1ps

module rx_proto #(
    parameter [31:0] LOCAL_IP   = 32'hC0A8_01C8,
    parameter [15:0] CMD_PORT   = 16'd4661      // 0x1235
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

    // ICMP echo request 检出
    output reg         icmp_req,
    output reg  [47:0] icmp_sender_mac,
    output reg  [31:0] icmp_sender_ip,
    output reg  [15:0] icmp_id,
    output reg  [15:0] icmp_seq,
    output reg  [6:0]  icmp_data_len,

    // UDP 命令检出
    output reg         udp_cmd_req,
    output reg  [47:0] udp_sender_mac,
    output reg  [31:0] udp_sender_ip,
    output reg  [15:0] udp_src_port,
    output reg  [7:0]  udp_app_type,
    output reg  [15:0] udp_app_seq,

    // data RAM 写口
    output wire        ram_wen,
    output wire [6:0]  ram_waddr,
    output wire [7:0]  ram_wdata,

    // 帧计数（供状态上报）
    output reg  [15:0] rx_frame_cnt
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
    // UDP
    reg [15:0] udp_src_port_r, udp_dst_port_r;
    reg [15:0] app_magic_r;
    reg [7:0]  app_type_r;
    reg [15:0] app_seq_r;

    // RAM 写口
    assign ram_wen   = frame_byte_v && (frame_idx >= 11'd42) && (frame_idx < 11'd170);
    assign ram_waddr = frame_idx[6:0] - 7'd42;
    assign ram_wdata = frame_data;

    // 检测条件
    wire arp_match   = is_arp && (arp_op_hi == 8'h00) && (arp_op_lo == 8'h01)
                       && (arp_tip_r == LOCAL_IP);
    wire icmp_match  = is_ip && (ip_proto_r == 8'h01) && (icmp_type_r == 8'h08)
                       && (ip_dst_r == LOCAL_IP);
    wire udp_match   = is_ip && (ip_proto_r == 8'h11)
                       && (udp_dst_port_r == CMD_PORT)
                       && (app_magic_r == 16'h4746);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arp_req <= 1'b0;  icmp_req <= 1'b0;  udp_cmd_req <= 1'b0;
            sa_mac <= 48'd0;  et_hi <= 8'd0;
            is_arp <= 1'b0;   is_ip <= 1'b0;
            arp_op_hi <= 8'd0; arp_op_lo <= 8'd0;
            arp_sip_r <= 32'd0; arp_tip_r <= 32'd0;
            ip_proto_r <= 8'd0; ip_total_r <= 16'd0;
            ip_src_r <= 32'd0; ip_dst_r <= 32'd0;
            icmp_type_r <= 8'd0; icmp_id_r <= 16'd0; icmp_seq_r <= 16'd0;
            udp_src_port_r <= 16'd0; udp_dst_port_r <= 16'd0;
            app_magic_r <= 16'd0; app_type_r <= 8'd0; app_seq_r <= 16'd0;
            arp_sender_mac <= 48'd0; arp_sender_ip <= 32'd0;
            icmp_sender_mac <= 48'd0; icmp_sender_ip <= 32'd0;
            icmp_id <= 16'd0; icmp_seq <= 16'd0; icmp_data_len <= 7'd0;
            udp_sender_mac <= 48'd0; udp_sender_ip <= 32'd0;
            udp_src_port <= 16'd0; udp_app_type <= 8'd0; udp_app_seq <= 16'd0;
            rx_frame_cnt <= 16'd0;
        end else begin
            arp_req     <= 1'b0;
            icmp_req    <= 1'b0;
            udp_cmd_req <= 1'b0;

            if (frame_start) begin
                is_arp <= 1'b0;  is_ip <= 1'b0;
                arp_op_hi <= 8'd0; arp_op_lo <= 8'd0;
                ip_proto_r <= 8'd0; icmp_type_r <= 8'd0;
                app_magic_r <= 16'd0; app_type_r <= 8'd0;
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
                    // ARP opcode（idx 20-21）
                    11'd20: arp_op_hi <= frame_data;
                    11'd21: arp_op_lo <= frame_data;
                    // IP protocol（idx 23）
                    11'd23: ip_proto_r <= frame_data;
                    // IP src IP 高两字节（idx 26-27）
                    11'd26: ip_src_r[31:24] <= frame_data;
                    11'd27: ip_src_r[23:16] <= frame_data;
                    // ARP sender IP / IP src IP 低两字节（idx 28-29）
                    11'd28: begin arp_sip_r[31:24] <= frame_data; ip_src_r[15:8] <= frame_data; end
                    11'd29: begin arp_sip_r[23:16] <= frame_data; ip_src_r[7:0]  <= frame_data; end
                    // ARP sender IP 低两字节 / IP dst IP 高两字节（idx 30-31）
                    11'd30: begin arp_sip_r[15:8]  <= frame_data; ip_dst_r[31:24] <= frame_data; end
                    11'd31: begin arp_sip_r[7:0]   <= frame_data; ip_dst_r[23:16] <= frame_data; end
                    // IP dst IP 低两字节（idx 32-33）
                    11'd32: ip_dst_r[15:8]  <= frame_data;
                    11'd33: ip_dst_r[7:0]   <= frame_data;
                    // ICMP type / UDP src port hi（idx 34）
                    11'd34: begin icmp_type_r <= frame_data; udp_src_port_r[15:8] <= frame_data; end
                    // UDP src port lo（idx 35）
                    11'd35: udp_src_port_r[7:0] <= frame_data;
                    // UDP dst port（idx 36-37）
                    11'd36: udp_dst_port_r[15:8] <= frame_data;
                    11'd37: udp_dst_port_r[7:0]  <= frame_data;
                    // ARP target IP / ICMP id（idx 38-39）
                    11'd38: begin arp_tip_r[31:24] <= frame_data; icmp_id_r[15:8] <= frame_data; end
                    11'd39: begin arp_tip_r[23:16] <= frame_data; icmp_id_r[7:0]  <= frame_data; end
                    // ARP target IP 低两字节 / ICMP seq（idx 40-41）
                    11'd40: begin arp_tip_r[15:8]  <= frame_data; icmp_seq_r[15:8] <= frame_data; end
                    11'd41: begin arp_tip_r[7:0]   <= frame_data; icmp_seq_r[7:0]  <= frame_data; end
                    // 应用头 magic（idx 42-43）
                    11'd42: app_magic_r[15:8] <= frame_data;
                    11'd43: app_magic_r[7:0]  <= frame_data;
                    // 应用头 type（idx 44）
                    11'd44: app_type_r <= frame_data;
                    // 应用头 seq（idx 46-47）
                    11'd46: app_seq_r[15:8] <= frame_data;
                    11'd47: app_seq_r[7:0]  <= frame_data;
                    default: ;
                endcase
            end

            // frame_end 时判定
            if (frame_end && !frame_err) begin
                rx_frame_cnt <= rx_frame_cnt + 16'd1;
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
                    icmp_data_len    <= (ip_total_r >= 16'd28) ? (ip_total_r[6:0] - 7'd28) : 7'd0;
                end
                if (udp_match) begin
                    udp_cmd_req      <= 1'b1;
                    udp_sender_mac   <= sa_mac;
                    udp_sender_ip    <= ip_src_r;
                    udp_src_port     <= udp_src_port_r;
                    udp_app_type     <= app_type_r;
                    udp_app_seq      <= app_seq_r;
                end
            end
        end
    end
endmodule
