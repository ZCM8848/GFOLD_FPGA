// arp_reply_tx.v — ARP 应答帧发送器（clk_125 域）
// 收到 req 脉冲后发送一个 64 字节最小帧（DA..FCS）的 ARP reply。
//
// 帧（idx 0..71）：
//   0..6   0x55 × 7        前导码
//   7      0xD5            SFD
//   8..13  DA = 请求方 MAC
//   14..19 SA = 本机 MAC（02:00:00:00:00:01）
//   20..21 Type 0x0806
//   22..49 ARP reply（28 字节）
//   50..67 填充 0x00（18 字节，凑到最小帧 46 字节 payload）
//   68..71 FCS（CRC32 小端）
//
`timescale 1ns/1ps

module arp_reply_tx #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01,
    parameter [31:0] LOCAL_IP  = 32'hC0A8_01C8
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        req,           // 脉冲启动
    input  wire [47:0] dst_mac,       // 请求方 MAC
    input  wire [31:0] dst_ip,        // 请求方 IP

    output wire        busy,
    output reg         done,          // 帧结束脉冲
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en
);
    localparam [6:0] IDX_FCS = 7'd68;
    localparam [6:0] IDX_END = 7'd72;

    localparam S_IDLE = 2'd0,
               S_TX   = 2'd1,
               S_IFG  = 2'd2;

    reg [1:0]  st;
    reg [6:0]  idx;
    reg [3:0]  ifg_cnt;
    reg        crc_init;
    reg [7:0]  crc_data;

    assign busy = (st != S_IDLE);

    // CRC
    wire [31:0] crc;
    wire [31:0] crc_next;
    wire        crc_en = (st == S_TX) && (idx >= 7'd9) && (idx <= IDX_FCS);

    // 字节选择
    reg [7:0] b;
    always @(*) begin
        b = 8'h00;
        case (idx)
            7'd0,7'd1,7'd2,7'd3,7'd4,7'd5,7'd6 : b = 8'h55;
            7'd7                                : b = 8'hD5;
            // DA
            7'd8  : b = dst_mac[47:40];
            7'd9  : b = dst_mac[39:32];
            7'd10 : b = dst_mac[31:24];
            7'd11 : b = dst_mac[23:16];
            7'd12 : b = dst_mac[15:8];
            7'd13 : b = dst_mac[7:0];
            // SA = LOCAL_MAC
            7'd14 : b = LOCAL_MAC[47:40];
            7'd15 : b = LOCAL_MAC[39:32];
            7'd16 : b = LOCAL_MAC[31:24];
            7'd17 : b = LOCAL_MAC[23:16];
            7'd18 : b = LOCAL_MAC[15:8];
            7'd19 : b = LOCAL_MAC[7:0];
            // Type ARP
            7'd20 : b = 8'h08;
            7'd21 : b = 8'h06;
            // ARP: hw type
            7'd22 : b = 8'h00;
            7'd23 : b = 8'h01;
            // proto type
            7'd24 : b = 8'h08;
            7'd25 : b = 8'h00;
            // hw len / proto len
            7'd26 : b = 8'h06;
            7'd27 : b = 8'h04;
            // opcode = reply
            7'd28 : b = 8'h00;
            7'd29 : b = 8'h02;
            // sender HW = LOCAL_MAC
            7'd30 : b = LOCAL_MAC[47:40];
            7'd31 : b = LOCAL_MAC[39:32];
            7'd32 : b = LOCAL_MAC[31:24];
            7'd33 : b = LOCAL_MAC[23:16];
            7'd34 : b = LOCAL_MAC[15:8];
            7'd35 : b = LOCAL_MAC[7:0];
            // sender IP = LOCAL_IP
            7'd36 : b = LOCAL_IP[31:24];
            7'd37 : b = LOCAL_IP[23:16];
            7'd38 : b = LOCAL_IP[15:8];
            7'd39 : b = LOCAL_IP[7:0];
            // target HW = dst_mac
            7'd40 : b = dst_mac[47:40];
            7'd41 : b = dst_mac[39:32];
            7'd42 : b = dst_mac[31:24];
            7'd43 : b = dst_mac[23:16];
            7'd44 : b = dst_mac[15:8];
            7'd45 : b = dst_mac[7:0];
            // target IP = dst_ip
            7'd46 : b = dst_ip[31:24];
            7'd47 : b = dst_ip[23:16];
            7'd48 : b = dst_ip[15:8];
            7'd49 : b = dst_ip[7:0];
            // padding（50..67 = 0x00，default 已覆盖）
            default: b = 8'h00;
        endcase

        // FCS
        if (idx == IDX_FCS)         b = ~crc_next[7:0];
        if (idx == IDX_FCS + 7'd1)  b = ~crc[15:8];
        if (idx == IDX_FCS + 7'd2)  b = ~crc[23:16];
        if (idx == IDX_FCS + 7'd3)  b = ~crc[31:24];
    end

    crc32_eth_nib u_crc (
        .clk      ( clk      ),
        .rst_n    ( rst_n    ),
        .init     ( crc_init ),
        .en       ( crc_en   ),
        .data     ( crc_data ),
        .crc      ( crc      ),
        .crc_next ( crc_next )
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) crc_data <= 8'h0;
        else        crc_data <= b;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            idx        <= 7'd0;
            ifg_cnt    <= 4'd0;
            gmii_tx_en <= 1'b0;
            gmii_txd   <= 8'h00;
            done       <= 1'b0;
            crc_init   <= 1'b0;
        end else begin
            done     <= 1'b0;
            crc_init <= 1'b0;

            case (st)
                S_IDLE: begin
                    gmii_tx_en <= 1'b0;
                    if (req) begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= b;       // byte(0) = 0x55
                        idx        <= 7'd1;
                        crc_init   <= 1'b1;
                        st         <= S_TX;
                    end
                end

                S_TX: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd   <= b;
                    if (idx == IDX_END) begin
                        gmii_tx_en <= 1'b0;
                        done       <= 1'b1;
                        ifg_cnt    <= 4'd0;
                        st         <= S_IFG;
                    end else begin
                        idx <= idx + 7'd1;
                    end
                end

                S_IFG: begin
                    gmii_tx_en <= 1'b0;
                    gmii_txd   <= 8'h00;
                    if (ifg_cnt >= 4'd11) st <= S_IDLE;
                    else ifg_cnt <= ifg_cnt + 4'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
