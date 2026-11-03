// udp_reply_tx.v — UDP ACK 帧发送器（clk_125 域）
//
// 收到 req 脉冲后发送一个 72 字节帧（含前导码）：
//   Eth + IPv4(total=44) + UDP(len=24) + 应用头(8) + payload(8) + pad(2) + FCS(4)
//   = 14 + 44 + 2 + 4 = 64 字节（最小帧）+ 前导码 8 = 72
//
// payload = {u32 rx_frame_count, u32 link_status}（小端）
//
`timescale 1ns/1ps

module udp_reply_tx #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01,
    parameter [31:0] LOCAL_IP  = 32'hC0A8_01C8,
    parameter [15:0] CMD_PORT  = 16'd4661
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        req,
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,
    input  wire [7:0]  ack_type,       // 0x02 = ACK
    input  wire [15:0] ack_seq,
    input  wire [31:0] payload_w0,      // LE 32-bit word at idx 58-61
    input  wire [31:0] payload_w1,      // LE 32-bit word at idx 62-65

    output wire        busy,
    output reg         done,
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en
);
    localparam [6:0] IDX_FCS = 7'd68;
    localparam [6:0] IDX_END = 7'd72;

    localparam S_IDLE = 2'd0, S_TX = 2'd1, S_IFG = 2'd2;

    reg [2:0]  st;
    reg [6:0]  idx;
    reg [3:0]  ifg_cnt;
    reg        crc_init;
    reg [7:0]  crc_data;

    assign busy = (st != S_IDLE);

    // IP checksum（组合）
    wire [31:0] ip_sum = 32'h4500 + 32'h002C + 32'h4011
                       + {16'd0, LOCAL_IP[31:16]} + {16'd0, LOCAL_IP[15:0]}
                       + {16'd0, dst_ip[31:16]}   + {16'd0, dst_ip[15:0]};
    wire [16:0] ip_f1  = ip_sum[15:0] + ip_sum[31:16];
    wire [16:0] ip_f2  = ip_f1[15:0] + {16'd0, ip_f1[16]};
    wire [15:0] ip_ck  = ~ip_f2[15:0];

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
            7'd7 : b = 8'hD5;
            7'd8  : b = dst_mac[47:40];
            7'd9  : b = dst_mac[39:32];
            7'd10 : b = dst_mac[31:24];
            7'd11 : b = dst_mac[23:16];
            7'd12 : b = dst_mac[15:8];
            7'd13 : b = dst_mac[7:0];
            7'd14 : b = LOCAL_MAC[47:40];
            7'd15 : b = LOCAL_MAC[39:32];
            7'd16 : b = LOCAL_MAC[31:24];
            7'd17 : b = LOCAL_MAC[23:16];
            7'd18 : b = LOCAL_MAC[15:8];
            7'd19 : b = LOCAL_MAC[7:0];
            7'd20 : b = 8'h08;
            7'd21 : b = 8'h00;
            // IPv4
            7'd22 : b = 8'h45;
            7'd23 : b = 8'h00;
            7'd24 : b = 8'h00;       // IP total hi = 44 = 0x002C
            7'd25 : b = 8'h2C;
            7'd26 : b = 8'h00;
            7'd27 : b = 8'h00;
            7'd28 : b = 8'h00;
            7'd29 : b = 8'h00;
            7'd30 : b = 8'h40;       // TTL 64
            7'd31 : b = 8'h11;       // UDP
            7'd32 : b = ip_ck[15:8];
            7'd33 : b = ip_ck[7:0];
            7'd34 : b = LOCAL_IP[31:24];
            7'd35 : b = LOCAL_IP[23:16];
            7'd36 : b = LOCAL_IP[15:8];
            7'd37 : b = LOCAL_IP[7:0];
            7'd38 : b = dst_ip[31:24];
            7'd39 : b = dst_ip[23:16];
            7'd40 : b = dst_ip[15:8];
            7'd41 : b = dst_ip[7:0];
            // UDP header
            7'd42 : b = CMD_PORT[15:8];   // src port
            7'd43 : b = CMD_PORT[7:0];
            7'd44 : b = dst_port[15:8];   // dst port
            7'd45 : b = dst_port[7:0];
            7'd46 : b = 8'h00;            // UDP length = 24 = 0x0018
            7'd47 : b = 8'h18;
            7'd48 : b = 8'h00;            // checksum = 0
            7'd49 : b = 8'h00;
            // 应用头
            7'd50 : b = 8'h47;            // magic hi
            7'd51 : b = 8'h46;            // magic lo
            7'd52 : b = ack_type;
            7'd53 : b = 8'h00;            // flags
            7'd54 : b = ack_seq[15:8];
            7'd55 : b = ack_seq[7:0];
            7'd56 : b = 8'h00;            // nwords = 1
            7'd57 : b = 8'h01;
            // payload (LE)
            7'd58 : b = payload_w0[7:0];
            7'd59 : b = payload_w0[15:8];
            7'd60 : b = payload_w0[23:16];
            7'd61 : b = payload_w0[31:24];
            7'd62 : b = payload_w1[7:0];
            7'd63 : b = payload_w1[15:8];
            7'd64 : b = payload_w1[23:16];
            7'd65 : b = payload_w1[31:24];
            // padding（66-67 = 0x00, default 已覆盖）
            default: b = 8'h00;
        endcase

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
                        gmii_txd   <= b;
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
