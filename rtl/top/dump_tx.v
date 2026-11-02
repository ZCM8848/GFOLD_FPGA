// dump_tx.v — DUMP_DATA UDP 帧发送器（clk_125 域）
// 预读最多 10 个 64 位字，发送一个 UDP/IP 帧。
`timescale 1ns/1ps

module dump_tx #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01,
    parameter [31:0] LOCAL_IP  = 32'hC0A8_01C8,
    parameter [15:0] CMD_PORT  = 16'd4661,
    parameter [3:0]  MAX_WORDS = 4'd10
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,
    input  wire [9:0]  dump_start,
    input  wire [3:0]  dump_nwords,
    // image_ram 读口
    output reg  [9:0]  ram_raddr,
    input  wire [63:0] ram_rdata,
    output wire        busy,
    output reg         done,
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en
);
    localparam S_IDLE  = 3'd0,
               S_RWAIT = 3'd1,
               S_READ  = 3'd2,
               S_TX    = 3'd3,
               S_IFG   = 3'd4;

    reg [2:0]  st;
    reg [10:0] idx;
    reg [3:0]  ifg_cnt;
    reg        crc_init;
    reg [3:0]  rd_idx;      // 预读索引
    reg [63:0] dbuf [0:9];    // 字缓冲
    reg [3:0]  nwords_r;

    assign busy = (st != S_IDLE);

    // 帧布局参数
    wire [15:0] ip_total = {8'd0, nwords_r} * 16'd8 + 16'd36;  // 20+8+8+nwords*8
    wire [15:0] udp_len   = {8'd0, nwords_r} * 16'd8 + 16'd16;  // 8+8+nwords*8
    wire [10:0] idx_payload = 11'd58;  // payload 起始
    wire [10:0] idx_fcs = 11'd58 + {{7'd0, nwords_r} << 3};     // FCS 起始
    wire [10:0] idx_end = idx_fcs + 11'd4;

    // IP checksum
    wire [31:0] ip_sum = 32'h4500 + {16'd0, ip_total} + 32'h4011
                       + {16'd0, LOCAL_IP[31:16]} + {16'd0, LOCAL_IP[15:0]}
                       + {16'd0, dst_ip[31:16]}   + {16'd0, dst_ip[15:0]};
    wire [16:0] ip_f1  = ip_sum[15:0] + ip_sum[31:16];
    wire [16:0] ip_f2  = ip_f1[15:0] + {16'd0, ip_f1[16]};
    wire [15:0] ip_ck  = ~ip_f2[15:0];

    // CRC
    wire [31:0] crc, crc_next;
    wire        crc_en = (st == S_TX) && (idx >= 11'd8) && (idx < idx_fcs);

    // payload 字节偏移
    wire [10:0] pay_off = idx - idx_payload;       // 0..nwords*8-1
    wire [3:0]  word_idx = pay_off[10:3];           // pay_off / 8
    wire [2:0]  byte_idx = pay_off[2:0];            // pay_off % 8

    reg [7:0] b;
    always @(*) begin
        b = 8'h00;
        case (idx)
            11'd0,11'd1,11'd2,11'd3,11'd4,11'd5,11'd6 : b = 8'h55;
            11'd7 : b = 8'hD5;
            11'd8 : b = dst_mac[47:40]; 11'd9 : b = dst_mac[39:32];
            11'd10: b = dst_mac[31:24]; 11'd11: b = dst_mac[23:16];
            11'd12: b = dst_mac[15:8];  11'd13: b = dst_mac[7:0];
            11'd14: b = LOCAL_MAC[47:40]; 11'd15: b = LOCAL_MAC[39:32];
            11'd16: b = LOCAL_MAC[31:24]; 11'd17: b = LOCAL_MAC[23:16];
            11'd18: b = LOCAL_MAC[15:8];  11'd19: b = LOCAL_MAC[7:0];
            11'd20: b = 8'h08; 11'd21: b = 8'h00;
            11'd22: b = 8'h45; 11'd23: b = 8'h00;
            11'd24: b = ip_total[15:8]; 11'd25: b = ip_total[7:0];
            11'd26: b = 8'h00; 11'd27: b = 8'h00;
            11'd28: b = 8'h00; 11'd29: b = 8'h00;
            11'd30: b = 8'h40; 11'd31: b = 8'h11;
            11'd32: b = ip_ck[15:8]; 11'd33: b = ip_ck[7:0];
            11'd34: b = LOCAL_IP[31:24]; 11'd35: b = LOCAL_IP[23:16];
            11'd36: b = LOCAL_IP[15:8];  11'd37: b = LOCAL_IP[7:0];
            11'd38: b = dst_ip[31:24];   11'd39: b = dst_ip[23:16];
            11'd40: b = dst_ip[15:8];    11'd41: b = dst_ip[7:0];
            11'd42: b = CMD_PORT[15:8];  11'd43: b = CMD_PORT[7:0];
            11'd44: b = dst_port[15:8];  11'd45: b = dst_port[7:0];
            11'd46: b = udp_len[15:8];   11'd47: b = udp_len[7:0];
            11'd48: b = 8'h00; 11'd49: b = 8'h00;
            // app header
            11'd50: b = 8'h47; 11'd51: b = 8'h46;
            11'd52: b = 8'h08;  // type = DUMP_DATA
            11'd53: b = 8'h00;
            11'd54: b = 8'h00;  11'd55: b = 8'h00;  // seq
            11'd56: b = 8'h00;  11'd57: b = {4'd0, nwords_r};  // nwords BE
            default: b = 8'h00;
        endcase
        // payload
        if (idx >= idx_payload && idx < idx_fcs)
            b = dbuf[word_idx][byte_idx*8 +: 8];
        // FCS
        if (idx == idx_fcs)       b = ~crc[7:0];
        if (idx == idx_fcs + 11'd1) b = ~crc[15:8];
        if (idx == idx_fcs + 11'd2) b = ~crc[23:16];
        if (idx == idx_fcs + 11'd3) b = ~crc[31:24];
    end

    crc32_eth_nib u_crc (
        .clk(clk), .rst_n(rst_n), .init(crc_init), .en(crc_en),
        .data(b), .crc(crc), .crc_next(crc_next));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; idx <= 0; ifg_cnt <= 0; crc_init <= 0;
            rd_idx <= 0; nwords_r <= 0; ram_raddr <= 0;
            gmii_tx_en <= 0; gmii_txd <= 0; done <= 0;
        end else begin
            done <= 0; crc_init <= 0;
            case (st)
                S_IDLE: begin
                    gmii_tx_en <= 0;
                    if (req) begin
                        nwords_r <= dump_nwords;
                        rd_idx <= 0;
                        ram_raddr <= dump_start;
                        st <= S_RWAIT;
                    end
                end
                S_RWAIT: st <= S_READ;
                S_READ: begin
                    // 每拍读一个字（raddr 已设好，rdata 下一拍有效）
                    dbuf[rd_idx] <= ram_rdata;
                    if (rd_idx + 1 >= nwords_r) begin
                        st <= S_TX;
                        idx <= 0;
                        crc_init <= 1;
                    end else begin
                        rd_idx <= rd_idx + 1;
                        ram_raddr <= dump_start + rd_idx + 1;
                    end
                end
                S_TX: begin
                    gmii_tx_en <= 1;
                    gmii_txd <= b;
                    if (idx == idx_end) begin
                        gmii_tx_en <= 0; done <= 1;
                        ifg_cnt <= 0; st <= S_IFG;
                    end else idx <= idx + 1;
                end
                S_IFG: begin
                    gmii_tx_en <= 0; gmii_txd <= 0;
                    if (ifg_cnt >= 4'd11) st <= S_IDLE;
                    else ifg_cnt <= ifg_cnt + 1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
