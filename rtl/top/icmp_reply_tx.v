// icmp_reply_tx.v — ICMP echo reply 帧发送器（clk_125 域）
//
// 流程：req → S_CKSUM（读 RAM 累加 ICMP 校验和）→ S_CKFOLD → S_TX → S_IFG
//
// 帧布局（idx 0..end）：
//   0..6    0x55 × 7        前导码
//   7       0xD5            SFD
//   8..13   DA = 请求方 MAC
//   14..19  SA = 本机 MAC
//   20..21  Type 0x0800
//   22..41  IPv4 头 20B（total = data_len + 28，proto 0x01，校验和动态算）
//   42..49  ICMP 头 8B（type 0x00 reply，code 0x00，校验和预计算，id，seq）
//   50..50+data_len-1  data（从 dc_ram 读）
//   50+data_len..+3    FCS
//
`timescale 1ns/1ps

module icmp_reply_tx #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01,
    parameter [31:0] LOCAL_IP  = 32'hC0A8_01C8
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        req,
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] icmp_id,
    input  wire [15:0] icmp_seq,
    input  wire [6:0]  data_len,

    // RAM 读口
    output wire [6:0]  ram_raddr,
    input  wire [7:0]  ram_rdata,

    output wire        busy,
    output reg         done,
    output reg  [7:0]  gmii_txd,
    output reg         gmii_tx_en
);
    localparam S_IDLE   = 3'd0,
               S_CKSUM  = 3'd1,
               S_CKFOLD = 3'd2,
               S_TX     = 3'd3,
               S_IFG    = 3'd4;

    reg [2:0]  st;
    reg [10:0] idx;
    reg [6:0]  ck_idx;
    reg        ck_even;
    reg [31:0] cksum;
    reg [15:0] icmp_cksum_r;
    reg [3:0]  ifg_cnt;
    reg        crc_init;
    reg [7:0]  crc_data;

    // 锁存的参数
    reg [47:0] dst_mac_r;
    reg [31:0] dst_ip_r;
    reg [15:0] icmp_id_r, icmp_seq_r;
    reg [6:0]  data_len_r;
    reg [15:0] ip_total_r, ip_ck_r;

    assign busy = (st != S_IDLE);

    // RAM 读地址：CKSUM 阶段读 ck_idx，TX 阶段读 idx-50
    assign ram_raddr = (st == S_CKSUM) ? ck_idx :
                       (st == S_TX)    ? (idx[6:0] - 7'd50) :
                                         7'd0;

    // IP total length & checksum（组合）
    wire [15:0] ip_total = {9'd0, data_len_r} + 16'd28;
    wire [31:0] ip_sum   = 32'h4500 + {16'd0, ip_total} + 32'h4001
                         + {16'd0, LOCAL_IP[31:16]} + {16'd0, LOCAL_IP[15:0]}
                         + {16'd0, dst_ip_r[31:16]} + {16'd0, dst_ip_r[15:0]};
    wire [16:0] ip_f1    = ip_sum[15:0] + ip_sum[31:16];
    wire [16:0] ip_f2    = ip_f1[15:0] + {16'd0, ip_f1[16]};
    wire [15:0] ip_ck    = ~ip_f2[15:0];

    // FCS 位置
    wire [10:0] idx_fcs  = {4'd0, data_len_r} + 11'd50;
    wire [10:0] idx_end  = {4'd0, data_len_r} + 11'd54;

    // CRC
    wire [31:0] crc;
    wire [31:0] crc_next;
    wire        crc_en  = (st == S_TX) && (idx >= 11'd9) && (idx <= idx_fcs);

    // 字节选择
    reg [7:0] b;
    always @(*) begin
        b = 8'h00;
        case (idx)
            11'd0,11'd1,11'd2,11'd3,11'd4,11'd5,11'd6 : b = 8'h55;
            11'd7 : b = 8'hD5;
            // DA
            11'd8  : b = dst_mac_r[47:40];
            11'd9  : b = dst_mac_r[39:32];
            11'd10 : b = dst_mac_r[31:24];
            11'd11 : b = dst_mac_r[23:16];
            11'd12 : b = dst_mac_r[15:8];
            11'd13 : b = dst_mac_r[7:0];
            // SA
            11'd14 : b = LOCAL_MAC[47:40];
            11'd15 : b = LOCAL_MAC[39:32];
            11'd16 : b = LOCAL_MAC[31:24];
            11'd17 : b = LOCAL_MAC[23:16];
            11'd18 : b = LOCAL_MAC[15:8];
            11'd19 : b = LOCAL_MAC[7:0];
            // Type IPv4
            11'd20 : b = 8'h08;
            11'd21 : b = 8'h00;
            // IPv4 header
            11'd22 : b = 8'h45;
            11'd23 : b = 8'h00;
            11'd24 : b = ip_total_r[15:8];
            11'd25 : b = ip_total_r[7:0];
            11'd26 : b = 8'h00;          // ID
            11'd27 : b = 8'h00;
            11'd28 : b = 8'h00;          // flags/frag
            11'd29 : b = 8'h00;
            11'd30 : b = 8'h40;          // TTL 64
            11'd31 : b = 8'h01;          // ICMP
            11'd32 : b = ip_ck_r[15:8];
            11'd33 : b = ip_ck_r[7:0];
            11'd34 : b = LOCAL_IP[31:24];
            11'd35 : b = LOCAL_IP[23:16];
            11'd36 : b = LOCAL_IP[15:8];
            11'd37 : b = LOCAL_IP[7:0];
            11'd38 : b = dst_ip_r[31:24];
            11'd39 : b = dst_ip_r[23:16];
            11'd40 : b = dst_ip_r[15:8];
            11'd41 : b = dst_ip_r[7:0];
            // ICMP header
            11'd42 : b = 8'h00;          // type = echo reply
            11'd43 : b = 8'h00;          // code
            11'd44 : b = icmp_cksum_r[15:8];
            11'd45 : b = icmp_cksum_r[7:0];
            11'd46 : b = icmp_id_r[15:8];
            11'd47 : b = icmp_id_r[7:0];
            11'd48 : b = icmp_seq_r[15:8];
            11'd49 : b = icmp_seq_r[7:0];
            default: b = 8'h00;
        endcase

        // data 区域
        if (idx >= 11'd50 && idx < idx_fcs) b = ram_rdata;

        // FCS
        if (idx == idx_fcs)        b = ~crc_next[7:0];
        if (idx == idx_fcs + 11'd1) b = ~crc[15:8];
        if (idx == idx_fcs + 11'd2) b = ~crc[23:16];
        if (idx == idx_fcs + 11'd3) b = ~crc[31:24];
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

    // 状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= S_IDLE;
            idx           <= 11'd0;
            ck_idx        <= 7'd0;
            ck_even       <= 1'b1;
            cksum         <= 32'd0;
            icmp_cksum_r  <= 16'd0;
            ifg_cnt       <= 4'd0;
            crc_init      <= 1'b0;
            gmii_tx_en    <= 1'b0;
            gmii_txd      <= 8'h00;
            done          <= 1'b0;
            dst_mac_r     <= 48'd0;
            dst_ip_r      <= 32'd0;
            icmp_id_r     <= 16'd0;
            icmp_seq_r    <= 16'd0;
            data_len_r    <= 7'd0;
            ip_total_r    <= 16'd0;
            ip_ck_r       <= 16'd0;
        end else begin
            done     <= 1'b0;
            crc_init <= 1'b0;

            case (st)
                S_IDLE: begin
                    gmii_tx_en <= 1'b0;
                    if (req) begin
                        dst_mac_r  <= dst_mac;
                        dst_ip_r   <= dst_ip;
                        icmp_id_r  <= icmp_id;
                        icmp_seq_r <= icmp_seq;
                        data_len_r <= data_len;
                        // 预载 ICMP 校验和累加器：id + seq（type=0, code=0, cksum=0 贡献 0）
                        cksum  <= {16'd0, icmp_id} + {16'd0, icmp_seq};
                        ck_idx <= 7'd0;
                        ck_even<= 1'b1;
                        if (data_len == 7'd0) st <= S_CKFOLD;
                        else                  st <= S_CKSUM;
                    end
                end

                S_CKSUM: begin
                    // ram_raddr = ck_idx（组合），ram_rdata = mem[ck_idx]（组合）
                    if (ck_even)
                        cksum <= cksum + {ram_rdata, 8'h00};
                    else
                        cksum <= cksum + {24'd0, ram_rdata};
                    ck_idx  <= ck_idx + 7'd1;
                    ck_even <= ~ck_even;
                    if (ck_idx + 7'd1 >= data_len_r) st <= S_CKFOLD;
                end

                S_CKFOLD: begin
                    // 折叠到 16 位
                    icmp_cksum_r <= ~fold_f2[15:0];
                    ip_total_r   <= ip_total;
                    ip_ck_r      <= ip_ck;
                    st  <= S_TX;
                    idx <= 11'd0;
                    crc_init <= 1'b1;
                end

                S_TX: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd   <= b;
                    if (idx == idx_end) begin
                        gmii_tx_en <= 1'b0;
                        done       <= 1'b1;
                        ifg_cnt    <= 4'd0;
                        st         <= S_IFG;
                    end else begin
                        idx <= idx + 11'd1;
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

    // 折叠到 16 位（组合）
    wire [16:0] fold_f1 = cksum[15:0] + cksum[31:16];
    wire [16:0] fold_f2 = fold_f1[15:0] + {16'd0, fold_f1[16]};

endmodule
