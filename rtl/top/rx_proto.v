// rx_proto.v — RX 协议解析 + 镜像 CRC + 字存储 + DUMP（RX_CLK 域）
`timescale 1ns/1ps

module rx_proto #(
    parameter [31:0] LOCAL_IP = 32'hC0A8_01C8,
    parameter [15:0] CMD_PORT = 16'd4661
)(
    input  wire        clk, input wire rst_n,
    input  wire        frame_start, input wire [7:0] frame_data,
    input  wire [10:0] frame_idx, input wire frame_byte_v,
    input  wire        frame_end, input wire [10:0] frame_len, input wire frame_err,

    output reg         arp_req, output reg [47:0] arp_sender_mac, output reg [31:0] arp_sender_ip,
    output reg         icmp_req, output reg [47:0] icmp_sender_mac, output reg [31:0] icmp_sender_ip,
    output reg [15:0]  icmp_id, output reg [15:0] icmp_seq, output reg [6:0] icmp_data_len,
    output reg         udp_cmd_req, output reg [47:0] udp_sender_mac, output reg [31:0] udp_sender_ip,
    output reg [15:0]  udp_src_port, output reg [7:0] udp_app_type, output reg [15:0] udp_app_seq,
    output wire        ram_wen, output wire [6:0] ram_waddr, output wire [7:0] ram_wdata,
    output reg [15:0]  rx_frame_cnt,
    output reg [31:0]  fin_actual_crc, output reg [31:0] fin_actual_words,
    output reg [7:0]   fin_status, output reg img_valid,
    // image_ram 写口
    output reg         img_wen, output reg [9:0] img_waddr, output reg [63:0] img_wdata,
    // DUMP
    output reg         dump_req, output reg [9:0] dump_start_out, output reg [3:0] dump_nwords_out
);
    reg [47:0] sa_mac; reg [7:0] et_hi; reg is_arp, is_ip;
    reg [7:0] arp_op_hi, arp_op_lo; reg [31:0] arp_sip_r, arp_tip_r;
    reg [7:0] ip_proto_r; reg [15:0] ip_total_r;
    reg [31:0] ip_src_r, ip_dst_r;
    reg [7:0] icmp_type_r; reg [15:0] icmp_id_r, icmp_seq_r;
    reg [15:0] udp_src_port_r, udp_dst_port_r;
    reg [15:0] app_magic_r; reg [7:0] app_type_r; reg [15:0] app_seq_r, app_nwords_r;
    reg is_load, is_finish, is_dump;
    reg [31:0] fin_exp_crc_r, fin_exp_words_r;
    reg [31:0] word_cnt; reg crc_init_pulse;
    wire [31:0] img_crc, img_crc_next;
    // 字组装
    reg [63:0] word_shift; reg [2:0] byte_cnt; reg [15:0] word_addr;
    // DUMP payload
    reg [31:0] dump_start_r, dump_nwords_r;

    wire [10:0] crc_end_idx = ip_total_r[10:0] + 11'd14;
    wire load_crc_en = frame_byte_v && is_load && (frame_idx >= 11'd50) && (frame_idx < crc_end_idx);

    assign ram_wen = frame_byte_v && (frame_idx >= 11'd42) && (frame_idx < 11'd170);
    assign ram_waddr = frame_idx[6:0] - 7'd42;
    assign ram_wdata = frame_data;

    wire arp_match = is_arp && (arp_op_hi==8'h00) && (arp_op_lo==8'h01) && (arp_tip_r==LOCAL_IP);
    wire icmp_match = is_ip && (ip_proto_r==8'h01) && (icmp_type_r==8'h08) && (ip_dst_r==LOCAL_IP);
    wire udp_match = is_ip && (ip_proto_r==8'h11) && (udp_dst_port_r==CMD_PORT) && (app_magic_r==16'h4746);

    crc32_eth_nib u_img_crc (.clk(clk), .rst_n(rst_n), .init(crc_init_pulse), .en(load_crc_en), .data(frame_data), .crc(img_crc), .crc_next(img_crc_next));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arp_req<=0; icmp_req<=0; udp_cmd_req<=0; dump_req<=0;
            sa_mac<=0; et_hi<=0; is_arp<=0; is_ip<=0;
            arp_op_hi<=0; arp_op_lo<=0; arp_sip_r<=0; arp_tip_r<=0;
            ip_proto_r<=0; ip_total_r<=0; ip_src_r<=0; ip_dst_r<=0;
            icmp_type_r<=0; icmp_id_r<=0; icmp_seq_r<=0;
            udp_src_port_r<=0; udp_dst_port_r<=0;
            app_magic_r<=0; app_type_r<=0; app_seq_r<=0; app_nwords_r<=0;
            is_load<=0; is_finish<=0; is_dump<=0;
            fin_exp_crc_r<=0; fin_exp_words_r<=0;
            word_cnt<=0; crc_init_pulse<=0; img_valid<=0;
            fin_actual_crc<=0; fin_actual_words<=0; fin_status<=0;
            word_shift<=0; byte_cnt<=0; word_addr<=0; img_wen<=0;
            img_waddr<=0; img_wdata<=0;
            dump_start_r<=0; dump_nwords_r<=0;
            dump_start_out<=0; dump_nwords_out<=0;
            rx_frame_cnt<=0;
            arp_sender_mac<=0; arp_sender_ip<=0;
            icmp_sender_mac<=0; icmp_sender_ip<=0;
            icmp_id<=0; icmp_seq<=0; icmp_data_len<=0;
            udp_sender_mac<=0; udp_sender_ip<=0; udp_src_port<=0; udp_app_type<=0; udp_app_seq<=0;
        end else begin
            arp_req<=0; icmp_req<=0; udp_cmd_req<=0; dump_req<=0; img_wen<=0;
            crc_init_pulse<=0;

            if (frame_start) begin
                is_arp<=0; is_ip<=0; arp_op_hi<=0; arp_op_lo<=0;
                ip_proto_r<=0; icmp_type_r<=0;
                app_magic_r<=0; app_type_r<=0;
                is_load<=0; is_finish<=0; is_dump<=0; byte_cnt<=0;
            end

            if (frame_byte_v) begin
                case (frame_idx)
                    11'd6 :sa_mac[47:40]<=frame_data; 11'd7 :sa_mac[39:32]<=frame_data;
                    11'd8 :sa_mac[31:24]<=frame_data; 11'd9 :sa_mac[23:16]<=frame_data;
                    11'd10:sa_mac[15:8] <=frame_data; 11'd11:sa_mac[7:0]  <=frame_data;
                    11'd12:et_hi<=frame_data;
                    11'd13:begin if(et_hi==8'h08&&frame_data==8'h06)is_arp<=1; if(et_hi==8'h08&&frame_data==8'h00)is_ip<=1; end
                    11'd16:ip_total_r[15:8]<=frame_data; 11'd17:ip_total_r[7:0]<=frame_data;
                    11'd20:arp_op_hi<=frame_data; 11'd21:arp_op_lo<=frame_data;
                    11'd23:ip_proto_r<=frame_data;
                    11'd26:ip_src_r[31:24]<=frame_data; 11'd27:ip_src_r[23:16]<=frame_data;
                    11'd28:begin arp_sip_r[31:24]<=frame_data; ip_src_r[15:8]<=frame_data; end
                    11'd29:begin arp_sip_r[23:16]<=frame_data; ip_src_r[7:0] <=frame_data; end
                    11'd30:begin arp_sip_r[15:8] <=frame_data; ip_dst_r[31:24]<=frame_data; end
                    11'd31:begin arp_sip_r[7:0]  <=frame_data; ip_dst_r[23:16]<=frame_data; end
                    11'd32:ip_dst_r[15:8]<=frame_data; 11'd33:ip_dst_r[7:0]<=frame_data;
                    11'd34:begin icmp_type_r<=frame_data; udp_src_port_r[15:8]<=frame_data; end
                    11'd35:udp_src_port_r[7:0]<=frame_data;
                    11'd36:udp_dst_port_r[15:8]<=frame_data; 11'd37:udp_dst_port_r[7:0]<=frame_data;
                    11'd38:begin arp_tip_r[31:24]<=frame_data; icmp_id_r[15:8]<=frame_data; end
                    11'd39:begin arp_tip_r[23:16]<=frame_data; icmp_id_r[7:0] <=frame_data; end
                    11'd40:begin arp_tip_r[15:8] <=frame_data; icmp_seq_r[15:8]<=frame_data; end
                    11'd41:begin arp_tip_r[7:0]  <=frame_data; icmp_seq_r[7:0] <=frame_data; end
                    11'd42:app_magic_r[15:8]<=frame_data; 11'd43:app_magic_r[7:0]<=frame_data;
                    11'd44:begin
                        app_type_r<=frame_data;
                        if(udp_match&&frame_data==8'h01)is_load<=1;
                        if(udp_match&&frame_data==8'h03)is_finish<=1;
                        if(udp_match&&frame_data==8'h07)is_dump<=1;
                    end
                    11'd46:app_seq_r[15:8]<=frame_data; 11'd47:app_seq_r[7:0]<=frame_data;
                    11'd48:begin
                        app_nwords_r[15:8]<=frame_data;
                        if(is_load) word_addr<=app_seq_r*16'd180;
                    end
                    11'd49:app_nwords_r[7:0]<=frame_data;
                    // FINISH / DUMP payload（idx 50-57）
                    11'd50:begin if(is_finish)fin_exp_words_r[7:0]<=frame_data; if(is_dump)dump_start_r[7:0]<=frame_data; end
                    11'd51:begin if(is_finish)fin_exp_words_r[15:8]<=frame_data; if(is_dump)dump_start_r[15:8]<=frame_data; end
                    11'd52:begin if(is_finish)fin_exp_words_r[23:16]<=frame_data; if(is_dump)dump_start_r[23:16]<=frame_data; end
                    11'd53:begin if(is_finish)fin_exp_words_r[31:24]<=frame_data; if(is_dump)dump_start_r[31:24]<=frame_data; end
                    11'd54:begin if(is_finish)fin_exp_crc_r[7:0]<=frame_data; if(is_dump)dump_nwords_r[7:0]<=frame_data; end
                    11'd55:begin if(is_finish)fin_exp_crc_r[15:8]<=frame_data; if(is_dump)dump_nwords_r[15:8]<=frame_data; end
                    11'd56:begin if(is_finish)fin_exp_crc_r[23:16]<=frame_data; if(is_dump)dump_nwords_r[23:16]<=frame_data; end
                    11'd57:begin if(is_finish)fin_exp_crc_r[31:24]<=frame_data; if(is_dump)dump_nwords_r[31:24]<=frame_data; end
                    default: ;
                endcase

                // 字组装（LOAD payload → 64 位字 → image_ram）
                if (load_crc_en) begin
                    word_shift <= {frame_data, word_shift[63:8]};
                    if (byte_cnt == 3'd7) begin
                        img_wen   <= 1'b1;
                        img_waddr <= word_addr[9:0];
                        img_wdata <= {frame_data, word_shift[63:8]};
                        word_addr <= word_addr + 16'd1;
                        byte_cnt  <= 3'd0;
                    end else byte_cnt <= byte_cnt + 3'd1;
                end
            end

            if (frame_end && !frame_err) begin
                rx_frame_cnt <= rx_frame_cnt + 16'd1;
                if (arp_match) begin arp_req<=1; arp_sender_mac<=sa_mac; arp_sender_ip<=arp_sip_r; end
                if (icmp_match) begin
                    icmp_req<=1; icmp_sender_mac<=sa_mac; icmp_sender_ip<=ip_src_r;
                    icmp_id<=icmp_id_r; icmp_seq<=icmp_seq_r;
                    icmp_data_len<=(ip_total_r>=16'd28)?(ip_total_r[6:0]-7'd28):7'd0;
                end
                if (udp_match) begin
                    udp_cmd_req<=1; udp_sender_mac<=sa_mac; udp_sender_ip<=ip_src_r;
                    udp_src_port<=udp_src_port_r; udp_app_type<=app_type_r; udp_app_seq<=app_seq_r;
                    if (app_type_r==8'h01) word_cnt<=word_cnt+{16'd0,app_nwords_r};
                    if (app_type_r==8'h03) begin
                        fin_actual_crc<=~img_crc; fin_actual_words<=word_cnt;
                        if(~img_crc==fin_exp_crc_r && word_cnt==fin_exp_words_r) begin fin_status<=8'd0; img_valid<=1; end
                        else fin_status<=(word_cnt!=fin_exp_words_r)?8'd2:8'd1;
                    end
                    if (app_type_r==8'h07) begin
                        dump_req<=1;
                        dump_start_out<=dump_start_r[9:0];
                        dump_nwords_out<=(dump_nwords_r>32'd10)?4'd10:dump_nwords_r[3:0];
                    end
                    if (app_type_r==8'h09) begin crc_init_pulse<=1; word_cnt<=0; img_valid<=0; end
                end
            end
        end
    end
endmodule
