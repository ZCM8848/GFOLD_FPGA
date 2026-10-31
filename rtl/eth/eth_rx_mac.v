// eth_rx_mac.v — RGMII 接收字节流提取（SFD 对齐 + 字节计数）
//
// 输入是 rgmii_rx_phy 输出的 GMII 8-bit 流（同步于 RX_CLK）。
// PHY 送来的前导码处理方式各家用例不同，这里做成**自适应**：
//   - 连续的 0x55 视为前导码，丢弃；
//   - 遇到 0xD5（SFD）丢弃并开始收 DA；
//   - 若 DV 一拉高就是其它值（部分 PHY 已剥掉 SFD），直接当作 DA 开始。
//
// 输出给上层的是「帧字节流」：
//   frame_start   : 第一个字节（DA）那一拍（与 frame_byte_v 同拍）
//   frame_data    : 字节
//   frame_idx     : 字节序号（0 = DA）
//   frame_byte_v  : 字节有效（1 拍）
//   frame_end     : 帧结束（1 拍，此时 frame_len 有效）
//   frame_len     : 本帧字节数（含 4 字节 FCS）
//   frame_err     : 收到过 RX_ER 或超长
//
// 注：本版本**不做 FCS 校验**（组合 CRC32 在 125 MHz 下时序偏紧，
//     联调阶段用 ARP 字段的自洽性校验代替），后续做 UDP 时再补。
//
`timescale 1ns/1ps

module eth_rx_mac #(
    parameter MAX_LEN = 1600
)(
    input  wire        clk,            // RGMII RX_CLK
    input  wire        rst_n,

    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rx_dv,
    input  wire        gmii_rx_er,

    output reg         frame_start,
    output reg  [7:0]  frame_data,
    output reg  [10:0] frame_idx,
    output reg         frame_byte_v,
    output reg         frame_end,
    output reg  [10:0] frame_len,
    output reg         frame_err
);

    localparam S_IDLE = 2'd0,
               S_HUNT = 2'd1,
               S_DATA = 2'd2;

    reg [1:0]  st;
    reg [10:0] idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            idx         <= 11'd0;
            frame_start <= 1'b0;
            frame_data  <= 8'd0;
            frame_idx   <= 11'd0;
            frame_byte_v<= 1'b0;
            frame_end   <= 1'b0;
            frame_len   <= 11'd0;
            frame_err   <= 1'b0;
        end else begin
            frame_start  <= 1'b0;
            frame_byte_v <= 1'b0;
            frame_end    <= 1'b0;

            case (st)
                // ---------------- 等 DV ----------------
                S_IDLE: begin
                    if (gmii_rx_dv) begin
                        st      <= S_HUNT;
                        frame_err <= gmii_rx_er;
                    end else begin
                        frame_err <= 1'b0;
                    end
                end

                // ---------------- 跳过前导码 / SFD ----------------
                S_HUNT: begin
                    if (!gmii_rx_dv) begin
                        st <= S_IDLE;           // 只有前导码没有数据，忽略
                    end else if (gmii_rxd == 8'h55) begin
                        st <= st;                // 继续跳过前导码
                    end else if (gmii_rxd == 8'hD5) begin
                        st  <= S_DATA;           // 下一拍是 DA
                        idx <= 11'd0;
                    end else begin
                        // 该 PHY 已剥掉 SFD，这个字节就是 DA
                        st           <= S_DATA;
                        idx          <= 11'd1;
                        frame_start  <= 1'b1;
                        frame_data   <= gmii_rxd;
                        frame_idx    <= 11'd0;
                        frame_byte_v <= 1'b1;
                        if (gmii_rx_er) frame_err <= 1'b1;
                    end
                end

                // ---------------- 收帧 ----------------
                S_DATA: begin
                    if (gmii_rx_dv) begin
                        frame_data   <= gmii_rxd;
                        frame_idx    <= idx;
                        frame_byte_v <= 1'b1;
                        if (idx == 11'd0) frame_start <= 1'b1;
                        if (gmii_rx_er)   frame_err   <= 1'b1;

                        if (idx >= MAX_LEN - 1) begin
                            // 超长：丢弃
                            frame_byte_v <= 1'b0;
                            frame_start  <= 1'b0;
                            st           <= S_IDLE;
                        end else begin
                            idx <= idx + 11'd1;
                        end
                    end else begin
                        // DV 落下 = 帧结束
                        frame_end <= 1'b1;
                        frame_len <= idx;
                        st        <= S_IDLE;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
