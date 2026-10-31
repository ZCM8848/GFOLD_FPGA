// phy_rst.v — 以太网 PHY 复位序列器
//
// YT8511 复位时序要求（datasheet §1.3 / §6）：
//   - RESET_N 低电平脉宽 ≥ 150 µs（推荐 ≥ 1 ms）
//   - 复位释放后等 5 ms 内部寄存器稳定
//   - 复位期间 MDIO 不可用
//
// 本模块实现：上电后拉低 RESET_N 一段时间，释放后等待，然后置 rst_done
//
`timescale 1ns/1ps

module phy_rst #(
    parameter RESET_TIME_MS = 2,         // 复位低电平持续 2 ms（YT8511 ≥150µs，留 13× 余量）
    parameter POST_RST_MS   = 5,         // 复位释放后等待 5 ms（datasheet 推荐）
    parameter CLK_FREQ_HZ   = 50_000_000
)(
    input  wire clk,
    input  wire rst_n,                   // 外部复位（如 PLL lock / 上电复位）
    output reg  phy_rst_n,               // 直接连 PHY 的 RESET_N 引脚
    output reg  rst_done                 // 复位序列完成后拉高
);
    localparam RST_CYCLES  = (RESET_TIME_MS * CLK_FREQ_HZ) / 1000;  // 2ms @ 50MHz = 100K
    localparam WAIT_CYCLES = (POST_RST_MS   * CLK_FREQ_HZ) / 1000; // 5ms @ 50MHz = 250K

    reg [31:0] cnt;
    reg [1:0]  state;
    localparam S_ASSERT_RST = 2'd0;   // 拉 RESET_N 低
    localparam S_WAIT       = 2'd1;   // 释放，等内部稳定
    localparam S_DONE       = 2'd2;   // 完成

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_ASSERT_RST;
            cnt        <= 0;
            phy_rst_n  <= 1'b0;
            rst_done   <= 1'b0;
        end else begin
            case (state)
                S_ASSERT_RST: begin
                    phy_rst_n <= 1'b0;
                    rst_done  <= 1'b0;
                    if (cnt >= RST_CYCLES - 1) begin
                        cnt   <= 0;
                        state <= S_WAIT;
                    end else begin
                        cnt   <= cnt + 1;
                    end
                end
                S_WAIT: begin
                    phy_rst_n <= 1'b1;
                    rst_done  <= 1'b0;
                    if (cnt >= WAIT_CYCLES - 1) begin
                        cnt   <= 0;
                        state <= S_DONE;
                    end else begin
                        cnt   <= cnt + 1;
                    end
                end
                S_DONE: begin
                    phy_rst_n <= 1'b1;
                    rst_done  <= 1'b1;
                end
            endcase
        end
    end
endmodule
