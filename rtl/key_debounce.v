module key_debounce #(						  // This is correct
    parameter CLK_FREQ    = 50_000_000,
    parameter DEBOUNCE_MS = 10
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key,        // 按键输入，DE2-115 按键低电平有效
    output reg  key_pulse   // 消抖后脉冲，高电平有效，持续 1 个时钟周期
                            // 仅在按键"抬起"时产生
);

    //--------------------------------------------------
    // 参数计算：20ms @ 50MHz = 1,000,000 个时钟周期
    //--------------------------------------------------
    localparam CNT_MAX   = (CLK_FREQ / 1000) * DEBOUNCE_MS;  // 1_000_000
    localparam CNT_WIDTH = 20;  // 2^20 = 1,048,576 > 1_000_000

    //--------------------------------------------------
    // 按键同步（三级移位寄存器，消除亚稳态 + 简单滤波）
    //--------------------------------------------------
    reg [2:0] key_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            key_sync <= 3'b111;
        else
            key_sync <= {key_sync[1:0], key};
    end
    wire key_in = key_sync[2];  // 同步后的按键信号

    //--------------------------------------------------
    // 状态机定义
    //--------------------------------------------------
    localparam IDLE             = 2'd0;
    localparam PRESS_DEBOUNCE   = 2'd1;
    localparam WAIT_RELEASE     = 2'd2;
    localparam RELEASE_DEBOUNCE = 2'd3;

    reg [1:0] state;
    reg [CNT_WIDTH-1:0] cnt;

    //--------------------------------------------------
    // 消抖计数器
    //--------------------------------------------------
    wire cnt_full = (cnt == CNT_MAX - 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt <= 'd0;
        else begin
            case (state)
                PRESS_DEBOUNCE, RELEASE_DEBOUNCE: begin
                    if (!cnt_full)
                        cnt <= cnt + 1'b1;
                    else
                        cnt <= 'd0;  // 计数满后清零，准备下次使用
                end
                default: cnt <= 'd0;
            endcase
        end
    end

    //--------------------------------------------------
    // 状态转移
    //--------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else begin
            case (state)
                IDLE: begin
                    if (key_in == 1'b0)      // 检测到按下（低电平）
                        state <= PRESS_DEBOUNCE;
                end

                PRESS_DEBOUNCE: begin
                    if (key_in == 1'b1)      // 按下过程中抖动，回 IDLE
                        state <= IDLE;
                    else if (cnt_full)       // 确认按下稳定
                        state <= WAIT_RELEASE;
                end

                WAIT_RELEASE: begin
                    if (key_in == 1'b1)      // 检测到抬起
                        state <= RELEASE_DEBOUNCE;
                end

                RELEASE_DEBOUNCE: begin
                    if (key_in == 1'b0)      // 抬起过程中抖动，回 WAIT_RELEASE
                        state <= WAIT_RELEASE;
                    else if (cnt_full)       // 确认抬起稳定，输出脉冲
                        state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    //--------------------------------------------------
    // 脉冲输出：仅在 RELEASE_DEBOUNCE 且计数满时产生单周期脉冲
    //--------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            key_pulse <= 1'b0;
        else
            key_pulse <= (state == RELEASE_DEBOUNCE) && cnt_full;
    end

endmodule