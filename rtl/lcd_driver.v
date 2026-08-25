module lcd_driver (
    input  wire        clk,          // 50MHz
    input  wire        rst_n,
    // User interface
    input  wire [7:0]  cmd_data,     // 8-bit command or data
    input  wire        cmd_valid,    // single-cycle pulse
    input  wire        is_data,      // 1=data, 0=command
    output reg         busy,         // 1=controller busy
    // LCD hardware interface
    output reg  [7:0]  LCD_DATA,
    output reg         LCD_EN,
    output reg         LCD_RS,
    output wire        LCD_RW,
    output wire        LCD_ON
);

    assign LCD_RW = 1'b0;   // Write only
    assign LCD_ON = 1'b1;   // Keep LCD powered

    // Timing constants @ 50 MHz (20 ns / cycle)
    localparam SETUP_CYCLES   = 15;         // 300 ns  > 140 ns (t_AS min)
    localparam E_PULSE_CYCLES = 30;         // 600 ns  > 450 ns (t_PW min)
    localparam NORMAL_WAIT    = 2500;       // 50  us  > 37  us  (most instructions)
    localparam CLEAR_WAIT     = 90000;      // 1.8 ms  > 1.52 ms (clear/home)
    localparam INIT30A_WAIT   = 205000;     // 4.1 ms  > 4.1  ms (first 0x30 after power)
    localparam INIT30B_WAIT   = 5000;       // 100 us  > 100 us (subsequent 0x30)
    localparam POWER_WAIT     = 1000000;    // 20  ms  > 15  ms (power-on)

    // State encoding
    localparam S_PWR_WAIT  = 4'd0;
    localparam S_INIT0     = 4'd1;   // send 0x30, wait 4.1ms
    localparam S_INIT1     = 4'd2;   // send 0x30, wait 100us
    localparam S_INIT2     = 4'd3;   // send 0x30, wait 100us
    localparam S_INIT3     = 4'd4;   // send 0x38, wait 50us
    localparam S_INIT4     = 4'd5;   // send 0x0C, wait 50us
    localparam S_INIT5     = 4'd6;   // send 0x01, wait 1.8ms
    localparam S_INIT6     = 4'd7;   // send 0x06, wait 50us
    localparam S_IDLE      = 4'd8;
    localparam S_SETUP     = 4'd9;
    localparam S_SETUP_WAIT= 4'd10;
    localparam S_PULSE     = 4'd11;
    localparam S_HOLD      = 4'd12;
    localparam S_WAIT      = 4'd13;

    reg [3:0]  state;
    reg [3:0]  return_state;
    reg [7:0]  latched_data;
    reg        latched_is_data;
    reg [19:0] wait_cnt;
    reg [7:0]  e_cnt;

    // Main FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_PWR_WAIT;
            return_state   <= S_IDLE;
            LCD_EN         <= 1'b0;
            LCD_RS         <= 1'b0;
            LCD_DATA       <= 8'h00;
            busy           <= 1'b1;
            wait_cnt       <= 20'd0;
            e_cnt          <= 8'd0;
            latched_data   <= 8'd0;
            latched_is_data<= 1'b0;
        end else begin
            case (state)

                S_PWR_WAIT: begin
                    if (wait_cnt >= POWER_WAIT - 1) begin
                        wait_cnt <= 20'd0;
                        state    <= S_INIT0;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                //--------------------------------------------------
                // Init sequence: 0x30 x3 -> 0x38 -> 0x0C -> 0x01 -> 0x06
                //--------------------------------------------------
                S_INIT0: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h30;
                    return_state <= S_INIT1;
                    wait_cnt     <= INIT30A_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT1: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h30;
                    return_state <= S_INIT2;
                    wait_cnt     <= INIT30B_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT2: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h30;
                    return_state <= S_INIT3;
                    wait_cnt     <= INIT30B_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT3: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h38;      // Function set: 8-bit, 2-line, 5x7
                    return_state <= S_INIT4;
                    wait_cnt     <= NORMAL_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT4: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h0C;      // Display ON, cursor OFF, blink OFF
                    return_state <= S_INIT5;
                    wait_cnt     <= NORMAL_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT5: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h01;      // Clear display
                    return_state <= S_INIT6;
                    wait_cnt     <= CLEAR_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                S_INIT6: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h06;      // Entry mode: increment, no shift
                    return_state <= S_IDLE;
                    wait_cnt     <= NORMAL_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                //--------------------------------------------------
                // User write transaction
                //--------------------------------------------------
                S_IDLE: begin
                    if (cmd_valid) begin
                        latched_data    <= cmd_data;
                        latched_is_data <= is_data;
                        busy            <= 1'b1;
                        state           <= S_SETUP;
                    end
                end

                S_SETUP: begin
                    LCD_RS   <= latched_is_data;
                    LCD_DATA <= latched_data;
                    return_state <= S_IDLE;
                    wait_cnt     <= NORMAL_WAIT;
                    e_cnt        <= 8'd0;
                    state        <= S_SETUP_WAIT;
                end

                //--------------------------------------------------
                // Common physical-layer timing
                //--------------------------------------------------
                S_SETUP_WAIT: begin
                    if (e_cnt >= SETUP_CYCLES - 1) begin
                        e_cnt <= 8'd0;
                        state <= S_PULSE;
                    end else begin
                        e_cnt <= e_cnt + 1'b1;
                    end
                end

                S_PULSE: begin
                    LCD_EN <= 1'b1;
                    if (e_cnt >= E_PULSE_CYCLES - 1) begin
                        e_cnt <= 8'd0;
                        state <= S_HOLD;
                    end else begin
                        e_cnt <= e_cnt + 1'b1;
                    end
                end

                S_HOLD: begin
                    LCD_EN <= 1'b0;
                    state  <= S_WAIT;
                end

                S_WAIT: begin
                    if (wait_cnt <= 1) begin
                        busy  <= (return_state == S_IDLE) ? 1'b0 : 1'b1;
                        state <= return_state;
                    end else begin
                        wait_cnt <= wait_cnt - 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
