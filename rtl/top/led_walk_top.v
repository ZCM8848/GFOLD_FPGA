`timescale 1ns/1ps
// led_walk_top — identify the four user LEDs / FPGA pins.
// Lights exactly one of D10, E10, C6, D6 at a time, ~1.34 s per step:
//   step 0 -> D10 (O_led[0])
//   step 1 -> E10 (O_led[1])
//   step 2 -> C6  (O_led[2])
//   step 3 -> D6  (O_led[3])
// Watch which silkscreen LED1..LED4 lights in each step.
module led_walk_top (
    input  wire       I_sys_clk,
    output reg  [3:0] O_led
);
    reg [25:0] cnt;
    reg [1:0]  step;
    initial begin cnt = 26'd0; step = 2'd0; end

    always @(posedge I_sys_clk) begin
        cnt <= cnt + 1'b1;
        if (cnt == 26'h3FFFFFF) begin
            cnt  <= 26'd0;
            step <= step + 1'b1;
        end
    end

    always @(*) begin
        O_led = 4'b0000;
        case (step)
            2'd0: O_led[0] = 1'b1;
            2'd1: O_led[1] = 1'b1;
            2'd2: O_led[2] = 1'b1;
            default: O_led[3] = 1'b1;
        endcase
    end
endmodule
