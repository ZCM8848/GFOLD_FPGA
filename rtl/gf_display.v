`timescale 1ns/1ps
// gf_display: 16x2 LCD status display for the GFOLD_FPGA DRS solver.
// Line1: "ITER 00012345"   Line2: "SCALE 3FE80000  MASS 40C2..."
// Refreshes every ~50ms (50MHz). Talks to the HD44780 driver (lcd_driver.v,
// cmd_data/cmd_valid/is_data/busy handshake) - same protocol as lcd_status_display.
module gf_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] iter,        // current DRS iteration
    input  wire [63:0] scale,       // current scale (float64 bits)
    input  wire [63:0] x1099,       // x[1099] = log final_mass (float64 bits)
    // lcd_driver interface
    output reg  [7:0]  cmd_data,
    output reg         cmd_valid,
    output reg         is_data,
    input  wire        busy
);
    localparam DS_IDLE=0, DS_SEND=1, DS_WAIT_ACK=2, DS_WAIT_DONE=3, DS_NEXT=4;
    reg [2:0] ds_state;
    reg [5:0] step;                 // 0..33 refresh sequence
    reg       refresh_req;
    reg [24:0] timer;               // ~50ms refresh period

    wire is_addr_step = (step == 6'd0) || (step == 6'd17);
    wire [7:0] addr_cmd = (step == 6'd0) ? 8'h80 : 8'hC0;
    wire [4:0] char_idx = (step < 6'd17) ? (step[4:0] - 5'd1) : (step[4:0] - 5'd2);

    // ---- decimal digits of iter (0..65535) via successive subtraction ----
    reg [15:0] iter_work;
    reg [15:0] dig [0:4];
    wire [15:0] d0_rem, d1_rem, d2_rem, d3_rem, d4_rem;
    // digit 0 (ten-thousands): how many 10000s
    reg [4:0] dgt [0:4];
    // simple: compute on refresh with a small FSM inside get_char is complex;
    // instead precompute digits each refresh (5 subtract-loops, 16 cyc each = fast)
    reg [3:0] cd_state;
    reg [15:0] cd_rem;
    reg [4:0] cd_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cd_state <= 0; cd_rem <= 0; cd_i <= 0; dgt[0] <= 0; dgt[1] <= 0;
            dgt[2] <= 0; dgt[3] <= 0; dgt[4] <= 0;
        end else case (cd_state)
        0: begin cd_rem <= iter; cd_i <= 0; cd_state <= 1; end
        1: begin  // digit cd_i: count how many divs
            dgt[cd_i] <= 0; cd_state <= 2;
        end
        2: begin
            case (cd_i)
            0: if (cd_rem >= 16'd10000) begin cd_rem <= cd_rem - 16'd10000; dgt[0] <= dgt[0] + 1; end else cd_state <= 3;
            1: if (cd_rem >= 16'd1000)  begin cd_rem <= cd_rem - 16'd1000;  dgt[1] <= dgt[1] + 1; end else cd_state <= 3;
            2: if (cd_rem >= 16'd100)   begin cd_rem <= cd_rem - 16'd100;   dgt[2] <= dgt[2] + 1; end else cd_state <= 3;
            3: if (cd_rem >= 16'd10)    begin cd_rem <= cd_rem - 16'd10;    dgt[3] <= dgt[3] + 1; end else cd_state <= 3;
            default: cd_state <= 3;
            endcase
        end
        3: begin
            dgt[4] <= cd_rem[3:0];      // units
            if (cd_i == 4) begin cd_state <= 4; end
            else begin cd_i <= cd_i + 1; cd_state <= 1; end
        end
        4: begin cd_state <= 0; end
        endcase
    end

    // hex nibble -> ASCII
    function [7:0] hex2ascii;
        input [3:0] n;
        begin
            hex2ascii = (n < 10) ? (8'h30 + n) : (8'h37 + n);
        end
    endfunction

    // character lookup for the 16x2 layout
    function [7:0] get_char;
        input [4:0] idx;
        begin
            case (idx)
            5'd0:  get_char = "I";
            5'd1:  get_char = "T";
            5'd2:  get_char = "E";
            5'd3:  get_char = "R";
            5'd4:  get_char = " ";
            5'd5:  get_char = 8'h30 + dgt[0];
            5'd6:  get_char = 8'h30 + dgt[1];
            5'd7:  get_char = 8'h30 + dgt[2];
            5'd8:  get_char = 8'h30 + dgt[3];
            5'd9:  get_char = 8'h30 + dgt[4];
            5'd10: get_char = " ";
            5'd11: get_char = "S";
            5'd12: get_char = "C";
            5'd13: get_char = "A";
            5'd14: get_char = "L";
            5'd15: get_char = "E";
            // line 2 (idx 16..31): " 3FE80000  MASS 40C2..."
            5'd16: get_char = " ";
            5'd17: get_char = hex2ascii(scale[63:60]);
            5'd18: get_char = hex2ascii(scale[59:56]);
            5'd19: get_char = hex2ascii(scale[55:52]);
            5'd20: get_char = hex2ascii(scale[51:48]);
            5'd21: get_char = hex2ascii(scale[47:44]);
            5'd22: get_char = hex2ascii(scale[43:40]);
            5'd23: get_char = hex2ascii(scale[39:36]);
            5'd24: get_char = hex2ascii(scale[35:32]);
            5'd25: get_char = " ";
            5'd26: get_char = "M";
            5'd27: get_char = "A";
            5'd28: get_char = "S";
            5'd29: get_char = "S";
            5'd30: get_char = " ";
            5'd31: get_char = hex2ascii(x1099[63:60]);
            default: get_char = " ";
            endcase
        end
    endfunction

    // refresh request every ~50ms
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin timer <= 0; refresh_req <= 0; end
        else begin
            if (timer >= 25'd2500000) begin timer <= 0; refresh_req <= 1; end
            else timer <= timer + 1;
        end
    end

    // refresh FSM (same handshake as lcd_status_display)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ds_state <= DS_IDLE; step <= 0; cmd_valid <= 0;
        end else begin
            cmd_valid <= 0;
            case (ds_state)
            DS_IDLE: begin
                if (refresh_req) begin step <= 0; ds_state <= DS_SEND; end
            end
            DS_SEND: begin
                cmd_data <= is_addr_step ? addr_cmd : get_char(char_idx);
                is_data  <= ~is_addr_step;
                cmd_valid <= 1;
                ds_state <= DS_WAIT_ACK;
            end
            DS_WAIT_ACK: begin
                if (!busy) ds_state <= DS_WAIT_DONE;
            end
            DS_WAIT_DONE: begin
                if (!busy) begin
                    if (step == 6'd33) begin ds_state <= DS_IDLE; end
                    else begin step <= step + 1; ds_state <= DS_SEND; end
                end
            end
            default: ds_state <= DS_IDLE;
            endcase
        end
    end
endmodule
