`timescale 1ns/1ps
// gf_display: 16x2 LCD status display for the GFOLD_FPGA DRS solver (AGC DSKY style).
// Line1: "ITER 00012345"            (decimal iteration, latched per refresh)
// Line2: page 0 "MASS 401DF4C2"     8 hex = x1099 (log final_mass)
//         page 1 "SCAL 3F5124..."   8 hex = scale_cur
//         page 2 "TAU  3F6E..."     8 hex = root_plus tau
//         page 3 "ITER 00012345"    decimal iteration
// Refreshes every ~10ms (@30MHz). Talks to the HD44780 driver (lcd_driver.v).
module gf_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] iter,        // current DRS iteration
    input  wire [1:0]  page,        // 0=MASS,1=SCALE,2=TAU,3=ITER
    input  wire [63:0] x1099,       // x[1099] = log final_mass (float64 bits)
    input  wire [63:0] scale_cur,   // adaptive scale (float64 bits)
    input  wire [63:0] tau,         // root_plus tau (float64 bits)
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
    reg [19:0] timer;               // ~10ms refresh period @30MHz

    reg [15:0] iter_hold;           // latched at refresh start (stable digits)

    wire is_addr_step = (step == 6'd0) || (step == 6'd17);
    wire [7:0] addr_cmd = (step == 6'd0) ? 8'h80 : 8'hC0;
    wire [4:0] char_idx = (step < 6'd17) ? (step[4:0] - 5'd1) : (step[4:0] - 5'd2);

    // ---- decimal digits of iter_hold (0..65535) via successive subtraction ----
    reg [15:0] cd_rem;
    reg [4:0]  cd_i;
    reg [4:0]  dgt [0:4];
    reg [3:0]  cd_state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cd_state <= 0; cd_rem <= 0; cd_i <= 0;
            dgt[0] <= 0; dgt[1] <= 0; dgt[2] <= 0; dgt[3] <= 0; dgt[4] <= 0;
        end else case (cd_state)
        0: begin cd_rem <= iter_hold; cd_i <= 0; cd_state <= 1; end
        1: begin dgt[cd_i] <= 0; cd_state <= 2; end
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
            dgt[4] <= cd_rem[3:0];
            if (cd_i == 4) cd_state <= 4;
            else begin cd_i <= cd_i + 1; cd_state <= 1; end
        end
        4: cd_state <= 0;
        endcase
    end

    // hex nibble -> ASCII
    function [7:0] hex2ascii;
        input [3:0] n;
        begin
            hex2ascii = (n < 10) ? (8'h30 + n) : (8'h37 + n);
        end
    endfunction

    // Line2 data source for the 8-hex display (pages 0/1/2)
    wire [63:0] line2_val = (page == 2'd0) ? x1099 :
                            (page == 2'd1) ? scale_cur : tau;

    // character lookup for the 16x2 layout
    function [7:0] get_char;
        input [4:0] idx;
        reg [7:0] c;
        begin
            case (idx)
            // Line1: "ITER xxxxx     "
            5'd0:  c = "I";
            5'd1:  c = "T";
            5'd2:  c = "E";
            5'd3:  c = "R";
            5'd4:  c = " ";
            5'd5:  c = 8'h30 + dgt[0];
            5'd6:  c = 8'h30 + dgt[1];
            5'd7:  c = 8'h30 + dgt[2];
            5'd8:  c = 8'h30 + dgt[3];
            5'd9:  c = 8'h30 + dgt[4];
            5'd10: c = " ";
            5'd11: c = " ";
            5'd12: c = " ";
            5'd13: c = " ";
            5'd14: c = " ";
            5'd15: c = " ";
            // Line2: label(4) + " " + data(8) + "   "
            5'd16: c = (page == 2'd0) ? "M" : (page == 2'd1) ? "S" : (page == 2'd2) ? "T" : "I";
            5'd17: c = (page == 2'd0) ? "A" : (page == 2'd1) ? "C" : (page == 2'd2) ? "A" : "T";
            5'd18: c = (page == 2'd0) ? "S" : (page == 2'd1) ? "A" : (page == 2'd2) ? "U" : "E";
            5'd19: c = (page == 2'd0) ? "S" : (page == 2'd1) ? "L" : (page == 2'd2) ? " " : "R";
            5'd20: c = " ";
            5'd21: c = (page == 2'd3) ? (8'h30 + dgt[0]) : hex2ascii(line2_val[63:60]);
            5'd22: c = (page == 2'd3) ? (8'h30 + dgt[1]) : hex2ascii(line2_val[59:56]);
            5'd23: c = (page == 2'd3) ? (8'h30 + dgt[2]) : hex2ascii(line2_val[55:52]);
            5'd24: c = (page == 2'd3) ? (8'h30 + dgt[3]) : hex2ascii(line2_val[51:48]);
            5'd25: c = (page == 2'd3) ? (8'h30 + dgt[4]) : hex2ascii(line2_val[47:44]);
            5'd26: c = (page == 2'd3) ? " " : hex2ascii(line2_val[43:40]);
            5'd27: c = (page == 2'd3) ? " " : hex2ascii(line2_val[39:36]);
            5'd28: c = (page == 2'd3) ? " " : hex2ascii(line2_val[35:32]);
            5'd29: c = " ";
            5'd30: c = " ";
            5'd31: c = " ";
            default: c = " ";
            endcase
            get_char = c;
        end
    endfunction

    // refresh request pulse every ~10ms @30MHz (300000 cycles)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin timer <= 0; refresh_req <= 0; end
        else begin
            refresh_req <= 1'b0;   // single-cycle pulse
            if (timer >= 20'd300000) begin timer <= 0; refresh_req <= 1'b1; end
            else timer <= timer + 1;
        end
    end

    // refresh FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ds_state <= DS_IDLE; step <= 0; cmd_valid <= 0;
        end else begin
            cmd_valid <= 0;
            case (ds_state)
            DS_IDLE: begin
                if (refresh_req) begin
                    iter_hold <= iter;      // latch one snapshot for the whole refresh
                    step <= 0; ds_state <= DS_SEND;
                end
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
                    if (step == 6'd33) ds_state <= DS_IDLE;
                    else begin step <= step + 1; ds_state <= DS_SEND; end
                end
            end
            default: ds_state <= DS_IDLE;
            endcase
        end
    end
endmodule
