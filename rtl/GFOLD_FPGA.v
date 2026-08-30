// ============================================================
// GFOLD_FPGA — top-level for DE2-115 (EP4CE115F29C7)
// drs_iter + internal main/KKT RAM + external SRAM + CFI Flash
// + LCD status display (ITER/SCALE/MASS) + 7-seg + KEY control.
// KEY0 = reset, KEY1 = start (debounced edge).
// Solver input (COO/c/nb/zmask/band/g) is burned into CFI Flash and loaded
// by drs_iter's boot FSM at power-up (see gen_flash_image.py).
// ============================================================
`timescale 1ns/1ps
module GFOLD_FPGA (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG,
    // ---- LCD (HD44780 via lcd_driver) ----
    output wire [7:0]  LCD_DATA,
    output wire        LCD_EN, LCD_RS, LCD_RW, LCD_ON,
    // ---- 7-seg displays ----
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
    // ---- 2MB async SRAM (IS61WV25616BLL: COO + LDL band/history) ----
    output wire [19:0] SRAM_ADDR,
    inout  wire [15:0] SRAM_DQ,
    output wire        SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N,
    // ---- 8MB CFI Flash (boot: solver input burned in) ----
    output wire [22:0] FL_ADDR,
    inout  wire [7:0]  FL_DQ,
    output wire        FL_CE_N, FL_OE_N, FL_WE_N, FL_RESET_N, FL_WP_N,
    input  wire        FL_RY
);
    localparam N = 1100, M = 2107, NNZ = 4783, HB = 17;

    wire clk;
    wire pll_locked;
    pll30 u_pll(.areset(1'b0), .inclk0(CLOCK_50), .c0(clk), .locked(pll_locked));

    // ---- reset (KEY0 raw, async ok) + start (KEY1 debounced pulse) ----
    wire rst_n = KEY[0];   // KEY0 pressed = low = reset
    wire key1_pulse;
    key_debounce u_k1(.clk(clk), .rst_n(1'b1), .key(~KEY[1]), .key_pulse(key1_pulse));

    // ---- drs_iter ----
    wire start, done;
    wire band_ready;
    wire [15:0] iter;
    wire refactor, band_valid;
    wire [63:0] band_in;
    wire scale_valid;
    wire [63:0] scale_r;
    wire [16:0] ram_addr;
    wire [63:0] ram_wdata;
    wire        ram_we;
    wire [13:0] kkt_addr;
    wire [63:0] kkt_wdata;
    wire        kkt_we;

    // ---- main RAM (internal M9K, packed layout 31983 words: CB packed after
    // VPR, spmv workspace 2*LMAX, D_y M) ----
    // NOTE: must be a pure synchronous read (no combinational read anywhere, e.g.
    // gf_display) or Quartus fails to infer M9K and explodes into 276003 registers
    // (Error 276003). The mass display value is captured below from ram_wdata.
    reg [63:0] smem [0:31982];
    reg [63:0] ram_rdata;
    // ---- KKT RAM (sync read for M9K inference); kkt_solve max addr = 4*M-1 = 8427 ----
    reg [63:0] kmem [0:8427];
    reg [63:0] kkt_rdata;

    drs_iter #(.N(N), .M(M), .NNZ(NNZ), .HB(HB)) dut(
        .clk(clk), .rst_n(rst_n), .start(start),
        .refactor(refactor), .band_in(band_in), .band_valid(band_valid), .iter(iter),
        .scale_valid(scale_valid), .scale_r(scale_r),
        .ram_addr(ram_addr), .ram_wdata(ram_wdata), .ram_we(ram_we), .ram_rdata(ram_rdata),
        .kkt_addr(kkt_addr), .kkt_wdata(kkt_wdata), .kkt_we(kkt_we), .kkt_rdata(kkt_rdata),
        .done(done), .band_ready(band_ready),
        .SRAM_ADDR(SRAM_ADDR), .SRAM_DQ(SRAM_DQ),
        .SRAM_CE_N(SRAM_CE_N), .SRAM_OE_N(SRAM_OE_N), .SRAM_WE_N(SRAM_WE_N),
        .SRAM_UB_N(SRAM_UB_N), .SRAM_LB_N(SRAM_LB_N),
        .FL_ADDR(FL_ADDR), .FL_DQ(FL_DQ), .FL_CE_N(FL_CE_N), .FL_OE_N(FL_OE_N),
        .FL_WE_N(FL_WE_N), .FL_RESET_N(FL_RESET_N), .FL_WP_N(FL_WP_N), .FL_RY(FL_RY));

    // ---- main RAM read/write ----
    always @(posedge clk) ram_rdata <= smem[ram_addr];
    always @(posedge clk) if (ram_we) smem[ram_addr] <= ram_wdata;

    // mass display value: v[1099] (log final_mass) lives at address 1099 (V_BASE=0);
    // capture it when drs_iter writes it, avoiding a combinational read of smem.
    reg [63:0] disp_mass;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    disp_mass <= 64'd0;
        else if (ram_we && ram_addr == 17'd1099) disp_mass <= ram_wdata;
    end

    // ---- KKT RAM read/write ----
    always @(posedge clk) kkt_rdata <= kmem[kkt_addr];
    always @(posedge clk) if (kkt_we) kmem[kkt_addr] <= kkt_wdata;

    // ---- start: KEY1 edge (after reset, drs_iter in S_IDLE) ----
    reg started = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      started <= 0;
        else if (key1_pulse) started <= 1;
    end
    assign start = started && !done;

    // ---- inputs (no data loaded yet; refactor once at start) ----
    assign refactor    = 1'b1;
    assign band_valid  = 1'b0;
    assign band_in     = 64'd0;
    assign iter        = 16'd0;
    assign scale_valid = 1'b0;
    assign scale_r     = 64'h3FF0000000000000;

    // ---- iteration counter for display (increments on done) ----
    reg [15:0] iter_cnt;
    reg done_p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin iter_cnt <= 0; done_p <= 0; end
        else begin
            done_p <= done;
            if (done && !done_p) iter_cnt <= iter_cnt + 1;
        end
    end

    // ---- LCD status display ----
    wire [7:0] lcd_cmd_data; wire lcd_cmd_valid, lcd_is_data, lcd_busy;
    gf_display u_disp(.clk(clk), .rst_n(rst_n), .iter(iter_cnt),
        .scale(scale_r), .x1099(disp_mass),
        .cmd_data(lcd_cmd_data), .cmd_valid(lcd_cmd_valid), .is_data(lcd_is_data), .busy(lcd_busy));
    lcd_driver u_lcd(.clk(clk), .rst_n(rst_n),
        .cmd_data(lcd_cmd_data), .cmd_valid(lcd_cmd_valid), .is_data(lcd_is_data), .busy(lcd_busy),
        .LCD_DATA(LCD_DATA), .LCD_EN(LCD_EN), .LCD_RS(LCD_RS), .LCD_RW(LCD_RW), .LCD_ON(LCD_ON));

    // ---- 7-seg: HEX3..0 = iter (hex), HEX7..4 = scale exponent/pattern ----
    num_7seg u_h0(.c(iter_cnt[3:0]),  .hex(HEX0));
    num_7seg u_h1(.c(iter_cnt[7:4]),  .hex(HEX1));
    num_7seg u_h2(.c(iter_cnt[11:8]), .hex(HEX2));
    num_7seg u_h3(.c(iter_cnt[15:12]),.hex(HEX3));
    num_7seg u_h4(.c(4'hf), .hex(HEX4));
    num_7seg u_h5(.c(4'h0), .hex(HEX5));
    num_7seg u_h6(.c(4'h0), .hex(HEX6));
    num_7seg u_h7(.c(4'h0), .hex(HEX7));

    // ---- status LEDs ----
    assign LEDR = { done, started, 16'd0 };
    assign LEDG = { 8'd0, pll_locked };
endmodule
