`timescale 1ns/1ps
// sram64_ctrl: 64-bit word interface to the DE2-115 async SRAM
// (IS61WV25616BLL, 1M x 16-bit, 2MB). A 64-bit word = 4 x 16-bit halfwords;
// one word access = 6 cycles (async tAA ~10ns @ 50MHz, address->data next cycle).
// Protocol: pulse req (we=1: write wdata to waddr; we=0: read waddr).
// busy goes high the cycle after req and falls when the access completes;
// rdata is valid when busy falls (read).
// Multiple clients share one instance through an external arbiter.
module sram64_ctrl #(parameter AW = 18) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,
    input  wire        we,
    input  wire [AW-1:0] waddr,
    input  wire [63:0] wdata,
    output reg         busy,
    output reg  [63:0] rdata,
    // async SRAM pins
    output wire [19:0] SRAM_ADDR,
    inout  wire [15:0] SRAM_DQ,
    output wire        SRAM_CE_N,
    output wire        SRAM_OE_N,
    output wire        SRAM_WE_N,
    output wire        SRAM_UB_N,
    output wire        SRAM_LB_N
);
    reg [1:0]  hw;
    reg [19:0] base;
    reg [63:0] wd_lat;
    reg [15:0] rd0, rd1, rd2, rd3;
    reg        oe_n, we_n, ce_n, dq_oe;
    reg [15:0] dq_out;
    reg [3:0]  st;

    assign SRAM_ADDR = base + hw;
    assign SRAM_DQ   = dq_oe ? dq_out : 16'hz;
    assign SRAM_CE_N = ce_n;
    assign SRAM_OE_N = oe_n;
    assign SRAM_WE_N = we_n;
    assign SRAM_UB_N = 1'b0;
    assign SRAM_LB_N = 1'b0;

    localparam S_IDLE=0, S_RD0=1, S_RD1=2, S_RD2=3, S_RD3=4, S_RD4=5, S_RDD=6,
               S_WR0=7, S_WR1=8, S_WR2=9, S_WR3=10, S_WR4=11, S_WR5=12,
               S_WR6=13, S_WR7=14, S_WRD=15;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 0; rdata <= 0;
            hw <= 0; base <= 0; wd_lat <= 0;
            rd0 <= 0; rd1 <= 0; rd2 <= 0; rd3 <= 0;
            oe_n <= 1; we_n <= 1; ce_n <= 1; dq_oe <= 0; dq_out <= 0;
        end else begin
            case (st)
            S_IDLE: begin
                if (req) begin
                    base <= {waddr, 2'b00};
                    wd_lat <= wdata;
                    hw <= 0;
                    busy <= 1;
                    ce_n <= 0;
                    if (we) st <= S_WR0;
                    else begin oe_n <= 0; st <= S_RD0; end
                end
            end
            // ---- read: addr@T0 -> data@T+1..T+4, rdata@T+5 ----
            S_RD0: begin st <= S_RD1; end              // addr=base+0, OE low
            S_RD1: begin rd0 <= SRAM_DQ; hw <= 1; st <= S_RD2; end
            S_RD2: begin rd1 <= SRAM_DQ; hw <= 2; st <= S_RD3; end
            S_RD3: begin rd2 <= SRAM_DQ; hw <= 3; st <= S_RD4; end
            S_RD4: begin rd3 <= SRAM_DQ; oe_n <= 1; ce_n <= 1; st <= S_RDD; end
            S_RDD: begin rdata <= {rd3, rd2, rd1, rd0}; busy <= 0; st <= S_IDLE; end
            // ---- write: WE low 1 cycle per halfword (2 states each) ----
            S_WR0: begin
                dq_oe <= 1; dq_out <= wd_lat[15:0];
                we_n <= 0; st <= S_WR1;                // write hw0 @ base+0
            end
            S_WR1: begin
                we_n <= 1; st <= S_WR2;                // end hw0 write
            end
            S_WR2: begin
                hw <= 1; dq_out <= wd_lat[31:16];
                we_n <= 0; st <= S_WR3;                // write hw1 @ base+1
            end
            S_WR3: begin
                we_n <= 1; st <= S_WR4;                // end hw1 write
            end
            S_WR4: begin
                hw <= 2; dq_out <= wd_lat[47:32];
                we_n <= 0; st <= S_WR5;                // write hw2 @ base+2
            end
            S_WR5: begin
                we_n <= 1; st <= S_WR6;                // end hw2 write
            end
            S_WR6: begin
                hw <= 3; dq_out <= wd_lat[63:48];
                we_n <= 0; st <= S_WR7;                // write hw3 @ base+3
            end
            S_WR7: begin
                we_n <= 1; dq_oe <= 0; ce_n <= 1; st <= S_WRD;   // end hw3 write
            end
            S_WRD: begin busy <= 0; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
