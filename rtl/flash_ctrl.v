`timescale 1ns/1ps
// flash_ctrl: 8-bit async NOR CFI Flash READ controller for the DE2-115
// (8M x 8, FL_ADDR[22:0]). A 64-bit word = 8 consecutive bytes, read in
// ascending byte order (little-endian, matching sram64_ctrl's halfword order):
//   rdata = {DQ[7], DQ[6], ..., DQ[0]}
// Read-only (the solver input is burned into Flash; never written at runtime).
//
// Protocol (like sram64_ctrl): pulse req with waddr (64-bit word address,
// byte address = waddr<<3). busy goes high the cycle after req and falls when
// the access completes; rdata is valid when busy falls. ready rises after a
// ~1ms power-up delay (Flash t_vcs/t_vih) AND FL_RY deasserted (chip ready).
//
// Async read timing: address + CE#/OE# low, then t_acc ~90ns (~5 cyc @ 50MHz)
// before DQ is valid; we wait 6 cycles to be safe.
module flash_ctrl #(parameter AW = 20) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,
    input  wire [AW-1:0] waddr,
    output reg         busy,
    output reg         ready,
    output reg  [63:0] rdata,
    // ---- async Flash pins ----
    output wire [22:0] FL_ADDR,
    inout  wire [7:0]  FL_DQ,
    output wire        FL_CE_N,
    output wire        FL_OE_N,
    output wire        FL_WE_N,
    output wire        FL_RESET_N,
    output wire        FL_WP_N,
    input  wire        FL_RY
);
    // read-only: DQ always high-Z (Flash drives it); WE/RESET/WP all inactive-high
    assign FL_DQ      = 8'hz;
    assign FL_WE_N    = 1'b1;
    assign FL_RESET_N = 1'b1;
    assign FL_WP_N    = 1'b1;

    reg [22:0] addr;
    reg [2:0]  by;          // byte index 0..7
    reg [2:0]  wait_cnt;    // access-time wait (0..5)
    reg        ce_n, oe_n;
    reg [15:0] pwr_cnt;     // power-up delay counter (~1ms @ 50MHz = 50000)
    reg        req_p;       // req rising-edge detect
    reg [2:0]  st;

    assign FL_ADDR = addr;
    assign FL_CE_N = ce_n;
    assign FL_OE_N = oe_n;

    localparam S_PWRUP=0, S_READY=1, S_WAIT=2, S_SAMPLE=3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_PWRUP; busy <= 0; ready <= 0; rdata <= 0;
            addr <= 0; by <= 0; wait_cnt <= 0; ce_n <= 1; oe_n <= 1;
            pwr_cnt <= 0; req_p <= 0;
        end else begin
            req_p <= req;
            case (st)
            S_PWRUP: begin
                ce_n <= 1; oe_n <= 1;
                // ~1ms power-up (65535 cyc @ 50MHz = 1.3ms); require FL_RY too
                if (pwr_cnt == 16'hFFFF && FL_RY) begin ready <= 1; st <= S_READY; end
                else pwr_cnt <= pwr_cnt + 1;
            end
            S_READY: begin
                busy <= 0;
                if (req && !req_p) begin
                    addr <= {waddr, 3'b000}; by <= 0; wait_cnt <= 0;
                    ce_n <= 0; oe_n <= 0; busy <= 1;
                    st <= S_WAIT;
                end
            end
            // ---- wait t_acc (6 cyc), then sample DQ[by] ----
            S_WAIT: begin
                if (wait_cnt == 3'd5) begin
                    rdata[by*8 +: 8] <= FL_DQ;
                    st <= S_SAMPLE;
                end else wait_cnt <= wait_cnt + 1;
            end
            S_SAMPLE: begin
                if (by == 3'd7) begin
                    ce_n <= 1; oe_n <= 1; st <= S_READY;
                end else begin
                    by <= by + 1; addr <= addr + 1; wait_cnt <= 0; st <= S_WAIT;
                end
            end
            default: st <= S_READY;
            endcase
        end
    end
endmodule
