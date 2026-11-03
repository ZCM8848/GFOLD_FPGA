`timescale 1ns/1ps
// boot_rom_ctrl — presents the CFI-flash controller handshake that drs_iter's
// boot FSM expects, but reads 64-bit words from the ETH-loaded image RAM.
// The image must be stable before the solver reset is released.
module boot_rom_ctrl #(parameter AW = 20) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,
    input  wire [AW-1:0] waddr,
    output reg         busy,
    output reg         ready,
    output reg  [63:0] rdata,
    // image RAM read port (synchronous, 1-cycle)
    output reg  [9:0]  img_addr,
    input  wire [63:0] img_rdata
);
    reg        req_p;
    reg [2:0]  cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; ready <= 0; rdata <= 0; img_addr <= 0; req_p <= 0; cnt <= 0;
        end else begin
            req_p <= req;
            ready <= 1'b1;               // image loaded before reset release
            if (req && !req_p) begin
                img_addr <= waddr[9:0];
                busy <= 1; cnt <= 0;
            end else if (busy) begin
                cnt <= cnt + 1'b1;
                if (cnt == 3'd1) begin
                    rdata <= img_rdata;    // image RAM read latency = 1
                    busy  <= 0;
                end
            end
        end
    end
endmodule
