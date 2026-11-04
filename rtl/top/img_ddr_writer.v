`timescale 1ns/1ps
// img_ddr_writer -- drains one LOAD packet from async_fifo_256x64 into DDR2 at
// the packet's global 64-bit word address. Runs on clk_sol; the FIFO write side
// is the 125 MHz RX clock.
module img_ddr_writer (
    input  wire        clk,
    input  wire        rst_n,
    // packet descriptor (RX domain; stable around the toggle)
    input  wire        pkt_start_tog,
    input  wire [15:0] pkt_base,
    input  wire [15:0] pkt_nwords,
    // FIFO read side
    input  wire        fifo_empty,
    input  wire [63:0] fifo_dout,
    output reg         fifo_rd_en,
    // DDR2 client port
    output reg         req,
    output reg         we,
    output reg  [17:0] addr,
    output reg  [63:0] wdata,
    input  wire        busy,
    input  wire [63:0] rdata,
    // completion toggle (clk domain)
    output reg         done_tog
);
    reg [1:0] start_sync;
    reg       start_seen;
    reg [15:0] base_r, nwords_r, idx;
    reg [2:0]  st;
    reg        wait_busy;

    localparam S_IDLE=0, S_WAIT=1, S_READ=2, S_REQ=3, S_BSY=4;

    wire start_edge = start_sync[1] ^ start_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_sync <= 2'd0; start_seen <= 1'b0;
            base_r <= 0; nwords_r <= 0; idx <= 0; st <= S_IDLE;
            req <= 0; we <= 0; addr <= 0; wdata <= 0; fifo_rd_en <= 0;
            wait_busy <= 0; done_tog <= 0;
        end else begin
            start_sync <= {start_sync[0], pkt_start_tog};
            fifo_rd_en <= 1'b0;
            case (st)
            S_IDLE: begin
                req <= 0;
                if (start_edge) begin
                    start_seen <= start_sync[1];
                    base_r <= pkt_base;
                    nwords_r <= pkt_nwords;
                    idx <= 0;
                    st <= (pkt_nwords == 0) ? S_IDLE : S_WAIT;
                    if (pkt_nwords == 0) done_tog <= ~done_tog;
                end
            end
            S_WAIT: begin
                req <= 0;
                if (!fifo_empty) begin
                    fifo_rd_en <= 1'b1;
                    st <= S_READ;
                end
            end
            S_READ: begin
                // fifo_dout is valid this cycle (registered read)
                addr <= {2'b0, base_r + idx};
                wdata <= fifo_dout;
                we <= 1'b1;
                req <= 1'b1;
                st <= S_REQ;
            end
            S_REQ: begin
                if (busy) begin
                    req <= 1'b0;
                    wait_busy <= 1'b1;
                    st <= S_BSY;
                end
            end
            S_BSY: begin
                if (!wait_busy && busy) wait_busy <= 1'b1;
                else if (wait_busy && !busy) begin
                    if (idx + 1 >= nwords_r) begin
                        idx <= 0;
                        done_tog <= ~done_tog;
                        st <= S_IDLE;
                    end else begin
                        idx <= idx + 1;
                        st <= S_WAIT;
                    end
                end
            end
            endcase
        end
    end
endmodule
