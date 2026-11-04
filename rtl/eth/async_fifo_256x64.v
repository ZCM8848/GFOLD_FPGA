`timescale 1ns/1ps
// async_fifo_256x64 -- packet buffer between the 125 MHz Ethernet RX clock and
// the 25 MHz solver/DDR2-writer clock. One LOAD packet is at most 180 words.
module async_fifo_256x64 (
    input  wire        wclk,
    input  wire        wrst_n,
    input  wire        wr_en,
    input  wire [63:0] din,
    output wire        full,
    input  wire        rclk,
    input  wire        rrst_n,
    input  wire        rd_en,
    output reg  [63:0] dout,
    output wire        empty
);
    (* ram_style = "bram" *) reg [63:0] mem [0:255];
    reg [8:0] wbin, wgray, rbin, rgray;
    reg [8:0] wgray_s1, wgray_s2, rgray_s1, rgray_s2;

    wire [8:0] wbin_next = wbin + {8'd0, (wr_en && !full)};
    wire [8:0] rbin_next = rbin + {8'd0, (rd_en && !empty)};
    wire [8:0] wgray_next = wbin_next ^ (wbin_next >> 1);
    wire [8:0] rgray_next = rbin_next ^ (rbin_next >> 1);

    assign full  = (wgray_next == {~rgray_s2[8:7], rgray_s2[6:0]});
    assign empty = (rgray == wgray_s2);

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin <= 9'd0; wgray <= 9'd0;
            wgray_s1 <= 9'd0; wgray_s2 <= 9'd0;
        end else begin
            wgray_s1 <= rgray;
            wgray_s2 <= wgray_s1;
            if (wr_en && !full) begin
                mem[wbin[7:0]] <= din;
                wbin  <= wbin_next;
                wgray <= wgray_next;
            end
        end
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin <= 9'd0; rgray <= 9'd0;
            rgray_s1 <= 9'd0; rgray_s2 <= 9'd0;
            dout <= 64'd0;
        end else begin
            rgray_s1 <= wgray;
            rgray_s2 <= rgray_s1;
            if (rd_en && !empty) begin
                dout <= mem[rbin[7:0]];
                rbin  <= rbin_next;
                rgray <= rgray_next;
            end
        end
    end
endmodule
