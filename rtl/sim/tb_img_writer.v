`timescale 1ns/1ps
module tb_img_writer;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        pkt_start_tog = 0;
    reg [15:0] pkt_base = 16'd100;
    reg [15:0] pkt_nwords = 16'd4;

    reg  [63:0] fifo_mem [0:7];
    integer     rp = 0;
    wire [63:0] fifo_dout = fifo_mem[rp];
    wire        fifo_empty = (rp >= 4);
    wire        fifo_rd_en;

    wire        img_req, img_we, img_busy;
    wire [17:0] img_addr;
    wire [63:0] img_wdata, img_rdata;
    wire        done_tog;

    wire        mem_req, mem_we;
    wire [17:0] mem_addr;
    wire [63:0] mem_wdata;
    reg         mem_busy;
    reg  [63:0] mem_rdata;
    reg  [63:0] mem [0:255];

    wire boot_busy, sol_busy;
    wire [63:0] boot_rdata, sol_rdata;
    wire [17:0] boot_addr = 0, sol_addr = 0;
    wire [63:0] sol_wdata = 0;
    wire sol_req = 0, sol_we = 0;

    always @(posedge clk) if (fifo_rd_en) rp <= rp + 1;

    ddr2_mem_arbiter u_arb (
        .clk(clk), .rst_n(rst_n),
        .img_req(img_req), .img_we(img_we), .img_addr(img_addr), .img_wdata(img_wdata),
        .img_busy(img_busy), .img_rdata(img_rdata),
        .boot_req(1'b0), .boot_addr(boot_addr), .boot_busy(boot_busy), .boot_rdata(boot_rdata),
        .sol_req(sol_req), .sol_we(sol_we), .sol_addr(sol_addr), .sol_wdata(sol_wdata),
        .sol_busy(sol_busy), .sol_rdata(sol_rdata),
        .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr), .mem_wdata(mem_wdata),
        .mem_busy(mem_busy), .mem_rdata(mem_rdata)
    );

    img_ddr_writer u_wr (
        .clk(clk), .rst_n(rst_n),
        .pkt_start_tog(pkt_start_tog), .pkt_base(pkt_base), .pkt_nwords(pkt_nwords),
        .fifo_empty(fifo_empty), .fifo_dout(fifo_dout), .fifo_rd_en(fifo_rd_en),
        .req(img_req), .we(img_we), .addr(img_addr), .wdata(img_wdata),
        .busy(img_busy), .rdata(img_rdata), .done_tog(done_tog)
    );

    // Simple DDR2 client model: 3-cycle busy, write stores data.
    reg [2:0] mst;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mst <= 0; mem_busy <= 0; mem_rdata <= 0;
        end else begin
            case (mst)
            0: if (mem_req) begin mem_busy <= 1; mst <= 1; end
            1: mst <= 2;
            2: mst <= 3;
            3: begin
                if (mem_we) begin
                    mem[mem_addr[7:0]] <= mem_wdata;
                    $display("WR addr=%0d data=%h", mem_addr, mem_wdata);
                end else mem_rdata <= mem[mem_addr[7:0]];
                mem_busy <= 0; mst <= 0;
            end
            endcase
        end
    end

    integer i;
    initial begin
        fifo_mem[0] = 64'hA0;
        fifo_mem[1] = 64'hA1;
        fifo_mem[2] = 64'hA2;
        fifo_mem[3] = 64'hA3;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (4) @(negedge clk);
        @(negedge clk); pkt_start_tog = 1;
        @(negedge clk); pkt_start_tog = 0;
        repeat (200) @(posedge clk);
        for (i = 0; i < 4; i = i + 1)
            $display("mem[%0d] = %h", 100+i, mem[100+i]);
        if (mem[100] == 64'hA0 && mem[101] == 64'hA1 &&
            mem[102] == 64'hA2 && mem[103] == 64'hA3)
            $display("IMG WRITER PASS");
        else
            $display("IMG WRITER FAIL");
        $finish;
    end
endmodule
