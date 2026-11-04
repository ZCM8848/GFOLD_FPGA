`timescale 1ns/1ps
// ddr2_mem_arbiter -- three clients (image writer, boot reader, solver) share
// the single DDR2 sram64-style client port. All clients are in clk_sol.
module ddr2_mem_arbiter (
    input  wire        clk,
    input  wire        rst_n,
    // image writer
    input  wire        img_req, img_we,
    input  wire [17:0] img_addr,
    input  wire [63:0] img_wdata,
    output reg         img_busy,
    output reg  [63:0] img_rdata,
    // boot reader
    input  wire        boot_req,
    input  wire [17:0] boot_addr,
    output reg         boot_busy,
    output reg  [63:0] boot_rdata,
    // solver external memory
    input  wire        sol_req, sol_we,
    input  wire [17:0] sol_addr,
    input  wire [63:0] sol_wdata,
    output reg         sol_busy,
    output reg  [63:0] sol_rdata,
    // DDR2 client port
    output reg         mem_req, mem_we,
    output reg  [17:0] mem_addr,
    output reg  [63:0] mem_wdata,
    input  wire        mem_busy,
    input  wire [63:0] mem_rdata
);
    localparam A_IMG=0, A_BOOT=1, A_SOL=2;
    reg [1:0] active;
    reg [2:0] st;
    reg       pend_img, pend_boot, pend_sol;
    reg       busy_r;
    reg       we_r;
    reg [17:0] addr_r;
    reg [63:0] wdata_r;

    wire any_pend = pend_img | pend_boot | pend_sol;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= A_IMG; st <= 0;
            pend_img <= 0; pend_boot <= 0; pend_sol <= 0;
            img_busy <= 0; boot_busy <= 0; sol_busy <= 0;
            img_rdata <= 0; boot_rdata <= 0; sol_rdata <= 0;
            mem_req <= 0; mem_we <= 0; mem_addr <= 0; mem_wdata <= 0;
            busy_r <= 0; we_r <= 0; addr_r <= 0; wdata_r <= 0;
        end else begin
            // Latch one-cycle requests only while idle; clients hold their
            // request until busy rises, so this avoids a duplicate transaction.
            if (st == 3'd0) begin
                if (img_req)  pend_img <= 1'b1;
                if (boot_req) pend_boot <= 1'b1;
                if (sol_req)  pend_sol <= 1'b1;
            end

            case (st)
            3'd0: begin // idle / select
                mem_req <= 1'b0;
                if (any_pend) begin
                    if (pend_img) begin
                        active <= A_IMG; pend_img <= 0;
                        we_r <= img_we; addr_r <= img_addr; wdata_r <= img_wdata;
                        mem_we <= img_we; mem_addr <= img_addr; mem_wdata <= img_wdata;
                        img_busy <= 1'b1;
                    end else if (pend_boot) begin
                        active <= A_BOOT; pend_boot <= 0;
                        we_r <= 1'b0; addr_r <= boot_addr; wdata_r <= 64'd0;
                        mem_we <= 1'b0; mem_addr <= boot_addr; mem_wdata <= 64'd0;
                        boot_busy <= 1'b1;
                    end else begin
                        active <= A_SOL; pend_sol <= 0;
                        we_r <= sol_we; addr_r <= sol_addr; wdata_r <= sol_wdata;
                        mem_we <= sol_we; mem_addr <= sol_addr; mem_wdata <= sol_wdata;
                        sol_busy <= 1'b1;
                    end
                    mem_req   <= 1'b1;
                    st <= 3'd1;
                end
            end
            3'd1: begin // wait for DDR busy
                if (mem_busy) begin
                    mem_req <= 1'b0;
                    st <= 3'd2;
                end
            end
            3'd2: begin // wait completion
                if (!mem_busy) begin
                    case (active)
                    A_IMG:  begin img_busy <= 0;  img_rdata <= mem_rdata;  end
                    A_BOOT: begin boot_busy <= 0; boot_rdata <= mem_rdata; end
                    default:begin sol_busy <= 0;  sol_rdata <= mem_rdata;  end
                    endcase
                    st <= 3'd0;
                end
            end
            default: st <= 3'd0;
            endcase
        end
    end
endmodule
