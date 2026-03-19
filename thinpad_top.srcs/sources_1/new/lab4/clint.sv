`timescale 1ns/1ps

module clint #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire clk_i,
    input wire rst_i,

    // Wishbone slave interface
    input wire wb_cyc_i,
    input wire wb_stb_i,
    output reg wb_ack_o,
    input wire [ADDR_WIDTH-1:0] wb_adr_i,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input wire wb_we_i,

    output reg timer_irq_o
);

    logic [63:0] mtime;
    logic [63:0] mtimecmp;
    logic [31:0] prescaler;

    // Timer increment
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            mtime <= 64'd0;
            prescaler <= 32'd0;
        end else begin
            if (prescaler >= 32'd49) begin
                mtime <= mtime + 64'd1;
                prescaler <= 32'd0;
            end else begin
                prescaler <= prescaler + 32'd1;
            end
            
            // Allow writing to mtime (optional, for completeness)
            if (wb_stb_i && wb_cyc_i && wb_we_i && wb_ack_o) begin
                 case (wb_adr_i[15:0])
                    16'hBFF8: begin // mtime low
                        if (wb_sel_i[0]) mtime[7:0]   <= wb_dat_i[7:0];
                        if (wb_sel_i[1]) mtime[15:8]  <= wb_dat_i[15:8];
                        if (wb_sel_i[2]) mtime[23:16] <= wb_dat_i[23:16];
                        if (wb_sel_i[3]) mtime[31:24] <= wb_dat_i[31:24];
                    end
                    16'hBFFC: begin // mtime high
                        if (wb_sel_i[0]) mtime[39:32] <= wb_dat_i[7:0];
                        if (wb_sel_i[1]) mtime[47:40] <= wb_dat_i[15:8];
                        if (wb_sel_i[2]) mtime[55:48] <= wb_dat_i[23:16];
                        if (wb_sel_i[3]) mtime[63:56] <= wb_dat_i[31:24];
                    end
                endcase
            end
        end
    end

    // Interrupt generation
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            timer_irq_o <= 1'b0;
        end else begin
            if (mtime >= mtimecmp) begin
                timer_irq_o <= 1'b1;
            end else begin
                timer_irq_o <= 1'b0;
            end
        end
    end

    // Wishbone interface
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_ack_o <= 1'b0;
            mtimecmp <= 64'hFFFF_FFFF_FFFF_FFFF; // Default to max
        end else begin
            // Handshake
            if (wb_stb_i && wb_cyc_i && !wb_ack_o) begin
                wb_ack_o <= 1'b1;
                
                // Write logic for mtimecmp
                if (wb_we_i) begin
                    case (wb_adr_i[15:0])
                        16'h4000: begin // mtimecmp low
                            if (wb_sel_i[0]) mtimecmp[7:0]   <= wb_dat_i[7:0];
                            if (wb_sel_i[1]) mtimecmp[15:8]  <= wb_dat_i[15:8];
                            if (wb_sel_i[2]) mtimecmp[23:16] <= wb_dat_i[23:16];
                            if (wb_sel_i[3]) mtimecmp[31:24] <= wb_dat_i[31:24];
                        end
                        16'h4004: begin // mtimecmp high
                            if (wb_sel_i[0]) mtimecmp[39:32] <= wb_dat_i[7:0];
                            if (wb_sel_i[1]) mtimecmp[47:40] <= wb_dat_i[15:8];
                            if (wb_sel_i[2]) mtimecmp[55:48] <= wb_dat_i[23:16];
                            if (wb_sel_i[3]) mtimecmp[63:56] <= wb_dat_i[31:24];
                        end
                    endcase
                end
            end else begin
                wb_ack_o <= 1'b0;
            end
        end
    end

    // Read logic
    always_comb begin
        wb_dat_o = 32'h0;
        case (wb_adr_i[15:0])
            16'h4000: wb_dat_o = mtimecmp[31:0];
            16'h4004: wb_dat_o = mtimecmp[63:32];
            16'hBFF8: wb_dat_o = mtime[31:0];
            16'hBFFC: wb_dat_o = mtime[63:32];
            default:  wb_dat_o = 32'h0;
        endcase
    end

endmodule
