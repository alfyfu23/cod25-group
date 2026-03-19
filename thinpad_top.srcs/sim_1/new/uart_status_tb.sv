`timescale 1ns / 1ps
`default_nettype none

module uart_status_tb;
  localparam CLK_FREQ = 50_000_000;
  localparam BAUD     = 115200;

  logic clk = 1'b0;
  logic rst = 1'b1;

  always #10 clk = ~clk;  // 50 MHz clock

  // Wishbone signals
  logic        wb_cyc;
  logic        wb_stb;
  logic        wb_we;
  logic [31:0] wb_adr;
  logic [31:0] wb_dat_m2s;
  logic [31:0] wb_dat_s2m;
  logic [3:0]  wb_sel;
  logic        wb_ack;

  logic txd;
  logic tx_ready;
  bit   ready_seen;

  uart_controller #(
      .ADDR_WIDTH(32),
      .DATA_WIDTH(32),
      .CLK_FREQ  (CLK_FREQ),
      .BAUD      (BAUD)
  ) dut (
      .clk_i    (clk),
      .rst_i    (rst),
      .wb_cyc_i (wb_cyc),
      .wb_stb_i (wb_stb),
      .wb_ack_o (wb_ack),
      .wb_adr_i (wb_adr),
      .wb_dat_i (wb_dat_m2s),
      .wb_dat_o (wb_dat_s2m),
      .wb_sel_i (wb_sel),
      .wb_we_i  (wb_we),
      .uart_txd_o(txd),
      .uart_rxd_i(1'b1),
      .tx_ready_o(tx_ready)
  );

  initial begin
    wb_cyc      = 0;
    wb_stb      = 0;
    wb_we       = 0;
    wb_adr      = 0;
    wb_dat_m2s  = 0;
    wb_sel      = 4'b0001;

    repeat (5) @(posedge clk);
    rst = 1'b0;

    // send character 'd'
    wb_write(32'h0000_0000, 32'h0000_0064, 4'b0001);
    $display("[%0t] Wrote 'd', waiting for TX ready", $time);

    // poll status register
  ready_seen = 1'b0;
  for (int i = 0; i < 20000; i++) begin
      logic [31:0] status;
      wb_read(32'h0000_0005, status, 4'b0010);
      if (status[5]) begin
        ready_seen = 1;
        $display("[%0t] Status ready = %0b after %0d polls", $time, status[5], i+1);
        break;
      end
    end

    if (!ready_seen) begin
      $display("[%0t] ERROR: TX ready never asserted", $time);
    end else begin
      $display("[%0t] PASS: TX ready observed", $time);
    end

    $finish;
  end

  task automatic wb_write(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] sel_mask);
    begin
      @(posedge clk);
      wb_adr     <= addr;
      wb_dat_m2s <= data;
      wb_sel     <= sel_mask;
      wb_we      <= 1'b1;
      wb_cyc     <= 1'b1;
      wb_stb     <= 1'b1;
      wait_ack();
      wb_cyc     <= 1'b0;
      wb_stb     <= 1'b0;
      wb_we      <= 1'b0;
    end
  endtask

  task automatic wb_read(input logic [31:0] addr, output logic [31:0] data, input logic [3:0] sel_mask);
    begin
      @(posedge clk);
      wb_adr     <= addr;
      wb_sel     <= sel_mask;
      wb_we      <= 1'b0;
      wb_cyc     <= 1'b1;
      wb_stb     <= 1'b1;
      wait_ack();
      data = wb_dat_s2m;
      wb_cyc     <= 1'b0;
      wb_stb     <= 1'b0;
    end
  endtask

  task automatic wait_ack();
    begin
      do begin
        @(posedge clk);
      end while (!wb_ack);
    end
  endtask

endmodule

`default_nettype wire
