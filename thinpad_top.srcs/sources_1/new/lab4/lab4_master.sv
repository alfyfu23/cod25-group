// 杜逸凡实现

module lab4_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire clk_i,
    input  wire rst_i,

    // Control signals
    input  wire push_btn_i,
    input  wire reset_btn_i,
    input  wire [ADDR_WIDTH-1:0] dip_sw_i,

    // Wishbone master interface
    output reg  wb_cyc_o,
    output reg  wb_stb_o,
    input  wire wb_ack_i,
    output reg  [ADDR_WIDTH-1:0] wb_adr_o,
    output reg  [DATA_WIDTH-1:0] wb_dat_o,
    input  wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg  [DATA_WIDTH/8-1:0] wb_sel_o,
    output reg  wb_we_o
);

  // UART 寄存器地址
  localparam UART_DATA_ADDR   = 32'h1000_0000;
  localparam UART_STATUS_ADDR = 32'h1000_0005;

  // 状态定义
  typedef enum logic [3:0] {
    IDLE,

    READ_WAIT_ACTION,
    READ_WAIT_CHECK,
    READ_DATA_ACTION,
    READ_DATA_DONE,

    WRITE_SRAM_ACTION,
    WRITE_SRAM_DONE,

    WRITE_WAIT_ACTION,
    WRITE_WAIT_CHECK,
    WRITE_DATA_ACTION,
    WRITE_DATA_DONE,

    DEPRECATED
  } state_t;

  state_t state;

  // 内部寄存器
  reg [3:0] byte_count;
  reg [7:0] data_buffer [0:9];
  reg [ADDR_WIDTH-1:0] base_addr;

  // 主状态机
  always_ff @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
      state       <= READ_WAIT_ACTION;
      wb_cyc_o    <= 1'b0;
      wb_stb_o    <= 1'b0;
      wb_adr_o    <= '0;
      wb_dat_o    <= '0;
      wb_sel_o    <= 4'h0;
      wb_we_o     <= 1'b0;
      byte_count  <= 4'h0;
      base_addr   <= dip_sw_i;
    end else begin
      // 默认信号
      wb_cyc_o <= 1'b0;
      wb_stb_o <= 1'b0;
      wb_we_o  <= 1'b0;

      case (state)

        // ------------------------------
        // IDLE
        // ------------------------------
        IDLE: begin
          if (reset_btn_i) begin
            byte_count <= 0;
            base_addr  <= dip_sw_i;
            state      <= READ_WAIT_ACTION;
          end
        end

        // ------------------------------
        // 读取 UART 等待区
        // ------------------------------
        READ_WAIT_ACTION: begin
          wb_adr_o <= UART_STATUS_ADDR;
          wb_sel_o <= 4'b0010;
          wb_we_o  <= 1'b0;
          wb_cyc_o <= 1'b1;
          wb_stb_o <= 1'b1;
          state    <= READ_WAIT_CHECK;
        end

        READ_WAIT_CHECK: begin
          if (wb_ack_i) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
            if (wb_dat_i[8])   // 有数据可读
              state <= READ_DATA_ACTION;
            else
              state <= READ_WAIT_ACTION;  // 继续轮询
          end
        end

        // ------------------------------
        // 读取 UART 数据
        // ------------------------------
        READ_DATA_ACTION: begin
          wb_adr_o <= UART_DATA_ADDR;
          wb_sel_o <= 4'b0001;
          wb_we_o  <= 1'b0;
          wb_cyc_o <= 1'b1;
          wb_stb_o <= 1'b1;
          state    <= READ_DATA_DONE;
        end

        READ_DATA_DONE: begin
          if (wb_ack_i) begin
            data_buffer[byte_count] <= wb_dat_i[7:0];
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
            state <= WRITE_SRAM_ACTION;
          end
        end

        // ------------------------------
        // 写入 SRAM
        // ------------------------------
        WRITE_SRAM_ACTION: begin
          wb_adr_o <= base_addr + {28'h0, byte_count, 2'b00};
          wb_dat_o <= {24'h0, data_buffer[byte_count]};
          wb_sel_o <= 4'b0001;
          wb_we_o  <= 1'b1;
          wb_cyc_o <= 1'b1;
          wb_stb_o <= 1'b1;
          state    <= WRITE_SRAM_DONE;
        end

        WRITE_SRAM_DONE: begin
          if (wb_ack_i) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
            wb_we_o  <= 1'b0;
            state <= WRITE_WAIT_ACTION;
          end
        end

        // ------------------------------
        // 等待 UART 可写
        // ------------------------------
        WRITE_WAIT_ACTION: begin
          wb_adr_o <= UART_STATUS_ADDR;
          wb_sel_o <= 4'b0010;
          wb_we_o  <= 1'b0;
          wb_cyc_o <= 1'b1;
          wb_stb_o <= 1'b1;
          state    <= WRITE_WAIT_CHECK;
        end

        WRITE_WAIT_CHECK: begin
          if (wb_ack_i) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
            if (wb_dat_i[13])   // 可发送
              state <= WRITE_DATA_ACTION;
            else
              state <= WRITE_WAIT_ACTION;  // 继续轮询
          end
        end

        // ------------------------------
        // 写 UART 数据
        // ------------------------------
        WRITE_DATA_ACTION: begin
          wb_adr_o <= UART_DATA_ADDR;
          wb_dat_o <= {24'h0, data_buffer[byte_count]};
          wb_sel_o <= 4'b0001;
          wb_we_o  <= 1'b1;
          wb_cyc_o <= 1'b1;
          wb_stb_o <= 1'b1;
          state    <= WRITE_DATA_DONE;
        end

        WRITE_DATA_DONE: begin
          if (wb_ack_i) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
            wb_we_o  <= 1'b0;
            if (byte_count < 9) begin
              byte_count <= byte_count + 1;
              state <= READ_WAIT_ACTION;
            end else begin
              state <= IDLE;
            end
          end
        end

        default: state <= IDLE;

      endcase
    end
  end

endmodule



// module lab4_master #(
//     parameter ADDR_WIDTH = 32,
//     parameter DATA_WIDTH = 32
// ) (
//     input wire clk_i,
//     input wire rst_i,

//     // TODO: 添加需要的控制信号，例如按键开关？

//     // wishbone master
//     output reg wb_cyc_o,
//     output reg wb_stb_o,
//     input wire wb_ack_i,
//     output reg [ADDR_WIDTH-1:0] wb_adr_o,
//     output reg [DATA_WIDTH-1:0] wb_dat_o,
//     input wire [DATA_WIDTH-1:0] wb_dat_i,
//     output reg [DATA_WIDTH/8-1:0] wb_sel_o,
//     output reg wb_we_o
// );

//   // TODO: 实现实验 5 的内存+串口 Master

// endmodule
