`default_nettype none

module thinpad_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

    // 时钟与复位（统一到 50MHz 系统时钟）
    // 说明：为保证 UART 波特率精确、各模块同域，使用板载 50MHz 作为系统主时钟
    // 若在硬件上仍需使用 PLL，可在后续版本恢复，但需同时更新 UART 的 CLK_FREQ
    logic sys_clk;
    logic sys_rst;
    assign sys_clk = clk_50M;
    // 使用按钮直接作为简单复位（低成本做法）：按下时为 1
    // 如果需要消抖/同步复位，可接入更严格的同步电路
    assign sys_rst = reset_btn;

  // 禁用 CPLD 串口
  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;

  /* =========== CPU Wishbone 主设备 =========== */
  logic [31:0] cpu_wb_adr;
  logic [31:0] cpu_wb_dat_m;
  logic [31:0] cpu_wb_dat_s;
  logic [3:0]  cpu_wb_sel;
  logic        cpu_wb_we;
  logic        cpu_wb_cyc;
  logic        cpu_wb_stb;
  logic        cpu_wb_ack;
  logic        cpu_wb_err;

  logic [31:0] cpu_dbg_pc;
  logic [31:0] cpu_dbg_instr;

  wire timer_irq;

  rv32i_core #(
      .RESET_VECTOR(32'h8000_0000)
  ) u_cpu (
      .clk       (sys_clk),
      .rst       (sys_rst),
      .wbm_adr_o (cpu_wb_adr),
      .wbm_dat_o (cpu_wb_dat_m),
      .wbm_dat_i (cpu_wb_dat_s),
      .wbm_we_o  (cpu_wb_we),
      .wbm_sel_o (cpu_wb_sel),
      .wbm_cyc_o (cpu_wb_cyc),
      .wbm_stb_o (cpu_wb_stb),
      .wbm_ack_i (cpu_wb_ack),
      .wbm_err_i (cpu_wb_err),
      .external_interrupt_i(uart_irq),
      .timer_interrupt_i(timer_irq),
      .dbg_pc    (cpu_dbg_pc),
      .dbg_instr (cpu_dbg_instr)
  );

  /* =========== Wishbone MUX =========== */
  logic wbs0_cyc_o, wbs0_stb_o, wbs0_ack_i, wbs0_we_o;
  logic [31:0] wbs0_adr_o, wbs0_dat_o, wbs0_dat_i;
  logic [3:0]  wbs0_sel_o;

  logic wbs1_cyc_o, wbs1_stb_o, wbs1_ack_i, wbs1_we_o;
  logic [31:0] wbs1_adr_o, wbs1_dat_o, wbs1_dat_i;
  logic [3:0]  wbs1_sel_o;

  logic wbs2_cyc_o, wbs2_stb_o, wbs2_ack_i, wbs2_we_o;
  logic [31:0] wbs2_adr_o, wbs2_dat_o, wbs2_dat_i;
  logic [3:0]  wbs2_sel_o;

  logic wbs3_cyc_o, wbs3_stb_o, wbs3_ack_i, wbs3_we_o;
  logic [31:0] wbs3_adr_o, wbs3_dat_o, wbs3_dat_i;
  logic [3:0]  wbs3_sel_o;

  wb_mux_4 wb_crossbar (
      .clk(sys_clk),
      .rst(sys_rst),

      .wbm_adr_i(cpu_wb_adr),
      .wbm_dat_i(cpu_wb_dat_m),
      .wbm_dat_o(cpu_wb_dat_s),
      .wbm_we_i (cpu_wb_we),
      .wbm_sel_i(cpu_wb_sel),
      .wbm_stb_i(cpu_wb_stb),
      .wbm_ack_o(cpu_wb_ack),
      .wbm_err_o(cpu_wb_err),
      .wbm_rty_o(),
      .wbm_cyc_i(cpu_wb_cyc),

      .wbs0_addr    (32'h8000_0000),
      .wbs0_addr_msk(32'hFFC0_0000),
      .wbs0_adr_o   (wbs0_adr_o),
      .wbs0_dat_i   (wbs0_dat_i),
      .wbs0_dat_o   (wbs0_dat_o),
      .wbs0_we_o    (wbs0_we_o),
      .wbs0_sel_o   (wbs0_sel_o),
      .wbs0_stb_o   (wbs0_stb_o),
      .wbs0_ack_i   (wbs0_ack_i),
      .wbs0_err_i   ('0),
      .wbs0_rty_i   ('0),
      .wbs0_cyc_o   (wbs0_cyc_o),

      .wbs1_addr    (32'h8040_0000),
      .wbs1_addr_msk(32'hFFC0_0000),
      .wbs1_adr_o   (wbs1_adr_o),
      .wbs1_dat_i   (wbs1_dat_i),
      .wbs1_dat_o   (wbs1_dat_o),
      .wbs1_we_o    (wbs1_we_o),
      .wbs1_sel_o   (wbs1_sel_o),
      .wbs1_stb_o   (wbs1_stb_o),
      .wbs1_ack_i   (wbs1_ack_i),
      .wbs1_err_i   ('0),
      .wbs1_rty_i   ('0),
      .wbs1_cyc_o   (wbs1_cyc_o),

      .wbs2_addr    (32'h1000_0000),
      .wbs2_addr_msk(32'hFFFF_0000),
      .wbs2_adr_o   (wbs2_adr_o),
      .wbs2_dat_i   (wbs2_dat_i),
      .wbs2_dat_o   (wbs2_dat_o),
      .wbs2_we_o    (wbs2_we_o),
      .wbs2_sel_o   (wbs2_sel_o),
      .wbs2_stb_o   (wbs2_stb_o),
      .wbs2_ack_i   (wbs2_ack_i),
      .wbs2_err_i   ('0),
      .wbs2_rty_i   ('0),
      .wbs2_cyc_o   (wbs2_cyc_o),

      .wbs3_addr    (32'h0200_0000),
      .wbs3_addr_msk(32'hFFFF_0000),
      .wbs3_adr_o   (wbs3_adr_o),
      .wbs3_dat_i   (wbs3_dat_i),
      .wbs3_dat_o   (wbs3_dat_o),
      .wbs3_we_o    (wbs3_we_o),
      .wbs3_sel_o   (wbs3_sel_o),
      .wbs3_stb_o   (wbs3_stb_o),
      .wbs3_ack_i   (wbs3_ack_i),
      .wbs3_err_i   ('0),
      .wbs3_rty_i   ('0),
      .wbs3_cyc_o   (wbs3_cyc_o)
  );

  /* =========== Wishbone 从设备 =========== */
  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) u_base_sram (
      .clk_i(sys_clk),
      .rst_i(sys_rst),
      .wb_cyc_i(wbs0_cyc_o),
      .wb_stb_i(wbs0_stb_o),
      .wb_ack_o(wbs0_ack_i),
      .wb_adr_i(wbs0_adr_o),
      .wb_dat_i(wbs0_dat_o),
      .wb_dat_o(wbs0_dat_i),
      .wb_sel_i(wbs0_sel_o),
      .wb_we_i (wbs0_we_o),
      .sram_addr(base_ram_addr),
      .sram_data(base_ram_data),
      .sram_ce_n(base_ram_ce_n),
      .sram_oe_n(base_ram_oe_n),
      .sram_we_n(base_ram_we_n),
      .sram_be_n(base_ram_be_n)
  );

  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) u_ext_sram (
      .clk_i(sys_clk),
      .rst_i(sys_rst),
      .wb_cyc_i(wbs1_cyc_o),
      .wb_stb_i(wbs1_stb_o),
      .wb_ack_o(wbs1_ack_i),
      .wb_adr_i(wbs1_adr_o),
      .wb_dat_i(wbs1_dat_o),
      .wb_dat_o(wbs1_dat_i),
      .wb_sel_i(wbs1_sel_o),
      .wb_we_i (wbs1_we_o),
      .sram_addr(ext_ram_addr),
      .sram_data(ext_ram_data),
      .sram_ce_n(ext_ram_ce_n),
      .sram_oe_n(ext_ram_oe_n),
      .sram_we_n(ext_ram_we_n),
      .sram_be_n(ext_ram_be_n)
  );

  wire uart_tx_ready;
  wire uart_irq;

  uart_controller #(
      .CLK_FREQ(50_000_000),
      .BAUD    (115200)
  ) u_uart (
      .clk_i(sys_clk),
      .rst_i(sys_rst),
      .wb_cyc_i(wbs2_cyc_o),
      .wb_stb_i(wbs2_stb_o),
      .wb_ack_o(wbs2_ack_i),
      .wb_adr_i(wbs2_adr_o),
      .wb_dat_i(wbs2_dat_o),
      .wb_dat_o(wbs2_dat_i),
      .wb_sel_i(wbs2_sel_o),
      .wb_we_i (wbs2_we_o),
      .uart_txd_o(txd),
      .uart_rxd_i(rxd),
      .tx_ready_o(uart_tx_ready),
      .uart_irq(uart_irq)
  );

  clint u_clint (
      .clk_i(sys_clk),
      .rst_i(sys_rst),
      .wb_cyc_i(wbs3_cyc_o),
      .wb_stb_i(wbs3_stb_o),
      .wb_ack_o(wbs3_ack_i),
      .wb_adr_i(wbs3_adr_o),
      .wb_dat_i(wbs3_dat_o),
      .wb_dat_o(wbs3_dat_i),
      .wb_sel_i(wbs3_sel_o),
      .wb_we_i (wbs3_we_o),
      .timer_irq_o(timer_irq)
  );

  /* =========== 调试输出 =========== */
    // 低位 LED0 显示 UART ready 状态，其余显示 PC 低 15 位，便于调试
    assign leds[0]    = uart_tx_ready;
    assign leds[15:1] = cpu_dbg_pc[15:1];
  assign dpy0 = 8'hFF;
  assign dpy1 = 8'hFF;

  /* =========== 未使用外设的安全默认值 =========== */
  assign flash_a      = 23'h0;
  assign flash_d      = 16'hzzzz;
  assign flash_rp_n   = 1'b1;
  assign flash_vpen   = 1'b1;
  assign flash_ce_n   = 1'b1;
  assign flash_oe_n   = 1'b1;
  assign flash_we_n   = 1'b1;
  assign flash_byte_n = 1'b1;

  assign sl811_a0    = 1'b0;
  assign sl811_wr_n  = 1'b1;
  assign sl811_rd_n  = 1'b1;
  assign sl811_cs_n  = 1'b1;
  assign sl811_rst_n = 1'b1;
  assign sl811_dack_n= 1'b1;

  assign dm9k_cmd    = 1'b0;
  assign dm9k_sd     = 16'hzzzz;
  assign dm9k_iow_n  = 1'b1;
  assign dm9k_ior_n  = 1'b1;
  assign dm9k_cs_n   = 1'b1;
  assign dm9k_pwrst_n= 1'b1;

  assign video_red   = 3'b000;
  assign video_green = 3'b000;
  assign video_blue  = 2'b00;
  assign video_hsync = 1'b0;
  assign video_vsync = 1'b0;
  assign video_clk   = clk_50M;
  assign video_de    = 1'b0;

endmodule
