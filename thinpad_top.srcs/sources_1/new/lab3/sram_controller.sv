// 这里用的是杜逸凡的实现


module sram_controller #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,

    parameter SRAM_ADDR_WIDTH = 20,
    parameter SRAM_DATA_WIDTH = 32,

    localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
    localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
) (
    // clk and reset
    input wire clk_i,
    input wire rst_i,

    // wishbone slave interface
    input wire wb_cyc_i,
    input wire wb_stb_i,
    output reg wb_ack_o,  // 高有效
    input wire [ADDR_WIDTH-1:0] wb_adr_i,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input wire wb_we_i,  // 读还是写

    // sram interface
    output reg [SRAM_ADDR_WIDTH-1:0] sram_addr,
    inout wire [SRAM_DATA_WIDTH-1:0] sram_data,		
    output reg sram_ce_n,		// 片选，低有效
    output reg sram_oe_n,		// 读使能，低有效
    output reg sram_we_n,		// 写使能，低有效
    output reg [SRAM_BYTES-1:0] sram_be_n  // 字节使能，低有效。如果不使用字节使能，请保持为0
);

  	// TODO: 实现 SRAM 控制器

	typedef enum logic [2:0] { 
		IDLE = 0,
		READ = 1,
		READ_2 = 2,
		WRITE = 3,
		WRITE_2 = 4,
		WRITE_3 = 5,
		DONE = 6
	} States;

	States state;


	reg [SRAM_DATA_WIDTH-1:0] sram_data_o;
	wire [SRAM_DATA_WIDTH-1:0] sram_data_i;
	logic sram_data_t;  // 在需要向sram写入的时候置0
	assign sram_data = (sram_data_t) ? {SRAM_DATA_WIDTH{1'bz}} : sram_data_o;
	assign sram_data_i = sram_data;

	always_ff @(posedge clk_i or posedge rst_i) begin
		if (rst_i) begin
			state <= IDLE;
			wb_ack_o <= 0;
			sram_data_t <= 1;
			sram_ce_n <= 1;
			sram_oe_n <= 1;
			sram_we_n <= 1;
			sram_be_n <= {SRAM_BYTES{1'b1}};
		end else begin
			case (state)
				IDLE: begin
					if (wb_stb_i && wb_cyc_i) begin
						sram_ce_n <= 0;
						// 设置地址
						sram_addr <= wb_adr_i[SRAM_ADDR_WIDTH + 1 : 2];
						// 字节使能
						sram_be_n <= ~wb_sel_i;
						if (wb_we_i) begin
							// 写
							sram_we_n <= 1;
							sram_data_o <= wb_dat_i;
							sram_data_t <= 0;
							state <= WRITE;
						end else begin
							// 读
							sram_data_t <= 1;
							sram_oe_n <= 0;
							state <= READ;
						end 
					end
				end 
				READ: begin
					state <= READ_2;
				end 
				READ_2: begin
					sram_ce_n <= 1;
					sram_oe_n <= 1;
					wb_dat_o <= sram_data_i;
					wb_ack_o <= 1;
					state <= DONE;
				end
				WRITE: begin
					sram_we_n <= 0;
					state <= WRITE_2;
				end
				WRITE_2: begin
					sram_we_n <= 1;
					state <= WRITE_3;
				end
				WRITE_3: begin
					sram_ce_n <= 1;
					sram_data_t <= 1;
					wb_ack_o <= 1;
					state <= DONE;
				end
				DONE: begin
					wb_ack_o <= 0;
					state <= IDLE;
				end 
			endcase
		end
	end


endmodule


// module sram_controller #(
//     parameter DATA_WIDTH = 32,
//     parameter ADDR_WIDTH = 32,

//     parameter SRAM_ADDR_WIDTH = 20,
//     parameter SRAM_DATA_WIDTH = 32,

//     localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
//     localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
// ) (
//     // clk and reset
//     input wire clk_i,
//     input wire rst_i,

//     // wishbone slave interface
//     input wire wb_cyc_i,
//     input wire wb_stb_i,
//     output reg wb_ack_o,
//     input wire [ADDR_WIDTH-1:0] wb_adr_i,
//     input wire [DATA_WIDTH-1:0] wb_dat_i,
//     output reg [DATA_WIDTH-1:0] wb_dat_o,
//     input wire [DATA_WIDTH/8-1:0] wb_sel_i,
//     input wire wb_we_i,

//     // sram interface
//     output reg [SRAM_ADDR_WIDTH-1:0] sram_addr,
//     inout wire [SRAM_DATA_WIDTH-1:0] sram_data,
//     output reg sram_ce_n,
//     output reg sram_oe_n,
//     output reg sram_we_n,
//     output reg [SRAM_BYTES-1:0] sram_be_n
// );

//   // TODO: 实现 SRAM 控制器

// endmodule