module AddressDecoder_Req_BRIDGE (
	data_req_i,
	destination_i,
	data_gnt_o,
	data_gnt_i,
	data_req_o,
	data_ID_o
);
	reg _sv2v_0;
	parameter ID_WIDTH = 17;
	parameter ID = 1;
	parameter N_SLAVE = 16;
	parameter ADDR_WIDTH = 32;
	input wire data_req_i;
	input wire [N_SLAVE - 1:0] destination_i;
	output reg data_gnt_o;
	input wire [N_SLAVE - 1:0] data_gnt_i;
	output reg [N_SLAVE - 1:0] data_req_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	assign data_ID_o = ID;
	always @(*) begin : Combinational_ADDR_DEC_REQ
		if (_sv2v_0)
			;
		data_req_o = {N_SLAVE {data_req_i}} & destination_i;
		data_gnt_o = |(data_gnt_i & destination_i) & data_req_i;
	end
	initial _sv2v_0 = 0;
endmodule
module AddressDecoder_Resp_BRIDGE (
	data_r_valid_i,
	data_ID_i,
	data_r_valid_o
);
	parameter ID_WIDTH = 20;
	parameter N_MASTER = 20;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_ID_i;
	output wire [N_MASTER - 1:0] data_r_valid_o;
	assign data_r_valid_o = {ID_WIDTH {data_r_valid_i}} & data_ID_i;
endmodule
module ArbitrationTree_BRIDGE (
	clk,
	rst_n,
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_ID_i,
	data_aux_i,
	data_gnt_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_aux_o,
	data_gnt_i
);
	parameter ADDR_WIDTH = 32;
	parameter ID_WIDTH = 20;
	parameter N_MASTER = 16;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter AUX_WIDTH = 6;
	parameter MAX_COUNT = N_MASTER;
	input wire clk;
	input wire rst_n;
	input wire [N_MASTER - 1:0] data_req_i;
	input wire [(N_MASTER * ADDR_WIDTH) - 1:0] data_add_i;
	input wire [N_MASTER - 1:0] data_wen_i;
	input wire [(N_MASTER * DATA_WIDTH) - 1:0] data_wdata_i;
	input wire [(N_MASTER * BE_WIDTH) - 1:0] data_be_i;
	input wire [(N_MASTER * ID_WIDTH) - 1:0] data_ID_i;
	input wire [(N_MASTER * AUX_WIDTH) - 1:0] data_aux_i;
	output wire [N_MASTER - 1:0] data_gnt_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	output wire [AUX_WIDTH - 1:0] data_aux_o;
	input wire data_gnt_i;
	localparam LOG_MASTER = $clog2(N_MASTER);
	localparam N_WIRE = N_MASTER - 2;
	wire [LOG_MASTER - 1:0] RR_FLAG;
	genvar _gv_j_1;
	genvar _gv_k_1;
	generate
		case (N_MASTER)
			1: ;
			2: begin : DUAL_MASTER
				FanInPrimitive_Req_BRIDGE #(
					.ADDR_WIDTH(ADDR_WIDTH),
					.ID_WIDTH(ID_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_FanInPrimitive_Req_BRIDGE(
					.RR_FLAG(RR_FLAG),
					.data_wdata0_i(data_wdata_i[0+:DATA_WIDTH]),
					.data_wdata1_i(data_wdata_i[DATA_WIDTH+:DATA_WIDTH]),
					.data_add0_i(data_add_i[0+:ADDR_WIDTH]),
					.data_add1_i(data_add_i[ADDR_WIDTH+:ADDR_WIDTH]),
					.data_req0_i(data_req_i[0]),
					.data_req1_i(data_req_i[1]),
					.data_wen0_i(data_wen_i[0]),
					.data_wen1_i(data_wen_i[1]),
					.data_ID0_i(data_ID_i[0+:ID_WIDTH]),
					.data_ID1_i(data_ID_i[ID_WIDTH+:ID_WIDTH]),
					.data_be0_i(data_be_i[0+:BE_WIDTH]),
					.data_be1_i(data_be_i[BE_WIDTH+:BE_WIDTH]),
					.data_aux0_i(data_aux_i[0+:AUX_WIDTH]),
					.data_aux1_i(data_aux_i[AUX_WIDTH+:AUX_WIDTH]),
					.data_gnt0_o(data_gnt_o[0]),
					.data_gnt1_o(data_gnt_o[1]),
					.data_wdata_o(data_wdata_o),
					.data_add_o(data_add_o),
					.data_req_o(data_req_o),
					.data_wen_o(data_wen_o),
					.data_ID_o(data_ID_o),
					.data_be_o(data_be_o),
					.data_aux_o(data_aux_o),
					.data_gnt_i(data_gnt_i)
				);
			end
			default: begin : BINARY_TREE
				wire [DATA_WIDTH - 1:0] data_wdata_LEVEL [N_WIRE - 1:0];
				wire [ADDR_WIDTH - 1:0] data_add_LEVEL [N_WIRE - 1:0];
				wire data_req_LEVEL [N_WIRE - 1:0];
				wire data_wen_LEVEL [N_WIRE - 1:0];
				wire [ID_WIDTH - 1:0] data_ID_LEVEL [N_WIRE - 1:0];
				wire [BE_WIDTH - 1:0] data_be_LEVEL [N_WIRE - 1:0];
				wire [AUX_WIDTH - 1:0] data_aux_LEVEL [N_WIRE - 1:0];
				wire data_gnt_LEVEL [N_WIRE - 1:0];
				for (_gv_j_1 = 0; _gv_j_1 < LOG_MASTER; _gv_j_1 = _gv_j_1 + 1) begin : STAGE
					localparam j = _gv_j_1;
					for (_gv_k_1 = 0; _gv_k_1 < (2 ** j); _gv_k_1 = _gv_k_1 + 1) begin : INCR_VERT
						localparam k = _gv_k_1;
						if (j == 0) begin : LAST_NODE
							FanInPrimitive_Req_BRIDGE #(
								.ADDR_WIDTH(ADDR_WIDTH),
								.ID_WIDTH(ID_WIDTH),
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH),
								.BE_WIDTH(BE_WIDTH)
							) i_FanInPrimitive_Req_BRIDGE(
								.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
								.data_wdata0_i(data_wdata_LEVEL[2 * k]),
								.data_wdata1_i(data_wdata_LEVEL[(2 * k) + 1]),
								.data_add0_i(data_add_LEVEL[2 * k]),
								.data_add1_i(data_add_LEVEL[(2 * k) + 1]),
								.data_req0_i(data_req_LEVEL[2 * k]),
								.data_req1_i(data_req_LEVEL[(2 * k) + 1]),
								.data_wen0_i(data_wen_LEVEL[2 * k]),
								.data_wen1_i(data_wen_LEVEL[(2 * k) + 1]),
								.data_ID0_i(data_ID_LEVEL[2 * k]),
								.data_ID1_i(data_ID_LEVEL[(2 * k) + 1]),
								.data_be0_i(data_be_LEVEL[2 * k]),
								.data_be1_i(data_be_LEVEL[(2 * k) + 1]),
								.data_aux0_i(data_aux_LEVEL[2 * k]),
								.data_aux1_i(data_aux_LEVEL[(2 * k) + 1]),
								.data_gnt0_o(data_gnt_LEVEL[2 * k]),
								.data_gnt1_o(data_gnt_LEVEL[(2 * k) + 1]),
								.data_wdata_o(data_wdata_o),
								.data_add_o(data_add_o),
								.data_req_o(data_req_o),
								.data_wen_o(data_wen_o),
								.data_ID_o(data_ID_o),
								.data_be_o(data_be_o),
								.data_aux_o(data_aux_o),
								.data_gnt_i(data_gnt_i)
							);
						end
						else if (j < (LOG_MASTER - 1)) begin : MIDDLE_NODES
							FanInPrimitive_Req_BRIDGE #(
								.ADDR_WIDTH(ADDR_WIDTH),
								.ID_WIDTH(ID_WIDTH),
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH),
								.BE_WIDTH(BE_WIDTH)
							) i_FanInPrimitive_Req_BRIDGE(
								.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
								.data_wdata0_i(data_wdata_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_wdata1_i(data_wdata_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_add0_i(data_add_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_add1_i(data_add_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_req0_i(data_req_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_req1_i(data_req_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_wen0_i(data_wen_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_wen1_i(data_wen_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_ID0_i(data_ID_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_ID1_i(data_ID_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_be0_i(data_be_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_be1_i(data_be_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_aux0_i(data_aux_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_aux1_i(data_aux_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_gnt0_o(data_gnt_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_gnt1_o(data_gnt_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_wdata_o(data_wdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_add_o(data_add_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_req_o(data_req_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_wen_o(data_wen_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_ID_o(data_ID_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_be_o(data_be_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_aux_o(data_aux_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_gnt_i(data_gnt_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
						else begin : LEAF_NODES
							FanInPrimitive_Req_BRIDGE #(
								.ADDR_WIDTH(ADDR_WIDTH),
								.ID_WIDTH(ID_WIDTH),
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH),
								.BE_WIDTH(BE_WIDTH)
							) i_FanInPrimitive_Req_BRIDGE(
								.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
								.data_wdata0_i(data_wdata_i[(2 * k) * DATA_WIDTH+:DATA_WIDTH]),
								.data_wdata1_i(data_wdata_i[((2 * k) + 1) * DATA_WIDTH+:DATA_WIDTH]),
								.data_add0_i(data_add_i[(2 * k) * ADDR_WIDTH+:ADDR_WIDTH]),
								.data_add1_i(data_add_i[((2 * k) + 1) * ADDR_WIDTH+:ADDR_WIDTH]),
								.data_req0_i(data_req_i[2 * k]),
								.data_req1_i(data_req_i[(2 * k) + 1]),
								.data_wen0_i(data_wen_i[2 * k]),
								.data_wen1_i(data_wen_i[(2 * k) + 1]),
								.data_ID0_i(data_ID_i[(2 * k) * ID_WIDTH+:ID_WIDTH]),
								.data_ID1_i(data_ID_i[((2 * k) + 1) * ID_WIDTH+:ID_WIDTH]),
								.data_be0_i(data_be_i[(2 * k) * BE_WIDTH+:BE_WIDTH]),
								.data_be1_i(data_be_i[((2 * k) + 1) * BE_WIDTH+:BE_WIDTH]),
								.data_aux0_i(data_aux_i[(2 * k) * AUX_WIDTH+:AUX_WIDTH]),
								.data_aux1_i(data_aux_i[((2 * k) + 1) * AUX_WIDTH+:AUX_WIDTH]),
								.data_gnt0_o(data_gnt_o[2 * k]),
								.data_gnt1_o(data_gnt_o[(2 * k) + 1]),
								.data_wdata_o(data_wdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_add_o(data_add_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_req_o(data_req_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_wen_o(data_wen_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_ID_o(data_ID_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_be_o(data_be_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_aux_o(data_aux_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_gnt_i(data_gnt_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
					end
				end
			end
		endcase
	endgenerate
	RR_Flag_Req_BRIDGE #(
		.WIDTH(LOG_MASTER),
		.MAX_COUNT(MAX_COUNT)
	) RR_REQ(
		.clk(clk),
		.rst_n(rst_n),
		.RR_FLAG_o(RR_FLAG),
		.data_req_i(data_req_o),
		.data_gnt_i(data_gnt_i)
	);
endmodule
module FanInPrimitive_Req_BRIDGE (
	RR_FLAG,
	data_wdata0_i,
	data_wdata1_i,
	data_add0_i,
	data_add1_i,
	data_req0_i,
	data_req1_i,
	data_wen0_i,
	data_wen1_i,
	data_be0_i,
	data_be1_i,
	data_ID0_i,
	data_ID1_i,
	data_aux0_i,
	data_aux1_i,
	data_gnt0_o,
	data_gnt1_o,
	data_wdata_o,
	data_add_o,
	data_req_o,
	data_ID_o,
	data_wen_o,
	data_be_o,
	data_aux_o,
	data_gnt_i
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter ID_WIDTH = 16;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire RR_FLAG;
	input wire [DATA_WIDTH - 1:0] data_wdata0_i;
	input wire [DATA_WIDTH - 1:0] data_wdata1_i;
	input wire [ADDR_WIDTH - 1:0] data_add0_i;
	input wire [ADDR_WIDTH - 1:0] data_add1_i;
	input wire data_req0_i;
	input wire data_req1_i;
	input wire data_wen0_i;
	input wire data_wen1_i;
	input wire [BE_WIDTH - 1:0] data_be0_i;
	input wire [BE_WIDTH - 1:0] data_be1_i;
	input wire [ID_WIDTH - 1:0] data_ID0_i;
	input wire [ID_WIDTH - 1:0] data_ID1_i;
	input wire [AUX_WIDTH - 1:0] data_aux0_i;
	input wire [AUX_WIDTH - 1:0] data_aux1_i;
	output wire data_gnt0_o;
	output wire data_gnt1_o;
	output reg [DATA_WIDTH - 1:0] data_wdata_o;
	output reg [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_req_o;
	output reg [ID_WIDTH - 1:0] data_ID_o;
	output reg data_wen_o;
	output reg [BE_WIDTH - 1:0] data_be_o;
	output reg [AUX_WIDTH - 1:0] data_aux_o;
	input wire data_gnt_i;
	wire SEL;
	assign data_req_o = data_req0_i | data_req1_i;
	assign SEL = ~data_req0_i | (RR_FLAG & data_req1_i);
	assign data_gnt0_o = ((data_req0_i & ~data_req1_i) | (data_req0_i & ~RR_FLAG)) & data_gnt_i;
	assign data_gnt1_o = ((~data_req0_i & data_req1_i) | (data_req1_i & RR_FLAG)) & data_gnt_i;
	always @(*) begin : FanIn_MUX2
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: begin
				data_wdata_o = data_wdata0_i;
				data_add_o = data_add0_i;
				data_wen_o = data_wen0_i;
				data_ID_o = data_ID0_i;
				data_be_o = data_be0_i;
				data_aux_o = data_aux0_i;
			end
			1'b1: begin
				data_wdata_o = data_wdata1_i;
				data_add_o = data_add1_i;
				data_wen_o = data_wen1_i;
				data_ID_o = data_ID1_i;
				data_be_o = data_be1_i;
				data_aux_o = data_aux1_i;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module FanInPrimitive_Resp_BRIDGE (
	data_r_rdata0_i,
	data_r_rdata1_i,
	data_r_valid0_i,
	data_r_valid1_i,
	data_r_opc0_i,
	data_r_opc1_i,
	data_r_aux0_i,
	data_r_aux1_i,
	data_r_rdata_o,
	data_r_valid_o,
	data_r_opc_o,
	data_r_aux_o
);
	reg _sv2v_0;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 6;
	input wire [DATA_WIDTH - 1:0] data_r_rdata0_i;
	input wire [DATA_WIDTH - 1:0] data_r_rdata1_i;
	input wire data_r_valid0_i;
	input wire data_r_valid1_i;
	input wire data_r_opc0_i;
	input wire data_r_opc1_i;
	input wire [AUX_WIDTH - 1:0] data_r_aux0_i;
	input wire [AUX_WIDTH - 1:0] data_r_aux1_i;
	output reg [DATA_WIDTH - 1:0] data_r_rdata_o;
	output wire data_r_valid_o;
	output reg data_r_opc_o;
	output reg [AUX_WIDTH - 1:0] data_r_aux_o;
	wire SEL;
	assign data_r_valid_o = data_r_valid1_i | data_r_valid0_i;
	assign SEL = data_r_valid1_i;
	always @(*) begin : FanOut_MUX2
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: begin
				data_r_rdata_o = data_r_rdata0_i;
				data_r_opc_o = data_r_opc0_i;
				data_r_aux_o = data_r_aux0_i;
			end
			1'b1: begin
				data_r_rdata_o = data_r_rdata1_i;
				data_r_opc_o = data_r_opc1_i;
				data_r_aux_o = data_r_aux1_i;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module MUX2_REQ_BRIDGE (
	data_req_CH0_i,
	data_add_CH0_i,
	data_wen_CH0_i,
	data_wdata_CH0_i,
	data_be_CH0_i,
	data_ID_CH0_i,
	data_aux_CH0_i,
	data_gnt_CH0_o,
	data_req_CH1_i,
	data_add_CH1_i,
	data_wen_CH1_i,
	data_wdata_CH1_i,
	data_be_CH1_i,
	data_ID_CH1_i,
	data_aux_CH1_i,
	data_gnt_CH1_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_aux_o,
	data_gnt_i,
	clk,
	rst_n
);
	reg _sv2v_0;
	parameter ID_WIDTH = 20;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 6;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire data_req_CH0_i;
	input wire [ADDR_WIDTH - 1:0] data_add_CH0_i;
	input wire data_wen_CH0_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_CH0_i;
	input wire [BE_WIDTH - 1:0] data_be_CH0_i;
	input wire [ID_WIDTH - 1:0] data_ID_CH0_i;
	input wire [AUX_WIDTH - 1:0] data_aux_CH0_i;
	output wire data_gnt_CH0_o;
	input wire data_req_CH1_i;
	input wire [ADDR_WIDTH - 1:0] data_add_CH1_i;
	input wire data_wen_CH1_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_CH1_i;
	input wire [BE_WIDTH - 1:0] data_be_CH1_i;
	input wire [ID_WIDTH - 1:0] data_ID_CH1_i;
	input wire [AUX_WIDTH - 1:0] data_aux_CH1_i;
	output wire data_gnt_CH1_o;
	output wire data_req_o;
	output reg [ADDR_WIDTH - 1:0] data_add_o;
	output reg data_wen_o;
	output reg [DATA_WIDTH - 1:0] data_wdata_o;
	output reg [BE_WIDTH - 1:0] data_be_o;
	output reg [ID_WIDTH - 1:0] data_ID_o;
	output reg [AUX_WIDTH - 1:0] data_aux_o;
	input wire data_gnt_i;
	input wire clk;
	input wire rst_n;
	wire SEL;
	reg RR_FLAG;
	assign data_req_o = data_req_CH0_i | data_req_CH1_i;
	assign SEL = ~data_req_CH0_i | (RR_FLAG & data_req_CH1_i);
	assign data_gnt_CH0_o = ((data_req_CH0_i & ~data_req_CH1_i) | (data_req_CH0_i & ~RR_FLAG)) & data_gnt_i;
	assign data_gnt_CH1_o = ((~data_req_CH0_i & data_req_CH1_i) | (data_req_CH1_i & RR_FLAG)) & data_gnt_i;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			RR_FLAG <= 1'b0;
		else if ((data_req_o == 1'b1) && (data_gnt_i == 1'b1))
			RR_FLAG <= ~RR_FLAG;
	always @(*) begin : MUX2_REQ_COMB_L2
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: begin
				data_add_o = data_add_CH0_i;
				data_wen_o = data_wen_CH0_i;
				data_wdata_o = data_wdata_CH0_i;
				data_be_o = data_be_CH0_i;
				data_ID_o = data_ID_CH0_i;
				data_aux_o = data_aux_CH0_i;
			end
			1'b1: begin
				data_add_o = data_add_CH1_i;
				data_wen_o = data_wen_CH1_i;
				data_wdata_o = data_wdata_CH1_i;
				data_be_o = data_be_CH1_i;
				data_ID_o = data_ID_CH1_i;
				data_aux_o = data_aux_CH1_i;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module RR_Flag_Req_BRIDGE (
	clk,
	rst_n,
	RR_FLAG_o,
	data_req_i,
	data_gnt_i
);
	parameter WIDTH = 3;
	parameter MAX_COUNT = (2 ** WIDTH) - 1;
	input wire clk;
	input wire rst_n;
	output reg [WIDTH - 1:0] RR_FLAG_o;
	input wire data_req_i;
	input wire data_gnt_i;
	always @(posedge clk or negedge rst_n) begin : RR_Flag_Req_SEQ
		if (rst_n == 1'b0)
			RR_FLAG_o <= 1'sb0;
		else if (data_req_i & data_gnt_i) begin
			if (RR_FLAG_o < MAX_COUNT)
				RR_FLAG_o <= RR_FLAG_o + 1'b1;
			else
				RR_FLAG_o <= 1'sb0;
		end
	end
endmodule
module RequestBlock1CH_BRIDGE (
	data_req_CH0_i,
	data_add_CH0_i,
	data_wen_CH0_i,
	data_wdata_CH0_i,
	data_be_CH0_i,
	data_ID_CH0_i,
	data_aux_CH0_i,
	data_gnt_CH0_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_aux_o,
	data_gnt_i,
	data_r_valid_i,
	data_r_ID_i,
	data_r_valid_CH0_o,
	clk,
	rst_n
);
	parameter ADDR_WIDTH = 32;
	parameter N_CH0 = 16;
	parameter ID_WIDTH = N_CH0;
	parameter N_SLAVE = 16;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire [N_CH0 - 1:0] data_req_CH0_i;
	input wire [(N_CH0 * ADDR_WIDTH) - 1:0] data_add_CH0_i;
	input wire [N_CH0 - 1:0] data_wen_CH0_i;
	input wire [(N_CH0 * DATA_WIDTH) - 1:0] data_wdata_CH0_i;
	input wire [(N_CH0 * BE_WIDTH) - 1:0] data_be_CH0_i;
	input wire [(N_CH0 * ID_WIDTH) - 1:0] data_ID_CH0_i;
	input wire [(N_CH0 * AUX_WIDTH) - 1:0] data_aux_CH0_i;
	output wire [N_CH0 - 1:0] data_gnt_CH0_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	output wire [AUX_WIDTH - 1:0] data_aux_o;
	input wire data_gnt_i;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_r_ID_i;
	output wire [N_CH0 - 1:0] data_r_valid_CH0_o;
	input wire clk;
	input wire rst_n;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_req_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ADDR_WIDTH) - 1:0] data_add_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_wen_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * DATA_WIDTH) - 1:0] data_wdata_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * BE_WIDTH) - 1:0] data_be_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ID_WIDTH) - 1:0] data_ID_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * AUX_WIDTH) - 1:0] data_aux_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_gnt_CH0_int;
	generate
		if ((2 ** $clog2(N_CH0)) != N_CH0) begin : _DUMMY_CH0_PORTS_
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_req_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ADDR_WIDTH) - 1:0] data_add_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_wen_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * DATA_WIDTH) - 1:0] data_wdata_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * BE_WIDTH) - 1:0] data_be_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ID_WIDTH) - 1:0] data_ID_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * AUX_WIDTH) - 1:0] data_aux_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_gnt_CH0_dummy;
			assign data_req_CH0_dummy = 1'sb0;
			assign data_add_CH0_dummy = 1'sb0;
			assign data_wen_CH0_dummy = 1'sb0;
			assign data_wdata_CH0_dummy = 1'sb0;
			assign data_be_CH0_dummy = 1'sb0;
			assign data_ID_CH0_dummy = 1'sb0;
			assign data_aux_CH0_dummy = 1'sb0;
			assign data_req_CH0_int = {data_req_CH0_dummy, data_req_CH0_i};
			assign data_add_CH0_int = {data_add_CH0_dummy, data_add_CH0_i};
			assign data_wen_CH0_int = {data_wen_CH0_dummy, data_wen_CH0_i};
			assign data_wdata_CH0_int = {data_wdata_CH0_dummy, data_wdata_CH0_i};
			assign data_be_CH0_int = {data_be_CH0_dummy, data_be_CH0_i};
			assign data_ID_CH0_int = {data_ID_CH0_dummy, data_ID_CH0_i};
			assign data_aux_CH0_int = {data_aux_CH0_dummy, data_aux_CH0_i};
			genvar _gv_j_2;
			for (_gv_j_2 = 0; _gv_j_2 < N_CH0; _gv_j_2 = _gv_j_2 + 1) begin : _MERGING_CH0_DUMMY_PORTS_OUT_
				localparam j = _gv_j_2;
				assign data_gnt_CH0_o[j] = data_gnt_CH0_int[j];
			end
		end
		else begin : genblk1
			assign data_req_CH0_int = data_req_CH0_i;
			assign data_add_CH0_int = data_add_CH0_i;
			assign data_wen_CH0_int = data_wen_CH0_i;
			assign data_wdata_CH0_int = data_wdata_CH0_i;
			assign data_be_CH0_int = data_be_CH0_i;
			assign data_ID_CH0_int = data_ID_CH0_i;
			assign data_aux_CH0_int = data_aux_CH0_i;
			assign data_gnt_CH0_o = data_gnt_CH0_int;
		end
		if (N_CH0 > 1) begin : POLY_CH0
			ArbitrationTree_BRIDGE #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(N_CH0),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.AUX_WIDTH(AUX_WIDTH),
				.MAX_COUNT(N_CH0 - 1)
			) i_ArbitrationTree_BRIDGE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH0_int),
				.data_add_i(data_add_CH0_int),
				.data_wen_i(data_wen_CH0_int),
				.data_wdata_i(data_wdata_CH0_int),
				.data_be_i(data_be_CH0_int),
				.data_ID_i(data_ID_CH0_int),
				.data_aux_i(data_aux_CH0_int),
				.data_gnt_o(data_gnt_CH0_int),
				.data_req_o(data_req_o),
				.data_add_o(data_add_o),
				.data_wen_o(data_wen_o),
				.data_wdata_o(data_wdata_o),
				.data_be_o(data_be_o),
				.data_ID_o(data_ID_o),
				.data_aux_o(data_aux_o),
				.data_gnt_i(data_gnt_i)
			);
		end
		else begin : MONO_CH0
			assign data_req_o = data_req_CH0_int;
			assign data_add_o = data_add_CH0_int;
			assign data_wen_o = data_wen_CH0_int;
			assign data_wdata_o = data_wdata_CH0_int;
			assign data_be_o = data_be_CH0_int;
			assign data_ID_o = data_ID_CH0_int;
			assign data_aux_o = data_aux_CH0_int;
			assign data_gnt_CH0_int = data_gnt_i;
		end
	endgenerate
	AddressDecoder_Resp_BRIDGE #(
		.ID_WIDTH(ID_WIDTH),
		.N_MASTER(N_CH0)
	) i_AddressDecoder_Resp_PE(
		.data_r_valid_i(data_r_valid_i),
		.data_ID_i(data_r_ID_i),
		.data_r_valid_o(data_r_valid_CH0_o)
	);
endmodule
module RequestBlock2CH_BRIDGE (
	data_req_CH0_i,
	data_add_CH0_i,
	data_wen_CH0_i,
	data_wdata_CH0_i,
	data_be_CH0_i,
	data_ID_CH0_i,
	data_aux_CH0_i,
	data_gnt_CH0_o,
	data_req_CH1_i,
	data_add_CH1_i,
	data_wen_CH1_i,
	data_wdata_CH1_i,
	data_be_CH1_i,
	data_ID_CH1_i,
	data_aux_CH1_i,
	data_gnt_CH1_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_aux_o,
	data_gnt_i,
	data_r_valid_i,
	data_r_ID_i,
	data_r_valid_CH0_o,
	data_r_valid_CH1_o,
	clk,
	rst_n
);
	parameter ADDR_WIDTH = 32;
	parameter N_CH0 = 16;
	parameter N_CH1 = 1;
	parameter ID_WIDTH = N_CH0 + N_CH1;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 6;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire [N_CH0 - 1:0] data_req_CH0_i;
	input wire [(N_CH0 * ADDR_WIDTH) - 1:0] data_add_CH0_i;
	input wire [N_CH0 - 1:0] data_wen_CH0_i;
	input wire [(N_CH0 * DATA_WIDTH) - 1:0] data_wdata_CH0_i;
	input wire [(N_CH0 * BE_WIDTH) - 1:0] data_be_CH0_i;
	input wire [(N_CH0 * ID_WIDTH) - 1:0] data_ID_CH0_i;
	input wire [(N_CH0 * AUX_WIDTH) - 1:0] data_aux_CH0_i;
	output wire [N_CH0 - 1:0] data_gnt_CH0_o;
	input wire [N_CH1 - 1:0] data_req_CH1_i;
	input wire [(N_CH1 * ADDR_WIDTH) - 1:0] data_add_CH1_i;
	input wire [N_CH1 - 1:0] data_wen_CH1_i;
	input wire [(N_CH1 * DATA_WIDTH) - 1:0] data_wdata_CH1_i;
	input wire [(N_CH1 * BE_WIDTH) - 1:0] data_be_CH1_i;
	input wire [(N_CH1 * ID_WIDTH) - 1:0] data_ID_CH1_i;
	input wire [(N_CH1 * AUX_WIDTH) - 1:0] data_aux_CH1_i;
	output wire [N_CH1 - 1:0] data_gnt_CH1_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	output wire [AUX_WIDTH - 1:0] data_aux_o;
	input wire data_gnt_i;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_r_ID_i;
	output wire [N_CH0 - 1:0] data_r_valid_CH0_o;
	output wire [N_CH1 - 1:0] data_r_valid_CH1_o;
	input wire clk;
	input wire rst_n;
	wire data_req_CH0;
	wire [ADDR_WIDTH - 1:0] data_add_CH0;
	wire data_wen_CH0;
	wire [DATA_WIDTH - 1:0] data_wdata_CH0;
	wire [BE_WIDTH - 1:0] data_be_CH0;
	wire [ID_WIDTH - 1:0] data_ID_CH0;
	wire [AUX_WIDTH - 1:0] data_aux_CH0;
	wire data_gnt_CH0;
	wire data_req_CH1;
	wire [ADDR_WIDTH - 1:0] data_add_CH1;
	wire data_wen_CH1;
	wire [DATA_WIDTH - 1:0] data_wdata_CH1;
	wire [BE_WIDTH - 1:0] data_be_CH1;
	wire [ID_WIDTH - 1:0] data_ID_CH1;
	wire [AUX_WIDTH - 1:0] data_aux_CH1;
	wire data_gnt_CH1;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_req_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ADDR_WIDTH) - 1:0] data_add_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_wen_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * DATA_WIDTH) - 1:0] data_wdata_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * BE_WIDTH) - 1:0] data_be_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ID_WIDTH) - 1:0] data_ID_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * AUX_WIDTH) - 1:0] data_aux_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_gnt_CH0_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_req_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * ADDR_WIDTH) - 1:0] data_add_CH1_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_wen_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * DATA_WIDTH) - 1:0] data_wdata_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * BE_WIDTH) - 1:0] data_be_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * ID_WIDTH) - 1:0] data_ID_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * AUX_WIDTH) - 1:0] data_aux_CH1_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_gnt_CH1_int;
	generate
		if ((2 ** $clog2(N_CH0)) != N_CH0) begin : _DUMMY_CH0_PORTS_
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_req_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ADDR_WIDTH) - 1:0] data_add_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_wen_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * DATA_WIDTH) - 1:0] data_wdata_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * BE_WIDTH) - 1:0] data_be_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ID_WIDTH) - 1:0] data_ID_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * AUX_WIDTH) - 1:0] data_aux_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_gnt_CH0_dummy;
			assign data_req_CH0_dummy = 1'sb0;
			assign data_add_CH0_dummy = 1'sb0;
			assign data_wen_CH0_dummy = 1'sb0;
			assign data_wdata_CH0_dummy = 1'sb0;
			assign data_be_CH0_dummy = 1'sb0;
			assign data_ID_CH0_dummy = 1'sb0;
			assign data_aux_CH0_dummy = 1'sb0;
			assign data_req_CH0_int = {data_req_CH0_dummy, data_req_CH0_i};
			assign data_add_CH0_int = {data_add_CH0_dummy, data_add_CH0_i};
			assign data_wen_CH0_int = {data_wen_CH0_dummy, data_wen_CH0_i};
			assign data_wdata_CH0_int = {data_wdata_CH0_dummy, data_wdata_CH0_i};
			assign data_be_CH0_int = {data_be_CH0_dummy, data_be_CH0_i};
			assign data_ID_CH0_int = {data_ID_CH0_dummy, data_ID_CH0_i};
			assign data_aux_CH0_int = {data_aux_CH0_dummy, data_aux_CH0_i};
			genvar _gv_j_3;
			for (_gv_j_3 = 0; _gv_j_3 < N_CH0; _gv_j_3 = _gv_j_3 + 1) begin : _MERGING_CH0_DUMMY_PORTS_OUT_
				localparam j = _gv_j_3;
				assign data_gnt_CH0_o[j] = data_gnt_CH0_int[j];
			end
		end
		else begin : genblk1
			assign data_req_CH0_int = data_req_CH0_i;
			assign data_add_CH0_int = data_add_CH0_i;
			assign data_wen_CH0_int = data_wen_CH0_i;
			assign data_wdata_CH0_int = data_wdata_CH0_i;
			assign data_be_CH0_int = data_be_CH0_i;
			assign data_ID_CH0_int = data_ID_CH0_i;
			assign data_aux_CH0_int = data_aux_CH0_i;
			assign data_gnt_CH0_o = data_gnt_CH0_int;
		end
		if ((2 ** $clog2(N_CH1)) != N_CH1) begin : _DUMMY_CH1_PORTS_
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_req_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * ADDR_WIDTH) - 1:0] data_add_CH1_dummy;
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_wen_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * DATA_WIDTH) - 1:0] data_wdata_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * BE_WIDTH) - 1:0] data_be_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * ID_WIDTH) - 1:0] data_ID_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * AUX_WIDTH) - 1:0] data_aux_CH1_dummy;
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_gnt_CH1_dummy;
			assign data_req_CH1_dummy = 1'sb0;
			assign data_add_CH1_dummy = 1'sb0;
			assign data_wen_CH1_dummy = 1'sb0;
			assign data_wdata_CH1_dummy = 1'sb0;
			assign data_be_CH1_dummy = 1'sb0;
			assign data_ID_CH1_dummy = 1'sb0;
			assign data_aux_CH1_dummy = 1'sb0;
			assign data_req_CH1_int = {data_req_CH1_dummy, data_req_CH1_i};
			assign data_add_CH1_int = {data_add_CH1_dummy, data_add_CH1_i};
			assign data_wen_CH1_int = {data_wen_CH1_dummy, data_wen_CH1_i};
			assign data_wdata_CH1_int = {data_wdata_CH1_dummy, data_wdata_CH1_i};
			assign data_be_CH1_int = {data_be_CH1_dummy, data_be_CH1_i};
			assign data_ID_CH1_int = {data_ID_CH1_dummy, data_ID_CH1_i};
			assign data_aux_CH1_int = {data_aux_CH1_dummy, data_aux_CH1_i};
			genvar _gv_j_4;
			for (_gv_j_4 = 0; _gv_j_4 < N_CH1; _gv_j_4 = _gv_j_4 + 1) begin : _MERGING_CH1_DUMMY_PORTS_OUT_
				localparam j = _gv_j_4;
				assign data_gnt_CH1_o[j] = data_gnt_CH1_int[j];
			end
		end
		else begin : genblk2
			assign data_req_CH1_int = data_req_CH1_i;
			assign data_add_CH1_int = data_add_CH1_i;
			assign data_wen_CH1_int = data_wen_CH1_i;
			assign data_wdata_CH1_int = data_wdata_CH1_i;
			assign data_be_CH1_int = data_be_CH1_i;
			assign data_ID_CH1_int = data_ID_CH1_i;
			assign data_aux_CH1_int = data_aux_CH1_i;
			assign data_gnt_CH1_o = data_gnt_CH1_int;
		end
		if (N_CH0 > 1) begin : CH0_ARB_TREE
			ArbitrationTree_BRIDGE #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(2 ** $clog2(N_CH0)),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.AUX_WIDTH(AUX_WIDTH),
				.MAX_COUNT(N_CH0 - 1)
			) i_ArbitrationTree_BRIDGE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH0_int),
				.data_add_i(data_add_CH0_int),
				.data_wen_i(data_wen_CH0_int),
				.data_wdata_i(data_wdata_CH0_int),
				.data_be_i(data_be_CH0_int),
				.data_ID_i(data_ID_CH0_int),
				.data_aux_i(data_aux_CH0_int),
				.data_gnt_o(data_gnt_CH0_int),
				.data_req_o(data_req_CH0),
				.data_add_o(data_add_CH0),
				.data_wen_o(data_wen_CH0),
				.data_wdata_o(data_wdata_CH0),
				.data_be_o(data_be_CH0),
				.data_ID_o(data_ID_CH0),
				.data_aux_o(data_aux_CH0),
				.data_gnt_i(data_gnt_CH0)
			);
		end
		if (N_CH1 > 1) begin : CH1_ARB_TREE
			ArbitrationTree_BRIDGE #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(2 ** $clog2(N_CH1)),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.AUX_WIDTH(AUX_WIDTH),
				.MAX_COUNT(N_CH1 - 1)
			) i_ArbitrationTree_BRIDGE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH1_int),
				.data_add_i(data_add_CH1_int),
				.data_wen_i(data_wen_CH1_int),
				.data_wdata_i(data_wdata_CH1_int),
				.data_be_i(data_be_CH1_int),
				.data_ID_i(data_ID_CH1_int),
				.data_aux_i(data_aux_CH1_int),
				.data_gnt_o(data_gnt_CH1_int),
				.data_req_o(data_req_CH1),
				.data_add_o(data_add_CH1),
				.data_wen_o(data_wen_CH1),
				.data_wdata_o(data_wdata_CH1),
				.data_be_o(data_be_CH1),
				.data_ID_o(data_ID_CH1),
				.data_aux_o(data_aux_CH1),
				.data_gnt_i(data_gnt_CH1)
			);
		end
		if (N_CH1 == 1) begin : MONO_CH1
			if (N_CH0 == 1) begin : MONO_CH0
				MUX2_REQ_BRIDGE #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_MUX2_REQ_BRIDGE(
					.data_req_CH0_i(data_req_CH0_int),
					.data_add_CH0_i(data_add_CH0_int),
					.data_wen_CH0_i(data_wen_CH0_int),
					.data_wdata_CH0_i(data_wdata_CH0_int),
					.data_be_CH0_i(data_be_CH0_int),
					.data_ID_CH0_i(data_ID_CH0_int),
					.data_aux_CH0_i(data_aux_CH0_int),
					.data_gnt_CH0_o(data_gnt_CH0_int),
					.data_req_CH1_i(data_req_CH1_int),
					.data_add_CH1_i(data_add_CH1_int),
					.data_wen_CH1_i(data_wen_CH1_int),
					.data_wdata_CH1_i(data_wdata_CH1_int),
					.data_be_CH1_i(data_be_CH1_int),
					.data_ID_CH1_i(data_ID_CH1_int),
					.data_aux_CH1_i(data_aux_CH1_int),
					.data_gnt_CH1_o(data_gnt_CH1_int),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_aux_o(data_aux_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : POLY_CH0
				MUX2_REQ_BRIDGE #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_MUX2_REQ_BRIDGE(
					.data_req_CH0_i(data_req_CH0),
					.data_add_CH0_i(data_add_CH0),
					.data_wen_CH0_i(data_wen_CH0),
					.data_wdata_CH0_i(data_wdata_CH0),
					.data_be_CH0_i(data_be_CH0),
					.data_ID_CH0_i(data_ID_CH0),
					.data_aux_CH0_i(data_aux_CH0),
					.data_gnt_CH0_o(data_gnt_CH0),
					.data_req_CH1_i(data_req_CH1_int),
					.data_add_CH1_i(data_add_CH1_int),
					.data_wen_CH1_i(data_wen_CH1_int),
					.data_wdata_CH1_i(data_wdata_CH1_int),
					.data_be_CH1_i(data_be_CH1_int),
					.data_ID_CH1_i(data_ID_CH1_int),
					.data_aux_CH1_i(data_aux_CH1_int),
					.data_gnt_CH1_o(data_gnt_CH1_int),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_aux_o(data_aux_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
		else begin : POLY_CH1
			if (N_CH0 == 1) begin : MONO_CH0
				MUX2_REQ_BRIDGE #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_MUX2_REQ_BRIDGE(
					.data_req_CH0_i(data_req_CH0_int),
					.data_add_CH0_i(data_add_CH0_int),
					.data_wen_CH0_i(data_wen_CH0_int),
					.data_wdata_CH0_i(data_wdata_CH0_int),
					.data_be_CH0_i(data_be_CH0_int),
					.data_ID_CH0_i(data_ID_CH0_int),
					.data_aux_CH0_i(data_aux_CH0_int),
					.data_gnt_CH0_o(data_gnt_CH0_int),
					.data_req_CH1_i(data_req_CH1),
					.data_add_CH1_i(data_add_CH1),
					.data_wen_CH1_i(data_wen_CH1),
					.data_wdata_CH1_i(data_wdata_CH1),
					.data_be_CH1_i(data_be_CH1),
					.data_ID_CH1_i(data_ID_CH1),
					.data_aux_CH1_i(data_aux_CH1),
					.data_gnt_CH1_o(data_gnt_CH1),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_aux_o(data_aux_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : POLY_CH0
				MUX2_REQ_BRIDGE #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_MUX2_REQ_BRIDGE(
					.data_req_CH0_i(data_req_CH0),
					.data_add_CH0_i(data_add_CH0),
					.data_wen_CH0_i(data_wen_CH0),
					.data_wdata_CH0_i(data_wdata_CH0),
					.data_be_CH0_i(data_be_CH0),
					.data_ID_CH0_i(data_ID_CH0),
					.data_aux_CH0_i(data_aux_CH0),
					.data_gnt_CH0_o(data_gnt_CH0),
					.data_req_CH1_i(data_req_CH1),
					.data_add_CH1_i(data_add_CH1),
					.data_wen_CH1_i(data_wen_CH1),
					.data_wdata_CH1_i(data_wdata_CH1),
					.data_be_CH1_i(data_be_CH1),
					.data_ID_CH1_i(data_ID_CH1),
					.data_aux_CH1_i(data_aux_CH1),
					.data_gnt_CH1_o(data_gnt_CH1),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_aux_o(data_aux_o),
					.data_ID_o(data_ID_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
	endgenerate
	AddressDecoder_Resp_BRIDGE #(
		.ID_WIDTH(ID_WIDTH),
		.N_MASTER(N_CH0 + N_CH1)
	) i_AddressDecoder_Resp_BRIDGE(
		.data_r_valid_i(data_r_valid_i),
		.data_ID_i(data_r_ID_i),
		.data_r_valid_o({data_r_valid_CH1_o, data_r_valid_CH0_o})
	);
endmodule
module ResponseBlock_BRIDGE (
	data_r_valid_i,
	data_r_rdata_i,
	data_r_opc_i,
	data_r_aux_i,
	data_r_valid_o,
	data_r_rdata_o,
	data_r_opc_o,
	data_r_aux_o,
	data_req_i,
	destination_i,
	data_gnt_o,
	data_req_o,
	data_gnt_i,
	data_ID_o
);
	parameter ID = 1;
	parameter ID_WIDTH = 17;
	parameter N_SLAVE = 16;
	parameter AUX_WIDTH = 8;
	parameter DATA_WIDTH = 32;
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	input wire [N_SLAVE - 1:0] data_r_opc_i;
	input wire [(N_SLAVE * AUX_WIDTH) - 1:0] data_r_aux_i;
	output wire data_r_valid_o;
	output wire [DATA_WIDTH - 1:0] data_r_rdata_o;
	output wire data_r_opc_o;
	output wire [AUX_WIDTH - 1:0] data_r_aux_o;
	input wire data_req_i;
	input wire [N_SLAVE - 1:0] destination_i;
	output wire data_gnt_o;
	output wire [N_SLAVE - 1:0] data_req_o;
	input wire [N_SLAVE - 1:0] data_gnt_i;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	wire [(2 ** $clog2(N_SLAVE)) - 1:0] data_r_valid_int;
	wire [((2 ** $clog2(N_SLAVE)) * DATA_WIDTH) - 1:0] data_r_rdata_int;
	wire [(2 ** $clog2(N_SLAVE)) - 1:0] data_r_opc_int;
	wire [((2 ** $clog2(N_SLAVE)) * AUX_WIDTH) - 1:0] data_r_aux_int;
	generate
		if ((2 ** $clog2(N_SLAVE)) != N_SLAVE) begin : _DUMMY_SLAVE_PORTS_
			wire [((2 ** $clog2(N_SLAVE)) - N_SLAVE) - 1:0] data_r_valid_dummy;
			wire [(((2 ** $clog2(N_SLAVE)) - N_SLAVE) * DATA_WIDTH) - 1:0] data_r_rdata_dummy;
			wire [((2 ** $clog2(N_SLAVE)) - N_SLAVE) - 1:0] data_r_opc_dummy;
			wire [(((2 ** $clog2(N_SLAVE)) - N_SLAVE) * AUX_WIDTH) - 1:0] data_r_aux_dummy;
			assign data_r_valid_dummy = 1'sb0;
			assign data_r_rdata_dummy = 1'sb0;
			assign data_r_opc_dummy = 1'sb0;
			assign data_r_aux_dummy = 1'sb0;
			assign data_r_valid_int = {data_r_valid_dummy, data_r_valid_i};
			assign data_r_rdata_int = {data_r_rdata_dummy, data_r_rdata_i};
			assign data_r_opc_int = {data_r_opc_dummy, data_r_opc_i};
			assign data_r_aux_int = {data_r_aux_dummy, data_r_aux_i};
		end
		else begin : genblk1
			assign data_r_valid_int = data_r_valid_i;
			assign data_r_rdata_int = data_r_rdata_i;
			assign data_r_opc_int = data_r_opc_i;
			assign data_r_aux_int = data_r_aux_i;
		end
	endgenerate
	ResponseTree_BRIDGE #(
		.N_SLAVE(2 ** $clog2(N_SLAVE)),
		.DATA_WIDTH(DATA_WIDTH),
		.AUX_WIDTH(AUX_WIDTH)
	) i_ResponseTree_BRIDGE(
		.data_r_valid_i(data_r_valid_int),
		.data_r_rdata_i(data_r_rdata_int),
		.data_r_opc_i(data_r_opc_int),
		.data_r_aux_i(data_r_aux_int),
		.data_r_valid_o(data_r_valid_o),
		.data_r_rdata_o(data_r_rdata_o),
		.data_r_opc_o(data_r_opc_o),
		.data_r_aux_o(data_r_aux_o)
	);
	AddressDecoder_Req_BRIDGE #(
		.ID_WIDTH(ID_WIDTH),
		.ID(ID),
		.N_SLAVE(N_SLAVE)
	) i_AddressDecoder_Req_BRIDGE(
		.data_req_i(data_req_i),
		.destination_i(destination_i),
		.data_gnt_o(data_gnt_o),
		.data_gnt_i(data_gnt_i),
		.data_req_o(data_req_o),
		.data_ID_o(data_ID_o)
	);
endmodule
module ResponseTree_BRIDGE (
	data_r_valid_i,
	data_r_rdata_i,
	data_r_opc_i,
	data_r_aux_i,
	data_r_valid_o,
	data_r_rdata_o,
	data_r_opc_o,
	data_r_aux_o
);
	parameter N_SLAVE = 16;
	parameter DATA_WIDTH = 32;
	parameter AUX_WIDTH = 8;
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	input wire [N_SLAVE - 1:0] data_r_opc_i;
	input wire [(N_SLAVE * AUX_WIDTH) - 1:0] data_r_aux_i;
	output wire data_r_valid_o;
	output wire [DATA_WIDTH - 1:0] data_r_rdata_o;
	output wire data_r_opc_o;
	output wire [AUX_WIDTH - 1:0] data_r_aux_o;
	localparam LOG_SLAVE = $clog2(N_SLAVE);
	localparam N_WIRE = N_SLAVE - 2;
	genvar _gv_j_5;
	genvar _gv_k_2;
	generate
		case (N_SLAVE)
			1: begin : MONO_SLAVE
				assign data_r_rdata_o = data_r_rdata_i;
				assign data_r_valid_o = data_r_valid_i;
				assign data_r_opc_o = data_r_opc_i;
				assign data_r_aux_o = data_r_aux_i;
			end
			2: begin : DUAL_SLAVE
				FanInPrimitive_Resp_BRIDGE #(
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH)
				) i_FanInPrimitive_Resp_BRIDGE(
					.data_r_rdata0_i(data_r_rdata_i[0+:DATA_WIDTH]),
					.data_r_rdata1_i(data_r_rdata_i[DATA_WIDTH+:DATA_WIDTH]),
					.data_r_valid0_i(data_r_valid_i[0]),
					.data_r_valid1_i(data_r_valid_i[1]),
					.data_r_opc0_i(data_r_opc_i[0]),
					.data_r_opc1_i(data_r_opc_i[1]),
					.data_r_aux0_i(data_r_aux_i[0+:AUX_WIDTH]),
					.data_r_aux1_i(data_r_aux_i[AUX_WIDTH+:AUX_WIDTH]),
					.data_r_rdata_o(data_r_rdata_o),
					.data_r_valid_o(data_r_valid_o),
					.data_r_opc_o(data_r_opc_o),
					.data_r_aux_o(data_r_aux_o)
				);
			end
			default: begin : BINARY_TREE
				wire [DATA_WIDTH - 1:0] data_r_rdata_LEVEL [N_WIRE - 1:0];
				wire data_r_valid_LEVEL [N_WIRE - 1:0];
				wire data_r_opc_LEVEL [N_WIRE - 1:0];
				wire [AUX_WIDTH - 1:0] data_r_aux_LEVEL [N_WIRE - 1:0];
				for (_gv_j_5 = 0; _gv_j_5 < LOG_SLAVE; _gv_j_5 = _gv_j_5 + 1) begin : STAGE
					localparam j = _gv_j_5;
					for (_gv_k_2 = 0; _gv_k_2 < (2 ** j); _gv_k_2 = _gv_k_2 + 1) begin : INCR_VERT
						localparam k = _gv_k_2;
						if (j == 0) begin : LAST_NODE
							FanInPrimitive_Resp_BRIDGE #(
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH)
							) i_FanInPrimitive_Resp_BRIDGE(
								.data_r_rdata0_i(data_r_rdata_LEVEL[2 * k]),
								.data_r_rdata1_i(data_r_rdata_LEVEL[(2 * k) + 1]),
								.data_r_valid0_i(data_r_valid_LEVEL[2 * k]),
								.data_r_valid1_i(data_r_valid_LEVEL[(2 * k) + 1]),
								.data_r_opc0_i(data_r_opc_LEVEL[2 * k]),
								.data_r_opc1_i(data_r_opc_LEVEL[(2 * k) + 1]),
								.data_r_aux0_i(data_r_aux_LEVEL[2 * k]),
								.data_r_aux1_i(data_r_aux_LEVEL[(2 * k) + 1]),
								.data_r_rdata_o(data_r_rdata_o),
								.data_r_valid_o(data_r_valid_o),
								.data_r_opc_o(data_r_opc_o),
								.data_r_aux_o(data_r_aux_o)
							);
						end
						else if (j < (LOG_SLAVE - 1)) begin : MIDDLE_NODES
							FanInPrimitive_Resp_BRIDGE #(
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH)
							) i_FanInPrimitive_Resp_BRIDGE(
								.data_r_rdata0_i(data_r_rdata_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_rdata1_i(data_r_rdata_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_valid0_i(data_r_valid_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_valid1_i(data_r_valid_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_opc0_i(data_r_opc_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_opc1_i(data_r_opc_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_aux0_i(data_r_aux_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_aux1_i(data_r_aux_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_rdata_o(data_r_rdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_valid_o(data_r_valid_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_opc_o(data_r_opc_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_aux_o(data_r_aux_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
						else begin : LEAF_NODES
							FanInPrimitive_Resp_BRIDGE #(
								.DATA_WIDTH(DATA_WIDTH),
								.AUX_WIDTH(AUX_WIDTH)
							) i_FanInPrimitive_Resp_BRIDGE(
								.data_r_rdata0_i(data_r_rdata_i[(2 * k) * DATA_WIDTH+:DATA_WIDTH]),
								.data_r_rdata1_i(data_r_rdata_i[((2 * k) + 1) * DATA_WIDTH+:DATA_WIDTH]),
								.data_r_valid0_i(data_r_valid_i[2 * k]),
								.data_r_valid1_i(data_r_valid_i[(2 * k) + 1]),
								.data_r_opc0_i(data_r_opc_i[2 * k]),
								.data_r_opc1_i(data_r_opc_i[(2 * k) + 1]),
								.data_r_aux0_i(data_r_aux_i[(2 * k) * AUX_WIDTH+:AUX_WIDTH]),
								.data_r_aux1_i(data_r_aux_i[((2 * k) + 1) * AUX_WIDTH+:AUX_WIDTH]),
								.data_r_rdata_o(data_r_rdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_valid_o(data_r_valid_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_opc_o(data_r_opc_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_aux_o(data_r_aux_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
					end
				end
			end
		endcase
	endgenerate
endmodule
module XBAR_BRIDGE (
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_aux_i,
	data_gnt_o,
	data_r_valid_o,
	data_r_rdata_o,
	data_r_opc_o,
	data_r_aux_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_aux_o,
	data_gnt_i,
	data_r_rdata_i,
	data_r_valid_i,
	data_r_ID_i,
	data_r_opc_i,
	data_r_aux_i,
	clk,
	rst_n,
	START_ADDR,
	END_ADDR
);
	parameter N_CH0 = 5;
	parameter N_CH1 = 4;
	parameter N_SLAVE = 3;
	parameter ID_WIDTH = N_CH0 + N_CH1;
	parameter AUX_WIDTH = 8;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire [(N_CH0 + N_CH1) - 1:0] data_req_i;
	input wire [((N_CH0 + N_CH1) * ADDR_WIDTH) - 1:0] data_add_i;
	input wire [(N_CH0 + N_CH1) - 1:0] data_wen_i;
	input wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] data_wdata_i;
	input wire [((N_CH0 + N_CH1) * BE_WIDTH) - 1:0] data_be_i;
	input wire [((N_CH0 + N_CH1) * AUX_WIDTH) - 1:0] data_aux_i;
	output wire [(N_CH0 + N_CH1) - 1:0] data_gnt_o;
	output wire [(N_CH0 + N_CH1) - 1:0] data_r_valid_o;
	output wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] data_r_rdata_o;
	output wire [(N_CH0 + N_CH1) - 1:0] data_r_opc_o;
	output wire [((N_CH0 + N_CH1) * AUX_WIDTH) - 1:0] data_r_aux_o;
	output wire [N_SLAVE - 1:0] data_req_o;
	output wire [(N_SLAVE * ADDR_WIDTH) - 1:0] data_add_o;
	output wire [N_SLAVE - 1:0] data_wen_o;
	output wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_wdata_o;
	output wire [(N_SLAVE * BE_WIDTH) - 1:0] data_be_o;
	output wire [(N_SLAVE * ID_WIDTH) - 1:0] data_ID_o;
	output wire [(N_SLAVE * AUX_WIDTH) - 1:0] data_aux_o;
	input wire [N_SLAVE - 1:0] data_gnt_i;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * ID_WIDTH) - 1:0] data_r_ID_i;
	input wire [N_SLAVE - 1:0] data_r_opc_i;
	input wire [(N_SLAVE * AUX_WIDTH) - 1:0] data_r_aux_i;
	input wire clk;
	input wire rst_n;
	input wire [(N_SLAVE * ADDR_WIDTH) - 1:0] START_ADDR;
	input wire [(N_SLAVE * ADDR_WIDTH) - 1:0] END_ADDR;
	wire [((N_CH0 + N_CH1) * ID_WIDTH) - 1:0] data_ID;
	wire [(N_CH0 + N_CH1) - 1:0] data_gnt_from_MEM [N_SLAVE - 1:0];
	wire [N_SLAVE - 1:0] data_req_from_MASTER [(N_CH0 + N_CH1) - 1:0];
	wire [(N_CH0 + N_CH1) - 1:0] data_r_valid_from_MEM [N_SLAVE - 1:0];
	wire [N_SLAVE - 1:0] data_r_valid_to_MASTER [(N_CH0 + N_CH1) - 1:0];
	wire [(N_CH0 + N_CH1) - 1:0] data_req_to_MEM [N_SLAVE - 1:0];
	wire [N_SLAVE - 1:0] data_gnt_to_MASTER [(N_CH0 + N_CH1) - 1:0];
	reg [((N_CH0 + N_CH1) * N_SLAVE) - 1:0] destination_OH;
	initial begin
		$display("START_ADDR[0] = 0x%8h; END_ADDR[0] = 0X%8h", START_ADDR[0+:ADDR_WIDTH], END_ADDR[0+:ADDR_WIDTH]);
		$display("START_ADDR[1] = 0x%8h; END_ADDR[1] = 0X%8h", START_ADDR[ADDR_WIDTH+:ADDR_WIDTH], END_ADDR[ADDR_WIDTH+:ADDR_WIDTH]);
	end
	genvar _gv_j_6;
	genvar _gv_k_3;
	generate
		for (_gv_k_3 = 0; _gv_k_3 < (N_CH0 + N_CH1); _gv_k_3 = _gv_k_3 + 1) begin : genblk1
			localparam k = _gv_k_3;
			always @(*) begin
				destination_OH[k * N_SLAVE+:N_SLAVE] = 1'sb0;
				begin : sv2v_autoblock_1
					reg [31:0] x;
					for (x = 0; x < N_SLAVE; x = x + 1)
						if ((data_add_i[k * ADDR_WIDTH+:ADDR_WIDTH] >= START_ADDR[x * ADDR_WIDTH+:ADDR_WIDTH]) && (data_add_i[k * ADDR_WIDTH+:ADDR_WIDTH] < END_ADDR[x * ADDR_WIDTH+:ADDR_WIDTH]))
							destination_OH[(k * N_SLAVE) + x] = 1'b1;
				end
			end
			for (_gv_j_6 = 0; _gv_j_6 < N_SLAVE; _gv_j_6 = _gv_j_6 + 1) begin : genblk1
				localparam j = _gv_j_6;
				assign data_r_valid_to_MASTER[k][j] = data_r_valid_from_MEM[j][k];
				assign data_req_to_MEM[j][k] = data_req_from_MASTER[k][j];
				assign data_gnt_to_MASTER[k][j] = data_gnt_from_MEM[j][k];
			end
		end
		for (_gv_j_6 = 0; _gv_j_6 < N_SLAVE; _gv_j_6 = _gv_j_6 + 1) begin : RequestBlock
			localparam j = _gv_j_6;
			if (N_CH1 != 0) begin : CH0_CH1
				RequestBlock2CH_BRIDGE #(
					.ADDR_WIDTH(ADDR_WIDTH),
					.N_CH0(N_CH0),
					.N_CH1(N_CH1),
					.ID_WIDTH(ID_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_RequestBlock2CH_BRIDGE(
					.data_req_CH0_i(data_req_to_MEM[j][N_CH0 - 1:0]),
					.data_add_CH0_i(data_add_i[ADDR_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:ADDR_WIDTH * N_CH0]),
					.data_wen_CH0_i(data_wen_i[N_CH0 - 1:0]),
					.data_wdata_CH0_i(data_wdata_i[DATA_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:DATA_WIDTH * N_CH0]),
					.data_be_CH0_i(data_be_i[BE_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:BE_WIDTH * N_CH0]),
					.data_ID_CH0_i(data_ID[ID_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:ID_WIDTH * N_CH0]),
					.data_aux_CH0_i(data_aux_i[AUX_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:AUX_WIDTH * N_CH0]),
					.data_gnt_CH0_o(data_gnt_from_MEM[j][N_CH0 - 1:0]),
					.data_req_CH1_i(data_req_to_MEM[j][(N_CH1 + N_CH0) - 1:N_CH0]),
					.data_add_CH1_i(data_add_i[ADDR_WIDTH * ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (N_CH1 + N_CH0) - 1 : (((N_CH1 + N_CH0) - 1) + (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)) - 1) - ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1) - 1))+:ADDR_WIDTH * (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)]),
					.data_wen_CH1_i(data_wen_i[(N_CH1 + N_CH0) - 1:N_CH0]),
					.data_wdata_CH1_i(data_wdata_i[DATA_WIDTH * ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (N_CH1 + N_CH0) - 1 : (((N_CH1 + N_CH0) - 1) + (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)) - 1) - ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1) - 1))+:DATA_WIDTH * (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)]),
					.data_be_CH1_i(data_be_i[BE_WIDTH * ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (N_CH1 + N_CH0) - 1 : (((N_CH1 + N_CH0) - 1) + (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)) - 1) - ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1) - 1))+:BE_WIDTH * (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)]),
					.data_ID_CH1_i(data_ID[ID_WIDTH * ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (N_CH1 + N_CH0) - 1 : (((N_CH1 + N_CH0) - 1) + (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)) - 1) - ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1) - 1))+:ID_WIDTH * (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)]),
					.data_aux_CH1_i(data_aux_i[AUX_WIDTH * ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (N_CH1 + N_CH0) - 1 : (((N_CH1 + N_CH0) - 1) + (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)) - 1) - ((((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1) - 1))+:AUX_WIDTH * (((N_CH1 + N_CH0) - 1) >= N_CH0 ? (((N_CH1 + N_CH0) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH1 + N_CH0) - 1)) + 1)]),
					.data_gnt_CH1_o(data_gnt_from_MEM[j][(N_CH1 + N_CH0) - 1:N_CH0]),
					.data_req_o(data_req_o[j]),
					.data_add_o(data_add_o[j * ADDR_WIDTH+:ADDR_WIDTH]),
					.data_wen_o(data_wen_o[j]),
					.data_wdata_o(data_wdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_be_o(data_be_o[j * BE_WIDTH+:BE_WIDTH]),
					.data_ID_o(data_ID_o[j * ID_WIDTH+:ID_WIDTH]),
					.data_aux_o(data_aux_o[j * AUX_WIDTH+:AUX_WIDTH]),
					.data_gnt_i(data_gnt_i[j]),
					.data_r_valid_i(data_r_valid_i[j]),
					.data_r_ID_i(data_r_ID_i[j * ID_WIDTH+:ID_WIDTH]),
					.data_r_valid_CH0_o(data_r_valid_from_MEM[j][N_CH0 - 1:0]),
					.data_r_valid_CH1_o(data_r_valid_from_MEM[j][(N_CH0 + N_CH1) - 1:N_CH0]),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : CH0_ONLY
				RequestBlock1CH_BRIDGE #(
					.ADDR_WIDTH(ADDR_WIDTH),
					.N_CH0(N_CH0),
					.ID_WIDTH(ID_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.AUX_WIDTH(AUX_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) i_RequestBlock1CH_BRIDGE(
					.data_req_CH0_i(data_req_to_MEM[j]),
					.data_add_CH0_i(data_add_i),
					.data_wen_CH0_i(data_wen_i),
					.data_wdata_CH0_i(data_wdata_i),
					.data_be_CH0_i(data_be_i),
					.data_ID_CH0_i(data_ID),
					.data_aux_CH0_i(data_aux_i),
					.data_gnt_CH0_o(data_gnt_from_MEM[j]),
					.data_req_o(data_req_o[j]),
					.data_add_o(data_add_o[j * ADDR_WIDTH+:ADDR_WIDTH]),
					.data_wen_o(data_wen_o[j]),
					.data_wdata_o(data_wdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_be_o(data_be_o[j * BE_WIDTH+:BE_WIDTH]),
					.data_ID_o(data_ID_o[j * ID_WIDTH+:ID_WIDTH]),
					.data_aux_o(data_aux_o[j * AUX_WIDTH+:AUX_WIDTH]),
					.data_gnt_i(data_gnt_i[j]),
					.data_r_valid_i(data_r_valid_i[j]),
					.data_r_ID_i(data_r_ID_i[j * ID_WIDTH+:ID_WIDTH]),
					.data_r_valid_CH0_o(data_r_valid_from_MEM[j]),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
		if (N_SLAVE == 1) begin : ResponseBlock_mono
			for (_gv_j_6 = 0; _gv_j_6 < (N_CH0 + N_CH1); _gv_j_6 = _gv_j_6 + 1) begin : WIRING
				localparam j = _gv_j_6;
				assign data_r_rdata_o[j * DATA_WIDTH+:DATA_WIDTH] = data_r_rdata_i;
				assign data_r_opc_o[j] = data_r_opc_i;
				assign data_r_valid_o[j] = data_r_valid_to_MASTER[j];
				assign data_ID[j * ID_WIDTH+:ID_WIDTH] = 2 ** j;
				assign data_req_from_MASTER[j] = data_req_i[j];
				assign data_gnt_o[j] = data_gnt_to_MASTER[j];
			end
		end
		else begin : ResponseBlock_multi
			for (_gv_j_6 = 0; _gv_j_6 < (N_CH0 + N_CH1); _gv_j_6 = _gv_j_6 + 1) begin : ResponseBlock_PE_Block
				localparam j = _gv_j_6;
				ResponseBlock_BRIDGE #(
					.ID(2 ** j),
					.ID_WIDTH(ID_WIDTH),
					.N_SLAVE(N_SLAVE),
					.AUX_WIDTH(AUX_WIDTH),
					.DATA_WIDTH(DATA_WIDTH)
				) i_ResponseBlock_BRIDGE(
					.data_r_valid_i(data_r_valid_to_MASTER[j]),
					.data_r_rdata_i(data_r_rdata_i),
					.data_r_opc_i(data_r_opc_i),
					.data_r_aux_i(data_r_aux_i),
					.data_r_valid_o(data_r_valid_o[j]),
					.data_r_rdata_o(data_r_rdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_r_opc_o(data_r_opc_o[j]),
					.data_r_aux_o(data_r_aux_o[j * AUX_WIDTH+:AUX_WIDTH]),
					.data_req_i(data_req_i[j]),
					.destination_i(destination_OH[j * N_SLAVE+:N_SLAVE]),
					.data_gnt_o(data_gnt_o[j]),
					.data_gnt_i(data_gnt_to_MASTER[j]),
					.data_req_o(data_req_from_MASTER[j]),
					.data_ID_o(data_ID[j * ID_WIDTH+:ID_WIDTH])
				);
			end
		end
	endgenerate
endmodule
module AddressDecoder_Req_L2 (
	data_req_i,
	routing_addr_i,
	data_gnt_o,
	data_gnt_i,
	data_req_o,
	data_ID_o
);
	reg _sv2v_0;
	parameter ID_WIDTH = 5;
	parameter ID = 1;
	parameter N_SLAVE = 8;
	parameter ROUT_WIDTH = $clog2(N_SLAVE);
	input wire data_req_i;
	input wire [ROUT_WIDTH - 1:0] routing_addr_i;
	output reg data_gnt_o;
	input wire [N_SLAVE - 1:0] data_gnt_i;
	output reg [N_SLAVE - 1:0] data_req_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	assign data_ID_o = ID;
	always @(*) begin : Combinational_ADDR_DEC_REQ
		if (_sv2v_0)
			;
		data_req_o = 1'sb0;
		data_req_o[routing_addr_i] = data_req_i;
		data_gnt_o = data_gnt_i[routing_addr_i];
	end
	initial _sv2v_0 = 0;
endmodule
module AddressDecoder_Resp_L2 (
	data_r_valid_i,
	data_r_ID_i,
	data_r_valid_o
);
	parameter N_MASTER = 8;
	parameter ID_WIDTH = N_MASTER;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_r_ID_i;
	output wire [N_MASTER - 1:0] data_r_valid_o;
	assign data_r_valid_o = {ID_WIDTH {data_r_valid_i}} & data_r_ID_i;
endmodule
module ArbitrationTree_L2 (
	clk,
	rst_n,
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_ID_i,
	data_gnt_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_gnt_i
);
	parameter ADDR_WIDTH = 12;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter ID_WIDTH = 20;
	parameter N_MASTER = 16;
	parameter MAX_COUNT = N_MASTER;
	input wire clk;
	input wire rst_n;
	input wire [N_MASTER - 1:0] data_req_i;
	input wire [(N_MASTER * ADDR_WIDTH) - 1:0] data_add_i;
	input wire [N_MASTER - 1:0] data_wen_i;
	input wire [(N_MASTER * DATA_WIDTH) - 1:0] data_wdata_i;
	input wire [(N_MASTER * BE_WIDTH) - 1:0] data_be_i;
	input wire [(N_MASTER * ID_WIDTH) - 1:0] data_ID_i;
	output wire [N_MASTER - 1:0] data_gnt_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	input wire data_gnt_i;
	localparam LOG_MASTER = $clog2(N_MASTER);
	localparam N_WIRE = N_MASTER - 2;
	wire [LOG_MASTER - 1:0] RR_FLAG;
	genvar _gv_j_7;
	genvar _gv_k_4;
	generate
		if (N_MASTER == 2) begin : INCR
			FanInPrimitive_Req_L2 #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH)
			) FAN_IN_REQ(
				.RR_FLAG(RR_FLAG),
				.data_wdata0_i(data_wdata_i[0+:DATA_WIDTH]),
				.data_wdata1_i(data_wdata_i[DATA_WIDTH+:DATA_WIDTH]),
				.data_add0_i(data_add_i[0+:ADDR_WIDTH]),
				.data_add1_i(data_add_i[ADDR_WIDTH+:ADDR_WIDTH]),
				.data_req0_i(data_req_i[0]),
				.data_req1_i(data_req_i[1]),
				.data_wen0_i(data_wen_i[0]),
				.data_wen1_i(data_wen_i[1]),
				.data_ID0_i(data_ID_i[0+:ID_WIDTH]),
				.data_ID1_i(data_ID_i[ID_WIDTH+:ID_WIDTH]),
				.data_be0_i(data_be_i[0+:BE_WIDTH]),
				.data_be1_i(data_be_i[BE_WIDTH+:BE_WIDTH]),
				.data_gnt0_o(data_gnt_o[0]),
				.data_gnt1_o(data_gnt_o[1]),
				.data_wdata_o(data_wdata_o),
				.data_add_o(data_add_o),
				.data_req_o(data_req_o),
				.data_wen_o(data_wen_o),
				.data_ID_o(data_ID_o),
				.data_be_o(data_be_o),
				.data_gnt_i(data_gnt_i)
			);
		end
		else begin : BINARY_TREE
			wire [DATA_WIDTH - 1:0] data_wdata_LEVEL [N_WIRE - 1:0];
			wire [ADDR_WIDTH - 1:0] data_add_LEVEL [N_WIRE - 1:0];
			wire data_req_LEVEL [N_WIRE - 1:0];
			wire data_wen_LEVEL [N_WIRE - 1:0];
			wire [ID_WIDTH - 1:0] data_ID_LEVEL [N_WIRE - 1:0];
			wire [BE_WIDTH - 1:0] data_be_LEVEL [N_WIRE - 1:0];
			wire data_gnt_LEVEL [N_WIRE - 1:0];
			for (_gv_j_7 = 0; _gv_j_7 < LOG_MASTER; _gv_j_7 = _gv_j_7 + 1) begin : STAGE
				localparam j = _gv_j_7;
				for (_gv_k_4 = 0; _gv_k_4 < (2 ** j); _gv_k_4 = _gv_k_4 + 1) begin : INCR_VERT
					localparam k = _gv_k_4;
					if (j == 0) begin : LAST_NODE
						FanInPrimitive_Req_L2 #(
							.ADDR_WIDTH(ADDR_WIDTH),
							.ID_WIDTH(ID_WIDTH),
							.DATA_WIDTH(DATA_WIDTH),
							.BE_WIDTH(BE_WIDTH)
						) FAN_IN_REQ(
							.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
							.data_wdata0_i(data_wdata_LEVEL[2 * k]),
							.data_wdata1_i(data_wdata_LEVEL[(2 * k) + 1]),
							.data_add0_i(data_add_LEVEL[2 * k]),
							.data_add1_i(data_add_LEVEL[(2 * k) + 1]),
							.data_req0_i(data_req_LEVEL[2 * k]),
							.data_req1_i(data_req_LEVEL[(2 * k) + 1]),
							.data_wen0_i(data_wen_LEVEL[2 * k]),
							.data_wen1_i(data_wen_LEVEL[(2 * k) + 1]),
							.data_ID0_i(data_ID_LEVEL[2 * k]),
							.data_ID1_i(data_ID_LEVEL[(2 * k) + 1]),
							.data_be0_i(data_be_LEVEL[2 * k]),
							.data_be1_i(data_be_LEVEL[(2 * k) + 1]),
							.data_gnt0_o(data_gnt_LEVEL[2 * k]),
							.data_gnt1_o(data_gnt_LEVEL[(2 * k) + 1]),
							.data_wdata_o(data_wdata_o),
							.data_add_o(data_add_o),
							.data_req_o(data_req_o),
							.data_wen_o(data_wen_o),
							.data_ID_o(data_ID_o),
							.data_be_o(data_be_o),
							.data_gnt_i(data_gnt_i)
						);
					end
					else if (j < (LOG_MASTER - 1)) begin : MIDDLE_NODES
						FanInPrimitive_Req_L2 #(
							.ADDR_WIDTH(ADDR_WIDTH),
							.ID_WIDTH(ID_WIDTH),
							.DATA_WIDTH(DATA_WIDTH),
							.BE_WIDTH(BE_WIDTH)
						) FAN_IN_REQ(
							.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
							.data_wdata0_i(data_wdata_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_wdata1_i(data_wdata_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_add0_i(data_add_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_add1_i(data_add_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_req0_i(data_req_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_req1_i(data_req_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_wen0_i(data_wen_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_wen1_i(data_wen_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_ID0_i(data_ID_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_ID1_i(data_ID_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_be0_i(data_be_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_be1_i(data_be_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_gnt0_o(data_gnt_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
							.data_gnt1_o(data_gnt_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
							.data_wdata_o(data_wdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_add_o(data_add_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_req_o(data_req_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_wen_o(data_wen_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_ID_o(data_ID_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_be_o(data_be_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_gnt_i(data_gnt_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
						);
					end
					else begin : LEAF_NODES
						FanInPrimitive_Req_L2 #(
							.ADDR_WIDTH(ADDR_WIDTH),
							.ID_WIDTH(ID_WIDTH),
							.DATA_WIDTH(DATA_WIDTH),
							.BE_WIDTH(BE_WIDTH)
						) FAN_IN_REQ(
							.RR_FLAG(RR_FLAG[(LOG_MASTER - j) - 1]),
							.data_wdata0_i(data_wdata_i[(2 * k) * DATA_WIDTH+:DATA_WIDTH]),
							.data_wdata1_i(data_wdata_i[((2 * k) + 1) * DATA_WIDTH+:DATA_WIDTH]),
							.data_add0_i(data_add_i[(2 * k) * ADDR_WIDTH+:ADDR_WIDTH]),
							.data_add1_i(data_add_i[((2 * k) + 1) * ADDR_WIDTH+:ADDR_WIDTH]),
							.data_req0_i(data_req_i[2 * k]),
							.data_req1_i(data_req_i[(2 * k) + 1]),
							.data_wen0_i(data_wen_i[2 * k]),
							.data_wen1_i(data_wen_i[(2 * k) + 1]),
							.data_ID0_i(data_ID_i[(2 * k) * ID_WIDTH+:ID_WIDTH]),
							.data_ID1_i(data_ID_i[((2 * k) + 1) * ID_WIDTH+:ID_WIDTH]),
							.data_be0_i(data_be_i[(2 * k) * BE_WIDTH+:BE_WIDTH]),
							.data_be1_i(data_be_i[((2 * k) + 1) * BE_WIDTH+:BE_WIDTH]),
							.data_gnt0_o(data_gnt_o[2 * k]),
							.data_gnt1_o(data_gnt_o[(2 * k) + 1]),
							.data_wdata_o(data_wdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_add_o(data_add_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_req_o(data_req_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_wen_o(data_wen_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_ID_o(data_ID_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_be_o(data_be_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
							.data_gnt_i(data_gnt_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
						);
					end
				end
			end
		end
	endgenerate
	RR_Flag_Req_L2 #(
		.WIDTH(LOG_MASTER),
		.MAX_COUNT(MAX_COUNT)
	) RR_REQ(
		.clk(clk),
		.rst_n(rst_n),
		.RR_FLAG_o(RR_FLAG),
		.data_req_i(data_req_o),
		.data_gnt_i(data_gnt_i)
	);
endmodule
module FanInPrimitive_Req_L2 (
	RR_FLAG,
	data_wdata0_i,
	data_wdata1_i,
	data_add0_i,
	data_add1_i,
	data_req0_i,
	data_req1_i,
	data_wen0_i,
	data_wen1_i,
	data_be0_i,
	data_be1_i,
	data_ID0_i,
	data_ID1_i,
	data_gnt0_o,
	data_gnt1_o,
	data_wdata_o,
	data_add_o,
	data_req_o,
	data_ID_o,
	data_wen_o,
	data_be_o,
	data_gnt_i
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter ID_WIDTH = 16;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire RR_FLAG;
	input wire [DATA_WIDTH - 1:0] data_wdata0_i;
	input wire [DATA_WIDTH - 1:0] data_wdata1_i;
	input wire [ADDR_WIDTH - 1:0] data_add0_i;
	input wire [ADDR_WIDTH - 1:0] data_add1_i;
	input wire data_req0_i;
	input wire data_req1_i;
	input wire data_wen0_i;
	input wire data_wen1_i;
	input wire [BE_WIDTH - 1:0] data_be0_i;
	input wire [BE_WIDTH - 1:0] data_be1_i;
	input wire [ID_WIDTH - 1:0] data_ID0_i;
	input wire [ID_WIDTH - 1:0] data_ID1_i;
	output wire data_gnt0_o;
	output wire data_gnt1_o;
	output reg [DATA_WIDTH - 1:0] data_wdata_o;
	output reg [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_req_o;
	output reg [ID_WIDTH - 1:0] data_ID_o;
	output reg data_wen_o;
	output reg [BE_WIDTH - 1:0] data_be_o;
	input wire data_gnt_i;
	wire SEL;
	assign data_req_o = data_req0_i | data_req1_i;
	assign SEL = ~data_req0_i | (RR_FLAG & data_req1_i);
	assign data_gnt0_o = ((data_req0_i & ~data_req1_i) | (data_req0_i & ~RR_FLAG)) & data_gnt_i;
	assign data_gnt1_o = ((~data_req0_i & data_req1_i) | (data_req1_i & RR_FLAG)) & data_gnt_i;
	always @(*) begin : FanIn_MUX2
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: begin
				data_wdata_o = data_wdata0_i;
				data_add_o = data_add0_i;
				data_wen_o = data_wen0_i;
				data_ID_o = data_ID0_i;
				data_be_o = data_be0_i;
			end
			1'b1: begin
				data_wdata_o = data_wdata1_i;
				data_add_o = data_add1_i;
				data_wen_o = data_wen1_i;
				data_ID_o = data_ID1_i;
				data_be_o = data_be1_i;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module FanInPrimitive_Resp_L2 (
	data_r_rdata0_i,
	data_r_rdata1_i,
	data_r_valid0_i,
	data_r_valid1_i,
	data_r_rdata_o,
	data_r_valid_o
);
	reg _sv2v_0;
	parameter DATA_WIDTH = 64;
	input wire [DATA_WIDTH - 1:0] data_r_rdata0_i;
	input wire [DATA_WIDTH - 1:0] data_r_rdata1_i;
	input wire data_r_valid0_i;
	input wire data_r_valid1_i;
	output reg [DATA_WIDTH - 1:0] data_r_rdata_o;
	output wire data_r_valid_o;
	wire SEL;
	assign data_r_valid_o = data_r_valid1_i | data_r_valid0_i;
	assign SEL = data_r_valid1_i;
	always @(*) begin : FanOut_MUX2
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: data_r_rdata_o = data_r_rdata0_i;
			1'b1: data_r_rdata_o = data_r_rdata1_i;
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module MUX2_REQ_L2 (
	data_req_CH0_i,
	data_add_CH0_i,
	data_wen_CH0_i,
	data_wdata_CH0_i,
	data_be_CH0_i,
	data_ID_CH0_i,
	data_gnt_CH0_o,
	data_req_CH1_i,
	data_add_CH1_i,
	data_wen_CH1_i,
	data_wdata_CH1_i,
	data_be_CH1_i,
	data_ID_CH1_i,
	data_gnt_CH1_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_gnt_i,
	clk,
	rst_n
);
	reg _sv2v_0;
	parameter ID_WIDTH = 20;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	input wire data_req_CH0_i;
	input wire [ADDR_WIDTH - 1:0] data_add_CH0_i;
	input wire data_wen_CH0_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_CH0_i;
	input wire [BE_WIDTH - 1:0] data_be_CH0_i;
	input wire [ID_WIDTH - 1:0] data_ID_CH0_i;
	output wire data_gnt_CH0_o;
	input wire data_req_CH1_i;
	input wire [ADDR_WIDTH - 1:0] data_add_CH1_i;
	input wire data_wen_CH1_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_CH1_i;
	input wire [BE_WIDTH - 1:0] data_be_CH1_i;
	input wire [ID_WIDTH - 1:0] data_ID_CH1_i;
	output wire data_gnt_CH1_o;
	output wire data_req_o;
	output reg [ADDR_WIDTH - 1:0] data_add_o;
	output reg data_wen_o;
	output reg [DATA_WIDTH - 1:0] data_wdata_o;
	output reg [BE_WIDTH - 1:0] data_be_o;
	output reg [ID_WIDTH - 1:0] data_ID_o;
	input wire data_gnt_i;
	input wire clk;
	input wire rst_n;
	wire SEL;
	reg RR_FLAG;
	assign data_req_o = data_req_CH0_i | data_req_CH1_i;
	assign SEL = ~data_req_CH0_i | (RR_FLAG & data_req_CH1_i);
	assign data_gnt_CH0_o = ((data_req_CH0_i & ~data_req_CH1_i) | (data_req_CH0_i & ~RR_FLAG)) & data_gnt_i;
	assign data_gnt_CH1_o = ((~data_req_CH0_i & data_req_CH1_i) | (data_req_CH1_i & RR_FLAG)) & data_gnt_i;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			RR_FLAG <= 1'b0;
		else if ((data_req_o == 1'b1) && (data_gnt_i == 1'b1))
			RR_FLAG <= ~RR_FLAG;
	always @(*) begin : MUX2_REQ_COMB
		if (_sv2v_0)
			;
		case (SEL)
			1'b0: begin
				data_add_o = data_add_CH0_i;
				data_wen_o = data_wen_CH0_i;
				data_wdata_o = data_wdata_CH0_i;
				data_be_o = data_be_CH0_i;
				data_ID_o = data_ID_CH0_i;
			end
			1'b1: begin
				data_add_o = data_add_CH1_i;
				data_wen_o = data_wen_CH1_i;
				data_wdata_o = data_wdata_CH1_i;
				data_be_o = data_be_CH1_i;
				data_ID_o = data_ID_CH1_i;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module RR_Flag_Req_L2 (
	clk,
	rst_n,
	RR_FLAG_o,
	data_req_i,
	data_gnt_i
);
	parameter WIDTH = 3;
	parameter MAX_COUNT = (2 ** WIDTH) - 1;
	input wire clk;
	input wire rst_n;
	output reg [WIDTH - 1:0] RR_FLAG_o;
	input wire data_req_i;
	input wire data_gnt_i;
	always @(posedge clk or negedge rst_n) begin : RR_Flag_Req_SEQ
		if (rst_n == 1'b0)
			RR_FLAG_o <= 1'sb0;
		else if (data_req_i & data_gnt_i) begin
			if (RR_FLAG_o < MAX_COUNT)
				RR_FLAG_o <= RR_FLAG_o + 1'b1;
			else
				RR_FLAG_o <= 1'sb0;
		end
	end
endmodule
module RequestBlock_L2_1CH (
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_ID_i,
	data_gnt_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_gnt_i,
	data_r_valid_i,
	data_r_ID_i,
	data_r_valid_o,
	clk,
	rst_n
);
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter N_CH0 = 16;
	parameter ID_WIDTH = N_CH0;
	input wire [N_CH0 - 1:0] data_req_i;
	input wire [(N_CH0 * ADDR_WIDTH) - 1:0] data_add_i;
	input wire [N_CH0 - 1:0] data_wen_i;
	input wire [(N_CH0 * DATA_WIDTH) - 1:0] data_wdata_i;
	input wire [(N_CH0 * BE_WIDTH) - 1:0] data_be_i;
	input wire [(N_CH0 * ID_WIDTH) - 1:0] data_ID_i;
	output wire [N_CH0 - 1:0] data_gnt_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	input wire data_gnt_i;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_r_ID_i;
	output wire [N_CH0 - 1:0] data_r_valid_o;
	input wire clk;
	input wire rst_n;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_req_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ADDR_WIDTH) - 1:0] data_add_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_wen_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * DATA_WIDTH) - 1:0] data_wdata_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * BE_WIDTH) - 1:0] data_be_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ID_WIDTH) - 1:0] data_ID_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_gnt_CH0_int;
	generate
		if ((2 ** $clog2(N_CH0)) != N_CH0) begin : _DUMMY_CH0_PORTS_
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_req_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ADDR_WIDTH) - 1:0] data_add_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_wen_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * DATA_WIDTH) - 1:0] data_wdata_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * BE_WIDTH) - 1:0] data_be_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ID_WIDTH) - 1:0] data_ID_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_gnt_CH0_dummy;
			assign data_req_CH0_dummy = 1'sb0;
			assign data_add_CH0_dummy = 1'sb0;
			assign data_wen_CH0_dummy = 1'sb0;
			assign data_wdata_CH0_dummy = 1'sb0;
			assign data_be_CH0_dummy = 1'sb0;
			assign data_ID_CH0_dummy = 1'sb0;
			assign data_req_CH0_int = {data_req_CH0_dummy, data_req_i};
			assign data_add_CH0_int = {data_add_CH0_dummy, data_add_i};
			assign data_wen_CH0_int = {data_wen_CH0_dummy, data_wen_i};
			assign data_wdata_CH0_int = {data_wdata_CH0_dummy, data_wdata_i};
			assign data_be_CH0_int = {data_be_CH0_dummy, data_be_i};
			assign data_ID_CH0_int = {data_ID_CH0_dummy, data_ID_i};
			genvar _gv_j_8;
			for (_gv_j_8 = 0; _gv_j_8 < N_CH0; _gv_j_8 = _gv_j_8 + 1) begin : _MERGING_CH0_DUMMY_PORTS_OUT_
				localparam j = _gv_j_8;
				assign data_gnt_o[j] = data_gnt_CH0_int[j];
			end
		end
		else begin : genblk1
			assign data_req_CH0_int = data_req_i;
			assign data_add_CH0_int = data_add_i;
			assign data_wen_CH0_int = data_wen_i;
			assign data_wdata_CH0_int = data_wdata_i;
			assign data_be_CH0_int = data_be_i;
			assign data_ID_CH0_int = data_ID_i;
			assign data_gnt_o = data_gnt_CH0_int;
		end
		if (N_CH0 > 1) begin : POLY_CH0
			ArbitrationTree_L2 #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(N_CH0),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.MAX_COUNT(N_CH0 - 1)
			) CH0_ARB_TREE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH0_int),
				.data_add_i(data_add_CH0_int),
				.data_wen_i(data_wen_CH0_int),
				.data_wdata_i(data_wdata_CH0_int),
				.data_be_i(data_be_CH0_int),
				.data_ID_i(data_ID_CH0_int),
				.data_gnt_o(data_gnt_CH0_int),
				.data_req_o(data_req_o),
				.data_add_o(data_add_o),
				.data_wen_o(data_wen_o),
				.data_wdata_o(data_wdata_o),
				.data_be_o(data_be_o),
				.data_ID_o(data_ID_o),
				.data_gnt_i(data_gnt_i)
			);
		end
		else begin : MONO_CH0
			assign data_req_o = data_req_CH0_int;
			assign data_add_o = data_add_CH0_int;
			assign data_wen_o = data_wen_CH0_int;
			assign data_wdata_o = data_wdata_CH0_int;
			assign data_be_o = data_be_CH0_int;
			assign data_ID_o = data_ID_CH0_int;
			assign data_gnt_CH0_int = data_gnt_i;
		end
	endgenerate
	AddressDecoder_Resp_L2 #(
		.ID_WIDTH(ID_WIDTH),
		.N_MASTER(N_CH0)
	) ADDR_DEC_RESP(
		.data_r_valid_i(data_r_valid_i),
		.data_r_ID_i(data_r_ID_i),
		.data_r_valid_o(data_r_valid_o)
	);
endmodule
module RequestBlock_L2_2CH (
	data_req_CH0_i,
	data_add_CH0_i,
	data_wen_CH0_i,
	data_wdata_CH0_i,
	data_be_CH0_i,
	data_ID_CH0_i,
	data_gnt_CH0_o,
	data_req_CH1_i,
	data_add_CH1_i,
	data_wen_CH1_i,
	data_wdata_CH1_i,
	data_be_CH1_i,
	data_ID_CH1_i,
	data_gnt_CH1_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_gnt_i,
	data_r_valid_i,
	data_r_ID_i,
	data_r_valid_CH0_o,
	data_r_valid_CH1_o,
	clk,
	rst_n
);
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter N_CH0 = 5;
	parameter N_CH1 = 4;
	parameter ID_WIDTH = N_CH0 + N_CH1;
	input wire [N_CH0 - 1:0] data_req_CH0_i;
	input wire [(N_CH0 * ADDR_WIDTH) - 1:0] data_add_CH0_i;
	input wire [N_CH0 - 1:0] data_wen_CH0_i;
	input wire [(N_CH0 * DATA_WIDTH) - 1:0] data_wdata_CH0_i;
	input wire [(N_CH0 * BE_WIDTH) - 1:0] data_be_CH0_i;
	input wire [(N_CH0 * ID_WIDTH) - 1:0] data_ID_CH0_i;
	output wire [N_CH0 - 1:0] data_gnt_CH0_o;
	input wire [N_CH1 - 1:0] data_req_CH1_i;
	input wire [(N_CH1 * ADDR_WIDTH) - 1:0] data_add_CH1_i;
	input wire [N_CH1 - 1:0] data_wen_CH1_i;
	input wire [(N_CH1 * DATA_WIDTH) - 1:0] data_wdata_CH1_i;
	input wire [(N_CH1 * BE_WIDTH) - 1:0] data_be_CH1_i;
	input wire [(N_CH1 * ID_WIDTH) - 1:0] data_ID_CH1_i;
	output wire [N_CH1 - 1:0] data_gnt_CH1_o;
	output wire data_req_o;
	output wire [ADDR_WIDTH - 1:0] data_add_o;
	output wire data_wen_o;
	output wire [DATA_WIDTH - 1:0] data_wdata_o;
	output wire [BE_WIDTH - 1:0] data_be_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	input wire data_gnt_i;
	input wire data_r_valid_i;
	input wire [ID_WIDTH - 1:0] data_r_ID_i;
	output wire [N_CH0 - 1:0] data_r_valid_CH0_o;
	output wire [N_CH1 - 1:0] data_r_valid_CH1_o;
	input wire clk;
	input wire rst_n;
	wire data_req_CH0;
	wire [ADDR_WIDTH - 1:0] data_add_CH0;
	wire data_wen_CH0;
	wire [DATA_WIDTH - 1:0] data_wdata_CH0;
	wire [BE_WIDTH - 1:0] data_be_CH0;
	wire [ID_WIDTH - 1:0] data_ID_CH0;
	wire data_gnt_CH0;
	wire data_req_CH1;
	wire [ADDR_WIDTH - 1:0] data_add_CH1;
	wire data_wen_CH1;
	wire [DATA_WIDTH - 1:0] data_wdata_CH1;
	wire [BE_WIDTH - 1:0] data_be_CH1;
	wire [ID_WIDTH - 1:0] data_ID_CH1;
	wire data_gnt_CH1;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_req_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ADDR_WIDTH) - 1:0] data_add_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_wen_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * DATA_WIDTH) - 1:0] data_wdata_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * BE_WIDTH) - 1:0] data_be_CH0_int;
	wire [((2 ** $clog2(N_CH0)) * ID_WIDTH) - 1:0] data_ID_CH0_int;
	wire [(2 ** $clog2(N_CH0)) - 1:0] data_gnt_CH0_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_req_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * ADDR_WIDTH) - 1:0] data_add_CH1_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_wen_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * DATA_WIDTH) - 1:0] data_wdata_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * BE_WIDTH) - 1:0] data_be_CH1_int;
	wire [((2 ** $clog2(N_CH1)) * ID_WIDTH) - 1:0] data_ID_CH1_int;
	wire [(2 ** $clog2(N_CH1)) - 1:0] data_gnt_CH1_int;
	generate
		if ((2 ** $clog2(N_CH0)) != N_CH0) begin : _DUMMY_CH0_PORTS_
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_req_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ADDR_WIDTH) - 1:0] data_add_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_wen_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * DATA_WIDTH) - 1:0] data_wdata_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * BE_WIDTH) - 1:0] data_be_CH0_dummy;
			wire [(((2 ** $clog2(N_CH0)) - N_CH0) * ID_WIDTH) - 1:0] data_ID_CH0_dummy;
			wire [((2 ** $clog2(N_CH0)) - N_CH0) - 1:0] data_gnt_CH0_dummy;
			assign data_req_CH0_dummy = 1'sb0;
			assign data_add_CH0_dummy = 1'sb0;
			assign data_wen_CH0_dummy = 1'sb0;
			assign data_wdata_CH0_dummy = 1'sb0;
			assign data_be_CH0_dummy = 1'sb0;
			assign data_ID_CH0_dummy = 1'sb0;
			assign data_req_CH0_int = {data_req_CH0_dummy, data_req_CH0_i};
			assign data_add_CH0_int = {data_add_CH0_dummy, data_add_CH0_i};
			assign data_wen_CH0_int = {data_wen_CH0_dummy, data_wen_CH0_i};
			assign data_wdata_CH0_int = {data_wdata_CH0_dummy, data_wdata_CH0_i};
			assign data_be_CH0_int = {data_be_CH0_dummy, data_be_CH0_i};
			assign data_ID_CH0_int = {data_ID_CH0_dummy, data_ID_CH0_i};
			genvar _gv_j_9;
			for (_gv_j_9 = 0; _gv_j_9 < N_CH0; _gv_j_9 = _gv_j_9 + 1) begin : _MERGING_CH0_DUMMY_PORTS_OUT_
				localparam j = _gv_j_9;
				assign data_gnt_CH0_o[j] = data_gnt_CH0_int[j];
			end
		end
		else begin : genblk1
			assign data_req_CH0_int = data_req_CH0_i;
			assign data_add_CH0_int = data_add_CH0_i;
			assign data_wen_CH0_int = data_wen_CH0_i;
			assign data_wdata_CH0_int = data_wdata_CH0_i;
			assign data_be_CH0_int = data_be_CH0_i;
			assign data_ID_CH0_int = data_ID_CH0_i;
			assign data_gnt_CH0_o = data_gnt_CH0_int;
		end
		if ((2 ** $clog2(N_CH1)) != N_CH1) begin : _DUMMY_CH1_PORTS_
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_req_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * ADDR_WIDTH) - 1:0] data_add_CH1_dummy;
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_wen_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * DATA_WIDTH) - 1:0] data_wdata_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * BE_WIDTH) - 1:0] data_be_CH1_dummy;
			wire [(((2 ** $clog2(N_CH1)) - N_CH1) * ID_WIDTH) - 1:0] data_ID_CH1_dummy;
			wire [((2 ** $clog2(N_CH1)) - N_CH1) - 1:0] data_gnt_CH1_dummy;
			assign data_req_CH1_dummy = 1'sb0;
			assign data_add_CH1_dummy = 1'sb0;
			assign data_wen_CH1_dummy = 1'sb0;
			assign data_wdata_CH1_dummy = 1'sb0;
			assign data_be_CH1_dummy = 1'sb0;
			assign data_ID_CH1_dummy = 1'sb0;
			assign data_req_CH1_int = {data_req_CH1_dummy, data_req_CH1_i};
			assign data_add_CH1_int = {data_add_CH1_dummy, data_add_CH1_i};
			assign data_wen_CH1_int = {data_wen_CH1_dummy, data_wen_CH1_i};
			assign data_wdata_CH1_int = {data_wdata_CH1_dummy, data_wdata_CH1_i};
			assign data_be_CH1_int = {data_be_CH1_dummy, data_be_CH1_i};
			assign data_ID_CH1_int = {data_ID_CH1_dummy, data_ID_CH1_i};
			genvar _gv_j_10;
			for (_gv_j_10 = 0; _gv_j_10 < N_CH1; _gv_j_10 = _gv_j_10 + 1) begin : _MERGING_CH1_DUMMY_PORTS_OUT_
				localparam j = _gv_j_10;
				assign data_gnt_CH1_o[j] = data_gnt_CH1_int[j];
			end
		end
		else begin : genblk2
			assign data_req_CH1_int = data_req_CH1_i;
			assign data_add_CH1_int = data_add_CH1_i;
			assign data_wen_CH1_int = data_wen_CH1_i;
			assign data_wdata_CH1_int = data_wdata_CH1_i;
			assign data_be_CH1_int = data_be_CH1_i;
			assign data_ID_CH1_int = data_ID_CH1_i;
			assign data_gnt_CH1_o = data_gnt_CH1_int;
		end
		if (N_CH0 > 1) begin : CH0_ARB_TREE
			ArbitrationTree_L2 #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(2 ** $clog2(N_CH0)),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.MAX_COUNT(N_CH0 - 1)
			) CH0_ARB_TREE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH0_int),
				.data_add_i(data_add_CH0_int),
				.data_wen_i(data_wen_CH0_int),
				.data_wdata_i(data_wdata_CH0_int),
				.data_be_i(data_be_CH0_int),
				.data_ID_i(data_ID_CH0_int),
				.data_gnt_o(data_gnt_CH0_int),
				.data_req_o(data_req_CH0),
				.data_add_o(data_add_CH0),
				.data_wen_o(data_wen_CH0),
				.data_wdata_o(data_wdata_CH0),
				.data_be_o(data_be_CH0),
				.data_ID_o(data_ID_CH0),
				.data_gnt_i(data_gnt_CH0)
			);
		end
		if (N_CH1 > 1) begin : CH1_ARB_TREE
			ArbitrationTree_L2 #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.ID_WIDTH(ID_WIDTH),
				.N_MASTER(2 ** $clog2(N_CH1)),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.MAX_COUNT(N_CH1 - 1)
			) CH1_ARB_TREE(
				.clk(clk),
				.rst_n(rst_n),
				.data_req_i(data_req_CH1_int),
				.data_add_i(data_add_CH1_int),
				.data_wen_i(data_wen_CH1_int),
				.data_wdata_i(data_wdata_CH1_int),
				.data_be_i(data_be_CH1_int),
				.data_ID_i(data_ID_CH1_int),
				.data_gnt_o(data_gnt_CH1_int),
				.data_req_o(data_req_CH1),
				.data_add_o(data_add_CH1),
				.data_wen_o(data_wen_CH1),
				.data_wdata_o(data_wdata_CH1),
				.data_be_o(data_be_CH1),
				.data_ID_o(data_ID_CH1),
				.data_gnt_i(data_gnt_CH1)
			);
		end
		if (N_CH1 == 1) begin : MONO_CH1
			if (N_CH0 == 1) begin : MONO_CH0
				MUX2_REQ_L2 #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) MUX2_CH0_CH1(
					.data_req_CH0_i(data_req_CH0_int),
					.data_add_CH0_i(data_add_CH0_int),
					.data_wen_CH0_i(data_wen_CH0_int),
					.data_wdata_CH0_i(data_wdata_CH0_int),
					.data_be_CH0_i(data_be_CH0_int),
					.data_ID_CH0_i(data_ID_CH0_int),
					.data_gnt_CH0_o(data_gnt_CH0_int),
					.data_req_CH1_i(data_req_CH1_int),
					.data_add_CH1_i(data_add_CH1_int),
					.data_wen_CH1_i(data_wen_CH1_int),
					.data_wdata_CH1_i(data_wdata_CH1_int),
					.data_be_CH1_i(data_be_CH1_int),
					.data_ID_CH1_i(data_ID_CH1_int),
					.data_gnt_CH1_o(data_gnt_CH1_int),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : POLY_CH0
				MUX2_REQ_L2 #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) MUX2_CH0_CH1(
					.data_req_CH0_i(data_req_CH0),
					.data_add_CH0_i(data_add_CH0),
					.data_wen_CH0_i(data_wen_CH0),
					.data_wdata_CH0_i(data_wdata_CH0),
					.data_be_CH0_i(data_be_CH0),
					.data_ID_CH0_i(data_ID_CH0),
					.data_gnt_CH0_o(data_gnt_CH0),
					.data_req_CH1_i(data_req_CH1_int),
					.data_add_CH1_i(data_add_CH1_int),
					.data_wen_CH1_i(data_wen_CH1_int),
					.data_wdata_CH1_i(data_wdata_CH1_int),
					.data_be_CH1_i(data_be_CH1_int),
					.data_ID_CH1_i(data_ID_CH1_int),
					.data_gnt_CH1_o(data_gnt_CH1_int),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
		else begin : POLY_CH1
			if (N_CH0 == 1) begin : MONO_CH0
				MUX2_REQ_L2 #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) MUX2_CH0_CH1(
					.data_req_CH0_i(data_req_CH0_int),
					.data_add_CH0_i(data_add_CH0_int),
					.data_wen_CH0_i(data_wen_CH0_int),
					.data_wdata_CH0_i(data_wdata_CH0_int),
					.data_be_CH0_i(data_be_CH0_int),
					.data_ID_CH0_i(data_ID_CH0_int),
					.data_gnt_CH0_o(data_gnt_CH0_int),
					.data_req_CH1_i(data_req_CH1),
					.data_add_CH1_i(data_add_CH1),
					.data_wen_CH1_i(data_wen_CH1),
					.data_wdata_CH1_i(data_wdata_CH1),
					.data_be_CH1_i(data_be_CH1),
					.data_ID_CH1_i(data_ID_CH1),
					.data_gnt_CH1_o(data_gnt_CH1),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : POLY_CH0
				MUX2_REQ_L2 #(
					.ID_WIDTH(ID_WIDTH),
					.ADDR_WIDTH(ADDR_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) MUX2_CH0_CH1(
					.data_req_CH0_i(data_req_CH0),
					.data_add_CH0_i(data_add_CH0),
					.data_wen_CH0_i(data_wen_CH0),
					.data_wdata_CH0_i(data_wdata_CH0),
					.data_be_CH0_i(data_be_CH0),
					.data_ID_CH0_i(data_ID_CH0),
					.data_gnt_CH0_o(data_gnt_CH0),
					.data_req_CH1_i(data_req_CH1),
					.data_add_CH1_i(data_add_CH1),
					.data_wen_CH1_i(data_wen_CH1),
					.data_wdata_CH1_i(data_wdata_CH1),
					.data_be_CH1_i(data_be_CH1),
					.data_ID_CH1_i(data_ID_CH1),
					.data_gnt_CH1_o(data_gnt_CH1),
					.data_req_o(data_req_o),
					.data_add_o(data_add_o),
					.data_wen_o(data_wen_o),
					.data_wdata_o(data_wdata_o),
					.data_be_o(data_be_o),
					.data_ID_o(data_ID_o),
					.data_gnt_i(data_gnt_i),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
	endgenerate
	AddressDecoder_Resp_L2 #(
		.ID_WIDTH(ID_WIDTH),
		.N_MASTER(N_CH0 + N_CH1)
	) ADDR_DEC_RESP(
		.data_r_valid_i(data_r_valid_i),
		.data_r_ID_i(data_r_ID_i),
		.data_r_valid_o({data_r_valid_CH1_o, data_r_valid_CH0_o})
	);
endmodule
module ResponseBlock_L2 (
	data_r_valid_i,
	data_r_rdata_i,
	data_r_valid_o,
	data_r_rdata_o,
	data_req_i,
	routing_addr_i,
	data_gnt_o,
	data_gnt_i,
	data_req_o,
	data_ID_o
);
	parameter ID = 1;
	parameter ID_WIDTH = 20;
	parameter N_SLAVE = 2;
	parameter DATA_WIDTH = 64;
	parameter ROUT_WIDTH = $clog2(N_SLAVE);
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	output wire data_r_valid_o;
	output wire [DATA_WIDTH - 1:0] data_r_rdata_o;
	input wire data_req_i;
	input wire [ROUT_WIDTH - 1:0] routing_addr_i;
	output wire data_gnt_o;
	input wire [N_SLAVE - 1:0] data_gnt_i;
	output wire [N_SLAVE - 1:0] data_req_o;
	output wire [ID_WIDTH - 1:0] data_ID_o;
	ResponseTree_L2 #(
		.N_SLAVE(N_SLAVE),
		.DATA_WIDTH(DATA_WIDTH)
	) MEM_RESP_TREE(
		.data_r_valid_i(data_r_valid_i),
		.data_r_rdata_i(data_r_rdata_i),
		.data_r_valid_o(data_r_valid_o),
		.data_r_rdata_o(data_r_rdata_o)
	);
	AddressDecoder_Req_L2 #(
		.ID_WIDTH(ID_WIDTH),
		.ID(ID),
		.N_SLAVE(N_SLAVE)
	) ADDR_DEC_REQ(
		.data_req_i(data_req_i),
		.routing_addr_i(routing_addr_i),
		.data_gnt_o(data_gnt_o),
		.data_gnt_i(data_gnt_i),
		.data_req_o(data_req_o),
		.data_ID_o(data_ID_o)
	);
endmodule
module ResponseTree_L2 (
	data_r_valid_i,
	data_r_rdata_i,
	data_r_valid_o,
	data_r_rdata_o
);
	parameter N_SLAVE = 4;
	parameter DATA_WIDTH = 64;
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	output wire data_r_valid_o;
	output wire [DATA_WIDTH - 1:0] data_r_rdata_o;
	localparam LOG_SLAVE = $clog2(N_SLAVE);
	localparam N_WIRE = N_SLAVE - 2;
	genvar _gv_j_11;
	genvar _gv_k_5;
	generate
		case (N_SLAVE)
			1: begin : MONO_SLAVE
				assign data_r_rdata_o = data_r_rdata_i;
				assign data_r_valid_o = data_r_valid_i;
			end
			2: begin : DUAL_SLAVE
				FanInPrimitive_Resp_L2 #(.DATA_WIDTH(DATA_WIDTH)) FAN_IN_RESP(
					.data_r_rdata0_i(data_r_rdata_i[0+:DATA_WIDTH]),
					.data_r_rdata1_i(data_r_rdata_i[DATA_WIDTH+:DATA_WIDTH]),
					.data_r_valid0_i(data_r_valid_i[0]),
					.data_r_valid1_i(data_r_valid_i[1]),
					.data_r_rdata_o(data_r_rdata_o),
					.data_r_valid_o(data_r_valid_o)
				);
			end
			default: begin : MULTI_SLAVE
				wire [DATA_WIDTH - 1:0] data_r_rdata_LEVEL [N_WIRE - 1:0];
				wire data_r_valid_LEVEL [N_WIRE - 1:0];
				for (_gv_j_11 = 0; _gv_j_11 < LOG_SLAVE; _gv_j_11 = _gv_j_11 + 1) begin : STAGE
					localparam j = _gv_j_11;
					for (_gv_k_5 = 0; _gv_k_5 < (2 ** j); _gv_k_5 = _gv_k_5 + 1) begin : INCR_VERT
						localparam k = _gv_k_5;
						if (j == 0) begin : LAST_NODE
							FanInPrimitive_Resp_L2 #(.DATA_WIDTH(DATA_WIDTH)) FAN_IN_RESP(
								.data_r_rdata0_i(data_r_rdata_LEVEL[2 * k]),
								.data_r_rdata1_i(data_r_rdata_LEVEL[(2 * k) + 1]),
								.data_r_valid0_i(data_r_valid_LEVEL[2 * k]),
								.data_r_valid1_i(data_r_valid_LEVEL[(2 * k) + 1]),
								.data_r_rdata_o(data_r_rdata_o),
								.data_r_valid_o(data_r_valid_o)
							);
						end
						else if (j < (LOG_SLAVE - 1)) begin : MIDDLE_NODES
							FanInPrimitive_Resp_L2 #(.DATA_WIDTH(DATA_WIDTH)) FAN_IN_RESP(
								.data_r_rdata0_i(data_r_rdata_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_rdata1_i(data_r_rdata_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_valid0_i(data_r_valid_LEVEL[(((2 ** j) * 2) - 2) + (2 * k)]),
								.data_r_valid1_i(data_r_valid_LEVEL[((((2 ** j) * 2) - 2) + (2 * k)) + 1]),
								.data_r_rdata_o(data_r_rdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_valid_o(data_r_valid_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
						else begin : LEAF_NODES
							FanInPrimitive_Resp_L2 #(.DATA_WIDTH(DATA_WIDTH)) FAN_IN_RESP(
								.data_r_rdata0_i(data_r_rdata_i[(2 * k) * DATA_WIDTH+:DATA_WIDTH]),
								.data_r_rdata1_i(data_r_rdata_i[((2 * k) + 1) * DATA_WIDTH+:DATA_WIDTH]),
								.data_r_valid0_i(data_r_valid_i[2 * k]),
								.data_r_valid1_i(data_r_valid_i[(2 * k) + 1]),
								.data_r_rdata_o(data_r_rdata_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k]),
								.data_r_valid_o(data_r_valid_LEVEL[(((2 ** (j - 1)) * 2) - 2) + k])
							);
						end
					end
				end
			end
		endcase
	endgenerate
endmodule
module XBAR_L2 (
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_gnt_o,
	data_r_valid_o,
	data_r_rdata_o,
	data_req_o,
	data_add_o,
	data_wen_o,
	data_wdata_o,
	data_be_o,
	data_ID_o,
	data_r_rdata_i,
	data_r_valid_i,
	data_r_ID_i,
	clk,
	rst_n
);
	parameter N_CH0 = 5;
	parameter N_CH1 = 4;
	parameter ADDR_MEM_WIDTH = 12;
	parameter N_SLAVE = 4;
	parameter DATA_WIDTH = 64;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter ID_WIDTH = N_CH0 + N_CH1;
	parameter N_MASTER = N_CH0 + N_CH1;
	parameter ADDR_IN_WIDTH = ADDR_MEM_WIDTH + $clog2(N_SLAVE);
	input wire [N_MASTER - 1:0] data_req_i;
	input wire [(N_MASTER * ADDR_IN_WIDTH) - 1:0] data_add_i;
	input wire [N_MASTER - 1:0] data_wen_i;
	input wire [(N_MASTER * DATA_WIDTH) - 1:0] data_wdata_i;
	input wire [(N_MASTER * BE_WIDTH) - 1:0] data_be_i;
	output wire [N_MASTER - 1:0] data_gnt_o;
	output wire [N_MASTER - 1:0] data_r_valid_o;
	output wire [(N_MASTER * DATA_WIDTH) - 1:0] data_r_rdata_o;
	output wire [N_SLAVE - 1:0] data_req_o;
	output wire [(N_SLAVE * ADDR_MEM_WIDTH) - 1:0] data_add_o;
	output wire [N_SLAVE - 1:0] data_wen_o;
	output wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_wdata_o;
	output wire [(N_SLAVE * BE_WIDTH) - 1:0] data_be_o;
	output wire [(N_SLAVE * ID_WIDTH) - 1:0] data_ID_o;
	input wire [(N_SLAVE * DATA_WIDTH) - 1:0] data_r_rdata_i;
	input wire [N_SLAVE - 1:0] data_r_valid_i;
	input wire [(N_SLAVE * ID_WIDTH) - 1:0] data_r_ID_i;
	input wire clk;
	input wire rst_n;
	wire [(N_MASTER * ID_WIDTH) - 1:0] data_ID;
	wire [(N_MASTER * ADDR_MEM_WIDTH) - 1:0] data_add;
	wire [(N_MASTER * $clog2(N_SLAVE)) - 1:0] data_routing;
	wire [N_MASTER - 1:0] data_r_valid_from_MEM [N_SLAVE - 1:0];
	wire [N_SLAVE - 1:0] data_r_valid_to_MASTER [N_MASTER - 1:0];
	wire [N_SLAVE - 1:0] data_req_from_MASTER [N_MASTER - 1:0];
	wire [N_MASTER - 1:0] data_req_to_MEM [N_SLAVE - 1:0];
	wire [N_SLAVE - 1:0] data_gnt_to_MASTER [N_MASTER - 1:0];
	wire [N_MASTER - 1:0] data_gnt_from_MEM [N_SLAVE - 1:0];
	genvar _gv_i_1;
	genvar _gv_j_12;
	genvar _gv_k_6;
	generate
		for (_gv_k_6 = 0; _gv_k_6 < N_MASTER; _gv_k_6 = _gv_k_6 + 1) begin : wiring_req_rout
			localparam k = _gv_k_6;
			if (N_SLAVE > 1) begin : genblk1
				assign data_add[k * ADDR_MEM_WIDTH+:ADDR_MEM_WIDTH] = {data_add_i[(k * ADDR_IN_WIDTH) + (((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) >= $clog2(N_SLAVE) ? (ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1 : (((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) + (((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) >= $clog2(N_SLAVE) ? (((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) - $clog2(N_SLAVE)) + 1 : ($clog2(N_SLAVE) - ((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1)) + 1)) - 1)-:(((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) >= $clog2(N_SLAVE) ? (((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1) - $clog2(N_SLAVE)) + 1 : ($clog2(N_SLAVE) - ((ADDR_MEM_WIDTH + $clog2(N_SLAVE)) - 1)) + 1)]};
			end
			else begin : genblk1
				assign data_add[k * ADDR_MEM_WIDTH+:ADDR_MEM_WIDTH] = data_add_i[k * ADDR_IN_WIDTH+:ADDR_IN_WIDTH];
			end
			if (N_SLAVE > 1) begin : genblk2
				assign data_routing[k * $clog2(N_SLAVE)+:$clog2(N_SLAVE)] = data_add_i[(k * ADDR_IN_WIDTH) + ($clog2(N_SLAVE) - 1)-:$clog2(N_SLAVE)];
			end
			else begin : genblk2
				assign data_routing[k * $clog2(N_SLAVE)+:$clog2(N_SLAVE)] = 1'b0;
			end
			for (_gv_j_12 = 0; _gv_j_12 < N_SLAVE; _gv_j_12 = _gv_j_12 + 1) begin : Wiring_flow_ctrl
				localparam j = _gv_j_12;
				assign data_r_valid_to_MASTER[k][j] = data_r_valid_from_MEM[j][k];
				assign data_req_to_MEM[j][k] = data_req_from_MASTER[k][j];
				assign data_gnt_to_MASTER[k][j] = data_gnt_from_MEM[j][k];
			end
		end
		for (_gv_j_12 = 0; _gv_j_12 < N_SLAVE; _gv_j_12 = _gv_j_12 + 1) begin : genblk2
			localparam j = _gv_j_12;
			if (N_CH1 == 0) begin : CH0_ONLY
				RequestBlock_L2_1CH #(
					.ADDR_WIDTH(ADDR_MEM_WIDTH),
					.N_CH0(N_CH0),
					.ID_WIDTH(ID_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) REQ_BLOCK_CLUSTERS(
					.data_req_i(data_req_to_MEM[j]),
					.data_add_i(data_add),
					.data_wen_i(data_wen_i),
					.data_wdata_i(data_wdata_i),
					.data_be_i(data_be_i),
					.data_ID_i(data_ID),
					.data_gnt_o(data_gnt_from_MEM[j]),
					.data_req_o(data_req_o[j]),
					.data_add_o(data_add_o[j * ADDR_MEM_WIDTH+:ADDR_MEM_WIDTH]),
					.data_wen_o(data_wen_o[j]),
					.data_wdata_o(data_wdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_be_o(data_be_o[j * BE_WIDTH+:BE_WIDTH]),
					.data_ID_o(data_ID_o[j * ID_WIDTH+:ID_WIDTH]),
					.data_gnt_i(1'b1),
					.data_r_valid_i(data_r_valid_i[j]),
					.data_r_ID_i(data_r_ID_i[j * ID_WIDTH+:ID_WIDTH]),
					.data_r_valid_o(data_r_valid_from_MEM[j]),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
			else begin : CH0_CH1
				RequestBlock_L2_2CH #(
					.ADDR_WIDTH(ADDR_MEM_WIDTH),
					.N_CH0(N_CH0),
					.N_CH1(N_CH1),
					.ID_WIDTH(ID_WIDTH),
					.DATA_WIDTH(DATA_WIDTH),
					.BE_WIDTH(BE_WIDTH)
				) REQ_BLOCK_CLUSTERS_FC(
					.data_req_CH0_i(data_req_to_MEM[j][N_CH0 - 1:0]),
					.data_add_CH0_i(data_add[ADDR_MEM_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:ADDR_MEM_WIDTH * N_CH0]),
					.data_wen_CH0_i(data_wen_i[N_CH0 - 1:0]),
					.data_wdata_CH0_i(data_wdata_i[DATA_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:DATA_WIDTH * N_CH0]),
					.data_be_CH0_i(data_be_i[BE_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:BE_WIDTH * N_CH0]),
					.data_ID_CH0_i(data_ID[ID_WIDTH * ((N_CH0 - 1) - (N_CH0 - 1))+:ID_WIDTH * N_CH0]),
					.data_gnt_CH0_o(data_gnt_from_MEM[j][N_CH0 - 1:0]),
					.data_req_CH1_i(data_req_to_MEM[j][(N_CH0 + N_CH1) - 1:N_CH0]),
					.data_add_CH1_i(data_add[ADDR_MEM_WIDTH * ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (N_CH0 + N_CH1) - 1 : (((N_CH0 + N_CH1) - 1) + (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)) - 1) - ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1) - 1))+:ADDR_MEM_WIDTH * (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)]),
					.data_wen_CH1_i(data_wen_i[(N_CH0 + N_CH1) - 1:N_CH0]),
					.data_wdata_CH1_i(data_wdata_i[DATA_WIDTH * ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (N_CH0 + N_CH1) - 1 : (((N_CH0 + N_CH1) - 1) + (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)) - 1) - ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1) - 1))+:DATA_WIDTH * (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)]),
					.data_be_CH1_i(data_be_i[BE_WIDTH * ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (N_CH0 + N_CH1) - 1 : (((N_CH0 + N_CH1) - 1) + (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)) - 1) - ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1) - 1))+:BE_WIDTH * (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)]),
					.data_ID_CH1_i(data_ID[ID_WIDTH * ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (N_CH0 + N_CH1) - 1 : (((N_CH0 + N_CH1) - 1) + (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)) - 1) - ((((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1) - 1))+:ID_WIDTH * (((N_CH0 + N_CH1) - 1) >= N_CH0 ? (((N_CH0 + N_CH1) - 1) - N_CH0) + 1 : (N_CH0 - ((N_CH0 + N_CH1) - 1)) + 1)]),
					.data_gnt_CH1_o(data_gnt_from_MEM[j][(N_CH0 + N_CH1) - 1:N_CH0]),
					.data_req_o(data_req_o[j]),
					.data_add_o(data_add_o[j * ADDR_MEM_WIDTH+:ADDR_MEM_WIDTH]),
					.data_wen_o(data_wen_o[j]),
					.data_wdata_o(data_wdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_be_o(data_be_o[j * BE_WIDTH+:BE_WIDTH]),
					.data_ID_o(data_ID_o[j * ID_WIDTH+:ID_WIDTH]),
					.data_gnt_i(1'b1),
					.data_r_valid_i(data_r_valid_i[j]),
					.data_r_ID_i(data_r_ID_i[j * ID_WIDTH+:ID_WIDTH]),
					.data_r_valid_CH0_o(data_r_valid_from_MEM[j][N_CH0 - 1:0]),
					.data_r_valid_CH1_o(data_r_valid_from_MEM[j][(N_CH1 + N_CH0) - 1:N_CH0]),
					.clk(clk),
					.rst_n(rst_n)
				);
			end
		end
		if (N_SLAVE == 1) begin : genblk3
			for (_gv_j_12 = 0; _gv_j_12 < N_MASTER; _gv_j_12 = _gv_j_12 + 1) begin : WIRING
				localparam j = _gv_j_12;
				assign data_r_rdata_o[j * DATA_WIDTH+:DATA_WIDTH] = data_r_rdata_i;
				assign data_r_valid_o[j] = data_r_valid_to_MASTER[j];
				assign data_ID[j * ID_WIDTH+:ID_WIDTH] = 2 ** j;
				assign data_req_from_MASTER[j] = data_req_i[j];
				assign data_gnt_o[j] = data_gnt_to_MASTER[j];
			end
		end
		else begin : genblk3
			for (_gv_j_12 = 0; _gv_j_12 < N_MASTER; _gv_j_12 = _gv_j_12 + 1) begin : ResponseBlock
				localparam j = _gv_j_12;
				ResponseBlock_L2 #(
					.ID(2 ** j),
					.ID_WIDTH(ID_WIDTH),
					.N_SLAVE(N_SLAVE),
					.DATA_WIDTH(DATA_WIDTH)
				) RESP_BLOCK(
					.data_r_valid_i(data_r_valid_to_MASTER[j]),
					.data_r_rdata_i(data_r_rdata_i),
					.data_r_valid_o(data_r_valid_o[j]),
					.data_r_rdata_o(data_r_rdata_o[j * DATA_WIDTH+:DATA_WIDTH]),
					.data_req_i(data_req_i[j]),
					.routing_addr_i(data_routing[j * $clog2(N_SLAVE)+:$clog2(N_SLAVE)]),
					.data_gnt_o(data_gnt_o[j]),
					.data_req_o(data_req_from_MASTER[j]),
					.data_gnt_i(data_gnt_to_MASTER[j]),
					.data_ID_o(data_ID[j * ID_WIDTH+:ID_WIDTH])
				);
			end
		end
	endgenerate
endmodule
module axi64_2_lint32 (
	clk,
	rst_n,
	test_en_i,
	AW_ADDR_i,
	AW_PROT_i,
	AW_REGION_i,
	AW_LEN_i,
	AW_SIZE_i,
	AW_BURST_i,
	AW_LOCK_i,
	AW_CACHE_i,
	AW_QOS_i,
	AW_ID_i,
	AW_USER_i,
	AW_VALID_i,
	AW_READY_o,
	AR_ADDR_i,
	AR_PROT_i,
	AR_REGION_i,
	AR_LEN_i,
	AR_SIZE_i,
	AR_BURST_i,
	AR_LOCK_i,
	AR_CACHE_i,
	AR_QOS_i,
	AR_ID_i,
	AR_USER_i,
	AR_VALID_i,
	AR_READY_o,
	W_USER_i,
	W_DATA_i,
	W_STRB_i,
	W_LAST_i,
	W_VALID_i,
	W_READY_o,
	B_ID_o,
	B_RESP_o,
	B_USER_o,
	B_VALID_o,
	B_READY_i,
	R_ID_o,
	R_USER_o,
	R_DATA_o,
	R_RESP_o,
	R_LAST_o,
	R_VALID_o,
	R_READY_i,
	data_W_req_o,
	data_W_gnt_i,
	data_W_wdata_o,
	data_W_add_o,
	data_W_wen_o,
	data_W_be_o,
	data_W_aux_o,
	data_W_r_valid_i,
	data_W_r_rdata_i,
	data_W_r_aux_i,
	data_W_r_opc_i,
	data_R_req_o,
	data_R_gnt_i,
	data_R_wdata_o,
	data_R_add_o,
	data_R_wen_o,
	data_R_be_o,
	data_R_aux_o,
	data_R_r_valid_i,
	data_R_r_rdata_i,
	data_R_r_aux_i,
	data_R_r_opc_i
);
	parameter AXI_ADDR_WIDTH = 32;
	parameter AXI_DATA_WIDTH = 64;
	parameter AXI_STRB_WIDTH = 8;
	parameter AXI_USER_WIDTH = 6;
	parameter AXI_ID_WIDTH = 7;
	parameter BUFF_DEPTH_SLICES = 4;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter ADDR_WIDTH = 32;
	parameter AUX_WIDTH = 4;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire [AXI_ADDR_WIDTH - 1:0] AW_ADDR_i;
	input wire [2:0] AW_PROT_i;
	input wire [3:0] AW_REGION_i;
	input wire [7:0] AW_LEN_i;
	input wire [2:0] AW_SIZE_i;
	input wire [1:0] AW_BURST_i;
	input wire AW_LOCK_i;
	input wire [3:0] AW_CACHE_i;
	input wire [3:0] AW_QOS_i;
	input wire [AXI_ID_WIDTH - 1:0] AW_ID_i;
	input wire [AXI_USER_WIDTH - 1:0] AW_USER_i;
	input wire AW_VALID_i;
	output wire AW_READY_o;
	input wire [AXI_ADDR_WIDTH - 1:0] AR_ADDR_i;
	input wire [2:0] AR_PROT_i;
	input wire [3:0] AR_REGION_i;
	input wire [7:0] AR_LEN_i;
	input wire [2:0] AR_SIZE_i;
	input wire [1:0] AR_BURST_i;
	input wire AR_LOCK_i;
	input wire [3:0] AR_CACHE_i;
	input wire [3:0] AR_QOS_i;
	input wire [AXI_ID_WIDTH - 1:0] AR_ID_i;
	input wire [AXI_USER_WIDTH - 1:0] AR_USER_i;
	input wire AR_VALID_i;
	output wire AR_READY_o;
	input wire [AXI_USER_WIDTH - 1:0] W_USER_i;
	input wire [AXI_DATA_WIDTH - 1:0] W_DATA_i;
	input wire [AXI_STRB_WIDTH - 1:0] W_STRB_i;
	input wire W_LAST_i;
	input wire W_VALID_i;
	output wire W_READY_o;
	output wire [AXI_ID_WIDTH - 1:0] B_ID_o;
	output wire [1:0] B_RESP_o;
	output wire [AXI_USER_WIDTH - 1:0] B_USER_o;
	output wire B_VALID_o;
	input wire B_READY_i;
	output wire [AXI_ID_WIDTH - 1:0] R_ID_o;
	output wire [AXI_USER_WIDTH - 1:0] R_USER_o;
	output wire [AXI_DATA_WIDTH - 1:0] R_DATA_o;
	output wire [1:0] R_RESP_o;
	output wire R_LAST_o;
	output wire R_VALID_o;
	input wire R_READY_i;
	output wire [1:0] data_W_req_o;
	input wire [1:0] data_W_gnt_i;
	output wire [(2 * DATA_WIDTH) - 1:0] data_W_wdata_o;
	output wire [(2 * ADDR_WIDTH) - 1:0] data_W_add_o;
	output wire [1:0] data_W_wen_o;
	output wire [(2 * BE_WIDTH) - 1:0] data_W_be_o;
	output wire [(2 * AUX_WIDTH) - 1:0] data_W_aux_o;
	input wire [1:0] data_W_r_valid_i;
	input wire [(2 * DATA_WIDTH) - 1:0] data_W_r_rdata_i;
	input wire [(2 * AUX_WIDTH) - 1:0] data_W_r_aux_i;
	input wire [1:0] data_W_r_opc_i;
	output wire [1:0] data_R_req_o;
	input wire [1:0] data_R_gnt_i;
	output wire [(2 * DATA_WIDTH) - 1:0] data_R_wdata_o;
	output wire [(2 * ADDR_WIDTH) - 1:0] data_R_add_o;
	output wire [1:0] data_R_wen_o;
	output wire [(2 * BE_WIDTH) - 1:0] data_R_be_o;
	output wire [(2 * AUX_WIDTH) - 1:0] data_R_aux_o;
	input wire [1:0] data_R_r_valid_i;
	input wire [(2 * DATA_WIDTH) - 1:0] data_R_r_rdata_i;
	input wire [(2 * AUX_WIDTH) - 1:0] data_R_r_aux_i;
	input wire [1:0] data_R_r_opc_i;
	localparam ADDR_OFFSET = $clog2(DATA_WIDTH) - 3;
	wire [AXI_ADDR_WIDTH - 1:0] AW_ADDR_int;
	wire [2:0] AW_PROT_int;
	wire [3:0] AW_REGION_int;
	wire [7:0] AW_LEN_int;
	wire [2:0] AW_SIZE_int;
	wire [1:0] AW_BURST_int;
	wire AW_LOCK_int;
	wire [3:0] AW_CACHE_int;
	wire [3:0] AW_QOS_int;
	wire [AXI_ID_WIDTH - 1:0] AW_ID_int;
	wire [AXI_USER_WIDTH - 1:0] AW_USER_int;
	wire AW_VALID_int;
	wire AW_READY_int;
	wire [AXI_ADDR_WIDTH - 1:0] AR_ADDR_int;
	wire [2:0] AR_PROT_int;
	wire [3:0] AR_REGION_int;
	wire [7:0] AR_LEN_int;
	wire [2:0] AR_SIZE_int;
	wire [1:0] AR_BURST_int;
	wire AR_LOCK_int;
	wire [3:0] AR_CACHE_int;
	wire [3:0] AR_QOS_int;
	wire [AXI_ID_WIDTH - 1:0] AR_ID_int;
	wire [AXI_USER_WIDTH - 1:0] AR_USER_int;
	wire AR_VALID_int;
	wire AR_READY_int;
	wire [AXI_USER_WIDTH - 1:0] W_USER_int;
	wire [AXI_DATA_WIDTH - 1:0] W_DATA_int;
	wire [AXI_STRB_WIDTH - 1:0] W_STRB_int;
	wire W_LAST_int;
	wire W_VALID_int;
	wire W_READY_int;
	wire [AXI_ID_WIDTH - 1:0] B_ID_int;
	wire [1:0] B_RESP_int;
	wire [AXI_USER_WIDTH - 1:0] B_USER_int;
	wire B_VALID_int;
	wire B_READY_int;
	wire [AXI_ID_WIDTH - 1:0] R_ID_int;
	wire [AXI_USER_WIDTH - 1:0] R_USER_int;
	wire [AXI_DATA_WIDTH - 1:0] R_DATA_int;
	wire [1:0] R_RESP_int;
	wire R_LAST_int;
	wire R_VALID_int;
	wire R_READY_int;
	wire data_W_req_int;
	wire data_W_gnt_int;
	wire [63:0] data_W_wdata_int;
	wire [31:0] data_W_add_int;
	wire data_W_wen_int;
	wire [7:0] data_W_be_int;
	wire data_W_r_valid_int;
	wire [63:0] data_W_r_rdata_int;
	wire data_R_req_int;
	wire data_R_gnt_int;
	wire [63:0] data_R_wdata_int;
	wire [31:0] data_R_add_int;
	wire data_R_wen_int;
	wire [7:0] data_R_be_int;
	wire data_R_r_valid_int;
	wire [63:0] data_R_r_rdata_int;
	assign data_R_aux_o = 1'sb0;
	assign data_W_aux_o = 1'sb0;
	axi_aw_buffer #(
		.ID_WIDTH(AXI_ID_WIDTH),
		.ADDR_WIDTH(AXI_ADDR_WIDTH),
		.USER_WIDTH(AXI_USER_WIDTH),
		.BUFFER_DEPTH(BUFF_DEPTH_SLICES)
	) Slave_aw_buffer(
		.clk_i(clk),
		.rst_ni(rst_n),
		.test_en_i(test_en_i),
		.slave_valid_i(AW_VALID_i),
		.slave_addr_i(AW_ADDR_i),
		.slave_prot_i(AW_PROT_i),
		.slave_region_i(AW_REGION_i),
		.slave_len_i(AW_LEN_i),
		.slave_size_i(AW_SIZE_i),
		.slave_burst_i(AW_BURST_i),
		.slave_lock_i(AW_LOCK_i),
		.slave_cache_i(AW_CACHE_i),
		.slave_qos_i(AW_QOS_i),
		.slave_id_i(AW_ID_i),
		.slave_user_i(AW_USER_i),
		.slave_ready_o(AW_READY_o),
		.master_valid_o(AW_VALID_int),
		.master_addr_o(AW_ADDR_int),
		.master_prot_o(AW_PROT_int),
		.master_region_o(AW_REGION_int),
		.master_len_o(AW_LEN_int),
		.master_size_o(AW_SIZE_int),
		.master_burst_o(AW_BURST_int),
		.master_lock_o(AW_LOCK_int),
		.master_cache_o(AW_CACHE_int),
		.master_qos_o(AW_QOS_int),
		.master_id_o(AW_ID_int),
		.master_user_o(AW_USER_int),
		.master_ready_i(AW_READY_int)
	);
	axi_ar_buffer #(
		.ID_WIDTH(AXI_ID_WIDTH),
		.ADDR_WIDTH(AXI_ADDR_WIDTH),
		.USER_WIDTH(AXI_USER_WIDTH),
		.BUFFER_DEPTH(BUFF_DEPTH_SLICES)
	) Slave_ar_buffer(
		.clk_i(clk),
		.rst_ni(rst_n),
		.test_en_i(test_en_i),
		.slave_valid_i(AR_VALID_i),
		.slave_addr_i(AR_ADDR_i),
		.slave_prot_i(AR_PROT_i),
		.slave_region_i(AR_REGION_i),
		.slave_len_i(AR_LEN_i),
		.slave_size_i(AR_SIZE_i),
		.slave_burst_i(AR_BURST_i),
		.slave_lock_i(AR_LOCK_i),
		.slave_cache_i(AR_CACHE_i),
		.slave_qos_i(AR_QOS_i),
		.slave_id_i(AR_ID_i),
		.slave_user_i(AR_USER_i),
		.slave_ready_o(AR_READY_o),
		.master_valid_o(AR_VALID_int),
		.master_addr_o(AR_ADDR_int),
		.master_prot_o(AR_PROT_int),
		.master_region_o(AR_REGION_int),
		.master_len_o(AR_LEN_int),
		.master_size_o(AR_SIZE_int),
		.master_burst_o(AR_BURST_int),
		.master_lock_o(AR_LOCK_int),
		.master_cache_o(AR_CACHE_int),
		.master_qos_o(AR_QOS_int),
		.master_id_o(AR_ID_int),
		.master_user_o(AR_USER_int),
		.master_ready_i(AR_READY_int)
	);
	axi_w_buffer #(
		.DATA_WIDTH(AXI_DATA_WIDTH),
		.USER_WIDTH(AXI_USER_WIDTH),
		.BUFFER_DEPTH(BUFF_DEPTH_SLICES)
	) Slave_w_buffer(
		.clk_i(clk),
		.rst_ni(rst_n),
		.test_en_i(test_en_i),
		.slave_valid_i(W_VALID_i),
		.slave_data_i(W_DATA_i),
		.slave_strb_i(W_STRB_i),
		.slave_user_i(W_USER_i),
		.slave_last_i(W_LAST_i),
		.slave_ready_o(W_READY_o),
		.master_valid_o(W_VALID_int),
		.master_data_o(W_DATA_int),
		.master_strb_o(W_STRB_int),
		.master_user_o(W_USER_int),
		.master_last_o(W_LAST_int),
		.master_ready_i(W_READY_int)
	);
	axi_r_buffer #(
		.ID_WIDTH(AXI_ID_WIDTH),
		.DATA_WIDTH(AXI_DATA_WIDTH),
		.USER_WIDTH(AXI_USER_WIDTH),
		.BUFFER_DEPTH(BUFF_DEPTH_SLICES)
	) Slave_r_buffer(
		.clk_i(clk),
		.rst_ni(rst_n),
		.test_en_i(test_en_i),
		.slave_valid_i(R_VALID_int),
		.slave_data_i(R_DATA_int),
		.slave_resp_i(R_RESP_int),
		.slave_user_i(R_USER_int),
		.slave_id_i(R_ID_int),
		.slave_last_i(R_LAST_int),
		.slave_ready_o(R_READY_int),
		.master_valid_o(R_VALID_o),
		.master_data_o(R_DATA_o),
		.master_resp_o(R_RESP_o),
		.master_user_o(R_USER_o),
		.master_id_o(R_ID_o),
		.master_last_o(R_LAST_o),
		.master_ready_i(R_READY_i)
	);
	axi_b_buffer #(
		.ID_WIDTH(AXI_ID_WIDTH),
		.USER_WIDTH(AXI_USER_WIDTH),
		.BUFFER_DEPTH(BUFF_DEPTH_SLICES)
	) Slave_b_buffer(
		.clk_i(clk),
		.rst_ni(rst_n),
		.test_en_i(test_en_i),
		.slave_valid_i(B_VALID_int),
		.slave_resp_i(B_RESP_int),
		.slave_id_i(B_ID_int),
		.slave_user_i(B_USER_int),
		.slave_ready_o(B_READY_int),
		.master_valid_o(B_VALID_o),
		.master_resp_o(B_RESP_o),
		.master_id_o(B_ID_o),
		.master_user_o(B_USER_o),
		.master_ready_i(B_READY_i)
	);
	wire data_W_size_int;
	wire data_R_size_int;
	localparam sv2v_uu_i_axi_write_ctrl_AXI4_RDATA_WIDTH = AXI_DATA_WIDTH;
	localparam [sv2v_uu_i_axi_write_ctrl_AXI4_RDATA_WIDTH - 1:0] sv2v_uu_i_axi_write_ctrl_ext_MEM_Q_i_0 = 1'sb0;
	axi_write_ctrl #(
		.AXI4_ADDRESS_WIDTH(AXI_ADDR_WIDTH),
		.AXI4_RDATA_WIDTH(AXI_DATA_WIDTH),
		.AXI4_WDATA_WIDTH(AXI_DATA_WIDTH),
		.AXI4_ID_WIDTH(AXI_ID_WIDTH),
		.AXI4_USER_WIDTH(AXI_USER_WIDTH),
		.AXI_NUMBYTES(AXI_STRB_WIDTH),
		.MEM_ADDR_WIDTH(ADDR_WIDTH)
	) i_axi_write_ctrl(
		.clk(clk),
		.rst_n(rst_n),
		.AWID_i(AW_ID_int),
		.AWADDR_i(AW_ADDR_int),
		.AWLEN_i(AW_LEN_int),
		.AWSIZE_i(AW_SIZE_int),
		.AWBURST_i(AW_BURST_int),
		.AWLOCK_i(AW_LOCK_int),
		.AWCACHE_i(AW_CACHE_int),
		.AWPROT_i(AW_PROT_int),
		.AWREGION_i(AW_REGION_int),
		.AWUSER_i(AW_USER_int),
		.AWQOS_i(AW_QOS_int),
		.AWVALID_i(AW_VALID_int),
		.AWREADY_o(AW_READY_int),
		.WDATA_i(W_DATA_int),
		.WSTRB_i(W_STRB_int),
		.WLAST_i(W_LAST_int),
		.WUSER_i(W_USER_int),
		.WVALID_i(W_VALID_int),
		.WREADY_o(W_READY_int),
		.BID_o(B_ID_int),
		.BRESP_o(B_RESP_int),
		.BVALID_o(B_VALID_int),
		.BUSER_o(B_USER_int),
		.BREADY_i(B_READY_int),
		.MEM_CEN_o(),
		.MEM_WEN_o(data_W_wen_int),
		.MEM_A_o(data_W_add_int),
		.MEM_D_o(data_W_wdata_int),
		.MEM_BE_o(data_W_be_int),
		.MEM_Q_i(sv2v_uu_i_axi_write_ctrl_ext_MEM_Q_i_0),
		.MEM_size_o(data_W_size_int),
		.grant_i(data_W_gnt_int),
		.valid_o(data_W_req_int)
	);
	lint64_to_32 parallel_lint_write(
		.clk(clk),
		.rst_n(rst_n),
		.data_req_i(data_W_req_int),
		.data_gnt_o(data_W_gnt_int),
		.data_wdata_i(data_W_wdata_int),
		.data_add_i(data_W_add_int),
		.data_wen_i(data_W_wen_int),
		.data_be_i(data_W_be_int),
		.data_size_i(data_W_size_int),
		.data_r_valid_o(data_W_r_valid_int),
		.data_r_rdata_o(data_W_r_rdata_int),
		.data_req_o(data_W_req_o),
		.data_gnt_i(data_W_gnt_i),
		.data_wdata_o(data_W_wdata_o),
		.data_add_o(data_W_add_o),
		.data_wen_o(data_W_wen_o),
		.data_be_o(data_W_be_o),
		.data_r_valid_i(data_W_r_valid_i),
		.data_r_rdata_i(data_W_r_rdata_i)
	);
	axi_read_ctrl #(
		.AXI4_ADDRESS_WIDTH(AXI_ADDR_WIDTH),
		.AXI4_RDATA_WIDTH(AXI_DATA_WIDTH),
		.AXI4_WDATA_WIDTH(AXI_DATA_WIDTH),
		.AXI4_ID_WIDTH(AXI_ID_WIDTH),
		.AXI4_USER_WIDTH(AXI_USER_WIDTH),
		.AXI_NUMBYTES(AXI_STRB_WIDTH),
		.MEM_ADDR_WIDTH(ADDR_WIDTH)
	) i_axi_read_ctrl(
		.clk(clk),
		.rst_n(rst_n),
		.ARID_i(AR_ID_int),
		.ARADDR_i(AR_ADDR_int),
		.ARLEN_i(AR_LEN_int),
		.ARSIZE_i(AR_SIZE_int),
		.ARBURST_i(AR_BURST_int),
		.ARLOCK_i(AR_LOCK_int),
		.ARCACHE_i(AR_CACHE_int),
		.ARPROT_i(AR_PROT_int),
		.ARREGION_i(AR_REGION_int),
		.ARUSER_i(AR_USER_int),
		.ARQOS_i(AR_QOS_int),
		.ARVALID_i(AR_VALID_int),
		.ARREADY_o(AR_READY_int),
		.RID_o(R_ID_int),
		.RDATA_o(R_DATA_int),
		.RRESP_o(R_RESP_int),
		.RLAST_o(R_LAST_int),
		.RUSER_o(R_USER_int),
		.RVALID_o(R_VALID_int),
		.RREADY_i(R_READY_int),
		.MEM_CEN_o(),
		.MEM_WEN_o(data_R_wen_int),
		.MEM_A_o(data_R_add_int),
		.MEM_D_o(data_R_wdata_int),
		.MEM_BE_o(data_R_be_int),
		.MEM_Q_i(data_R_r_rdata_int),
		.grant_i(data_R_gnt_int),
		.valid_o(data_R_req_int),
		.r_valid_i(data_R_r_valid_int),
		.MEM_size_o(data_R_size_int)
	);
	lint64_to_32 parallel_lint_read(
		.clk(clk),
		.rst_n(rst_n),
		.data_req_i(data_R_req_int),
		.data_gnt_o(data_R_gnt_int),
		.data_wdata_i(data_R_wdata_int),
		.data_add_i(data_R_add_int),
		.data_wen_i(data_R_wen_int),
		.data_be_i(data_R_be_int),
		.data_r_valid_o(data_R_r_valid_int),
		.data_r_rdata_o(data_R_r_rdata_int),
		.data_size_i(data_R_size_int),
		.data_req_o(data_R_req_o),
		.data_gnt_i(data_R_gnt_i),
		.data_wdata_o(data_R_wdata_o),
		.data_add_o(data_R_add_o),
		.data_wen_o(data_R_wen_o),
		.data_be_o(data_R_be_o),
		.data_r_valid_i(data_R_r_valid_i),
		.data_r_rdata_i(data_R_r_rdata_i)
	);
endmodule
module axi_read_ctrl (
	clk,
	rst_n,
	ARID_i,
	ARADDR_i,
	ARLEN_i,
	ARSIZE_i,
	ARBURST_i,
	ARLOCK_i,
	ARCACHE_i,
	ARPROT_i,
	ARREGION_i,
	ARUSER_i,
	ARQOS_i,
	ARVALID_i,
	ARREADY_o,
	RID_o,
	RDATA_o,
	RRESP_o,
	RLAST_o,
	RUSER_o,
	RVALID_o,
	RREADY_i,
	MEM_CEN_o,
	MEM_WEN_o,
	MEM_A_o,
	MEM_D_o,
	MEM_BE_o,
	MEM_Q_i,
	grant_i,
	r_valid_i,
	valid_o,
	MEM_size_o
);
	reg _sv2v_0;
	parameter AXI4_ADDRESS_WIDTH = 32;
	parameter AXI4_RDATA_WIDTH = 64;
	parameter AXI4_WDATA_WIDTH = 64;
	parameter AXI4_ID_WIDTH = 16;
	parameter AXI4_USER_WIDTH = 10;
	parameter AXI_NUMBYTES = AXI4_WDATA_WIDTH / 8;
	parameter MEM_ADDR_WIDTH = 32;
	input wire clk;
	input wire rst_n;
	input wire [AXI4_ID_WIDTH - 1:0] ARID_i;
	input wire [AXI4_ADDRESS_WIDTH - 1:0] ARADDR_i;
	input wire [7:0] ARLEN_i;
	input wire [2:0] ARSIZE_i;
	input wire [1:0] ARBURST_i;
	input wire ARLOCK_i;
	input wire [3:0] ARCACHE_i;
	input wire [2:0] ARPROT_i;
	input wire [3:0] ARREGION_i;
	input wire [AXI4_USER_WIDTH - 1:0] ARUSER_i;
	input wire [3:0] ARQOS_i;
	input wire ARVALID_i;
	output reg ARREADY_o;
	output reg [AXI4_ID_WIDTH - 1:0] RID_o;
	output reg [AXI4_RDATA_WIDTH - 1:0] RDATA_o;
	output reg [1:0] RRESP_o;
	output reg RLAST_o;
	output reg [AXI4_USER_WIDTH - 1:0] RUSER_o;
	output reg RVALID_o;
	input wire RREADY_i;
	output reg MEM_CEN_o;
	output reg MEM_WEN_o;
	output reg [MEM_ADDR_WIDTH - 1:0] MEM_A_o;
	output wire [AXI4_RDATA_WIDTH - 1:0] MEM_D_o;
	output wire [AXI_NUMBYTES - 1:0] MEM_BE_o;
	input wire [AXI4_RDATA_WIDTH - 1:0] MEM_Q_i;
	input wire grant_i;
	input wire r_valid_i;
	output reg valid_o;
	output reg MEM_size_o;
	localparam OFFSET_BIT = $clog2(AXI4_RDATA_WIDTH) - 3;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [8:0] CountBurst_CS;
	reg [8:0] CountBurst_NS;
	reg sample_rdata;
	reg sample_ctrl;
	reg [AXI4_RDATA_WIDTH - 1:0] RDATA_REG;
	reg [AXI4_USER_WIDTH - 1:0] RUSER_REG;
	reg [AXI4_ID_WIDTH - 1:0] RID_REG;
	reg [MEM_ADDR_WIDTH - 1:0] ARADDR_REG;
	reg [7:0] ARLEN_REG;
	reg [2:0] ARSIZE_REG;
	assign MEM_D_o = 1'sb0;
	assign MEM_BE_o = 1'sb0;
	always @(posedge clk or negedge rst_n) begin : _UPDATE_CS_
		if (~rst_n) begin
			CS <= 3'd0;
			CountBurst_CS <= 1'sb0;
			RDATA_REG <= 1'sb0;
			RID_REG <= 1'sb0;
			RUSER_REG <= 1'sb0;
			ARADDR_REG <= 1'sb0;
			ARLEN_REG <= 1'sb0;
			ARSIZE_REG <= 1'sb0;
		end
		else begin
			CS <= NS;
			CountBurst_CS <= CountBurst_NS;
			if (sample_ctrl) begin
				RUSER_REG <= ARUSER_i;
				RID_REG <= ARID_i;
				ARADDR_REG <= ARADDR_i[MEM_ADDR_WIDTH - 1:0];
				ARLEN_REG <= ARLEN_i;
				ARSIZE_REG <= ARSIZE_i;
			end
			if (sample_rdata)
				RDATA_REG <= MEM_Q_i;
		end
	end
	always @(*) begin : COMPUTE_NS
		if (_sv2v_0)
			;
		ARREADY_o = 1'b0;
		RDATA_o = MEM_Q_i;
		RUSER_o = RUSER_REG;
		RID_o = RID_REG;
		RVALID_o = 1'b0;
		RLAST_o = 1'b0;
		RRESP_o = 2'b00;
		MEM_CEN_o = 1'b1;
		MEM_WEN_o = 1'b1;
		MEM_A_o = ARADDR_i;
		MEM_size_o = ARSIZE_i == 3'b011;
		valid_o = 1'b0;
		sample_rdata = 1'b0;
		sample_ctrl = 1'b0;
		CountBurst_NS = CountBurst_CS;
		case (CS)
			3'd0: begin
				valid_o = ARVALID_i;
				MEM_CEN_o = ~ARVALID_i;
				ARREADY_o = grant_i;
				if (ARVALID_i) begin
					sample_ctrl = 1'b1;
					if (grant_i) begin
						ARREADY_o = 1'b1;
						if (ARLEN_i == 0) begin
							NS = 3'd1;
							CountBurst_NS = 1'sb0;
						end
						else begin
							NS = 3'd2;
							CountBurst_NS = 1'b1;
						end
					end
					else
						NS = 3'd0;
				end
				else
					NS = 3'd0;
			end
			3'd1: begin
				sample_rdata = r_valid_i;
				RDATA_o = MEM_Q_i;
				RVALID_o = r_valid_i;
				RLAST_o = 1'b1;
				RRESP_o = 2'b00;
				if (r_valid_i) begin
					if (RREADY_i) begin
						ARREADY_o = grant_i;
						valid_o = ARVALID_i;
						MEM_CEN_o = ~ARVALID_i;
						if (ARVALID_i) begin
							sample_ctrl = 1'b1;
							if (grant_i) begin
								ARREADY_o = 1'b1;
								if (ARLEN_i == 0) begin
									NS = 3'd1;
									CountBurst_NS = 1'sb0;
								end
								else begin
									NS = 3'd2;
									CountBurst_NS = 1'b1;
								end
							end
							else
								NS = 3'd0;
						end
						else
							NS = 3'd0;
					end
					else
						NS = 3'd3;
				end
				else
					NS = 3'd1;
			end
			3'd3: begin
				RDATA_o = RDATA_REG;
				RVALID_o = 1'b1;
				RLAST_o = 1'b1;
				RRESP_o = 2'b00;
				if (RREADY_i) begin
					ARREADY_o = grant_i;
					valid_o = ARVALID_i;
					MEM_CEN_o = ~ARVALID_i;
					if (ARVALID_i) begin
						sample_ctrl = 1'b1;
						if (grant_i) begin
							ARREADY_o = 1'b1;
							if (ARLEN_i == 0) begin
								NS = 3'd1;
								CountBurst_NS = 1'sb0;
							end
							else begin
								NS = 3'd2;
								CountBurst_NS = 1'b1;
							end
						end
						else
							NS = 3'd0;
					end
					else
						NS = 3'd0;
				end
				else
					NS = 3'd3;
			end
			3'd2: begin
				sample_rdata = r_valid_i;
				RDATA_o = MEM_Q_i;
				RVALID_o = r_valid_i;
				RLAST_o = 1'b0;
				RRESP_o = 2'b00;
				MEM_A_o = ARADDR_REG + (CountBurst_CS << 3);
				MEM_size_o = ARSIZE_REG == 3'b011;
				if (r_valid_i) begin
					if (RREADY_i) begin
						sample_ctrl = 1'b0;
						MEM_CEN_o = 1'b0;
						valid_o = 1'b1;
						if (grant_i) begin
							NS = 3'd2;
							if (CountBurst_CS[7:0] == ARLEN_REG) begin
								CountBurst_NS = 1'sb0;
								NS = 3'd4;
							end
							else begin
								NS = 3'd2;
								CountBurst_NS = CountBurst_CS + 1'b1;
							end
						end
						else
							NS = 3'd5;
					end
					else
						NS = 3'd6;
				end
				else
					NS = 3'd2;
			end
			3'd5: begin
				MEM_CEN_o = 1'b0;
				MEM_A_o = ARADDR_REG + (CountBurst_CS << 3);
				MEM_size_o = ARSIZE_REG == 3'b011;
				valid_o = 1'b1;
				if (grant_i) begin
					if (CountBurst_CS[7:0] == ARLEN_REG) begin
						NS = 3'd4;
						CountBurst_NS = 1'sb0;
					end
					else begin
						NS = 3'd2;
						CountBurst_NS = CountBurst_CS + 1;
					end
				end
				else
					NS = 3'd5;
			end
			3'd6: begin
				RDATA_o = RDATA_REG;
				RVALID_o = 1'b1;
				RLAST_o = 1'b0;
				RRESP_o = 2'b00;
				MEM_A_o = ARADDR_REG + (CountBurst_CS << 3);
				MEM_size_o = ARSIZE_REG == 3'b011;
				if (RREADY_i) begin
					valid_o = 1'b1;
					MEM_CEN_o = 1'b0;
					if (grant_i) begin
						CountBurst_NS = CountBurst_CS + 1;
						if (CountBurst_CS[7:0] == ARLEN_REG)
							NS = 3'd4;
						else
							NS = 3'd2;
					end
					else
						NS = 3'd5;
				end
				else
					NS = 3'd6;
			end
			3'd4: begin
				RVALID_o = r_valid_i;
				RLAST_o = 1'b1;
				RDATA_o = MEM_Q_i;
				sample_rdata = r_valid_i;
				if (r_valid_i) begin
					if (RREADY_i) begin
						valid_o = ARVALID_i;
						MEM_CEN_o = ~ARVALID_i;
						ARREADY_o = grant_i;
						if (ARVALID_i) begin
							sample_ctrl = 1'b1;
							if (grant_i) begin
								ARREADY_o = 1'b1;
								if (ARLEN_i == 0) begin
									NS = 3'd1;
									CountBurst_NS = 1'sb0;
								end
								else begin
									NS = 3'd2;
									CountBurst_NS = 1'b1;
								end
							end
							else
								NS = 3'd0;
						end
						else
							NS = 3'd0;
					end
					else
						NS = 3'd7;
				end
				else
					NS = 3'd4;
			end
			3'd7: begin
				RVALID_o = 1'b1;
				RLAST_o = 1'b1;
				RDATA_o = RDATA_REG;
				sample_rdata = 1'b0;
				if (RREADY_i) begin
					valid_o = ARVALID_i;
					MEM_CEN_o = ~ARVALID_i;
					ARREADY_o = grant_i;
					if (ARVALID_i) begin
						sample_ctrl = 1'b1;
						if (grant_i) begin
							ARREADY_o = 1'b1;
							if (ARLEN_i == 0) begin
								NS = 3'd1;
								CountBurst_NS = 1'sb0;
							end
							else begin
								NS = 3'd2;
								CountBurst_NS = 1'b1;
							end
						end
						else
							NS = 3'd0;
					end
					else
						NS = 3'd0;
				end
				else
					NS = 3'd7;
			end
			default: NS = 3'd0;
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module axi_write_ctrl (
	clk,
	rst_n,
	AWID_i,
	AWADDR_i,
	AWLEN_i,
	AWSIZE_i,
	AWBURST_i,
	AWLOCK_i,
	AWCACHE_i,
	AWPROT_i,
	AWREGION_i,
	AWUSER_i,
	AWQOS_i,
	AWVALID_i,
	AWREADY_o,
	WDATA_i,
	WSTRB_i,
	WLAST_i,
	WUSER_i,
	WVALID_i,
	WREADY_o,
	BID_o,
	BRESP_o,
	BVALID_o,
	BUSER_o,
	BREADY_i,
	MEM_CEN_o,
	MEM_WEN_o,
	MEM_A_o,
	MEM_D_o,
	MEM_BE_o,
	MEM_Q_i,
	MEM_size_o,
	grant_i,
	valid_o
);
	reg _sv2v_0;
	parameter AXI4_ADDRESS_WIDTH = 32;
	parameter AXI4_RDATA_WIDTH = 64;
	parameter AXI4_WDATA_WIDTH = 64;
	parameter AXI4_ID_WIDTH = 16;
	parameter AXI4_USER_WIDTH = 10;
	parameter AXI_NUMBYTES = AXI4_WDATA_WIDTH / 8;
	parameter MEM_ADDR_WIDTH = 13;
	input wire clk;
	input wire rst_n;
	input wire [AXI4_ID_WIDTH - 1:0] AWID_i;
	input wire [AXI4_ADDRESS_WIDTH - 1:0] AWADDR_i;
	input wire [7:0] AWLEN_i;
	input wire [2:0] AWSIZE_i;
	input wire [1:0] AWBURST_i;
	input wire AWLOCK_i;
	input wire [3:0] AWCACHE_i;
	input wire [2:0] AWPROT_i;
	input wire [3:0] AWREGION_i;
	input wire [AXI4_USER_WIDTH - 1:0] AWUSER_i;
	input wire [3:0] AWQOS_i;
	input wire AWVALID_i;
	output reg AWREADY_o;
	input wire [AXI4_WDATA_WIDTH - 1:0] WDATA_i;
	input wire [AXI_NUMBYTES - 1:0] WSTRB_i;
	input wire WLAST_i;
	input wire [AXI4_USER_WIDTH - 1:0] WUSER_i;
	input wire WVALID_i;
	output reg WREADY_o;
	output wire [AXI4_ID_WIDTH - 1:0] BID_o;
	output reg [1:0] BRESP_o;
	output reg BVALID_o;
	output wire [AXI4_USER_WIDTH - 1:0] BUSER_o;
	input wire BREADY_i;
	output reg MEM_CEN_o;
	output reg MEM_WEN_o;
	output reg [MEM_ADDR_WIDTH - 1:0] MEM_A_o;
	output reg [AXI4_RDATA_WIDTH - 1:0] MEM_D_o;
	output reg [AXI_NUMBYTES - 1:0] MEM_BE_o;
	input wire [AXI4_RDATA_WIDTH - 1:0] MEM_Q_i;
	output wire MEM_size_o;
	input wire grant_i;
	output reg valid_o;
	localparam OFFSET_BIT = $clog2(AXI4_WDATA_WIDTH) - 3;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [8:0] CountBurst_CS;
	reg [8:0] CountBurst_NS;
	reg sample_ctrl;
	reg [AXI4_USER_WIDTH - 1:0] AWUSER_REG;
	reg [AXI4_ID_WIDTH - 1:0] AWID_REG;
	reg [MEM_ADDR_WIDTH - 1:0] AWADDR_REG;
	reg [7:0] AWLEN_REG;
	reg [2:0] AWSIZE_REG;
	reg MEM_size;
	assign MEM_size_o = MEM_size;
	always @(posedge clk or negedge rst_n) begin : _UPDATE_CS_
		if (~rst_n) begin
			CS <= 3'd0;
			CountBurst_CS <= 1'sb0;
			AWID_REG <= 1'sb0;
			AWUSER_REG <= 1'sb0;
			AWADDR_REG <= 1'sb0;
			AWLEN_REG <= 1'sb0;
			AWSIZE_REG <= 1'sb0;
		end
		else begin
			CS <= NS;
			CountBurst_CS <= CountBurst_NS;
			if (sample_ctrl) begin
				AWUSER_REG <= AWUSER_i;
				AWID_REG <= AWID_i;
				AWADDR_REG <= AWADDR_i[MEM_ADDR_WIDTH - 1:0];
				AWLEN_REG <= AWLEN_i;
				AWSIZE_REG <= AWSIZE_i;
			end
		end
	end
	assign BUSER_o = AWUSER_REG;
	assign BID_o = AWID_REG;
	always @(*) begin : COMPUTE_NS
		if (_sv2v_0)
			;
		sample_ctrl = 1'b0;
		valid_o = 1'b0;
		AWREADY_o = 1'b0;
		WREADY_o = 1'b0;
		BRESP_o = 2'b00;
		BVALID_o = 1'b0;
		MEM_CEN_o = 1'b1;
		MEM_WEN_o = 1'b0;
		MEM_A_o = AWADDR_i[MEM_ADDR_WIDTH - 1:0];
		MEM_D_o = WDATA_i;
		MEM_BE_o = WSTRB_i;
		NS = CS;
		MEM_size = AWSIZE_i == 3'b011;
		CountBurst_NS = CountBurst_CS;
		case (CS)
			3'd0: begin
				AWREADY_o = 1'b1;
				sample_ctrl = AWVALID_i;
				MEM_A_o = AWADDR_i[MEM_ADDR_WIDTH - 1:0];
				if (AWVALID_i) begin
					valid_o = WVALID_i;
					MEM_CEN_o = ~WVALID_i;
					WREADY_o = grant_i;
					if (WVALID_i & grant_i) begin
						if (AWLEN_i == 0) begin
							NS = 3'd1;
							CountBurst_NS = 1'sb0;
						end
						else begin
							NS = 3'd2;
							CountBurst_NS = 1;
						end
					end
					else begin
						NS = 3'd3;
						CountBurst_NS = 1'sb0;
					end
				end
				else
					NS = 3'd0;
			end
			3'd3: begin
				WREADY_o = grant_i;
				valid_o = WVALID_i;
				MEM_CEN_o = ~(WVALID_i & grant_i);
				MEM_A_o = AWADDR_REG + (CountBurst_CS << 3);
				MEM_size = AWSIZE_REG == 3'b011;
				if (grant_i & WVALID_i) begin
					if (AWLEN_REG == CountBurst_CS[7:0]) begin
						NS = 3'd1;
						CountBurst_NS = 1'sb0;
					end
					else begin
						NS = 3'd2;
						CountBurst_NS = CountBurst_CS + 1;
					end
				end
				else
					NS = 3'd3;
			end
			3'd1: begin
				BRESP_o = 2'b00;
				BVALID_o = 1'b1;
				MEM_A_o = AWADDR_i[MEM_ADDR_WIDTH - 1:0];
				MEM_size = AWSIZE_i == 3'b011;
				if (BREADY_i) begin
					AWREADY_o = 1'b1;
					sample_ctrl = AWVALID_i;
					if (AWVALID_i) begin
						valid_o = WVALID_i;
						MEM_CEN_o = ~WVALID_i;
						WREADY_o = grant_i;
						if (WVALID_i & grant_i) begin
							if (AWLEN_i == 0) begin
								CountBurst_NS = 1'sb0;
								if (WLAST_i == 1'b1)
									NS = 3'd1;
								else
									NS = 3'd4;
							end
							else begin
								NS = 3'd2;
								CountBurst_NS = 1;
							end
						end
						else begin
							NS = 3'd3;
							CountBurst_NS = 1'sb0;
						end
					end
					else
						NS = 3'd0;
				end
				else
					NS = 3'd1;
			end
			3'd2: begin
				WREADY_o = grant_i;
				MEM_CEN_o = ~(WVALID_i & grant_i);
				valid_o = WVALID_i;
				MEM_A_o = AWADDR_REG + (CountBurst_CS << 3);
				MEM_size = AWSIZE_REG == 3'b011;
				if (WVALID_i & grant_i) begin
					if (AWLEN_REG == CountBurst_CS[7:0]) begin
						if (WLAST_i == 1'b1) begin
							NS = 3'd1;
							CountBurst_NS = 1'sb0;
						end
						else begin
							NS = 3'd4;
							CountBurst_NS = 1'sb0;
						end
					end
					else begin
						NS = 3'd2;
						CountBurst_NS = CountBurst_CS + 1;
					end
				end
				else
					NS = 3'd3;
			end
			3'd4: begin
				BVALID_o = 1'b1;
				BRESP_o = 2'b10;
				if (BREADY_i)
					NS = 3'd0;
				else
					NS = 3'd4;
			end
			default: NS = 3'd0;
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module lint64_to_32 (
	clk,
	rst_n,
	data_req_i,
	data_gnt_o,
	data_wdata_i,
	data_add_i,
	data_wen_i,
	data_be_i,
	data_size_i,
	data_r_valid_o,
	data_r_rdata_o,
	data_req_o,
	data_gnt_i,
	data_wdata_o,
	data_add_o,
	data_wen_o,
	data_be_o,
	data_r_valid_i,
	data_r_rdata_i
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire data_req_i;
	output reg data_gnt_o;
	input wire [63:0] data_wdata_i;
	input wire [31:0] data_add_i;
	input wire data_wen_i;
	input wire [7:0] data_be_i;
	input wire data_size_i;
	output reg data_r_valid_o;
	output reg [63:0] data_r_rdata_o;
	output reg [1:0] data_req_o;
	input wire [1:0] data_gnt_i;
	output wire [63:0] data_wdata_o;
	output reg [63:0] data_add_o;
	output reg [1:0] data_wen_o;
	output wire [7:0] data_be_o;
	input wire [1:0] data_r_valid_i;
	input wire [63:0] data_r_rdata_i;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [63:0] data_r_rdata_q;
	wire [1:0] sample_rdata;
	reg [1:0] gnt_mask;
	reg [1:0] rvalid_mask;
	reg [1:0] size_offset_info;
	reg update_rvalid_mask;
	assign data_wdata_o = data_wdata_i;
	assign data_be_o = data_be_i;
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			CS <= 3'd0;
			data_r_rdata_q <= 1'sb0;
			size_offset_info <= 1'sb0;
			rvalid_mask <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (sample_rdata[0])
				data_r_rdata_q[0+:32] <= data_r_rdata_i[0+:32];
			if (sample_rdata[1])
				data_r_rdata_q[32+:32] <= data_r_rdata_i[32+:32];
			if (data_req_i & data_gnt_o)
				size_offset_info <= {data_size_i, data_add_i[2]};
			if (update_rvalid_mask)
				rvalid_mask <= gnt_mask;
		end
	assign sample_rdata = data_r_valid_i;
	always @(*) begin
		if (_sv2v_0)
			;
		data_req_o = 1'sb0;
		data_add_o = (data_size_i ? {{data_add_i[31:3], 3'b000} + 4, data_add_i[31:3], 3'b000} : {data_add_i, data_add_i});
		data_wen_o = {data_wen_i, data_wen_i};
		data_r_valid_o = 1'sb0;
		data_r_rdata_o = data_r_rdata_i;
		data_gnt_o = 1'sb0;
		gnt_mask = 2'b00;
		update_rvalid_mask = 1'sb0;
		NS = CS;
		case (CS)
			3'd0: begin
				if (data_size_i) begin
					data_req_o = {data_req_i, data_req_i};
					gnt_mask = 2'b00;
				end
				else
					case (data_add_i[2])
						1'b0: begin
							data_req_o = {1'b0, data_req_i};
							gnt_mask = 2'b10;
						end
						1'b1: begin
							data_req_o = {data_req_i, 1'b0};
							gnt_mask = 2'b01;
						end
					endcase
				if (data_req_i)
					case (data_gnt_i | gnt_mask)
						2'b00: NS = 3'd0;
						2'b01: begin
							NS = 3'd1;
							update_rvalid_mask = 1'b1;
						end
						2'b10: begin
							NS = 3'd2;
							update_rvalid_mask = 1'b1;
						end
						2'b11: begin
							NS = 3'd3;
							data_gnt_o = 1'b1;
							update_rvalid_mask = 1'b1;
						end
					endcase
				else
					NS = 3'd0;
			end
			3'd1: begin
				data_req_o = 2'b10;
				if (data_gnt_i[1]) begin
					NS = 3'd4;
					data_gnt_o = 1'b1;
				end
				else
					NS = 3'd1;
			end
			3'd2: begin
				data_req_o = 2'b01;
				if (data_gnt_i[0]) begin
					NS = 3'd5;
					data_gnt_o = 1'b1;
				end
				else
					NS = 3'd2;
			end
			3'd3: begin
				data_r_valid_o = &(data_r_valid_i | rvalid_mask);
				if (size_offset_info[1])
					data_r_rdata_o = data_r_rdata_i;
				else
					data_r_rdata_o = (size_offset_info[0] ? {data_r_rdata_i[32+:32], 32'h00000000} : {32'h00000000, data_r_rdata_i[0+:32]});
				if (&(data_r_valid_i | rvalid_mask)) begin
					if (data_size_i) begin
						data_req_o = {data_req_i, data_req_i};
						gnt_mask = 2'b00;
					end
					else
						case (data_add_i[2])
							1'b0: begin
								data_req_o = {1'b0, data_req_i};
								gnt_mask = 2'b10;
							end
							1'b1: begin
								data_req_o = {data_req_i, 1'b0};
								gnt_mask = 2'b01;
							end
						endcase
					if (data_req_i)
						case (data_gnt_i | gnt_mask)
							2'b00: NS = 3'd0;
							2'b01: begin
								NS = 3'd1;
								update_rvalid_mask = 1'b1;
							end
							2'b10: begin
								NS = 3'd2;
								update_rvalid_mask = 1'b1;
							end
							2'b11: begin
								NS = 3'd3;
								data_gnt_o = 1'b1;
								update_rvalid_mask = 1'b1;
							end
						endcase
					else
						NS = 3'd0;
				end
				else
					case (data_r_valid_i | rvalid_mask)
						2'b00: NS = 3'd3;
						2'b10: NS = 3'd5;
						2'b01: NS = 3'd4;
						default: NS = 3'd3;
					endcase
			end
			3'd4: begin
				data_r_valid_o = data_r_valid_i[1];
				data_r_rdata_o = {data_r_rdata_i[32+:32], data_r_rdata_q[0+:32]};
				if (data_r_valid_i[1]) begin
					if (data_size_i) begin
						data_req_o = {data_req_i, data_req_i};
						gnt_mask = 2'b00;
					end
					else
						case (data_add_i[2])
							1'b0: begin
								data_req_o = {1'b0, data_req_i};
								gnt_mask = 2'b10;
							end
							1'b1: begin
								data_req_o = {data_req_i, 1'b0};
								gnt_mask = 2'b01;
							end
						endcase
					if (data_req_i)
						case (data_gnt_i)
							2'b00: NS = 3'd0;
							2'b01: begin
								NS = 3'd1;
								update_rvalid_mask = 1'b1;
							end
							2'b10: begin
								NS = 3'd2;
								update_rvalid_mask = 1'b1;
							end
							2'b11: begin
								NS = 3'd3;
								data_gnt_o = 1'b1;
								update_rvalid_mask = 1'b1;
							end
						endcase
					else
						NS = 3'd0;
				end
				else
					NS = 3'd4;
			end
			3'd5: begin
				data_r_valid_o = data_r_valid_i[0];
				data_r_rdata_o = {data_r_rdata_q[32+:32], data_r_rdata_i[0+:32]};
				if (data_r_valid_i[0]) begin
					if (data_size_i) begin
						data_req_o = {data_req_i, data_req_i};
						gnt_mask = 2'b00;
					end
					else
						case (data_add_i[2])
							1'b0: begin
								data_req_o = {1'b0, data_req_i};
								gnt_mask = 2'b10;
							end
							1'b1: begin
								data_req_o = {data_req_i, 1'b0};
								gnt_mask = 2'b01;
							end
						endcase
					if (data_req_i)
						case (data_gnt_i)
							2'b00: NS = 3'd0;
							2'b01: begin
								NS = 3'd1;
								update_rvalid_mask = 1'b1;
							end
							2'b10: begin
								NS = 3'd2;
								update_rvalid_mask = 1'b1;
							end
							2'b11: begin
								NS = 3'd3;
								data_gnt_o = 1'b1;
								update_rvalid_mask = 1'b1;
							end
						endcase
					else
						NS = 3'd0;
				end
				else
					NS = 3'd5;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module l2_tcdm_demux (
	clk,
	rst_n,
	test_en_i,
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_aux_i,
	data_gnt_o,
	data_r_aux_o,
	data_r_valid_o,
	data_r_rdata_o,
	data_r_opc_o,
	data_req_o_TDCM,
	data_add_o_TDCM,
	data_wen_o_TDCM,
	data_wdata_o_TDCM,
	data_be_o_TDCM,
	data_gnt_i_TDCM,
	data_r_valid_i_TDCM,
	data_r_rdata_i_TDCM,
	data_req_o_PER,
	data_add_o_PER,
	data_wen_o_PER,
	data_wdata_o_PER,
	data_be_o_PER,
	data_aux_o_PER,
	data_gnt_i_PER,
	data_r_valid_i_PER,
	data_r_rdata_i_PER,
	data_r_opc_i_PER,
	data_r_aux_i_PER,
	PER_START_ADDR,
	PER_END_ADDR,
	TCDM_START_ADDR,
	TCDM_END_ADDR
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter AUX_WIDTH = 4;
	parameter [31:0] N_PERIPHS = 2;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire data_req_i;
	input wire [ADDR_WIDTH - 1:0] data_add_i;
	input wire data_wen_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_i;
	input wire [BE_WIDTH - 1:0] data_be_i;
	input wire [AUX_WIDTH - 1:0] data_aux_i;
	output reg data_gnt_o;
	output reg [AUX_WIDTH - 1:0] data_r_aux_o;
	output reg data_r_valid_o;
	output reg [DATA_WIDTH - 1:0] data_r_rdata_o;
	output reg data_r_opc_o;
	output reg data_req_o_TDCM;
	output wire [ADDR_WIDTH - 1:0] data_add_o_TDCM;
	output wire data_wen_o_TDCM;
	output wire [DATA_WIDTH - 1:0] data_wdata_o_TDCM;
	output wire [BE_WIDTH - 1:0] data_be_o_TDCM;
	input wire data_gnt_i_TDCM;
	input wire data_r_valid_i_TDCM;
	input wire [DATA_WIDTH - 1:0] data_r_rdata_i_TDCM;
	output reg data_req_o_PER;
	output wire [ADDR_WIDTH - 1:0] data_add_o_PER;
	output wire data_wen_o_PER;
	output wire [DATA_WIDTH - 1:0] data_wdata_o_PER;
	output wire [BE_WIDTH - 1:0] data_be_o_PER;
	output wire [AUX_WIDTH - 1:0] data_aux_o_PER;
	input wire data_gnt_i_PER;
	input wire data_r_valid_i_PER;
	input wire [DATA_WIDTH - 1:0] data_r_rdata_i_PER;
	input wire data_r_opc_i_PER;
	input wire [AUX_WIDTH - 1:0] data_r_aux_i_PER;
	input wire [(N_PERIPHS * ADDR_WIDTH) - 1:0] PER_START_ADDR;
	input wire [(N_PERIPHS * ADDR_WIDTH) - 1:0] PER_END_ADDR;
	input wire [ADDR_WIDTH - 1:0] TCDM_START_ADDR;
	input wire [ADDR_WIDTH - 1:0] TCDM_END_ADDR;
	reg [1:0] CS;
	reg [1:0] NS;
	wire [(N_PERIPHS >= 0 ? ((N_PERIPHS + 1) * ADDR_WIDTH) - 1 : ((1 - N_PERIPHS) * ADDR_WIDTH) + ((N_PERIPHS * ADDR_WIDTH) - 1)):(N_PERIPHS >= 0 ? 0 : N_PERIPHS * ADDR_WIDTH)] ADDR_START;
	wire [(N_PERIPHS >= 0 ? ((N_PERIPHS + 1) * ADDR_WIDTH) - 1 : ((1 - N_PERIPHS) * ADDR_WIDTH) + ((N_PERIPHS * ADDR_WIDTH) - 1)):(N_PERIPHS >= 0 ? 0 : N_PERIPHS * ADDR_WIDTH)] ADDR_END;
	reg [N_PERIPHS:0] destination_OH;
	assign ADDR_START = {TCDM_START_ADDR, PER_START_ADDR};
	assign ADDR_END = {TCDM_END_ADDR, PER_END_ADDR};
	assign data_add_o_TDCM = data_add_i;
	assign data_wen_o_TDCM = data_wen_i;
	assign data_wdata_o_TDCM = data_wdata_i;
	assign data_be_o_TDCM = data_be_i;
	assign data_add_o_PER = data_add_i;
	assign data_wen_o_PER = data_wen_i;
	assign data_wdata_o_PER = data_wdata_i;
	assign data_be_o_PER = data_be_i;
	assign data_aux_o_PER = data_aux_i;
	reg sample_aux;
	reg [AUX_WIDTH - 1:0] sampled_data_aux;
	always @(*) begin
		destination_OH = 1'sb0;
		begin : sv2v_autoblock_1
			reg [31:0] x;
			for (x = 0; x < (N_PERIPHS + 1); x = x + 1)
				if ((data_add_i >= ADDR_START[(N_PERIPHS >= 0 ? x : N_PERIPHS - x) * ADDR_WIDTH+:ADDR_WIDTH]) && (data_add_i < ADDR_END[(N_PERIPHS >= 0 ? x : N_PERIPHS - x) * ADDR_WIDTH+:ADDR_WIDTH]))
					destination_OH[x] = 1'b1;
		end
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			CS <= 2'd0;
			sampled_data_aux <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (sample_aux)
				sampled_data_aux <= data_aux_i;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		data_req_o_TDCM = 1'b0;
		data_req_o_PER = 1'b0;
		data_gnt_o = 1'b0;
		sample_aux = 1'sb0;
		data_r_opc_o = 1'b0;
		data_r_valid_o = 1'b0;
		data_r_aux_o = sampled_data_aux;
		data_r_rdata_o = data_r_rdata_i_TDCM;
		case (CS)
			2'd0:
				if (data_req_i) begin
					if (destination_OH[N_PERIPHS] == 1'b1) begin
						data_req_o_TDCM = 1'b1;
						data_gnt_o = data_gnt_i_TDCM;
						sample_aux = data_gnt_i_TDCM;
						if (data_gnt_i_TDCM)
							NS = 2'd1;
						else
							NS = 2'd0;
					end
					else if (|destination_OH[N_PERIPHS - 1:0] == 1'b1) begin
						data_req_o_PER = 1'b1;
						data_gnt_o = data_gnt_i_PER;
						if (data_gnt_i_PER)
							NS = 2'd2;
						else
							NS = 2'd0;
					end
					else begin
						NS = 2'd3;
						data_gnt_o = 1'b1;
					end
				end
				else
					NS = 2'd0;
			2'd1: begin
				data_r_valid_o = 1'b1;
				data_r_aux_o = sampled_data_aux;
				data_r_rdata_o = data_r_rdata_i_TDCM;
				if (data_req_i) begin
					if (destination_OH[N_PERIPHS] == 1'b1) begin
						data_req_o_TDCM = 1'b1;
						data_gnt_o = data_gnt_i_TDCM;
						sample_aux = data_gnt_i_TDCM;
						if (data_gnt_i_TDCM)
							NS = 2'd1;
						else
							NS = 2'd0;
					end
					else if (|destination_OH[N_PERIPHS - 1:0] == 1'b1) begin
						data_req_o_PER = 1'b1;
						data_gnt_o = data_gnt_i_PER;
						if (data_gnt_i_PER)
							NS = 2'd2;
						else
							NS = 2'd0;
					end
					else begin
						NS = 2'd3;
						data_gnt_o = 1'b1;
						sample_aux = 1'b1;
					end
				end
				else
					NS = 2'd0;
			end
			2'd2: begin
				data_r_valid_o = data_r_valid_i_PER;
				data_r_aux_o = data_r_aux_i_PER;
				data_r_rdata_o = data_r_rdata_i_PER;
				data_r_opc_o = data_r_opc_i_PER;
				if (data_r_valid_i_PER) begin
					if (data_req_i) begin
						if (destination_OH[N_PERIPHS] == 1'b1) begin
							data_req_o_TDCM = 1'b1;
							data_gnt_o = data_gnt_i_TDCM;
							sample_aux = data_gnt_i_TDCM;
							if (data_gnt_i_TDCM)
								NS = 2'd1;
							else
								NS = 2'd0;
						end
						else if (|destination_OH[N_PERIPHS - 1:0] == 1'b1) begin
							data_req_o_PER = 1'b1;
							data_gnt_o = data_gnt_i_PER;
							if (data_gnt_i_PER)
								NS = 2'd2;
							else
								NS = 2'd0;
						end
						else begin
							NS = 2'd3;
							data_gnt_o = 1'b1;
						end
					end
					else
						NS = 2'd0;
				end
				else
					NS = 2'd2;
			end
			2'd3: begin
				data_r_valid_o = 1'b1;
				data_r_aux_o = sampled_data_aux;
				data_r_rdata_o = 32'hbadacce5;
				NS = 2'd0;
				data_r_opc_o = 1'b1;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module lint_2_apb (
	clk,
	rst_n,
	data_req_i,
	data_add_i,
	data_wen_i,
	data_wdata_i,
	data_be_i,
	data_aux_i,
	data_ID_i,
	data_gnt_o,
	data_r_valid_o,
	data_r_rdata_o,
	data_r_opc_o,
	data_r_aux_o,
	data_r_ID_o,
	master_PADDR,
	master_PWDATA,
	master_PWRITE,
	master_PSEL,
	master_PENABLE,
	master_PRDATA,
	master_PREADY,
	master_PSLVERR
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter ID_WIDTH = 10;
	parameter AUX_WIDTH = 8;
	input wire clk;
	input wire rst_n;
	input wire data_req_i;
	input wire [ADDR_WIDTH - 1:0] data_add_i;
	input wire data_wen_i;
	input wire [DATA_WIDTH - 1:0] data_wdata_i;
	input wire [BE_WIDTH - 1:0] data_be_i;
	input wire [AUX_WIDTH - 1:0] data_aux_i;
	input wire [ID_WIDTH - 1:0] data_ID_i;
	output reg data_gnt_o;
	output reg data_r_valid_o;
	output reg [DATA_WIDTH - 1:0] data_r_rdata_o;
	output reg data_r_opc_o;
	output reg [AUX_WIDTH - 1:0] data_r_aux_o;
	output reg [ID_WIDTH - 1:0] data_r_ID_o;
	output wire [ADDR_WIDTH - 1:0] master_PADDR;
	output wire [DATA_WIDTH - 1:0] master_PWDATA;
	output wire master_PWRITE;
	output reg master_PSEL;
	output reg master_PENABLE;
	input wire [DATA_WIDTH - 1:0] master_PRDATA;
	input wire master_PREADY;
	input wire master_PSLVERR;
	reg [1:0] CS;
	reg [1:0] NS;
	reg sample_req_info;
	reg sample_rdata;
	reg data_r_valid_NS;
	reg [ADDR_WIDTH - 1:0] master_PADDR_Q;
	reg [DATA_WIDTH - 1:0] master_PWDATA_Q;
	reg master_PWRITE_Q;
	assign master_PADDR = master_PADDR_Q;
	assign master_PWDATA = master_PWDATA_Q;
	assign master_PWRITE = master_PWRITE_Q;
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			CS <= 2'd0;
			data_r_aux_o <= 1'sb0;
			data_r_ID_o <= 1'sb0;
			master_PADDR_Q <= 1'sb0;
			master_PWDATA_Q <= 1'sb0;
			master_PWRITE_Q <= 1'sb0;
			data_r_rdata_o <= 1'sb0;
			data_r_opc_o <= 1'sb0;
			data_r_valid_o <= 1'b0;
		end
		else begin
			CS <= NS;
			if (sample_req_info) begin
				data_r_aux_o <= data_aux_i;
				data_r_ID_o <= data_ID_i;
				master_PADDR_Q <= data_add_i;
				master_PWDATA_Q <= data_wdata_i;
				master_PWRITE_Q <= ~data_wen_i;
			end
			if (sample_rdata) begin
				data_r_rdata_o <= master_PRDATA;
				data_r_opc_o <= master_PSLVERR;
			end
			data_r_valid_o <= data_r_valid_NS;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		master_PSEL = 1'b0;
		master_PENABLE = 1'b0;
		sample_req_info = 1'b0;
		data_gnt_o = 1'b0;
		sample_rdata = 1'b0;
		data_r_valid_NS = 1'b0;
		case (CS)
			2'd0: begin
				data_gnt_o = 1'b1;
				data_r_valid_NS = 1'b0;
				if (data_req_i) begin
					sample_req_info = 1'b1;
					NS = 2'd1;
				end
				else
					NS = 2'd0;
			end
			2'd1: begin
				master_PSEL = 1'b1;
				master_PENABLE = 1'b1;
				sample_rdata = master_PREADY;
				data_r_valid_NS = master_PREADY;
				if (master_PREADY)
					NS = 2'd2;
				else
					NS = 2'd1;
			end
			2'd2: begin
				NS = 2'd0;
				data_gnt_o = 1'b0;
			end
			default: NS = 2'd0;
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module lint_2_axi (
	clk_i,
	rst_ni,
	data_req_i,
	data_addr_i,
	data_we_i,
	data_wdata_i,
	data_be_i,
	data_ID_i,
	data_aux_i,
	data_gnt_o,
	data_rvalid_o,
	data_rdata_o,
	data_ropc_o,
	data_raux_o,
	data_rID_o,
	aw_id_o,
	aw_addr_o,
	aw_len_o,
	aw_size_o,
	aw_burst_o,
	aw_lock_o,
	aw_cache_o,
	aw_prot_o,
	aw_region_o,
	aw_user_o,
	aw_qos_o,
	aw_valid_o,
	aw_ready_i,
	w_data_o,
	w_strb_o,
	w_last_o,
	w_user_o,
	w_valid_o,
	w_ready_i,
	b_id_i,
	b_resp_i,
	b_valid_i,
	b_user_i,
	b_ready_o,
	ar_id_o,
	ar_addr_o,
	ar_len_o,
	ar_size_o,
	ar_burst_o,
	ar_lock_o,
	ar_cache_o,
	ar_prot_o,
	ar_region_o,
	ar_user_o,
	ar_qos_o,
	ar_valid_o,
	ar_ready_i,
	r_id_i,
	r_data_i,
	r_resp_i,
	r_last_i,
	r_user_i,
	r_valid_i,
	r_ready_o
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = 32;
	parameter ID_WIDTH = 16;
	parameter USER_WIDTH = 10;
	parameter AUX_WIDTH = 10;
	parameter AXI_ID_WIDTH = 5;
	parameter AXI_STRB_WIDTH = DATA_WIDTH / 8;
	parameter REGISTERED_GRANT = "FALSE";
	input wire clk_i;
	input wire rst_ni;
	input wire data_req_i;
	input wire [ADDR_WIDTH - 1:0] data_addr_i;
	input wire data_we_i;
	input wire [31:0] data_wdata_i;
	input wire [BE_WIDTH - 1:0] data_be_i;
	input wire [ID_WIDTH - 1:0] data_ID_i;
	input wire [AUX_WIDTH - 1:0] data_aux_i;
	output wire data_gnt_o;
	output wire data_rvalid_o;
	output wire [31:0] data_rdata_o;
	output wire data_ropc_o;
	output reg [AUX_WIDTH - 1:0] data_raux_o;
	output reg [ID_WIDTH - 1:0] data_rID_o;
	output wire [AXI_ID_WIDTH - 1:0] aw_id_o;
	output wire [ADDR_WIDTH - 1:0] aw_addr_o;
	output wire [7:0] aw_len_o;
	output wire [2:0] aw_size_o;
	output wire [1:0] aw_burst_o;
	output wire aw_lock_o;
	output wire [3:0] aw_cache_o;
	output wire [2:0] aw_prot_o;
	output wire [3:0] aw_region_o;
	output wire [USER_WIDTH - 1:0] aw_user_o;
	output wire [3:0] aw_qos_o;
	output reg aw_valid_o;
	input wire aw_ready_i;
	output wire [DATA_WIDTH - 1:0] w_data_o;
	output wire [AXI_STRB_WIDTH - 1:0] w_strb_o;
	output wire w_last_o;
	output wire [USER_WIDTH - 1:0] w_user_o;
	output reg w_valid_o;
	input wire w_ready_i;
	input wire [AXI_ID_WIDTH - 1:0] b_id_i;
	input wire [1:0] b_resp_i;
	input wire b_valid_i;
	input wire [USER_WIDTH - 1:0] b_user_i;
	output reg b_ready_o;
	output wire [AXI_ID_WIDTH - 1:0] ar_id_o;
	output wire [ADDR_WIDTH - 1:0] ar_addr_o;
	output wire [7:0] ar_len_o;
	output wire [2:0] ar_size_o;
	output wire [1:0] ar_burst_o;
	output wire ar_lock_o;
	output wire [3:0] ar_cache_o;
	output wire [2:0] ar_prot_o;
	output wire [3:0] ar_region_o;
	output wire [USER_WIDTH - 1:0] ar_user_o;
	output wire [3:0] ar_qos_o;
	output reg ar_valid_o;
	input wire ar_ready_i;
	input wire [AXI_ID_WIDTH - 1:0] r_id_i;
	input wire [DATA_WIDTH - 1:0] r_data_i;
	input wire [1:0] r_resp_i;
	input wire r_last_i;
	input wire [USER_WIDTH - 1:0] r_user_i;
	input wire r_valid_i;
	output reg r_ready_o;
	reg [2:0] CS;
	reg [2:0] NS;
	wire [31:0] rdata;
	reg valid;
	reg granted;
	reg r_opc;
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		granted = 1'b0;
		valid = 1'b0;
		r_opc = 1'b0;
		aw_valid_o = 1'b0;
		ar_valid_o = 1'b0;
		r_ready_o = 1'b0;
		w_valid_o = 1'b0;
		b_ready_o = 1'b0;
		case (CS)
			3'd0:
				if (data_req_i) begin
					if (data_we_i) begin
						aw_valid_o = 1'b1;
						w_valid_o = 1'b1;
						if (aw_ready_i) begin
							if (w_ready_i) begin
								granted = 1'b1;
								NS = 3'd4;
							end
							else
								NS = 3'd2;
						end
						else if (w_ready_i)
							NS = 3'd3;
						else
							NS = 3'd0;
					end
					else begin
						ar_valid_o = 1'b1;
						if (ar_ready_i) begin
							granted = 1'b1;
							NS = 3'd1;
						end
						else
							NS = 3'd0;
					end
				end
				else
					NS = 3'd0;
			3'd2: begin
				w_valid_o = 1'b1;
				if (w_ready_i) begin
					granted = 1'b1;
					NS = 3'd4;
				end
			end
			3'd3: begin
				aw_valid_o = 1'b1;
				if (aw_ready_i) begin
					granted = 1'b1;
					NS = 3'd4;
				end
			end
			3'd4: begin
				b_ready_o = 1'b1;
				if (b_valid_i) begin
					valid = 1'b1;
					r_opc = b_resp_i[1];
					NS = 3'd0;
				end
			end
			3'd1:
				if (r_valid_i) begin
					valid = 1'b1;
					r_ready_o = 1'b1;
					r_opc = r_resp_i[1];
					NS = 3'd0;
				end
			default: NS = 3'd0;
		endcase
	end
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			CS <= 3'd0;
		else
			CS <= NS;
	generate
		if (DATA_WIDTH == 32) begin : genblk1
			assign rdata = r_data_i[31:0];
		end
		else if (DATA_WIDTH == 64) begin : genblk1
			reg [0:0] addr_q;
			always @(posedge clk_i or negedge rst_ni)
				if (~rst_ni)
					addr_q <= 1'sb0;
				else if (data_gnt_o)
					addr_q <= data_addr_i[2:2];
			assign rdata = (addr_q[0] ? r_data_i[63:32] : r_data_i[31:0]);
		end
		else begin : genblk1
			initial $display("Error [%0t] hackatdac18-2018-soc/ips/L2_tcdm_hybrid_interco/RTL/lint_2_axi.sv:272:25 - lint_2_axi.genblk1.<unnamed_block>\n msg: ", $time, "DATA_WIDTH has an      invalid value");
		end
	endgenerate
	genvar _gv_w_1;
	generate
		for (_gv_w_1 = 0; _gv_w_1 < (DATA_WIDTH / 32); _gv_w_1 = _gv_w_1 + 1) begin : genblk2
			localparam w = _gv_w_1;
			assign w_data_o[(w * 32) + 31:(w * 32) + 0] = data_wdata_i;
		end
		if (DATA_WIDTH == 32) begin : genblk3
			assign w_strb_o = data_be_i;
		end
		else if (DATA_WIDTH == 64) begin : genblk3
			assign w_strb_o = (data_addr_i[2] ? {data_be_i, 4'b0000} : {4'b0000, data_be_i});
		end
		else begin : genblk3
			initial $display("Error [%0t] hackatdac18-2018-soc/ips/L2_tcdm_hybrid_interco/RTL/lint_2_axi.sv:293:25 - lint_2_axi.genblk3.<unnamed_block>\n msg: ", $time, "DATA_WIDTH has an      invalid value");
		end
	endgenerate
	assign aw_id_o = 1'sb0;
	assign aw_addr_o = data_addr_i;
	assign aw_size_o = 3'b010;
	assign aw_len_o = 1'sb0;
	assign aw_burst_o = 1'sb0;
	assign aw_lock_o = 1'sb0;
	assign aw_cache_o = 1'sb0;
	assign aw_prot_o = 1'sb0;
	assign aw_region_o = 1'sb0;
	assign aw_user_o = 1'sb0;
	assign aw_qos_o = 1'sb0;
	assign ar_id_o = 1'sb0;
	assign ar_addr_o = data_addr_i;
	assign ar_size_o = 3'b010;
	assign ar_len_o = 1'sb0;
	assign ar_burst_o = 1'sb0;
	assign ar_prot_o = 1'sb0;
	assign ar_region_o = 1'sb0;
	assign ar_lock_o = 1'sb0;
	assign ar_cache_o = 1'sb0;
	assign ar_qos_o = 1'sb0;
	assign ar_user_o = 1'sb0;
	assign w_last_o = 1'b1;
	assign w_user_o = 1'sb0;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			data_rID_o <= 1'sb0;
			data_raux_o <= 1'sb0;
		end
		else if (granted) begin
			data_raux_o <= data_aux_i;
			data_rID_o <= data_ID_i;
		end
	generate
		if (REGISTERED_GRANT == "TRUE") begin : genblk4
			reg valid_q;
			reg [31:0] rdata_q;
			reg r_opc_q;
			always @(posedge clk_i or negedge rst_ni)
				if (~rst_ni) begin
					valid_q <= 1'b0;
					rdata_q <= 1'sb0;
					r_opc_q <= 1'sb0;
				end
				else begin
					valid_q <= valid;
					r_opc_q <= r_opc;
					if (valid)
						rdata_q <= rdata;
				end
			assign data_rdata_o = rdata_q;
			assign data_rvalid_o = valid_q;
			assign data_gnt_o = valid;
			assign data_ropc_o = r_opc_q;
		end
		else begin : genblk4
			assign data_rdata_o = rdata;
			assign data_rvalid_o = valid;
			assign data_gnt_o = granted;
			assign data_ropc_o = r_opc;
		end
	endgenerate
	initial _sv2v_0 = 0;
endmodule
module adbg_tap_top (
	tms_pad_i,
	tck_pad_i,
	trstn_pad_i,
	tdi_pad_i,
	tdo_pad_o,
	tdo_padoe_o,
	test_mode_i,
	test_logic_reset_o,
	run_test_idle_o,
	shift_dr_o,
	pause_dr_o,
	update_dr_o,
	capture_dr_o,
	extest_select_o,
	sample_preload_select_o,
	mbist_select_o,
	debug_select_o,
	tdi_o,
	debug_tdo_i,
	bs_chain_tdo_i,
	mbist_tdo_i
);
	input tms_pad_i;
	input tck_pad_i;
	input trstn_pad_i;
	input tdi_pad_i;
	output reg tdo_pad_o;
	output reg tdo_padoe_o;
	input test_mode_i;
	output wire test_logic_reset_o;
	output wire run_test_idle_o;
	output wire shift_dr_o;
	output wire pause_dr_o;
	output wire update_dr_o;
	output wire capture_dr_o;
	output wire extest_select_o;
	output wire sample_preload_select_o;
	output wire mbist_select_o;
	output wire debug_select_o;
	output wire tdi_o;
	input debug_tdo_i;
	input bs_chain_tdo_i;
	input mbist_tdo_i;
	reg test_logic_reset;
	reg run_test_idle;
	reg select_dr_scan;
	reg capture_dr;
	reg shift_dr;
	reg exit1_dr;
	reg pause_dr;
	reg exit2_dr;
	reg update_dr;
	reg select_ir_scan;
	reg capture_ir;
	reg shift_ir;
	reg exit1_ir;
	reg pause_ir;
	reg exit2_ir;
	reg update_ir;
	reg extest_select;
	reg sample_preload_select;
	reg idcode_select;
	reg mbist_select;
	reg debug_select;
	reg bypass_select;
	wire s_clk_neg;
	wire s_tck_inv;
	cluster_clock_inverter u_clk_inv(
		.clk_i(tck_pad_i),
		.clk_o(s_tck_inv)
	);
	cluster_clock_mux2 u_clk_mux(
		.clk0_i(s_tck_inv),
		.clk1_i(tck_pad_i),
		.clk_sel_i(test_mode_i),
		.clk_o(s_clk_neg)
	);
	assign tdi_o = tdi_pad_i;
	assign test_logic_reset_o = test_logic_reset;
	assign run_test_idle_o = run_test_idle;
	assign shift_dr_o = shift_dr;
	assign pause_dr_o = pause_dr;
	assign update_dr_o = update_dr;
	assign capture_dr_o = capture_dr;
	assign extest_select_o = extest_select;
	assign sample_preload_select_o = sample_preload_select;
	assign mbist_select_o = mbist_select;
	assign debug_select_o = debug_select;
	reg [3:0] TAP_state;
	reg [3:0] next_TAP_state;
	reg passchk;
	reg [31:0] correct;
	reg [31:0] pass;
	reg [4:0] bitindex;
	always @(posedge tck_pad_i or negedge trstn_pad_i)
		if (trstn_pad_i == 0) begin
			TAP_state = 4'hf;
			pass = 32'hdeadbeef;
		end
		else
			TAP_state = next_TAP_state;
	always @(TAP_state or tms_pad_i)
		case (TAP_state)
			4'hf: begin
				passchk = 0;
				if (tms_pad_i)
					next_TAP_state = 4'hf;
				else
					next_TAP_state = 4'hc;
			end
			4'hc:
				if (tms_pad_i && passchk)
					next_TAP_state = 4'h7;
				else begin
					next_TAP_state = 4'hc;
					if (correct >= 32'h0001ffff)
						passchk = 1;
					else if (tdi_o == pass[bitindex]) begin
						correct = correct + 1;
						bitindex = bitindex + 1;
					end
				end
			4'h7:
				if (tms_pad_i)
					next_TAP_state = 4'h4;
				else
					next_TAP_state = 4'h6;
			4'h6:
				if (tms_pad_i)
					next_TAP_state = 4'h1;
				else
					next_TAP_state = 4'h2;
			4'h2:
				if (tms_pad_i)
					next_TAP_state = 4'h1;
				else
					next_TAP_state = 4'h2;
			4'h1:
				if (tms_pad_i)
					next_TAP_state = 4'h5;
				else
					next_TAP_state = 4'h3;
			4'h3:
				if (tms_pad_i)
					next_TAP_state = 4'h0;
				else
					next_TAP_state = 4'h3;
			4'h0:
				if (tms_pad_i)
					next_TAP_state = 4'h5;
				else
					next_TAP_state = 4'h2;
			4'h5:
				if (tms_pad_i)
					next_TAP_state = 4'h7;
				else
					next_TAP_state = 4'hc;
			4'h4:
				if (tms_pad_i)
					next_TAP_state = 4'hf;
				else
					next_TAP_state = 4'he;
			4'he:
				if (tms_pad_i)
					next_TAP_state = 4'h9;
				else
					next_TAP_state = 4'ha;
			4'ha:
				if (tms_pad_i)
					next_TAP_state = 4'h9;
				else
					next_TAP_state = 4'ha;
			4'h9:
				if (tms_pad_i)
					next_TAP_state = 4'hd;
				else
					next_TAP_state = 4'hb;
			4'hb:
				if (tms_pad_i)
					next_TAP_state = 4'h8;
				else
					next_TAP_state = 4'hb;
			4'h8:
				if (tms_pad_i)
					next_TAP_state = 4'hd;
				else
					next_TAP_state = 4'ha;
			4'hd:
				if (tms_pad_i)
					next_TAP_state = 4'h7;
				else
					next_TAP_state = 4'hc;
			default: next_TAP_state = 4'hf;
		endcase
	always @(TAP_state) begin
		test_logic_reset = 1'b0;
		run_test_idle = 1'b0;
		select_dr_scan = 1'b0;
		capture_dr = 1'b0;
		shift_dr = 1'b0;
		exit1_dr = 1'b0;
		pause_dr = 1'b0;
		exit2_dr = 1'b0;
		update_dr = 1'b0;
		select_ir_scan = 1'b0;
		capture_ir = 1'b0;
		shift_ir = 1'b0;
		exit1_ir = 1'b0;
		pause_ir = 1'b0;
		exit2_ir = 1'b0;
		update_ir = 1'b0;
		case (TAP_state)
			4'hf: test_logic_reset = 1'b1;
			4'hc: run_test_idle = 1'b1;
			4'h7: select_dr_scan = 1'b1;
			4'h6: capture_dr = 1'b1;
			4'h2: shift_dr = 1'b1;
			4'h1: exit1_dr = 1'b1;
			4'h3: pause_dr = 1'b1;
			4'h0: exit2_dr = 1'b1;
			4'h5: update_dr = 1'b1;
			4'h4: select_ir_scan = 1'b1;
			4'he: capture_ir = 1'b1;
			4'ha: shift_ir = 1'b1;
			4'h9: exit1_ir = 1'b1;
			4'hb: pause_ir = 1'b1;
			4'h8: exit2_ir = 1'b1;
			4'hd: update_ir = 1'b1;
			default:
				;
		endcase
	end
	reg [3:0] jtag_ir;
	reg [3:0] latched_jtag_ir;
	wire instruction_tdo;
	always @(posedge tck_pad_i or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			jtag_ir[3:0] <= 4'b0000;
		else if (test_logic_reset == 1)
			jtag_ir[3:0] <= 4'b0000;
		else if (capture_ir)
			jtag_ir <= 4'b0101;
		else if (shift_ir)
			jtag_ir[3:0] <= {tdi_pad_i, jtag_ir[3:1]};
	assign instruction_tdo = jtag_ir[0];
	always @(posedge s_clk_neg or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			latched_jtag_ir <= 4'b0010;
		else if (test_logic_reset)
			latched_jtag_ir <= 4'b0010;
		else if (update_ir)
			latched_jtag_ir <= jtag_ir;
	reg [31:0] idcode_reg;
	wire idcode_tdo;
	always @(posedge tck_pad_i or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			idcode_reg <= 32'h249511c3;
		else if (test_logic_reset)
			idcode_reg <= 32'h249511c3;
		else if (idcode_select & capture_dr)
			idcode_reg <= 32'h249511c3;
		else if (idcode_select & shift_dr)
			idcode_reg <= {tdi_pad_i, idcode_reg[31:1]};
	assign idcode_tdo = idcode_reg[0];
	wire bypassed_tdo;
	reg bypass_reg;
	always @(posedge tck_pad_i or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			bypass_reg <= 1'b0;
		else if (test_logic_reset == 1)
			bypass_reg <= 1'b0;
		else if (bypass_select & capture_dr)
			bypass_reg <= 1'b0;
		else if (bypass_select & shift_dr)
			bypass_reg <= tdi_pad_i;
	assign bypassed_tdo = bypass_reg;
	always @(latched_jtag_ir) begin
		extest_select = 1'b0;
		sample_preload_select = 1'b0;
		idcode_select = 1'b0;
		mbist_select = 1'b0;
		debug_select = 1'b0;
		bypass_select = 1'b0;
		case (latched_jtag_ir)
			4'b0000: extest_select = 1'b1;
			4'b0001: sample_preload_select = 1'b1;
			4'b0010: idcode_select = 1'b1;
			4'b1001: mbist_select = 1'b1;
			4'b1000: debug_select = 1'b1;
			4'b1111: bypass_select = 1'b1;
			default: bypass_select = 1'b1;
		endcase
	end
	reg tdo_mux_out;
	always @(shift_ir or instruction_tdo or latched_jtag_ir or idcode_tdo or debug_tdo_i or bs_chain_tdo_i or mbist_tdo_i or bypassed_tdo or bs_chain_tdo_i)
		if (shift_ir)
			tdo_mux_out = instruction_tdo;
		else
			case (latched_jtag_ir)
				4'b0010: tdo_mux_out = idcode_tdo;
				4'b1000: tdo_mux_out = debug_tdo_i;
				4'b0001: tdo_mux_out = bs_chain_tdo_i;
				4'b0000: tdo_mux_out = bs_chain_tdo_i;
				4'b1001: tdo_mux_out = mbist_tdo_i;
				default: tdo_mux_out = bypassed_tdo;
			endcase
	always @(posedge s_clk_neg or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			tdo_pad_o <= 1'b0;
		else
			tdo_pad_o <= tdo_mux_out;
	always @(posedge s_clk_neg or negedge trstn_pad_i)
		if (trstn_pad_i == 0)
			tdo_padoe_o <= 1'b0;
		else
			tdo_padoe_o <= shift_ir | shift_dr;
endmodule
module apb_gpio (
	HCLK,
	HRESETn,
	dft_cg_enable_i,
	PADDR,
	PWDATA,
	PWRITE,
	PSEL,
	PENABLE,
	PRDATA,
	PREADY,
	PSLVERR,
	gpio_in,
	gpio_in_sync,
	gpio_out,
	gpio_dir,
	gpio_padcfg,
	interrupt
);
	reg _sv2v_0;
	parameter APB_ADDR_WIDTH = 12;
	input wire HCLK;
	input wire HRESETn;
	input wire dft_cg_enable_i;
	input wire [APB_ADDR_WIDTH - 1:0] PADDR;
	input wire [31:0] PWDATA;
	input wire PWRITE;
	input wire PSEL;
	input wire PENABLE;
	output reg [31:0] PRDATA;
	output wire PREADY;
	output wire PSLVERR;
	input wire [31:0] gpio_in;
	output wire [31:0] gpio_in_sync;
	output wire [31:0] gpio_out;
	output wire [31:0] gpio_dir;
	output reg [191:0] gpio_padcfg;
	output wire interrupt;
	reg [31:0] r_gpio_inten;
	reg [31:0] r_gpio_inttype0;
	reg [31:0] s_gpio_inttype0;
	reg [31:0] r_gpio_inttype1;
	reg [31:0] s_gpio_inttype1;
	reg [31:0] r_gpio_out;
	reg [31:0] r_gpio_dir;
	reg [31:0] r_gpio_sync0;
	reg [31:0] r_gpio_sync1;
	reg [31:0] r_gpio_in;
	reg [31:0] r_gpio_en;
	reg [31:0] r_gpio_lock;
	wire [31:0] s_gpio_rise;
	wire [31:0] s_gpio_fall;
	wire [31:0] s_is_int_rise;
	wire [31:0] s_is_int_rifa;
	wire [31:0] s_is_int_fall;
	wire [31:0] s_is_int_all;
	wire s_rise_int;
	wire [4:0] s_apb_addr;
	reg [31:0] r_status;
	reg [7:0] s_clk_en;
	wire [7:0] s_clkg;
	genvar _gv_i_2;
	assign s_apb_addr = PADDR[6:2];
	assign gpio_in_sync = r_gpio_sync1;
	assign s_gpio_rise = r_gpio_sync1 & ~r_gpio_in;
	assign s_gpio_fall = ~r_gpio_sync1 & r_gpio_in;
	assign s_is_int_fall = (~s_gpio_inttype1 & ~s_gpio_inttype0) & s_gpio_fall;
	assign s_is_int_rise = (~s_gpio_inttype1 & s_gpio_inttype0) & s_gpio_rise;
	assign s_is_int_rifa = (s_gpio_inttype1 & ~s_gpio_inttype0) & (s_gpio_rise | s_gpio_fall);
	assign s_is_int_all = (r_gpio_inten & r_gpio_en) & ((s_is_int_rise | s_is_int_fall) | s_is_int_rifa);
	assign s_rise_int = |s_is_int_all;
	assign interrupt = s_rise_int;
	always @(*) begin
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 16; i = i + 1)
				begin
					s_gpio_inttype0[i] = r_gpio_inttype0[i * 2];
					s_gpio_inttype0[16 + i] = r_gpio_inttype1[i * 2];
					s_gpio_inttype1[i] = r_gpio_inttype0[(i * 2) + 1];
					s_gpio_inttype1[16 + i] = r_gpio_inttype1[(i * 2) + 1];
				end
		end
	end
	always @(posedge HCLK or negedge HRESETn)
		if (~HRESETn)
			r_status <= 'h0;
		else if (s_rise_int)
			r_status <= r_status | s_is_int_all;
		else if (((PSEL && PENABLE) && !PWRITE) && (s_apb_addr == 5'b00110))
			r_status <= 'h0;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < 8; _gv_i_2 = _gv_i_2 + 1) begin : genblk1
			localparam i = _gv_i_2;
			pulp_clock_gating i_clk_gate(
				.clk_i(HCLK),
				.en_i(s_clk_en[i]),
				.test_en_i(dft_cg_enable_i),
				.clk_o(s_clkg[i])
			);
		end
	endgenerate
	always @(*) begin : proc_clk_en
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_2
			reg signed [31:0] i;
			for (i = 0; i < 8; i = i + 1)
				s_clk_en[i] = ((r_gpio_en[i * 4] | r_gpio_en[(i * 4) + 1]) | r_gpio_en[(i * 4) + 2]) | r_gpio_en[(i * 4) + 3];
		end
	end
	always @(posedge s_clkg[0] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[3:0] <= 'h0;
			r_gpio_sync1[3:0] <= 'h0;
			r_gpio_in[3:0] <= 'h0;
		end
		else begin
			r_gpio_sync0[3:0] <= gpio_in[3:0];
			r_gpio_sync1[3:0] <= r_gpio_sync0[3:0];
			r_gpio_in[3:0] <= r_gpio_sync1[3:0];
		end
	always @(posedge s_clkg[1] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[7:4] <= 'h0;
			r_gpio_sync1[7:4] <= 'h0;
			r_gpio_in[7:4] <= 'h0;
		end
		else begin
			r_gpio_sync0[7:4] <= gpio_in[7:4];
			r_gpio_sync1[7:4] <= r_gpio_sync0[7:4];
			r_gpio_in[7:4] <= r_gpio_sync1[7:4];
		end
	always @(posedge s_clkg[2] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[11:8] <= 'h0;
			r_gpio_sync1[11:8] <= 'h0;
			r_gpio_in[11:8] <= 'h0;
		end
		else begin
			r_gpio_sync0[11:8] <= gpio_in[11:8];
			r_gpio_sync1[11:8] <= r_gpio_sync0[11:8];
			r_gpio_in[11:8] <= r_gpio_sync1[11:8];
		end
	always @(posedge s_clkg[3] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[15:12] <= 'h0;
			r_gpio_sync1[15:12] <= 'h0;
			r_gpio_in[15:12] <= 'h0;
		end
		else begin
			r_gpio_sync0[15:12] <= gpio_in[15:12];
			r_gpio_sync1[15:12] <= r_gpio_sync0[15:12];
			r_gpio_in[15:12] <= r_gpio_sync1[15:12];
		end
	always @(posedge s_clkg[4] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[19:16] <= 'h0;
			r_gpio_sync1[19:16] <= 'h0;
			r_gpio_in[19:16] <= 'h0;
		end
		else begin
			r_gpio_sync0[19:16] <= gpio_in[19:16];
			r_gpio_sync1[19:16] <= r_gpio_sync0[19:16];
			r_gpio_in[19:16] <= r_gpio_sync1[19:16];
		end
	always @(posedge s_clkg[5] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[23:20] <= 'h0;
			r_gpio_sync1[23:20] <= 'h0;
			r_gpio_in[23:20] <= 'h0;
		end
		else begin
			r_gpio_sync0[23:20] <= gpio_in[23:20];
			r_gpio_sync1[23:20] <= r_gpio_sync0[23:20];
			r_gpio_in[23:20] <= r_gpio_sync1[23:20];
		end
	always @(posedge s_clkg[6] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[27:24] <= 'h0;
			r_gpio_sync1[27:24] <= 'h0;
			r_gpio_in[27:24] <= 'h0;
		end
		else begin
			r_gpio_sync0[27:24] <= gpio_in[27:24];
			r_gpio_sync1[27:24] <= r_gpio_sync0[27:24];
			r_gpio_in[27:24] <= r_gpio_sync1[27:24];
		end
	always @(posedge s_clkg[7] or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_sync0[31:28] <= 'h0;
			r_gpio_sync1[31:28] <= 'h0;
			r_gpio_in[31:28] <= 'h0;
		end
		else begin
			r_gpio_sync0[31:28] <= gpio_in[31:28];
			r_gpio_sync1[31:28] <= r_gpio_sync0[31:28];
			r_gpio_in[31:28] <= r_gpio_sync1[31:28];
		end
	always @(posedge HCLK or negedge HRESETn)
		if (~HRESETn) begin
			r_gpio_inten <= 1'sb0;
			r_gpio_inttype0 <= 1'sb0;
			r_gpio_inttype1 <= 1'sb0;
			r_gpio_out <= 1'sb0;
			r_gpio_dir <= 1'sb0;
			r_gpio_en <= 1'sb0;
			r_gpio_lock <= 1'sb0;
			begin : sv2v_autoblock_3
				reg signed [31:0] i;
				for (i = 0; i < 32; i = i + 1)
					gpio_padcfg[i * 6+:6] <= 6'b000010;
			end
		end
		else if ((PSEL && PENABLE) && PWRITE) begin : sv2v_autoblock_4
			reg signed [31:0] pwdata_l;
			case (s_apb_addr)
				5'b00000: begin
					if (r_gpio_lock[0] == 1'b0)
						pwdata_l = PWDATA;
					else
						pwdata_l = 1'sb0;
					r_gpio_dir <= pwdata_l;
				end
				5'b00010: begin
					if (r_gpio_lock[2] == 1'b0)
						pwdata_l = PWDATA;
					else
						pwdata_l = 1'sb0;
					r_gpio_out <= pwdata_l;
				end
				5'b10000: r_gpio_out <= r_gpio_out | PWDATA;
				5'b10001: r_gpio_out <= r_gpio_out & ~PWDATA;
				5'b00011: r_gpio_inten <= PWDATA;
				5'b00100: r_gpio_inttype0 <= PWDATA;
				5'b00101: r_gpio_inttype1 <= PWDATA;
				5'b00111: r_gpio_en <= PWDATA;
				5'b10010: r_gpio_lock <= PWDATA;
				5'b01000: begin
					gpio_padcfg[0+:6] <= PWDATA[5:0];
					gpio_padcfg[6+:6] <= PWDATA[13:8];
					gpio_padcfg[12+:6] <= PWDATA[21:16];
					gpio_padcfg[18+:6] <= PWDATA[29:24];
				end
				5'b01001: begin
					gpio_padcfg[24+:6] <= PWDATA[5:0];
					gpio_padcfg[30+:6] <= PWDATA[13:8];
					gpio_padcfg[36+:6] <= PWDATA[21:16];
					gpio_padcfg[42+:6] <= PWDATA[29:24];
				end
				5'b01010: begin
					gpio_padcfg[48+:6] <= PWDATA[5:0];
					gpio_padcfg[54+:6] <= PWDATA[13:8];
					gpio_padcfg[60+:6] <= PWDATA[21:16];
					gpio_padcfg[66+:6] <= PWDATA[29:24];
				end
				5'b01011: begin
					gpio_padcfg[72+:6] <= PWDATA[5:0];
					gpio_padcfg[78+:6] <= PWDATA[13:8];
					gpio_padcfg[84+:6] <= PWDATA[21:16];
					gpio_padcfg[90+:6] <= PWDATA[29:24];
				end
				5'b01100: begin
					gpio_padcfg[96+:6] <= PWDATA[5:0];
					gpio_padcfg[102+:6] <= PWDATA[13:8];
					gpio_padcfg[108+:6] <= PWDATA[21:16];
					gpio_padcfg[114+:6] <= PWDATA[29:24];
				end
				5'b01101: begin
					gpio_padcfg[120+:6] <= PWDATA[5:0];
					gpio_padcfg[126+:6] <= PWDATA[13:8];
					gpio_padcfg[132+:6] <= PWDATA[21:16];
					gpio_padcfg[138+:6] <= PWDATA[29:24];
				end
				5'b01110: begin
					gpio_padcfg[144+:6] <= PWDATA[5:0];
					gpio_padcfg[150+:6] <= PWDATA[13:8];
					gpio_padcfg[156+:6] <= PWDATA[21:16];
					gpio_padcfg[162+:6] <= PWDATA[29:24];
				end
				5'b01111: begin
					gpio_padcfg[168+:6] <= PWDATA[5:0];
					gpio_padcfg[174+:6] <= PWDATA[13:8];
					gpio_padcfg[180+:6] <= PWDATA[21:16];
					gpio_padcfg[186+:6] <= PWDATA[29:24];
				end
			endcase
		end
	always @(*) begin
		if (_sv2v_0)
			;
		case (s_apb_addr)
			5'b00000:
				if (r_gpio_lock[0] == 1'b0)
					PRDATA = r_gpio_dir;
				else
					PRDATA = 1'sb0;
			5'b00001:
				if (r_gpio_lock[1] == 1'b0)
					PRDATA = r_gpio_in;
				else
					PRDATA = 1'sb0;
			5'b00010:
				if (r_gpio_lock[2] == 1'b0)
					PRDATA = r_gpio_out;
				else
					PRDATA = 1'sb0;
			5'b00011: PRDATA = r_gpio_inten;
			5'b00100: PRDATA = r_gpio_inttype0;
			5'b00101: PRDATA = r_gpio_inttype1;
			5'b00110: PRDATA = r_status;
			5'b00111: PRDATA = r_gpio_en;
			5'b10010: PRDATA = r_gpio_lock;
			5'b01000: PRDATA = {2'b00, gpio_padcfg[18+:6], 2'b00, gpio_padcfg[12+:6], 2'b00, gpio_padcfg[6+:6], 2'b00, gpio_padcfg[0+:6]};
			5'b01001: PRDATA = {2'b00, gpio_padcfg[42+:6], 2'b00, gpio_padcfg[36+:6], 2'b00, gpio_padcfg[30+:6], 2'b00, gpio_padcfg[24+:6]};
			5'b01010: PRDATA = {2'b00, gpio_padcfg[66+:6], 2'b00, gpio_padcfg[60+:6], 2'b00, gpio_padcfg[54+:6], 2'b00, gpio_padcfg[48+:6]};
			5'b01011: PRDATA = {2'b00, gpio_padcfg[90+:6], 2'b00, gpio_padcfg[84+:6], 2'b00, gpio_padcfg[78+:6], 2'b00, gpio_padcfg[72+:6]};
			5'b01100: PRDATA = {2'b00, gpio_padcfg[114+:6], 2'b00, gpio_padcfg[108+:6], 2'b00, gpio_padcfg[102+:6], 2'b00, gpio_padcfg[96+:6]};
			5'b01101: PRDATA = {2'b00, gpio_padcfg[138+:6], 2'b00, gpio_padcfg[132+:6], 2'b00, gpio_padcfg[126+:6], 2'b00, gpio_padcfg[120+:6]};
			5'b01110: PRDATA = {2'b00, gpio_padcfg[162+:6], 2'b00, gpio_padcfg[156+:6], 2'b00, gpio_padcfg[150+:6], 2'b00, gpio_padcfg[144+:6]};
			5'b01111: PRDATA = {2'b00, gpio_padcfg[186+:6], 2'b00, gpio_padcfg[180+:6], 2'b00, gpio_padcfg[174+:6], 2'b00, gpio_padcfg[168+:6]};
			default: PRDATA = 'h0;
		endcase
	end
	assign gpio_out = r_gpio_out;
	assign gpio_dir = r_gpio_dir;
	assign PREADY = 1'b1;
	assign PSLVERR = 1'b0;
	initial _sv2v_0 = 0;
endmodule
module apb_node (
	penable_i,
	pwrite_i,
	paddr_i,
	pwdata_i,
	prdata_o,
	pready_o,
	pslverr_o,
	penable_o,
	pwrite_o,
	paddr_o,
	psel_o,
	pwdata_o,
	prdata_i,
	pready_i,
	pslverr_i,
	START_ADDR_i,
	END_ADDR_i
);
	reg _sv2v_0;
	parameter NB_MASTER = 8;
	parameter APB_DATA_WIDTH = 32;
	parameter APB_ADDR_WIDTH = 32;
	input wire penable_i;
	input wire pwrite_i;
	input wire [31:0] paddr_i;
	input wire [31:0] pwdata_i;
	output reg [31:0] prdata_o;
	output reg pready_o;
	output reg pslverr_o;
	output reg [NB_MASTER - 1:0] penable_o;
	output reg [NB_MASTER - 1:0] pwrite_o;
	output reg [(NB_MASTER * 32) - 1:0] paddr_o;
	output wire [NB_MASTER - 1:0] psel_o;
	output reg [(NB_MASTER * 32) - 1:0] pwdata_o;
	input wire [(NB_MASTER * 32) - 1:0] prdata_i;
	input wire [NB_MASTER - 1:0] pready_i;
	input wire [NB_MASTER - 1:0] pslverr_i;
	input wire [(NB_MASTER * APB_ADDR_WIDTH) - 1:0] START_ADDR_i;
	input wire [(NB_MASTER * APB_ADDR_WIDTH) - 1:0] END_ADDR_i;
	genvar _gv_i_3;
	integer s_loop1;
	integer s_loop2;
	integer s_loop3;
	integer s_loop4;
	integer s_loop5;
	integer s_loop6;
	integer s_loop7;
	generate
		for (_gv_i_3 = 0; _gv_i_3 < NB_MASTER; _gv_i_3 = _gv_i_3 + 1) begin : genblk1
			localparam i = _gv_i_3;
			assign psel_o[i] = (paddr_i >= START_ADDR_i[i * APB_ADDR_WIDTH+:APB_ADDR_WIDTH]) && (paddr_i <= END_ADDR_i[i * APB_ADDR_WIDTH+:APB_ADDR_WIDTH]);
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		for (s_loop1 = 0; s_loop1 < NB_MASTER; s_loop1 = s_loop1 + 1)
			if (psel_o[s_loop1] == 1'b1)
				penable_o[s_loop1] = penable_i;
			else
				penable_o[s_loop1] = 1'sb0;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		for (s_loop2 = 0; s_loop2 < NB_MASTER; s_loop2 = s_loop2 + 1)
			if (psel_o[s_loop2] == 1'b1)
				pwrite_o[s_loop2] = pwrite_i;
			else
				pwrite_o[s_loop2] = 1'sb0;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		for (s_loop3 = 0; s_loop3 < NB_MASTER; s_loop3 = s_loop3 + 1)
			if (psel_o[s_loop3] == 1'b1)
				paddr_o[s_loop3 * 32+:32] = paddr_i;
			else
				paddr_o[s_loop3 * 32+:32] = 1'sb0;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		for (s_loop4 = 0; s_loop4 < NB_MASTER; s_loop4 = s_loop4 + 1)
			if (psel_o[s_loop4] == 1'b1)
				pwdata_o[s_loop4 * 32+:32] = pwdata_i;
			else
				pwdata_o[s_loop4 * 32+:32] = 1'sb0;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		prdata_o = 1'sb0;
		for (s_loop5 = 0; s_loop5 < NB_MASTER; s_loop5 = s_loop5 + 1)
			if (psel_o[s_loop5] == 1'b1)
				prdata_o = prdata_i[s_loop5 * 32+:32];
	end
	always @(*) begin
		if (_sv2v_0)
			;
		pready_o = 1'sb0;
		for (s_loop6 = 0; s_loop6 < NB_MASTER; s_loop6 = s_loop6 + 1)
			if (psel_o[s_loop6] == 1'b1)
				pready_o = pready_i[s_loop6];
	end
	always @(*) begin
		if (_sv2v_0)
			;
		pslverr_o = 1'sb0;
		for (s_loop7 = 0; s_loop7 < NB_MASTER; s_loop7 = s_loop7 + 1)
			if (psel_o[s_loop7] == 1'b1)
				pslverr_o = pslverr_i[s_loop7];
	end
	initial _sv2v_0 = 0;
endmodule
module axi_address_decoder_AR (
	clk,
	rst_n,
	arvalid_i,
	araddr_i,
	arready_o,
	arvalid_o,
	arready_i,
	START_ADDR_i,
	END_ADDR_i,
	enable_region_i,
	connectivity_map_i,
	incr_req_o,
	full_counter_i,
	outstanding_trans_i,
	error_req_o,
	error_gnt_i,
	sample_ardata_info_o
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 32;
	parameter N_INIT_PORT = 8;
	parameter N_REGION = 4;
	input wire clk;
	input wire rst_n;
	input wire arvalid_i;
	input wire [ADDR_WIDTH - 1:0] araddr_i;
	output reg arready_o;
	output reg [N_INIT_PORT - 1:0] arvalid_o;
	input wire [N_INIT_PORT - 1:0] arready_i;
	input wire [((N_REGION * N_INIT_PORT) * ADDR_WIDTH) - 1:0] START_ADDR_i;
	input wire [((N_REGION * N_INIT_PORT) * ADDR_WIDTH) - 1:0] END_ADDR_i;
	input wire [(N_REGION * N_INIT_PORT) - 1:0] enable_region_i;
	input wire [N_INIT_PORT - 1:0] connectivity_map_i;
	output reg incr_req_o;
	input wire full_counter_i;
	input wire outstanding_trans_i;
	output reg error_req_o;
	input wire error_gnt_i;
	output reg sample_ardata_info_o;
	wire [N_INIT_PORT - 1:0] match_region;
	wire [N_INIT_PORT:0] match_region_masked;
	wire [(N_REGION * N_INIT_PORT) - 1:0] match_region_int;
	wire [(N_INIT_PORT * N_REGION) - 1:0] match_region_rev;
	reg arready_int;
	reg [N_INIT_PORT - 1:0] arvalid_int;
	genvar _gv_i_5;
	genvar _gv_j_13;
	reg CS;
	reg NS;
	generate
		for (_gv_j_13 = 0; _gv_j_13 < N_REGION; _gv_j_13 = _gv_j_13 + 1) begin : genblk1
			localparam j = _gv_j_13;
			for (_gv_i_5 = 0; _gv_i_5 < N_INIT_PORT; _gv_i_5 = _gv_i_5 + 1) begin : genblk1
				localparam i = _gv_i_5;
				assign match_region_int[(j * N_INIT_PORT) + i] = (enable_region_i[(j * N_INIT_PORT) + i] == 1'b1 ? (araddr_i >= START_ADDR_i[((j * N_INIT_PORT) + i) * ADDR_WIDTH+:ADDR_WIDTH]) && (araddr_i <= END_ADDR_i[((j * N_INIT_PORT) + i) * ADDR_WIDTH+:ADDR_WIDTH]) : 1'b0);
			end
		end
		for (_gv_j_13 = 0; _gv_j_13 < N_INIT_PORT; _gv_j_13 = _gv_j_13 + 1) begin : genblk2
			localparam j = _gv_j_13;
			for (_gv_i_5 = 0; _gv_i_5 < N_REGION; _gv_i_5 = _gv_i_5 + 1) begin : genblk1
				localparam i = _gv_i_5;
				assign match_region_rev[(j * N_REGION) + i] = match_region_int[(i * N_INIT_PORT) + j];
			end
		end
		for (_gv_i_5 = 0; _gv_i_5 < N_INIT_PORT; _gv_i_5 = _gv_i_5 + 1) begin : genblk3
			localparam i = _gv_i_5;
			assign match_region[i] = |match_region_rev[i * N_REGION+:N_REGION];
		end
	endgenerate
	assign match_region_masked[N_INIT_PORT - 1:0] = match_region & connectivity_map_i;
	assign match_region_masked[N_INIT_PORT] = ~(|match_region_masked[N_INIT_PORT - 1:0]);
	always @(*) begin
		if (_sv2v_0)
			;
		if (arvalid_i)
			{error_req_o, arvalid_int} = {N_INIT_PORT + 1 {arvalid_i}} & match_region_masked;
		else begin
			arvalid_int = 1'sb0;
			error_req_o = 1'b0;
		end
		arready_int = |({error_gnt_i, arready_i} & match_region_masked);
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			CS <= 1'd0;
		else
			CS <= NS;
	always @(*) begin
		if (_sv2v_0)
			;
		arready_o = 1'b0;
		arvalid_o = arvalid_int;
		sample_ardata_info_o = 1'b0;
		incr_req_o = 1'b0;
		case (CS)
			1'd0:
				if (error_req_o) begin
					NS = 1'd1;
					arready_o = 1'b1;
					sample_ardata_info_o = 1'b1;
					arvalid_o = 1'sb0;
				end
				else begin
					NS = 1'd0;
					arready_o = arready_int;
					sample_ardata_info_o = 1'b0;
					incr_req_o = |(arvalid_o & arready_i);
					arvalid_o = arvalid_int;
				end
			1'd1: begin
				arready_o = 1'b0;
				arvalid_o = 1'sb0;
				if (outstanding_trans_i)
					NS = 1'd0;
				else if (error_gnt_i)
					NS = 1'd0;
				else
					NS = 1'd1;
			end
			default: begin
				NS = 1'd0;
				arready_o = arready_int;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module axi_ar_buffer (
	clk_i,
	rst_ni,
	test_en_i,
	slave_valid_i,
	slave_addr_i,
	slave_prot_i,
	slave_region_i,
	slave_len_i,
	slave_size_i,
	slave_burst_i,
	slave_lock_i,
	slave_cache_i,
	slave_qos_i,
	slave_id_i,
	slave_user_i,
	slave_ready_o,
	master_valid_o,
	master_addr_o,
	master_prot_o,
	master_region_o,
	master_len_o,
	master_size_o,
	master_burst_o,
	master_lock_o,
	master_cache_o,
	master_qos_o,
	master_id_o,
	master_user_o,
	master_ready_i
);
	parameter ID_WIDTH = 4;
	parameter ADDR_WIDTH = 32;
	parameter USER_WIDTH = 6;
	parameter BUFFER_DEPTH = 2;
	input wire clk_i;
	input wire rst_ni;
	input wire test_en_i;
	input wire slave_valid_i;
	input wire [ADDR_WIDTH - 1:0] slave_addr_i;
	input wire [2:0] slave_prot_i;
	input wire [3:0] slave_region_i;
	input wire [7:0] slave_len_i;
	input wire [2:0] slave_size_i;
	input wire [1:0] slave_burst_i;
	input wire slave_lock_i;
	input wire [3:0] slave_cache_i;
	input wire [3:0] slave_qos_i;
	input wire [ID_WIDTH - 1:0] slave_id_i;
	input wire [USER_WIDTH - 1:0] slave_user_i;
	output wire slave_ready_o;
	output wire master_valid_o;
	output wire [ADDR_WIDTH - 1:0] master_addr_o;
	output wire [2:0] master_prot_o;
	output wire [3:0] master_region_o;
	output wire [7:0] master_len_o;
	output wire [2:0] master_size_o;
	output wire [1:0] master_burst_o;
	output wire master_lock_o;
	output wire [3:0] master_cache_o;
	output wire [3:0] master_qos_o;
	output wire [ID_WIDTH - 1:0] master_id_o;
	output wire [USER_WIDTH - 1:0] master_user_o;
	input wire master_ready_i;
	wire [(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_in;
	wire [(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_out;
	assign s_data_in = {slave_cache_i, slave_prot_i, slave_lock_i, slave_burst_i, slave_size_i, slave_len_i, slave_qos_i, slave_region_i, slave_addr_i, slave_user_i, slave_id_i};
	assign {master_cache_o, master_prot_o, master_lock_o, master_burst_o, master_size_o, master_len_o, master_qos_o, master_region_o, master_addr_o, master_user_o, master_id_o} = s_data_out;
	generic_fifo #(
		.DATA_WIDTH(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH),
		.DATA_DEPTH(BUFFER_DEPTH)
	) buffer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.data_i(s_data_in),
		.valid_i(slave_valid_i),
		.grant_o(slave_ready_o),
		.data_o(s_data_out),
		.valid_o(master_valid_o),
		.grant_i(master_ready_i),
		.test_mode_i(test_en_i)
	);
endmodule
module axi_aw_buffer (
	clk_i,
	rst_ni,
	test_en_i,
	slave_valid_i,
	slave_addr_i,
	slave_prot_i,
	slave_region_i,
	slave_len_i,
	slave_size_i,
	slave_burst_i,
	slave_lock_i,
	slave_cache_i,
	slave_qos_i,
	slave_id_i,
	slave_user_i,
	slave_ready_o,
	master_valid_o,
	master_addr_o,
	master_prot_o,
	master_region_o,
	master_len_o,
	master_size_o,
	master_burst_o,
	master_lock_o,
	master_cache_o,
	master_qos_o,
	master_id_o,
	master_user_o,
	master_ready_i
);
	parameter ID_WIDTH = 4;
	parameter ADDR_WIDTH = 32;
	parameter USER_WIDTH = 6;
	parameter BUFFER_DEPTH = 2;
	input wire clk_i;
	input wire rst_ni;
	input wire test_en_i;
	input wire slave_valid_i;
	input wire [ADDR_WIDTH - 1:0] slave_addr_i;
	input wire [2:0] slave_prot_i;
	input wire [3:0] slave_region_i;
	input wire [7:0] slave_len_i;
	input wire [2:0] slave_size_i;
	input wire [1:0] slave_burst_i;
	input wire slave_lock_i;
	input wire [3:0] slave_cache_i;
	input wire [3:0] slave_qos_i;
	input wire [ID_WIDTH - 1:0] slave_id_i;
	input wire [USER_WIDTH - 1:0] slave_user_i;
	output wire slave_ready_o;
	output wire master_valid_o;
	output wire [ADDR_WIDTH - 1:0] master_addr_o;
	output wire [2:0] master_prot_o;
	output wire [3:0] master_region_o;
	output wire [7:0] master_len_o;
	output wire [2:0] master_size_o;
	output wire [1:0] master_burst_o;
	output wire master_lock_o;
	output wire [3:0] master_cache_o;
	output wire [3:0] master_qos_o;
	output wire [ID_WIDTH - 1:0] master_id_o;
	output wire [USER_WIDTH - 1:0] master_user_o;
	input wire master_ready_i;
	wire [(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_in;
	wire [(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_out;
	assign s_data_in = {slave_cache_i, slave_prot_i, slave_lock_i, slave_burst_i, slave_size_i, slave_len_i, slave_qos_i, slave_region_i, slave_addr_i, slave_user_i, slave_id_i};
	assign {master_cache_o, master_prot_o, master_lock_o, master_burst_o, master_size_o, master_len_o, master_qos_o, master_region_o, master_addr_o, master_user_o, master_id_o} = s_data_out;
	generic_fifo #(
		.DATA_WIDTH(((29 + ADDR_WIDTH) + USER_WIDTH) + ID_WIDTH),
		.DATA_DEPTH(BUFFER_DEPTH)
	) buffer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.data_i(s_data_in),
		.valid_i(slave_valid_i),
		.grant_o(slave_ready_o),
		.data_o(s_data_out),
		.valid_o(master_valid_o),
		.grant_i(master_ready_i),
		.test_mode_i(test_en_i)
	);
endmodule
module axi_b_buffer (
	clk_i,
	rst_ni,
	test_en_i,
	slave_valid_i,
	slave_resp_i,
	slave_id_i,
	slave_user_i,
	slave_ready_o,
	master_valid_o,
	master_resp_o,
	master_id_o,
	master_user_o,
	master_ready_i
);
	parameter ID_WIDTH = 4;
	parameter USER_WIDTH = 6;
	parameter BUFFER_DEPTH = 8;
	input wire clk_i;
	input wire rst_ni;
	input wire test_en_i;
	input wire slave_valid_i;
	input wire [1:0] slave_resp_i;
	input wire [ID_WIDTH - 1:0] slave_id_i;
	input wire [USER_WIDTH - 1:0] slave_user_i;
	output wire slave_ready_o;
	output wire master_valid_o;
	output wire [1:0] master_resp_o;
	output wire [ID_WIDTH - 1:0] master_id_o;
	output wire [USER_WIDTH - 1:0] master_user_o;
	input wire master_ready_i;
	wire [((2 + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_in;
	wire [((2 + USER_WIDTH) + ID_WIDTH) - 1:0] s_data_out;
	assign s_data_in = {slave_id_i, slave_user_i, slave_resp_i};
	assign {master_id_o, master_user_o, master_resp_o} = s_data_out;
	generic_fifo #(
		.DATA_WIDTH((2 + USER_WIDTH) + ID_WIDTH),
		.DATA_DEPTH(BUFFER_DEPTH)
	) buffer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.data_i(s_data_in),
		.valid_i(slave_valid_i),
		.grant_o(slave_ready_o),
		.data_o(s_data_out),
		.valid_o(master_valid_o),
		.grant_i(master_ready_i),
		.test_mode_i(test_en_i)
	);
endmodule
module axi_r_buffer (
	clk_i,
	rst_ni,
	test_en_i,
	slave_valid_i,
	slave_data_i,
	slave_resp_i,
	slave_user_i,
	slave_id_i,
	slave_last_i,
	slave_ready_o,
	master_valid_o,
	master_data_o,
	master_resp_o,
	master_user_o,
	master_id_o,
	master_last_o,
	master_ready_i
);
	parameter ID_WIDTH = 4;
	parameter DATA_WIDTH = 64;
	parameter USER_WIDTH = 6;
	parameter BUFFER_DEPTH = 8;
	parameter STRB_WIDTH = DATA_WIDTH / 8;
	input wire clk_i;
	input wire rst_ni;
	input wire test_en_i;
	input wire slave_valid_i;
	input wire [DATA_WIDTH - 1:0] slave_data_i;
	input wire [1:0] slave_resp_i;
	input wire [USER_WIDTH - 1:0] slave_user_i;
	input wire [ID_WIDTH - 1:0] slave_id_i;
	input wire slave_last_i;
	output wire slave_ready_o;
	output wire master_valid_o;
	output wire [DATA_WIDTH - 1:0] master_data_o;
	output wire [1:0] master_resp_o;
	output wire [USER_WIDTH - 1:0] master_user_o;
	output wire [ID_WIDTH - 1:0] master_id_o;
	output wire master_last_o;
	input wire master_ready_i;
	wire [((2 + DATA_WIDTH) + USER_WIDTH) + ID_WIDTH:0] s_data_in;
	wire [((2 + DATA_WIDTH) + USER_WIDTH) + ID_WIDTH:0] s_data_out;
	assign s_data_in = {slave_id_i, slave_user_i, slave_data_i, slave_resp_i, slave_last_i};
	assign {master_id_o, master_user_o, master_data_o, master_resp_o, master_last_o} = s_data_out;
	generic_fifo #(
		.DATA_WIDTH(((3 + DATA_WIDTH) + USER_WIDTH) + ID_WIDTH),
		.DATA_DEPTH(BUFFER_DEPTH)
	) buffer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.data_i(s_data_in),
		.valid_i(slave_valid_i),
		.grant_o(slave_ready_o),
		.data_o(s_data_out),
		.valid_o(master_valid_o),
		.grant_i(master_ready_i),
		.test_mode_i(test_en_i)
	);
endmodule
module axi_w_buffer (
	clk_i,
	rst_ni,
	test_en_i,
	slave_valid_i,
	slave_data_i,
	slave_strb_i,
	slave_user_i,
	slave_last_i,
	slave_ready_o,
	master_valid_o,
	master_data_o,
	master_strb_o,
	master_user_o,
	master_last_o,
	master_ready_i
);
	parameter DATA_WIDTH = 64;
	parameter USER_WIDTH = 6;
	parameter BUFFER_DEPTH = 2;
	parameter STRB_WIDTH = DATA_WIDTH / 8;
	input wire clk_i;
	input wire rst_ni;
	input wire test_en_i;
	input wire slave_valid_i;
	input wire [DATA_WIDTH - 1:0] slave_data_i;
	input wire [STRB_WIDTH - 1:0] slave_strb_i;
	input wire [USER_WIDTH - 1:0] slave_user_i;
	input wire slave_last_i;
	output wire slave_ready_o;
	output wire master_valid_o;
	output wire [DATA_WIDTH - 1:0] master_data_o;
	output wire [STRB_WIDTH - 1:0] master_strb_o;
	output wire [USER_WIDTH - 1:0] master_user_o;
	output wire master_last_o;
	input wire master_ready_i;
	wire [(DATA_WIDTH + STRB_WIDTH) + USER_WIDTH:0] s_data_in;
	wire [(DATA_WIDTH + STRB_WIDTH) + USER_WIDTH:0] s_data_out;
	assign s_data_in = {slave_user_i, slave_strb_i, slave_data_i, slave_last_i};
	assign {master_user_o, master_strb_o, master_data_o, master_last_o} = s_data_out;
	generic_fifo #(
		.DATA_WIDTH(((1 + DATA_WIDTH) + STRB_WIDTH) + USER_WIDTH),
		.DATA_DEPTH(BUFFER_DEPTH)
	) buffer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.data_i(s_data_in),
		.valid_i(slave_valid_i),
		.grant_o(slave_ready_o),
		.data_o(s_data_out),
		.valid_o(master_valid_o),
		.grant_i(master_ready_i),
		.test_mode_i(test_en_i)
	);
endmodule
module generic_fifo (
	clk,
	rst_n,
	data_i,
	valid_i,
	grant_o,
	data_o,
	valid_o,
	grant_i,
	test_mode_i
);
	reg _sv2v_0;
	parameter [31:0] DATA_WIDTH = 32;
	parameter [31:0] DATA_DEPTH = 8;
	input wire clk;
	input wire rst_n;
	input wire [DATA_WIDTH - 1:0] data_i;
	input wire valid_i;
	output reg grant_o;
	output wire [DATA_WIDTH - 1:0] data_o;
	output reg valid_o;
	input wire grant_i;
	input wire test_mode_i;
	localparam [31:0] ADDR_DEPTH = $clog2(DATA_DEPTH);
	reg [1:0] CS;
	reg [1:0] NS;
	reg gate_clock;
	wire clk_gated;
	reg [ADDR_DEPTH - 1:0] Pop_Pointer_CS;
	reg [ADDR_DEPTH - 1:0] Pop_Pointer_NS;
	reg [ADDR_DEPTH - 1:0] Push_Pointer_CS;
	reg [ADDR_DEPTH - 1:0] Push_Pointer_NS;
	reg [DATA_WIDTH - 1:0] FIFO_REGISTERS [DATA_DEPTH - 1:0];
	reg [31:0] i;
	initial begin : parameter_check
		integer param_err_flg;
		param_err_flg = 0;
		if (DATA_WIDTH < 1) begin
			param_err_flg = 1;
			$display("ERROR: %m :\n  Invalid value (%d) for parameter DATA_WIDTH (legal range: greater than 1)", DATA_WIDTH);
		end
		if (DATA_DEPTH < 1) begin
			param_err_flg = 1;
			$display("ERROR: %m :\n  Invalid value (%d) for parameter DATA_DEPTH (legal range: greater than 1)", DATA_DEPTH);
		end
	end
	cluster_clock_gating cg_cell(
		.clk_i(clk),
		.en_i(~gate_clock),
		.test_en_i(test_mode_i),
		.clk_o(clk_gated)
	);
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			CS <= 2'd0;
			Pop_Pointer_CS <= {ADDR_DEPTH {1'b0}};
			Push_Pointer_CS <= {ADDR_DEPTH {1'b0}};
		end
		else begin
			CS <= NS;
			Pop_Pointer_CS <= Pop_Pointer_NS;
			Push_Pointer_CS <= Push_Pointer_NS;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		gate_clock = 1'b0;
		case (CS)
			2'd0: begin
				grant_o = 1'b1;
				valid_o = 1'b0;
				case (valid_i)
					1'b0: begin
						NS = 2'd0;
						Push_Pointer_NS = Push_Pointer_CS;
						Pop_Pointer_NS = Pop_Pointer_CS;
						gate_clock = 1'b1;
					end
					1'b1: begin
						NS = 2'd2;
						Push_Pointer_NS = Push_Pointer_CS + 1'b1;
						Pop_Pointer_NS = Pop_Pointer_CS;
					end
				endcase
			end
			2'd2: begin
				grant_o = 1'b1;
				valid_o = 1'b1;
				case ({valid_i, grant_i})
					2'b01: begin
						gate_clock = 1'b1;
						if ((Pop_Pointer_CS == (Push_Pointer_CS - 1)) || ((Pop_Pointer_CS == (DATA_DEPTH - 1)) && (Push_Pointer_CS == 0)))
							NS = 2'd0;
						else
							NS = 2'd2;
						Push_Pointer_NS = Push_Pointer_CS;
						if (Pop_Pointer_CS == (DATA_DEPTH - 1))
							Pop_Pointer_NS = 0;
						else
							Pop_Pointer_NS = Pop_Pointer_CS + 1'b1;
					end
					2'b00: begin
						gate_clock = 1'b1;
						NS = 2'd2;
						Push_Pointer_NS = Push_Pointer_CS;
						Pop_Pointer_NS = Pop_Pointer_CS;
					end
					2'b11: begin
						NS = 2'd2;
						if (Push_Pointer_CS == (DATA_DEPTH - 1))
							Push_Pointer_NS = 0;
						else
							Push_Pointer_NS = Push_Pointer_CS + 1'b1;
						if (Pop_Pointer_CS == (DATA_DEPTH - 1))
							Pop_Pointer_NS = 0;
						else
							Pop_Pointer_NS = Pop_Pointer_CS + 1'b1;
					end
					2'b10: begin
						if ((Push_Pointer_CS == (Pop_Pointer_CS - 1)) || ((Push_Pointer_CS == (DATA_DEPTH - 1)) && (Pop_Pointer_CS == 0)))
							NS = 2'd1;
						else
							NS = 2'd2;
						if (Push_Pointer_CS == (DATA_DEPTH - 1))
							Push_Pointer_NS = 0;
						else
							Push_Pointer_NS = Push_Pointer_CS + 1'b1;
						Pop_Pointer_NS = Pop_Pointer_CS;
					end
				endcase
			end
			2'd1: begin
				grant_o = 1'b0;
				valid_o = 1'b1;
				gate_clock = 1'b1;
				case (grant_i)
					1'b1: begin
						NS = 2'd2;
						Push_Pointer_NS = Push_Pointer_CS;
						if (Pop_Pointer_CS == (DATA_DEPTH - 1))
							Pop_Pointer_NS = 0;
						else
							Pop_Pointer_NS = Pop_Pointer_CS + 1'b1;
					end
					1'b0: begin
						NS = 2'd1;
						Push_Pointer_NS = Push_Pointer_CS;
						Pop_Pointer_NS = Pop_Pointer_CS;
					end
				endcase
			end
			default: begin
				gate_clock = 1'b1;
				grant_o = 1'b0;
				valid_o = 1'b0;
				NS = 2'd0;
				Pop_Pointer_NS = 0;
				Push_Pointer_NS = 0;
			end
		endcase
	end
	always @(posedge clk_gated or negedge rst_n)
		if (rst_n == 1'b0)
			for (i = 0; i < DATA_DEPTH; i = i + 1)
				FIFO_REGISTERS[i] <= {DATA_WIDTH {1'b0}};
		else if ((grant_o == 1'b1) && (valid_i == 1'b1))
			FIFO_REGISTERS[Push_Pointer_CS] <= data_i;
	assign data_o = FIFO_REGISTERS[Pop_Pointer_CS];
	initial _sv2v_0 = 0;
endmodule
module AddRoundKey (
	x,
	y,
	z
);
	input [127:0] x;
	input [127:0] y;
	output wire [127:0] z;
	assign z = x ^ y;
endmodule
module KeyExpansion (
	key,
	expandedKey
);
	localparam NB = 4;
	localparam NK = 8;
	localparam NR = 14;
	input [255:0] key;
	output wire [1919:0] expandedKey;
	wire [31:0] w [119:0];
	wire [31:0] t [13:0];
	wire [31:0] tt [59:0];
	wire [7:0] RCon [13:0];
	wire [31:0] rotWord [13:0];
	wire [31:0] Q [13:0];
	wire [95:0] unused [13:0];
	wire [95:0] unused2 [59:0];
	assign RCon[0] = 8'h01;
	assign RCon[1] = 8'h02;
	assign RCon[2] = 8'h04;
	assign RCon[3] = 8'h08;
	assign RCon[4] = 8'h10;
	assign RCon[5] = 8'h20;
	assign RCon[6] = 8'h40;
	assign RCon[7] = 8'h80;
	assign RCon[8] = 8'h1b;
	assign RCon[9] = 8'h36;
	assign RCon[10] = 8'h6c;
	assign RCon[11] = 8'hd8;
	assign RCon[12] = 8'hab;
	assign RCon[13] = 8'h4d;
	genvar _gv_i_6;
	generate
		for (_gv_i_6 = 0; _gv_i_6 < 60; _gv_i_6 = _gv_i_6 + 1) begin : EXPANDKEY
			localparam i = _gv_i_6;
			assign expandedKey[(32 * (i + 1)) - 1:32 * i] = w[i];
		end
		for (_gv_i_6 = 0; _gv_i_6 < 60; _gv_i_6 = _gv_i_6 + 1) begin : NR_W
			localparam i = _gv_i_6;
			if (i < NK) begin : INITIAL
				assign w[i] = key[(32 * (i + 1)) - 1:32 * i];
			end
			else begin : NEXT
				if ((i % NK) == 0) begin : FIRST
					assign w[i] = w[i - NK] ^ t[(i / NK) - 1];
				end
				else begin : NEXT
					if ((i % NK) == 4) begin : NEXT
						assign w[i] = w[i - NK] ^ tt[i];
					end
					else begin : genblk1
						assign w[i] = w[i - 1] ^ w[i - NK];
					end
				end
			end
		end
		for (_gv_i_6 = 0; _gv_i_6 < NR; _gv_i_6 = _gv_i_6 + 1) begin : COREKEY
			localparam i = _gv_i_6;
			assign rotWord[i] = {w[(NK + (NK * i)) - 1][7:0], w[(NK + (NK * i)) - 1][31:8]};
			SubBytes b(
				.x({rotWord[i], 96'b000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000}),
				.z({Q[i], unused[i]})
			);
			assign t[i] = {Q[i][31:8], RCon[i] ^ Q[i][7:0]};
		end
		for (_gv_i_6 = 0; _gv_i_6 < 60; _gv_i_6 = _gv_i_6 + 1) begin : SBOX_ONLY
			localparam i = _gv_i_6;
			if ((i % NK) == 4) begin : genblk1
				SubBytes c(
					.x({w[i - 1], 96'b000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000}),
					.z({tt[i], unused2[i]})
				);
			end
		end
	endgenerate
endmodule
module MixColumns (
	x,
	z
);
	input [127:0] x;
	output reg [127:0] z;
	function [7:0] xtime;
		input [7:0] a;
		xtime = (a & 8'h80 ? (a << 1) ^ 8'h1b : a << 1);
	endfunction
	genvar _gv_i_7;
	generate
		for (_gv_i_7 = 0; _gv_i_7 < 4; _gv_i_7 = _gv_i_7 + 1) begin : gen_loop_enc
			localparam i = _gv_i_7;
			always @(x) begin : enc_colMix
				reg [7:0] a0;
				reg [7:0] a1;
				reg [7:0] a2;
				reg [7:0] a3;
				reg [7:0] temp;
				reg [7:0] b0;
				reg [7:0] b1;
				reg [7:0] b2;
				reg [7:0] b3;
				a0 = x[(8 * ((4 * i) + 1)) - 1:8 * ((4 * i) + 0)];
				a1 = x[(8 * ((4 * i) + 2)) - 1:8 * ((4 * i) + 1)];
				a2 = x[(8 * ((4 * i) + 3)) - 1:8 * ((4 * i) + 2)];
				a3 = x[(8 * ((4 * i) + 4)) - 1:8 * ((4 * i) + 3)];
				temp = ((a0 ^ a1) ^ a2) ^ a3;
				b0 = (a0 ^ temp) ^ xtime(a0 ^ a1);
				b1 = (a1 ^ temp) ^ xtime(a1 ^ a2);
				b2 = (a2 ^ temp) ^ xtime(a2 ^ a3);
				b3 = (a3 ^ temp) ^ xtime(a3 ^ a0);
				z[(8 * ((4 * i) + 1)) - 1:8 * ((4 * i) + 0)] = b0;
				z[(8 * ((4 * i) + 2)) - 1:8 * ((4 * i) + 1)] = b1;
				z[(8 * ((4 * i) + 3)) - 1:8 * ((4 * i) + 2)] = b2;
				z[(8 * ((4 * i) + 4)) - 1:8 * ((4 * i) + 3)] = b3;
			end
		end
	endgenerate
endmodule
module ShiftRows (
	x,
	z
);
	input [127:0] x;
	output wire [127:0] z;
	wire [127:0] xr;
	wire [127:0] zr;
	genvar _gv_i_8;
	generate
		for (_gv_i_8 = 0; _gv_i_8 < 128; _gv_i_8 = _gv_i_8 + 1) begin : REV
			localparam i = _gv_i_8;
			assign xr[127 - i] = x[i];
			assign z[i] = zr[127 - i];
		end
	endgenerate
	wire w0;
	wire w1;
	wire w2;
	wire w3;
	wire w4;
	wire w5;
	wire w6;
	wire w7;
	wire w8;
	wire w9;
	wire w10;
	wire w11;
	wire w12;
	wire w13;
	wire w14;
	wire w15;
	wire w16;
	wire w17;
	wire w18;
	wire w19;
	wire w20;
	wire w21;
	wire w22;
	wire w23;
	wire w24;
	wire w25;
	wire w26;
	wire w27;
	wire w28;
	wire w29;
	wire w30;
	wire w31;
	wire w32;
	wire w33;
	wire w34;
	wire w35;
	wire w36;
	wire w37;
	wire w38;
	wire w39;
	wire w40;
	wire w41;
	wire w42;
	wire w43;
	wire w44;
	wire w45;
	wire w46;
	wire w47;
	wire w48;
	wire w49;
	wire w50;
	wire w51;
	wire w52;
	wire w53;
	wire w54;
	wire w55;
	wire w56;
	wire w57;
	wire w58;
	wire w59;
	wire w60;
	wire w61;
	wire w62;
	wire w63;
	wire w64;
	wire w65;
	wire w66;
	wire w67;
	wire w68;
	wire w69;
	wire w70;
	wire w71;
	wire w72;
	wire w73;
	wire w74;
	wire w75;
	wire w76;
	wire w77;
	wire w78;
	wire w79;
	wire w80;
	wire w81;
	wire w82;
	wire w83;
	wire w84;
	wire w85;
	wire w86;
	wire w87;
	wire w88;
	wire w89;
	wire w90;
	wire w91;
	wire w92;
	wire w93;
	wire w94;
	wire w95;
	wire w96;
	wire w97;
	wire w98;
	wire w99;
	wire w100;
	wire w101;
	wire w102;
	wire w103;
	wire w104;
	wire w105;
	wire w106;
	wire w107;
	wire w108;
	wire w109;
	wire w110;
	wire w111;
	wire w112;
	wire w113;
	wire w114;
	wire w115;
	wire w116;
	wire w117;
	wire w118;
	wire w119;
	wire w120;
	wire w121;
	wire w122;
	wire w123;
	wire w124;
	wire w125;
	wire w126;
	wire w127;
	wire w128;
	assign {w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, w26, w27, w28, w29, w30, w31, w32, w33, w34, w35, w36, w37, w38, w39, w40, w41, w42, w43, w44, w45, w46, w47, w48, w49, w50, w51, w52, w53, w54, w55, w56, w57, w58, w59, w60, w61, w62, w63, w64, w65, w66, w67, w68, w69, w70, w71, w72, w73, w74, w75, w76, w77, w78, w79, w80, w81, w82, w83, w84, w85, w86, w87, w88, w89, w90, w91, w92, w93, w94, w95, w96, w97, w98, w99, w100, w101, w102, w103, w104, w105, w106, w107, w108, w109, w110, w111, w112, w113, w114, w115, w116, w117, w118, w119, w120, w121, w122, w123, w124, w125, w126, w127} = xr;
	assign zr = {w0, w1, w2, w3, w4, w5, w6, w7, w40, w41, w42, w43, w44, w45, w46, w47, w80, w81, w82, w83, w84, w85, w86, w87, w120, w121, w122, w123, w124, w125, w126, w127, w32, w33, w34, w35, w36, w37, w38, w39, w72, w73, w74, w75, w76, w77, w78, w79, w112, w113, w114, w115, w116, w117, w118, w119, w24, w25, w26, w27, w28, w29, w30, w31, w64, w65, w66, w67, w68, w69, w70, w71, w104, w105, w106, w107, w108, w109, w110, w111, w16, w17, w18, w19, w20, w21, w22, w23, w56, w57, w58, w59, w60, w61, w62, w63, w96, w97, w98, w99, w100, w101, w102, w103, w8, w9, w10, w11, w12, w13, w14, w15, w48, w49, w50, w51, w52, w53, w54, w55, w88, w89, w90, w91, w92, w93, w94, w95};
endmodule
module sbox (
	SubByte,
	num
);
	output wire [7:0] SubByte;
	input [7:0] num;
	wire [0:7] s;
	wire [0:7] x;
	assign x = num;
	assign SubByte = s;
	wire [21:0] y;
	wire [67:0] t;
	wire [17:0] z;
	assign y[14] = x[3] ^ x[5];
	assign y[13] = x[0] ^ x[6];
	assign y[9] = x[0] ^ x[3];
	assign y[8] = x[0] ^ x[5];
	assign t[0] = x[1] ^ x[2];
	assign y[1] = t[0] ^ x[7];
	assign y[4] = y[1] ^ x[3];
	assign y[12] = y[13] ^ y[14];
	assign y[2] = y[1] ^ x[0];
	assign y[5] = y[1] ^ x[6];
	assign y[3] = y[5] ^ y[8];
	assign t[1] = x[4] ^ y[12];
	assign y[15] = t[1] ^ x[5];
	assign y[20] = t[1] ^ x[1];
	assign y[6] = y[15] ^ x[7];
	assign y[10] = y[15] ^ t[0];
	assign y[11] = y[20] ^ y[9];
	assign y[7] = x[7] ^ y[11];
	assign y[17] = y[10] ^ y[11];
	assign y[19] = y[10] ^ y[8];
	assign y[16] = t[0] ^ y[11];
	assign y[21] = y[13] ^ y[16];
	assign y[18] = x[0] ^ y[16];
	assign t[2] = y[12] & y[15];
	assign t[3] = y[3] & y[6];
	assign t[4] = t[3] ^ t[2];
	assign t[5] = y[4] & x[7];
	assign t[6] = t[5] ^ t[2];
	assign t[7] = y[13] & y[16];
	assign t[8] = y[5] & y[1];
	assign t[9] = t[8] ^ t[7];
	assign t[10] = y[2] & y[7];
	assign t[11] = t[10] ^ t[7];
	assign t[12] = y[9] & y[11];
	assign t[13] = y[14] & y[17];
	assign t[14] = t[13] ^ t[12];
	assign t[15] = y[8] & y[10];
	assign t[16] = t[15] ^ t[12];
	assign t[17] = t[4] ^ t[14];
	assign t[18] = t[6] ^ t[16];
	assign t[19] = t[9] ^ t[14];
	assign t[20] = t[11] ^ t[16];
	assign t[21] = t[17] ^ y[20];
	assign t[22] = t[18] ^ y[19];
	assign t[23] = t[19] ^ y[21];
	assign t[24] = t[20] ^ y[18];
	assign t[25] = t[21] ^ t[22];
	assign t[26] = t[21] & t[23];
	assign t[27] = t[24] ^ t[26];
	assign t[28] = t[25] & t[27];
	assign t[29] = t[28] ^ t[22];
	assign t[30] = t[23] ^ t[24];
	assign t[31] = t[22] ^ t[26];
	assign t[32] = t[31] & t[30];
	assign t[33] = t[32] ^ t[24];
	assign t[34] = t[23] ^ t[33];
	assign t[35] = t[27] ^ t[33];
	assign t[36] = t[24] & t[35];
	assign t[37] = t[36] ^ t[34];
	assign t[38] = t[27] ^ t[36];
	assign t[39] = t[29] & t[38];
	assign t[40] = t[25] ^ t[39];
	assign t[41] = t[40] ^ t[37];
	assign t[42] = t[29] ^ t[33];
	assign t[43] = t[29] ^ t[40];
	assign t[44] = t[33] ^ t[37];
	assign t[45] = t[42] ^ t[41];
	assign z[0] = t[44] & y[15];
	assign z[1] = t[37] & y[6];
	assign z[2] = t[33] & x[7];
	assign z[3] = t[43] & y[16];
	assign z[4] = t[40] & y[1];
	assign z[5] = t[29] & y[7];
	assign z[6] = t[42] & y[11];
	assign z[7] = t[45] & y[17];
	assign z[8] = t[41] & y[10];
	assign z[9] = t[44] & y[12];
	assign z[10] = t[37] & y[3];
	assign z[11] = t[33] & y[4];
	assign z[12] = t[43] & y[13];
	assign z[13] = t[40] & y[5];
	assign z[14] = t[29] & y[2];
	assign z[15] = t[42] & y[9];
	assign z[16] = t[45] & y[14];
	assign z[17] = t[41] & y[8];
	assign t[46] = z[15] ^ z[16];
	assign t[47] = z[10] ^ z[11];
	assign t[48] = z[5] ^ z[13];
	assign t[49] = z[9] ^ z[10];
	assign t[50] = z[2] ^ z[12];
	assign t[51] = z[2] ^ z[5];
	assign t[52] = z[7] ^ z[8];
	assign t[53] = z[0] ^ z[3];
	assign t[54] = z[6] ^ z[7];
	assign t[55] = z[16] ^ z[17];
	assign t[56] = z[12] ^ t[48];
	assign t[57] = t[50] ^ t[53];
	assign t[58] = z[4] ^ t[46];
	assign t[59] = z[3] ^ t[54];
	assign t[60] = t[46] ^ t[57];
	assign t[61] = z[14] ^ t[57];
	assign t[62] = t[52] ^ t[58];
	assign t[63] = t[49] ^ t[58];
	assign t[64] = z[4] ^ t[59];
	assign t[65] = t[61] ^ t[62];
	assign t[66] = z[1] ^ t[63];
	assign s[0] = t[59] ^ t[63];
	assign s[6] = ~t[56] ^ t[62];
	assign s[7] = ~t[48] ^ t[60];
	assign t[67] = t[64] ^ t[65];
	assign s[3] = t[53] ^ t[66];
	assign s[4] = t[51] ^ t[66];
	assign s[5] = t[47] ^ t[65];
	assign s[1] = ~t[64] ^ s[3];
	assign s[2] = ~t[55] ^ t[67];
endmodule
module SubBytes (
	x,
	z
);
	input [127:0] x;
	output wire [127:0] z;
	function [7:0] sbox;
		input [7:0] address;
		case (address)
			8'h00: sbox = 8'h63;
			8'h01: sbox = 8'h7c;
			8'h02: sbox = 8'h77;
			8'h03: sbox = 8'h7b;
			8'h04: sbox = 8'hf2;
			8'h05: sbox = 8'h6b;
			8'h06: sbox = 8'h6f;
			8'h07: sbox = 8'hc5;
			8'h08: sbox = 8'h30;
			8'h09: sbox = 8'h01;
			8'h0a: sbox = 8'h67;
			8'h0b: sbox = 8'h2b;
			8'h0c: sbox = 8'hfe;
			8'h0d: sbox = 8'hd7;
			8'h0e: sbox = 8'hab;
			8'h0f: sbox = 8'h76;
			8'h10: sbox = 8'hca;
			8'h11: sbox = 8'h82;
			8'h12: sbox = 8'hc9;
			8'h13: sbox = 8'h7d;
			8'h14: sbox = 8'hfa;
			8'h15: sbox = 8'h59;
			8'h16: sbox = 8'h47;
			8'h17: sbox = 8'hf0;
			8'h18: sbox = 8'had;
			8'h19: sbox = 8'hd4;
			8'h1a: sbox = 8'ha2;
			8'h1b: sbox = 8'haf;
			8'h1c: sbox = 8'h9c;
			8'h1d: sbox = 8'ha4;
			8'h1e: sbox = 8'h72;
			8'h1f: sbox = 8'hc0;
			8'h20: sbox = 8'hb7;
			8'h21: sbox = 8'hfd;
			8'h22: sbox = 8'h93;
			8'h23: sbox = 8'h26;
			8'h24: sbox = 8'h36;
			8'h25: sbox = 8'h3f;
			8'h26: sbox = 8'hf7;
			8'h27: sbox = 8'hcc;
			8'h28: sbox = 8'h34;
			8'h29: sbox = 8'ha5;
			8'h2a: sbox = 8'he5;
			8'h2b: sbox = 8'hf1;
			8'h2c: sbox = 8'h71;
			8'h2d: sbox = 8'hd8;
			8'h2e: sbox = 8'h31;
			8'h2f: sbox = 8'h15;
			8'h30: sbox = 8'h04;
			8'h31: sbox = 8'hc7;
			8'h32: sbox = 8'h23;
			8'h33: sbox = 8'hc3;
			8'h34: sbox = 8'h18;
			8'h35: sbox = 8'h96;
			8'h36: sbox = 8'h05;
			8'h37: sbox = 8'h9a;
			8'h38: sbox = 8'h07;
			8'h39: sbox = 8'h12;
			8'h3a: sbox = 8'h80;
			8'h3b: sbox = 8'he2;
			8'h3c: sbox = 8'heb;
			8'h3d: sbox = 8'h27;
			8'h3e: sbox = 8'hb2;
			8'h3f: sbox = 8'h75;
			8'h40: sbox = 8'h09;
			8'h41: sbox = 8'h83;
			8'h42: sbox = 8'h2c;
			8'h43: sbox = 8'h1a;
			8'h44: sbox = 8'h1b;
			8'h45: sbox = 8'h6e;
			8'h46: sbox = 8'h5a;
			8'h47: sbox = 8'ha0;
			8'h48: sbox = 8'h52;
			8'h49: sbox = 8'h3b;
			8'h4a: sbox = 8'hd6;
			8'h4b: sbox = 8'hb3;
			8'h4c: sbox = 8'h29;
			8'h4d: sbox = 8'he3;
			8'h4e: sbox = 8'h2f;
			8'h4f: sbox = 8'h84;
			8'h50: sbox = 8'h53;
			8'h51: sbox = 8'hd1;
			8'h52: sbox = 8'h00;
			8'h53: sbox = 8'hed;
			8'h54: sbox = 8'h20;
			8'h55: sbox = 8'hfc;
			8'h56: sbox = 8'hb1;
			8'h57: sbox = 8'h5b;
			8'h58: sbox = 8'h6a;
			8'h59: sbox = 8'hcb;
			8'h5a: sbox = 8'hbe;
			8'h5b: sbox = 8'h39;
			8'h5c: sbox = 8'h4a;
			8'h5d: sbox = 8'h4c;
			8'h5e: sbox = 8'h58;
			8'h5f: sbox = 8'hcf;
			8'h60: sbox = 8'hd0;
			8'h61: sbox = 8'hef;
			8'h62: sbox = 8'haa;
			8'h63: sbox = 8'hfb;
			8'h64: sbox = 8'h43;
			8'h65: sbox = 8'h4d;
			8'h66: sbox = 8'h33;
			8'h67: sbox = 8'h85;
			8'h68: sbox = 8'h45;
			8'h69: sbox = 8'hf9;
			8'h6a: sbox = 8'h02;
			8'h6b: sbox = 8'h7f;
			8'h6c: sbox = 8'h50;
			8'h6d: sbox = 8'h3c;
			8'h6e: sbox = 8'h9f;
			8'h6f: sbox = 8'ha8;
			8'h70: sbox = 8'h51;
			8'h71: sbox = 8'ha3;
			8'h72: sbox = 8'h40;
			8'h73: sbox = 8'h8f;
			8'h74: sbox = 8'h92;
			8'h75: sbox = 8'h9d;
			8'h76: sbox = 8'h38;
			8'h77: sbox = 8'hf5;
			8'h78: sbox = 8'hbc;
			8'h79: sbox = 8'hb6;
			8'h7a: sbox = 8'hda;
			8'h7b: sbox = 8'h21;
			8'h7c: sbox = 8'h10;
			8'h7d: sbox = 8'hff;
			8'h7e: sbox = 8'hf3;
			8'h7f: sbox = 8'hd2;
			8'h80: sbox = 8'hcd;
			8'h81: sbox = 8'h0c;
			8'h82: sbox = 8'h13;
			8'h83: sbox = 8'hec;
			8'h84: sbox = 8'h5f;
			8'h85: sbox = 8'h97;
			8'h86: sbox = 8'h44;
			8'h87: sbox = 8'h17;
			8'h88: sbox = 8'hc4;
			8'h89: sbox = 8'ha7;
			8'h8a: sbox = 8'h7e;
			8'h8b: sbox = 8'h3d;
			8'h8c: sbox = 8'h64;
			8'h8d: sbox = 8'h5d;
			8'h8e: sbox = 8'h19;
			8'h8f: sbox = 8'h73;
			8'h90: sbox = 8'h60;
			8'h91: sbox = 8'h81;
			8'h92: sbox = 8'h4f;
			8'h93: sbox = 8'hdc;
			8'h94: sbox = 8'h22;
			8'h95: sbox = 8'h2a;
			8'h96: sbox = 8'h90;
			8'h97: sbox = 8'h88;
			8'h98: sbox = 8'h46;
			8'h99: sbox = 8'hee;
			8'h9a: sbox = 8'hb8;
			8'h9b: sbox = 8'h14;
			8'h9c: sbox = 8'hde;
			8'h9d: sbox = 8'h5e;
			8'h9e: sbox = 8'h0b;
			8'h9f: sbox = 8'hdb;
			8'ha0: sbox = 8'he0;
			8'ha1: sbox = 8'h32;
			8'ha2: sbox = 8'h3a;
			8'ha3: sbox = 8'h0a;
			8'ha4: sbox = 8'h49;
			8'ha5: sbox = 8'h06;
			8'ha6: sbox = 8'h24;
			8'ha7: sbox = 8'h5c;
			8'ha8: sbox = 8'hc2;
			8'ha9: sbox = 8'hd3;
			8'haa: sbox = 8'hac;
			8'hab: sbox = 8'h62;
			8'hac: sbox = 8'h91;
			8'had: sbox = 8'h95;
			8'hae: sbox = 8'he4;
			8'haf: sbox = 8'h79;
			8'hb0: sbox = 8'he7;
			8'hb1: sbox = 8'hc8;
			8'hb2: sbox = 8'h37;
			8'hb3: sbox = 8'h6d;
			8'hb4: sbox = 8'h8d;
			8'hb5: sbox = 8'hd5;
			8'hb6: sbox = 8'h4e;
			8'hb7: sbox = 8'ha9;
			8'hb8: sbox = 8'h6c;
			8'hb9: sbox = 8'h56;
			8'hba: sbox = 8'hf4;
			8'hbb: sbox = 8'hea;
			8'hbc: sbox = 8'h65;
			8'hbd: sbox = 8'h7a;
			8'hbe: sbox = 8'hae;
			8'hbf: sbox = 8'h08;
			8'hc0: sbox = 8'hba;
			8'hc1: sbox = 8'h78;
			8'hc2: sbox = 8'h25;
			8'hc3: sbox = 8'h2e;
			8'hc4: sbox = 8'h1c;
			8'hc5: sbox = 8'ha6;
			8'hc6: sbox = 8'hb4;
			8'hc7: sbox = 8'hc6;
			8'hc8: sbox = 8'he8;
			8'hc9: sbox = 8'hdd;
			8'hca: sbox = 8'h74;
			8'hcb: sbox = 8'h1f;
			8'hcc: sbox = 8'h4b;
			8'hcd: sbox = 8'hbd;
			8'hce: sbox = 8'h8b;
			8'hcf: sbox = 8'h8a;
			8'hd0: sbox = 8'h70;
			8'hd1: sbox = 8'h3e;
			8'hd2: sbox = 8'hb5;
			8'hd3: sbox = 8'h66;
			8'hd4: sbox = 8'h48;
			8'hd5: sbox = 8'h03;
			8'hd6: sbox = 8'hf6;
			8'hd7: sbox = 8'h0e;
			8'hd8: sbox = 8'h61;
			8'hd9: sbox = 8'h35;
			8'hda: sbox = 8'h57;
			8'hdb: sbox = 8'hb9;
			8'hdc: sbox = 8'h86;
			8'hdd: sbox = 8'hc1;
			8'hde: sbox = 8'h1d;
			8'hdf: sbox = 8'h9e;
			8'he0: sbox = 8'he1;
			8'he1: sbox = 8'hf8;
			8'he2: sbox = 8'h98;
			8'he3: sbox = 8'h11;
			8'he4: sbox = 8'h69;
			8'he5: sbox = 8'hd9;
			8'he6: sbox = 8'h8e;
			8'he7: sbox = 8'h94;
			8'he8: sbox = 8'h9b;
			8'he9: sbox = 8'h1e;
			8'hea: sbox = 8'h87;
			8'heb: sbox = 8'he9;
			8'hec: sbox = 8'hce;
			8'hed: sbox = 8'h55;
			8'hee: sbox = 8'h28;
			8'hef: sbox = 8'hdf;
			8'hf0: sbox = 8'h8c;
			8'hf1: sbox = 8'ha1;
			8'hf2: sbox = 8'h89;
			8'hf3: sbox = 8'h0d;
			8'hf4: sbox = 8'hbf;
			8'hf5: sbox = 8'he6;
			8'hf6: sbox = 8'h42;
			8'hf7: sbox = 8'h68;
			8'hf8: sbox = 8'h41;
			8'hf9: sbox = 8'h99;
			8'hfa: sbox = 8'h2d;
			8'hfb: sbox = 8'h0f;
			8'hfc: sbox = 8'hb0;
			8'hfd: sbox = 8'h54;
			8'hfe: sbox = 8'hbb;
			8'hff: sbox = 8'h16;
			default: sbox = 8'h00;
		endcase
	endfunction
	genvar _gv_i_9;
	generate
		for (_gv_i_9 = 0; _gv_i_9 < 16; _gv_i_9 = _gv_i_9 + 1) begin : SBOX
			localparam i = _gv_i_9;
			assign z[(8 * (i + 1)) - 1:8 * i] = sbox(x[(8 * (i + 1)) - 1:8 * i]);
		end
	endgenerate
endmodule
module aes_1cc (
	clk,
	rst,
	g_input,
	e_input,
	o
);
	localparam NB = 4;
	localparam NK = 4;
	localparam NR = 10;
	input clk;
	input rst;
	input [127:0] g_input;
	input [127:0] e_input;
	output wire [127:0] o;
	wire [127:0] key;
	wire [127:0] msg;
	wire [127:0] out;
	wire [1407:0] expandedKey;
	wire [127:0] expandedKeyi [NR:0];
	wire [127:0] x1 [9:0];
	wire [127:0] x2 [9:0];
	wire [127:0] x3 [9:0];
	wire [127:0] x4 [8:0];
	assign key = g_input;
	assign msg = e_input;
	assign o = out;
	genvar _gv_i_10;
	KeyExpansion e(
		.key(key),
		.expandedKey(expandedKey)
	);
	generate
		for (_gv_i_10 = 0; _gv_i_10 < 11; _gv_i_10 = _gv_i_10 + 1) begin : EXPANDKEY
			localparam i = _gv_i_10;
			assign expandedKeyi[i] = expandedKey[(128 * (i + 1)) - 1:128 * i];
		end
	endgenerate
	AddRoundKey a(
		.x(msg),
		.y(expandedKeyi[0]),
		.z(x1[0])
	);
	generate
		for (_gv_i_10 = 0; _gv_i_10 < NR; _gv_i_10 = _gv_i_10 + 1) begin : SUBBYTES
			localparam i = _gv_i_10;
			SubBytes a(
				.x(x1[i]),
				.z(x2[i])
			);
		end
		for (_gv_i_10 = 0; _gv_i_10 < NR; _gv_i_10 = _gv_i_10 + 1) begin : SHIFTROWS
			localparam i = _gv_i_10;
			ShiftRows c(
				.x(x2[i]),
				.z(x3[i])
			);
		end
		for (_gv_i_10 = 0; _gv_i_10 < 9; _gv_i_10 = _gv_i_10 + 1) begin : MIXCOLUMNS
			localparam i = _gv_i_10;
			MixColumns d(
				.x(x3[i]),
				.z(x4[i])
			);
		end
		for (_gv_i_10 = 0; _gv_i_10 < NR; _gv_i_10 = _gv_i_10 + 1) begin : ADDROUNDKEY
			localparam i = _gv_i_10;
			if (i == 9) begin : LAST
				AddRoundKey a(
					.x(x3[i]),
					.y(expandedKeyi[i + 1]),
					.z(out)
				);
			end
			else begin : ELSE
				AddRoundKey a(
					.x(x4[i]),
					.y(expandedKeyi[i + 1]),
					.z(x1[i + 1])
				);
			end
		end
	endgenerate
endmodule
module f_permutation (
	clk,
	reset,
	in,
	in_ready,
	ack,
	out,
	out_ready
);
	input clk;
	input reset;
	input [575:0] in;
	input in_ready;
	output wire ack;
	output reg [1599:0] out;
	output reg out_ready;
	reg [10:0] i;
	wire [1599:0] round_in;
	wire [1599:0] round_out;
	wire [63:0] rc1;
	wire [63:0] rc2;
	wire update;
	wire accept;
	reg calc;
	assign accept = in_ready & ~calc;
	always @(posedge clk)
		if (reset)
			i <= 0;
		else
			i <= {i[9:0], accept};
	always @(posedge clk)
		if (reset)
			calc <= 0;
		else
			calc <= (calc & ~i[10]) | accept;
	assign update = calc | accept;
	assign ack = accept;
	always @(posedge clk)
		if (reset)
			out_ready <= 0;
		else if (accept)
			out_ready <= 0;
		else if (i[10])
			out_ready <= 1;
	assign round_in = (accept ? {in ^ out[1599:1024], out[1023:0]} : out);
	rconst2in1 rconst_(
		.i({i, accept}),
		.rc1(rc1),
		.rc2(rc2)
	);
	round2in1 round_(
		.in(round_in),
		.round_const_1(rc1),
		.round_const_2(rc2),
		.out(round_out)
	);
	always @(posedge clk)
		if (reset)
			out <= 0;
		else if (update)
			out <= round_out;
endmodule
module keccak (
	clk,
	reset,
	in,
	in_ready,
	is_last,
	byte_num,
	buffer_full,
	out,
	out_ready
);
	input clk;
	input reset;
	input [63:0] in;
	input in_ready;
	input is_last;
	input [2:0] byte_num;
	output wire buffer_full;
	output wire [511:0] out;
	output reg out_ready;
	reg state;
	wire [575:0] padder_out;
	wire [575:0] padder_out_1;
	wire padder_out_ready;
	wire f_ack;
	wire [1599:0] f_out;
	wire f_out_ready;
	wire [511:0] out1;
	reg [10:0] i;
	genvar _gv_w_2;
	genvar _gv_b_1;
	assign out1 = f_out[1599:1088];
	always @(posedge clk)
		if (reset)
			i <= 0;
		else
			i <= {i[9:0], state & f_ack};
	always @(posedge clk)
		if (reset)
			state <= 0;
		else if (is_last)
			state <= 1;
	generate
		for (_gv_w_2 = 0; _gv_w_2 < 8; _gv_w_2 = _gv_w_2 + 1) begin : L0
			localparam w = _gv_w_2;
			for (_gv_b_1 = 0; _gv_b_1 < 8; _gv_b_1 = _gv_b_1 + 1) begin : L1
				localparam b = _gv_b_1;
				assign out[((w * 64) + (b * 8)) + 7:(w * 64) + (b * 8)] = out1[((w * 64) + ((7 - b) * 8)) + 7:(w * 64) + ((7 - b) * 8)];
			end
		end
		for (_gv_w_2 = 0; _gv_w_2 < 9; _gv_w_2 = _gv_w_2 + 1) begin : L2
			localparam w = _gv_w_2;
			for (_gv_b_1 = 0; _gv_b_1 < 8; _gv_b_1 = _gv_b_1 + 1) begin : L3
				localparam b = _gv_b_1;
				assign padder_out[((w * 64) + (b * 8)) + 7:(w * 64) + (b * 8)] = padder_out_1[((w * 64) + ((7 - b) * 8)) + 7:(w * 64) + ((7 - b) * 8)];
			end
		end
	endgenerate
	always @(posedge clk)
		if (reset)
			out_ready <= 0;
		else if (i[10])
			out_ready <= 1;
	padder padder_(
		.clk(clk),
		.reset(reset),
		.in(in),
		.in_ready(in_ready),
		.is_last(is_last),
		.byte_num(byte_num),
		.buffer_full(buffer_full),
		.out(padder_out_1),
		.out_ready(padder_out_ready),
		.f_ack(f_ack)
	);
	f_permutation f_permutation_(
		.clk(clk),
		.reset(reset),
		.in(padder_out),
		.in_ready(padder_out_ready),
		.ack(f_ack),
		.out(f_out),
		.out_ready(f_out_ready)
	);
endmodule
module md5 (
	clk,
	reset,
	load_i,
	ready_o,
	newtext_i,
	data_i,
	data_o
);
	input clk;
	input reset;
	input load_i;
	output reg ready_o;
	input newtext_i;
	input [127:0] data_i;
	output reg [127:0] data_o;
	reg next_ready_o;
	reg [127:0] next_data_o;
	reg [5:0] round64;
	reg [5:0] next_round64;
	reg [43:0] t;
	reg [31:0] ar;
	reg [31:0] br;
	reg [31:0] cr;
	reg [31:0] dr;
	reg [31:0] func_out;
	reg [31:0] next_ar;
	reg [31:0] next_br;
	reg [31:0] next_cr;
	reg [31:0] next_dr;
	reg [31:0] A;
	reg [31:0] B;
	reg [31:0] C;
	reg [31:0] D;
	reg [31:0] next_A;
	reg [31:0] next_B;
	reg [31:0] next_C;
	reg [31:0] next_D;
	reg [511:0] message;
	reg [511:0] next_message;
	reg [2:0] round;
	reg [2:0] next_round;
	reg next_generate_hash;
	reg generate_hash;
	reg hash_generated;
	reg [2:0] next_getdata_state;
	reg [2:0] getdata_state;
	always @(round64)
		case (round64)
			0: t = 44'hd76aa478070;
			1: t = 44'he8c7b7560c1;
			2: t = 44'h242070db112;
			3: t = 44'hc1bdceee163;
			4: t = 44'hf57c0faf074;
			5: t = 44'h4787c62a0c5;
			6: t = 44'ha8304613116;
			7: t = 44'hfd469501167;
			8: t = 44'h698098d8078;
			9: t = 44'h8b44f7af0c9;
			10: t = 44'hffff5bb111a;
			11: t = 44'h895cd7be16b;
			12: t = 44'h6b90112207c;
			13: t = 44'hfd9871930cd;
			14: t = 44'ha679438e11e;
			15: t = 44'h49b4082116f;
			16: t = 44'hf61e2562051;
			17: t = 44'hc040b340096;
			18: t = 44'h265e5a510eb;
			19: t = 44'he9b6c7aa140;
			20: t = 44'hd62f105d055;
			21: t = 44'h0244145309a;
			22: t = 44'hd8a1e6810ef;
			23: t = 44'he7d3fbc8144;
			24: t = 44'h21e1cde6059;
			25: t = 44'hc33707d609e;
			26: t = 44'hf4d50d870e3;
			27: t = 44'h455a14ed148;
			28: t = 44'ha9e3e90505d;
			29: t = 44'hfcefa3f8092;
			30: t = 44'h676f02d90e7;
			31: t = 44'h8d2a4c8a14c;
			32: t = 44'hfffa3942045;
			33: t = 44'h8771f6810b8;
			34: t = 44'h6d9d612210b;
			35: t = 44'hfde5380c17e;
			36: t = 44'ha4beea44041;
			37: t = 44'h4bdecfa90b4;
			38: t = 44'hf6bb4b60107;
			39: t = 44'hbebfbc7017a;
			40: t = 44'h289b7ec604d;
			41: t = 44'heaa127fa0b0;
			42: t = 44'hd4ef3085103;
			43: t = 44'h04881d05176;
			44: t = 44'hd9d4d039049;
			45: t = 44'he6db99e50bc;
			46: t = 44'h1fa27cf810f;
			47: t = 44'hc4ac5665172;
			48: t = 44'hf4292244060;
			49: t = 44'h432aff970a7;
			50: t = 44'hab9423a70fe;
			51: t = 44'hfc93a039155;
			52: t = 44'h655b59c306c;
			53: t = 44'h8f0ccc920a3;
			54: t = 44'hffeff47d0fa;
			55: t = 44'h85845dd1151;
			56: t = 44'h6fa87e4f068;
			57: t = 44'hfe2ce6e00af;
			58: t = 44'ha30143140f6;
			59: t = 44'h4e0811a115d;
			60: t = 44'hf7537e82064;
			61: t = 44'hbd3af2350ab;
			62: t = 44'h2ad7d2bb0f2;
			63: t = 44'heb86d391159;
		endcase
	reg [31:0] aux31;
	reg [31:0] fr_var;
	reg [31:0] tr_var;
	reg [31:0] rotate1;
	reg [31:0] rotate2;
	reg [7:0] s_var;
	reg [3:0] nblock;
	reg [31:0] message_var [15:0];
	always @(t or ar or br or cr or dr or round or message or func_out or message_var[0] or message_var[1] or message_var[2] or message_var[3] or message_var[4] or message_var[5] or message_var[6] or message_var[7] or message_var[8] or message_var[9] or message_var[10] or message_var[11] or message_var[12] or message_var[13] or message_var[14] or message_var[15]) begin
		message_var[0] = message[511:480];
		message_var[1] = message[479:448];
		message_var[2] = message[447:416];
		message_var[3] = message[415:384];
		message_var[4] = message[383:352];
		message_var[5] = message[351:320];
		message_var[6] = message[319:288];
		message_var[7] = message[287:256];
		message_var[8] = message[255:224];
		message_var[9] = message[223:192];
		message_var[10] = message[191:160];
		message_var[11] = message[159:128];
		message_var[12] = message[127:96];
		message_var[13] = message[95:64];
		message_var[14] = message[63:32];
		message_var[15] = message[31:0];
		fr_var = 0;
		case (round)
			0: fr_var = (br & cr) | (~br & dr);
			1: fr_var = (br & dr) | (cr & ~dr);
			2: fr_var = (br ^ cr) ^ dr;
			3: fr_var = cr ^ (br | ~dr);
		endcase
		tr_var = t[43:12];
		s_var = t[11:4];
		nblock = t[3:0];
		aux31 = ((ar + fr_var) + message_var[nblock]) + tr_var;
		rotate1 = aux31 << s_var;
		rotate2 = aux31 >> (32 - s_var);
		func_out = br + (rotate1 | rotate2);
	end
	always @(newtext_i or round or round64 or ar or br or cr or dr or generate_hash or func_out or getdata_state or A or B or C or D) begin
		next_ar = ar;
		next_br = br;
		next_cr = cr;
		next_dr = dr;
		next_round64 = round64;
		next_round = round;
		hash_generated = 0;
		if (generate_hash != 0) begin
			next_ar = dr;
			next_br = func_out;
			next_cr = br;
			next_dr = cr;
		end
		case (round64)
			0: begin
				next_round = 0;
				if (generate_hash)
					next_round64 = 1;
			end
			15, 31, 47: begin
				next_round = round + 1;
				next_round64 = round64 + 1;
			end
			63: begin
				next_round = 0;
				next_round64 = 0;
				hash_generated = 1;
			end
			default: next_round64 = round64 + 1;
		endcase
		if (newtext_i) begin
			next_ar = 32'h67452301;
			next_br = 32'hefcdab89;
			next_cr = 32'h98badcfe;
			next_dr = 32'h10325476;
			next_round = 0;
			next_round64 = 0;
		end
		if (!getdata_state) begin
			next_ar = A;
			next_br = B;
			next_cr = C;
			next_dr = D;
		end
	end
	always @(posedge clk or negedge reset)
		if (!reset) begin
			ready_o = 0;
			data_o = 0;
			message = 0;
			ar = 32'h67452301;
			br = 32'hefcdab89;
			cr = 32'h98badcfe;
			dr = 32'h10325476;
			getdata_state = 0;
			generate_hash = 0;
			round = 0;
			round64 = 0;
			A = 32'h67452301;
			B = 32'hefcdab89;
			C = 32'h98badcfe;
			D = 32'h10325476;
		end
		else begin
			ready_o = next_ready_o;
			data_o = next_data_o;
			message = next_message;
			ar = next_ar;
			br = next_br;
			cr = next_cr;
			dr = next_dr;
			A = next_A;
			B = next_B;
			C = next_C;
			D = next_D;
			generate_hash = next_generate_hash;
			getdata_state = next_getdata_state;
			round = next_round;
			round64 = next_round64;
		end
	reg [127:0] data_o_var;
	reg [511:0] aux;
	wire [31:0] A_t;
	wire [31:0] B_t;
	wire [31:0] C_t;
	wire [31:0] D_t;
	assign A_t = dr + A;
	assign B_t = func_out + B;
	assign C_t = br + C;
	assign D_t = cr + D;
	always @(newtext_i or A_t or B_t or C_t or D_t or data_i or load_i or getdata_state or generate_hash or hash_generated or message or func_out or A or B or C or D or ar or br or cr or dr) begin
		next_A = A;
		next_B = B;
		next_C = C;
		next_D = D;
		next_generate_hash = generate_hash;
		next_ready_o = 0;
		next_getdata_state = getdata_state;
		next_data_o = 0;
		aux = message;
		next_message = message;
		if (newtext_i) begin
			next_A = 32'h67452301;
			next_B = 32'hefcdab89;
			next_C = 32'h98badcfe;
			next_D = 32'h10325476;
			next_getdata_state = 0;
		end
		case (getdata_state)
			0:
				if (load_i) begin
					aux[511:384] = data_i;
					next_message = aux;
					next_getdata_state = 1;
				end
			1:
				if (load_i) begin
					aux[383:256] = data_i;
					next_message = aux;
					next_getdata_state = 2;
				end
			2:
				if (load_i) begin
					aux[255:128] = data_i;
					next_message = aux;
					next_getdata_state = 3;
				end
			3:
				if (load_i) begin
					aux[127:0] = data_i;
					next_message = aux;
					next_getdata_state = 4;
					next_generate_hash = 1;
				end
			4: begin
				next_generate_hash = 1;
				data_o_var[127:96] = A_t;
				data_o_var[95:64] = B_t;
				data_o_var[63:32] = C_t;
				data_o_var[31:0] = D_t;
				next_data_o = data_o_var;
				if (hash_generated) begin
					next_A = A_t;
					next_B = B_t;
					next_C = C_t;
					next_D = D_t;
					next_getdata_state = 0;
					next_ready_o = 1;
					next_generate_hash = 0;
				end
			end
		endcase
	end
endmodule
module mux_func (
	a,
	b,
	c,
	d,
	clk,
	rst
);
	input wire [127:0] a;
	input wire [127:0] b;
	output reg [127:0] c;
	input wire [127:0] d;
	input wire clk;
	input wire rst;
	wire rdy_out_sha;
	wire rdy_out_md5;
	wire [127:0] aes_out;
	wire [127:0] sha_out;
	wire [127:0] md5_out;
	wire [127:0] temperature_out;
	aes_1cc aes(
		.clk(0),
		.rst(1),
		.g_input(b),
		.e_input(a),
		.o(aes_out)
	);
	wire rdy_out;
	keccak sha(
		.clk(clk),
		.reset(rst),
		.in(a),
		.in_ready(1),
		.out(sha_out),
		.out_ready(rdy_out)
	);
	md5 md5(
		.clk(clk),
		.reset(rst),
		.data_i(a[31:0]),
		.load_i(1),
		.ready_o(rdy_out_md5),
		.data_o(md5_out[31:0])
	);
	tempsen temperature_sensor(
		.clk(clk),
		.in(a),
		.out(temperature_out)
	);
	always @(posedge clk) begin
		if (d[0] == 1'b1)
			c = aes_out;
		if (d[1] == 1'b1)
			c = sha_out;
		if (d[2] == 1'b1)
			c = md5_out;
		if (d[3] == 1'b1)
			c = temperature_out;
		else
			c = 128'h00000000000000000000000000000000;
	end
endmodule
module padder (
	clk,
	reset,
	in,
	in_ready,
	is_last,
	byte_num,
	buffer_full,
	out,
	out_ready,
	f_ack
);
	input clk;
	input reset;
	input [63:0] in;
	input in_ready;
	input is_last;
	input [2:0] byte_num;
	output wire buffer_full;
	output reg [575:0] out;
	output wire out_ready;
	input f_ack;
	reg state;
	reg done;
	reg [8:0] i;
	wire [63:0] v0;
	reg [63:0] v1;
	wire accept;
	wire update;
	assign buffer_full = i[8];
	assign out_ready = buffer_full;
	assign accept = (~state & in_ready) & ~buffer_full;
	assign update = (accept | (state & ~buffer_full)) & ~done;
	always @(posedge clk)
		if (reset)
			out <= 0;
		else if (update)
			out <= {out[511:0], v1};
	always @(posedge clk)
		if (reset)
			i <= 0;
		else if (f_ack | update)
			i <= {i[7:0], 1'b1} & {9 {~f_ack}};
	always @(posedge clk)
		if (reset)
			state <= 0;
		else if (is_last)
			state <= 1;
	always @(posedge clk)
		if (reset)
			done <= 0;
		else if (state & out_ready)
			done <= 1;
	padder1 p0(
		.in(in),
		.byte_num(byte_num),
		.out(v0)
	);
	always @(*)
		if (state) begin
			v1 = 0;
			v1[7] = v1[7] | i[7];
		end
		else if (is_last == 0)
			v1 = in;
		else begin
			v1 = v0;
			v1[7] = v1[7] | i[7];
		end
endmodule
module padder1 (
	in,
	byte_num,
	out
);
	input [63:0] in;
	input [2:0] byte_num;
	output reg [63:0] out;
	always @(*)
		case (byte_num)
			0: out = 64'h0100000000000000;
			1: out = {in[63:56], 56'h01000000000000};
			2: out = {in[63:48], 48'h010000000000};
			3: out = {in[63:40], 40'h0100000000};
			4: out = {in[63:32], 32'h01000000};
			5: out = {in[63:24], 24'h010000};
			6: out = {in[63:16], 16'h0100};
			7: out = {in[63:8], 8'h01};
		endcase
endmodule
module rconst2in1 (
	i,
	rc1,
	rc2
);
	input [11:0] i;
	output reg [63:0] rc1;
	output reg [63:0] rc2;
	always @(i) begin
		rc1 = 0;
		rc1[0] = ((((((i[0] | i[2]) | i[3]) | i[5]) | i[6]) | i[7]) | i[10]) | i[11];
		rc1[1] = ((((i[1] | i[2]) | i[4]) | i[6]) | i[8]) | i[9];
		rc1[3] = (((((i[1] | i[2]) | i[4]) | i[5]) | i[6]) | i[7]) | i[9];
		rc1[7] = (((((i[1] | i[2]) | i[3]) | i[4]) | i[6]) | i[7]) | i[10];
		rc1[15] = (((((((i[1] | i[2]) | i[3]) | i[5]) | i[6]) | i[7]) | i[8]) | i[9]) | i[10];
		rc1[31] = (((i[3] | i[5]) | i[6]) | i[10]) | i[11];
		rc1[63] = (((i[1] | i[3]) | i[7]) | i[8]) | i[10];
	end
	always @(i) begin
		rc2 = 0;
		rc2[0] = ((i[2] | i[3]) | i[6]) | i[7];
		rc2[1] = (((i[0] | i[5]) | i[6]) | i[7]) | i[9];
		rc2[3] = ((((i[3] | i[4]) | i[5]) | i[6]) | i[9]) | i[11];
		rc2[7] = (((i[0] | i[4]) | i[6]) | i[8]) | i[10];
		rc2[15] = ((((i[0] | i[1]) | i[3]) | i[7]) | i[10]) | i[11];
		rc2[31] = (((i[1] | i[2]) | i[5]) | i[9]) | i[11];
		rc2[63] = ((((((i[1] | i[3]) | i[6]) | i[7]) | i[8]) | i[9]) | i[10]) | i[11];
	end
endmodule
module round2in1 (
	in,
	round_const_1,
	round_const_2,
	out
);
	input [1599:0] in;
	input [63:0] round_const_1;
	input [63:0] round_const_2;
	output wire [1599:0] out;
	wire [63:0] a [4:0][4:0];
	wire [63:0] b [4:0];
	wire [63:0] c [4:0][4:0];
	wire [63:0] d [4:0][4:0];
	wire [63:0] e [4:0][4:0];
	wire [63:0] f [4:0][4:0];
	wire [63:0] g [4:0][4:0];
	wire [63:0] bb [4:0];
	wire [63:0] cc [4:0][4:0];
	wire [63:0] dd [4:0][4:0];
	wire [63:0] ee [4:0][4:0];
	wire [63:0] ff [4:0][4:0];
	wire [63:0] gg [4:0][4:0];
	genvar _gv_x_1;
	genvar _gv_y_1;
	generate
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L0
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L1
				localparam x = _gv_x_1;
				assign a[x][y] = in[1599 - (64 * ((5 * y) + x)):(1599 - (64 * ((5 * y) + x))) - 63];
			end
		end
		for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L2
			localparam x = _gv_x_1;
			assign b[x] = (((a[x][0] ^ a[x][1]) ^ a[x][2]) ^ a[x][3]) ^ a[x][4];
		end
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L3
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L4
				localparam x = _gv_x_1;
				assign c[x][y] = (a[x][y] ^ b[(x == 0 ? 4 : x - 1)]) ^ {b[(x == 4 ? 0 : x + 1)][62:0], b[(x == 4 ? 0 : x + 1)][63]};
			end
		end
	endgenerate
	assign d[0][0] = c[0][0];
	assign d[1][0] = {c[1][0][62:0], c[1][0][63]};
	assign d[2][0] = {c[2][0][1:0], c[2][0][63:2]};
	assign d[3][0] = {c[3][0][35:0], c[3][0][63:36]};
	assign d[4][0] = {c[4][0][36:0], c[4][0][63:37]};
	assign d[0][1] = {c[0][1][27:0], c[0][1][63:28]};
	assign d[1][1] = {c[1][1][19:0], c[1][1][63:20]};
	assign d[2][1] = {c[2][1][57:0], c[2][1][63:58]};
	assign d[3][1] = {c[3][1][8:0], c[3][1][63:9]};
	assign d[4][1] = {c[4][1][43:0], c[4][1][63:44]};
	assign d[0][2] = {c[0][2][60:0], c[0][2][63:61]};
	assign d[1][2] = {c[1][2][53:0], c[1][2][63:54]};
	assign d[2][2] = {c[2][2][20:0], c[2][2][63:21]};
	assign d[3][2] = {c[3][2][38:0], c[3][2][63:39]};
	assign d[4][2] = {c[4][2][24:0], c[4][2][63:25]};
	assign d[0][3] = {c[0][3][22:0], c[0][3][63:23]};
	assign d[1][3] = {c[1][3][18:0], c[1][3][63:19]};
	assign d[2][3] = {c[2][3][48:0], c[2][3][63:49]};
	assign d[3][3] = {c[3][3][42:0], c[3][3][63:43]};
	assign d[4][3] = {c[4][3][55:0], c[4][3][63:56]};
	assign d[0][4] = {c[0][4][45:0], c[0][4][63:46]};
	assign d[1][4] = {c[1][4][61:0], c[1][4][63:62]};
	assign d[2][4] = {c[2][4][2:0], c[2][4][63:3]};
	assign d[3][4] = {c[3][4][7:0], c[3][4][63:8]};
	assign d[4][4] = {c[4][4][49:0], c[4][4][63:50]};
	assign e[0][0] = d[0][0];
	assign e[0][2] = d[1][0];
	assign e[0][4] = d[2][0];
	assign e[0][1] = d[3][0];
	assign e[0][3] = d[4][0];
	assign e[1][3] = d[0][1];
	assign e[1][0] = d[1][1];
	assign e[1][2] = d[2][1];
	assign e[1][4] = d[3][1];
	assign e[1][1] = d[4][1];
	assign e[2][1] = d[0][2];
	assign e[2][3] = d[1][2];
	assign e[2][0] = d[2][2];
	assign e[2][2] = d[3][2];
	assign e[2][4] = d[4][2];
	assign e[3][4] = d[0][3];
	assign e[3][1] = d[1][3];
	assign e[3][3] = d[2][3];
	assign e[3][0] = d[3][3];
	assign e[3][2] = d[4][3];
	assign e[4][2] = d[0][4];
	assign e[4][4] = d[1][4];
	assign e[4][1] = d[2][4];
	assign e[4][3] = d[3][4];
	assign e[4][0] = d[4][4];
	generate
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L5
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L6
				localparam x = _gv_x_1;
				assign f[x][y] = e[x][y] ^ (~e[(x == 4 ? 0 : x + 1)][y] & e[(x == 3 ? 0 : (x == 4 ? 1 : x + 2))][y]);
			end
		end
		for (_gv_x_1 = 0; _gv_x_1 < 64; _gv_x_1 = _gv_x_1 + 1) begin : L60
			localparam x = _gv_x_1;
			if (((((((x == 0) || (x == 1)) || (x == 3)) || (x == 7)) || (x == 15)) || (x == 31)) || (x == 63)) begin : genblk1
				assign g[0][0][x] = f[0][0][x] ^ round_const_1[x];
			end
			else begin : genblk1
				assign g[0][0][x] = f[0][0][x];
			end
		end
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L7
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L8
				localparam x = _gv_x_1;
				if ((x != 0) || (y != 0)) begin : genblk1
					assign g[x][y] = f[x][y];
				end
			end
		end
		for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L12
			localparam x = _gv_x_1;
			assign bb[x] = (((g[x][0] ^ g[x][1]) ^ g[x][2]) ^ g[x][3]) ^ g[x][4];
		end
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L13
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L14
				localparam x = _gv_x_1;
				assign cc[x][y] = (g[x][y] ^ bb[(x == 0 ? 4 : x - 1)]) ^ {bb[(x == 4 ? 0 : x + 1)][62:0], bb[(x == 4 ? 0 : x + 1)][63]};
			end
		end
	endgenerate
	assign dd[0][0] = cc[0][0];
	assign dd[1][0] = {cc[1][0][62:0], cc[1][0][63]};
	assign dd[2][0] = {cc[2][0][1:0], cc[2][0][63:2]};
	assign dd[3][0] = {cc[3][0][35:0], cc[3][0][63:36]};
	assign dd[4][0] = {cc[4][0][36:0], cc[4][0][63:37]};
	assign dd[0][1] = {cc[0][1][27:0], cc[0][1][63:28]};
	assign dd[1][1] = {cc[1][1][19:0], cc[1][1][63:20]};
	assign dd[2][1] = {cc[2][1][57:0], cc[2][1][63:58]};
	assign dd[3][1] = {cc[3][1][8:0], cc[3][1][63:9]};
	assign dd[4][1] = {cc[4][1][43:0], cc[4][1][63:44]};
	assign dd[0][2] = {cc[0][2][60:0], cc[0][2][63:61]};
	assign dd[1][2] = {cc[1][2][53:0], cc[1][2][63:54]};
	assign dd[2][2] = {cc[2][2][20:0], cc[2][2][63:21]};
	assign dd[3][2] = {cc[3][2][38:0], cc[3][2][63:39]};
	assign dd[4][2] = {cc[4][2][24:0], cc[4][2][63:25]};
	assign dd[0][3] = {cc[0][3][22:0], cc[0][3][63:23]};
	assign dd[1][3] = {cc[1][3][18:0], cc[1][3][63:19]};
	assign dd[2][3] = {cc[2][3][48:0], cc[2][3][63:49]};
	assign dd[3][3] = {cc[3][3][42:0], cc[3][3][63:43]};
	assign dd[4][3] = {cc[4][3][55:0], cc[4][3][63:56]};
	assign dd[0][4] = {cc[0][4][45:0], cc[0][4][63:46]};
	assign dd[1][4] = {cc[1][4][61:0], cc[1][4][63:62]};
	assign dd[2][4] = {cc[2][4][2:0], cc[2][4][63:3]};
	assign dd[3][4] = {cc[3][4][7:0], cc[3][4][63:8]};
	assign dd[4][4] = {cc[4][4][49:0], cc[4][4][63:50]};
	assign ee[0][0] = dd[0][0];
	assign ee[0][2] = dd[1][0];
	assign ee[0][4] = dd[2][0];
	assign ee[0][1] = dd[3][0];
	assign ee[0][3] = dd[4][0];
	assign ee[1][3] = dd[0][1];
	assign ee[1][0] = dd[1][1];
	assign ee[1][2] = dd[2][1];
	assign ee[1][4] = dd[3][1];
	assign ee[1][1] = dd[4][1];
	assign ee[2][1] = dd[0][2];
	assign ee[2][3] = dd[1][2];
	assign ee[2][0] = dd[2][2];
	assign ee[2][2] = dd[3][2];
	assign ee[2][4] = dd[4][2];
	assign ee[3][4] = dd[0][3];
	assign ee[3][1] = dd[1][3];
	assign ee[3][3] = dd[2][3];
	assign ee[3][0] = dd[3][3];
	assign ee[3][2] = dd[4][3];
	assign ee[4][2] = dd[0][4];
	assign ee[4][4] = dd[1][4];
	assign ee[4][1] = dd[2][4];
	assign ee[4][3] = dd[3][4];
	assign ee[4][0] = dd[4][4];
	generate
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L15
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L16
				localparam x = _gv_x_1;
				assign ff[x][y] = ee[x][y] ^ (~ee[(x == 4 ? 0 : x + 1)][y] & ee[(x == 3 ? 0 : (x == 4 ? 1 : x + 2))][y]);
			end
		end
		for (_gv_x_1 = 0; _gv_x_1 < 64; _gv_x_1 = _gv_x_1 + 1) begin : L160
			localparam x = _gv_x_1;
			if (((((((x == 0) || (x == 1)) || (x == 3)) || (x == 7)) || (x == 15)) || (x == 31)) || (x == 63)) begin : genblk1
				assign gg[0][0][x] = ff[0][0][x] ^ round_const_2[x];
			end
			else begin : genblk1
				assign gg[0][0][x] = ff[0][0][x];
			end
		end
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L17
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L18
				localparam x = _gv_x_1;
				if ((x != 0) || (y != 0)) begin : genblk1
					assign gg[x][y] = ff[x][y];
				end
			end
		end
		for (_gv_y_1 = 0; _gv_y_1 < 5; _gv_y_1 = _gv_y_1 + 1) begin : L99
			localparam y = _gv_y_1;
			for (_gv_x_1 = 0; _gv_x_1 < 5; _gv_x_1 = _gv_x_1 + 1) begin : L100
				localparam x = _gv_x_1;
				assign out[1599 - (64 * ((5 * y) + x)):(1599 - (64 * ((5 * y) + x))) - 63] = gg[x][y];
			end
		end
	endgenerate
endmodule
module tempsen (
	clk,
	in,
	out
);
	input clk;
	input [127:0] in;
	output wire [127:0] out;
	assign out = (in[127] == 1 ? in : ~in);
endmodule
module bscell (
	clk_i,
	rst_ni,
	mode_i,
	enable_i,
	shift_dr_i,
	capture_dr_i,
	update_dr_i,
	scan_in_i,
	jtagreg_in_i,
	scan_out_o,
	jtagreg_out_o
);
	input wire clk_i;
	input wire rst_ni;
	input wire mode_i;
	input wire enable_i;
	input wire shift_dr_i;
	input wire capture_dr_i;
	input wire update_dr_i;
	input wire scan_in_i;
	input wire jtagreg_in_i;
	output wire scan_out_o;
	output wire jtagreg_out_o;
	reg r_dataout;
	reg r_datasample;
	wire s_datasample_next;
	always @(negedge rst_ni or posedge clk_i)
		if (~rst_ni) begin
			r_datasample <= 1'b0;
			r_dataout <= 1'b0;
		end
		else begin
			if ((shift_dr_i | capture_dr_i) & enable_i)
				r_datasample <= s_datasample_next;
			if (update_dr_i & enable_i)
				r_dataout <= r_datasample;
		end
	assign s_datasample_next = (shift_dr_i ? scan_in_i : jtagreg_in_i);
	assign jtagreg_out_o = (mode_i ? r_dataout : jtagreg_in_i);
	assign scan_out_o = r_datasample;
endmodule
module jtagreg (
	clk_i,
	rst_ni,
	enable_i,
	capture_dr_i,
	shift_dr_i,
	update_dr_i,
	jtagreg_in_i,
	mode_i,
	scan_in_i,
	scan_out_o,
	jtagreg_out_o
);
	parameter JTAGREGSIZE = 96;
	parameter SYNC = 1;
	input wire clk_i;
	input wire rst_ni;
	input wire enable_i;
	input wire capture_dr_i;
	input wire shift_dr_i;
	input wire update_dr_i;
	input wire [JTAGREGSIZE - 1:0] jtagreg_in_i;
	input wire mode_i;
	input wire scan_in_i;
	output wire scan_out_o;
	output wire [JTAGREGSIZE - 1:0] jtagreg_out_o;
	wire [JTAGREGSIZE - 2:0] s_scanbit;
	wire scan_in_syn;
	bscell reg_bit_last(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.mode_i(mode_i),
		.enable_i(enable_i),
		.shift_dr_i(shift_dr_i),
		.capture_dr_i(capture_dr_i),
		.update_dr_i(update_dr_i),
		.scan_in_i(scan_in_syn),
		.jtagreg_in_i(jtagreg_in_i[JTAGREGSIZE - 1]),
		.scan_out_o(s_scanbit[0]),
		.jtagreg_out_o(jtagreg_out_o[JTAGREGSIZE - 1])
	);
	genvar _gv_i_11;
	generate
		for (_gv_i_11 = 1; _gv_i_11 < (JTAGREGSIZE - 1); _gv_i_11 = _gv_i_11 + 1) begin : genblk1
			localparam i = _gv_i_11;
			bscell reg_bit_mid(
				.clk_i(clk_i),
				.rst_ni(rst_ni),
				.mode_i(mode_i),
				.enable_i(enable_i),
				.shift_dr_i(shift_dr_i),
				.capture_dr_i(capture_dr_i),
				.update_dr_i(update_dr_i),
				.scan_in_i(s_scanbit[i - 1]),
				.jtagreg_in_i(jtagreg_in_i[(JTAGREGSIZE - 1) - i]),
				.scan_out_o(s_scanbit[i]),
				.jtagreg_out_o(jtagreg_out_o[(JTAGREGSIZE - 1) - i])
			);
		end
	endgenerate
	bscell reg_bit0(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.mode_i(mode_i),
		.enable_i(enable_i),
		.shift_dr_i(shift_dr_i),
		.capture_dr_i(capture_dr_i),
		.update_dr_i(update_dr_i),
		.scan_in_i(s_scanbit[JTAGREGSIZE - 2]),
		.jtagreg_in_i(jtagreg_in_i[0]),
		.scan_out_o(scan_out_o),
		.jtagreg_out_o(jtagreg_out_o[0])
	);
	generate
		if (SYNC == 1) begin : JTAG_SYNC
			jtag_sync jtag_sync_scanin(
				.clk_i(clk_i),
				.rst_ni(rst_ni),
				.tosynch(scan_in_i),
				.synched(scan_in_syn)
			);
		end
		else begin : JTAG_NO_SYNC
			assign scan_in_syn = scan_in_i;
		end
	endgenerate
endmodule
module tap_top (
	tms_i,
	tck_i,
	rst_ni,
	td_i,
	td_o,
	shift_dr_o,
	update_dr_o,
	capture_dr_o,
	axireg_sel_o,
	bbmuxreg_sel_o,
	clkgatereg_sel_o,
	confreg_sel_o,
	testmodereg_sel_o,
	bistreg_sel_o,
	scan_in_o,
	axireg_out_i,
	bbmuxreg_out_i,
	clkgatereg_out_i,
	confreg_out_i,
	testmodereg_out_i,
	bistreg_out_i
);
	input tms_i;
	input tck_i;
	input rst_ni;
	input td_i;
	output reg td_o;
	output wire shift_dr_o;
	output wire update_dr_o;
	output wire capture_dr_o;
	output wire axireg_sel_o;
	output wire bbmuxreg_sel_o;
	output wire clkgatereg_sel_o;
	output wire confreg_sel_o;
	output wire testmodereg_sel_o;
	output wire bistreg_sel_o;
	output wire scan_in_o;
	input axireg_out_i;
	input bbmuxreg_out_i;
	input clkgatereg_out_i;
	input confreg_out_i;
	input testmodereg_out_i;
	input bistreg_out_i;
	reg test_logic_reset;
	reg run_test_idle;
	reg sel_dr_scan;
	reg capture_dr;
	reg shift_dr;
	reg exit1_dr;
	reg pause_dr;
	reg exit2_dr;
	reg update_dr;
	reg sel_ir_scan;
	reg capture_ir;
	reg shift_ir;
	reg shift_ir_neg;
	reg exit1_ir;
	reg pause_ir;
	reg exit2_ir;
	reg update_ir;
	reg idcode_sel;
	reg axireg_sel;
	reg bbmuxreg_sel;
	reg clkgatereg_sel;
	reg confreg_sel;
	reg testmodereg_sel;
	reg bistreg_sel;
	reg bypass_sel;
	reg tdo_comb;
	reg tms_q1;
	reg tms_q2;
	reg tms_q3;
	reg tms_q4;
	wire tms_reset;
	assign scan_in_o = td_i;
	assign shift_dr_o = shift_dr;
	assign update_dr_o = update_dr;
	assign capture_dr_o = capture_dr;
	assign axireg_sel_o = axireg_sel;
	assign bbmuxreg_sel_o = bbmuxreg_sel;
	assign clkgatereg_sel_o = clkgatereg_sel;
	assign confreg_sel_o = confreg_sel;
	assign testmodereg_sel_o = testmodereg_sel;
	assign bistreg_sel_o = bistreg_sel;
	always @(posedge tck_i) begin
		tms_q1 <= tms_i;
		tms_q2 <= tms_q1;
		tms_q3 <= tms_q2;
		tms_q4 <= tms_q3;
	end
	assign tms_reset = (((tms_q1 & tms_q2) & tms_q3) & tms_q4) & tms_i;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			test_logic_reset <= 1'b1;
		else if (tms_reset)
			test_logic_reset <= 1'b1;
		else if (tms_i & (test_logic_reset | sel_ir_scan))
			test_logic_reset <= 1'b1;
		else
			test_logic_reset <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			run_test_idle <= 1'b0;
		else if (tms_reset)
			run_test_idle <= 1'b0;
		else if (~tms_i & (((test_logic_reset | run_test_idle) | update_dr) | update_ir))
			run_test_idle <= 1'b1;
		else
			run_test_idle <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			sel_dr_scan <= 1'b0;
		else if (tms_reset)
			sel_dr_scan <= 1'b0;
		else if (tms_i & ((run_test_idle | update_dr) | update_ir))
			sel_dr_scan <= 1'b1;
		else
			sel_dr_scan <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			capture_dr <= 1'b0;
		else if (tms_reset)
			capture_dr <= 1'b0;
		else if (~tms_i & sel_dr_scan)
			capture_dr <= 1'b1;
		else
			capture_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			shift_dr <= 1'b0;
		else if (tms_reset)
			shift_dr <= 1'b0;
		else if (~tms_i & ((capture_dr | shift_dr) | exit2_dr))
			shift_dr <= 1'b1;
		else
			shift_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			exit1_dr <= 1'b0;
		else if (tms_reset)
			exit1_dr <= 1'b0;
		else if (tms_i & (capture_dr | shift_dr))
			exit1_dr <= 1'b1;
		else
			exit1_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			pause_dr <= 1'b0;
		else if (tms_reset)
			pause_dr <= 1'b0;
		else if (~tms_i & (exit1_dr | pause_dr))
			pause_dr <= 1'b1;
		else
			pause_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			exit2_dr <= 1'b0;
		else if (tms_reset)
			exit2_dr <= 1'b0;
		else if (tms_i & pause_dr)
			exit2_dr <= 1'b1;
		else
			exit2_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			update_dr <= 1'b0;
		else if (tms_reset)
			update_dr <= 1'b0;
		else if (tms_i & (exit1_dr | exit2_dr))
			update_dr <= 1'b1;
		else
			update_dr <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			sel_ir_scan <= 1'b0;
		else if (tms_reset)
			sel_ir_scan <= 1'b0;
		else if (tms_i & sel_dr_scan)
			sel_ir_scan <= 1'b1;
		else
			sel_ir_scan <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			capture_ir <= 1'b0;
		else if (tms_reset)
			capture_ir <= 1'b0;
		else if (~tms_i & sel_ir_scan)
			capture_ir <= 1'b1;
		else
			capture_ir <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			shift_ir <= 1'b0;
		else if (tms_reset)
			shift_ir <= 1'b0;
		else if (~tms_i & ((capture_ir | shift_ir) | exit2_ir))
			shift_ir <= 1'b1;
		else
			shift_ir <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			exit1_ir <= 1'b0;
		else if (tms_reset)
			exit1_ir <= 1'b0;
		else if (tms_i & (capture_ir | shift_ir))
			exit1_ir <= 1'b1;
		else
			exit1_ir <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			pause_ir <= 1'b0;
		else if (tms_reset)
			pause_ir <= 1'b0;
		else if (~tms_i & (exit1_ir | pause_ir))
			pause_ir <= 1'b1;
		else
			pause_ir <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			exit2_ir <= 1'b0;
		else if (tms_reset)
			exit2_ir <= 1'b0;
		else if (tms_i & pause_ir)
			exit2_ir <= 1'b1;
		else
			exit2_ir <= 1'b0;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			update_ir <= 1'b0;
		else if (tms_reset)
			update_ir <= 1'b0;
		else if (tms_i & (exit1_ir | exit2_ir))
			update_ir <= 1'b1;
		else
			update_ir <= 1'b0;
	reg [3:0] jtag_ir;
	reg [3:0] latched_jtag_ir;
	reg [3:0] latched_jtag_ir_neg;
	wire instruction_tdo;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			jtag_ir[3:0] <= 4'b0000;
		else if (capture_ir)
			jtag_ir <= 4'b0101;
		else if (shift_ir)
			jtag_ir[3:0] <= {td_i, jtag_ir[3:1]};
	assign instruction_tdo = jtag_ir[0];
	reg [31:0] idcode_reg;
	wire idcode_tdo;
	always @(posedge tck_i)
		if (idcode_sel & shift_dr)
			idcode_reg <= {td_i, idcode_reg[31:1]};
		else
			idcode_reg <= 32'h149511c3;
	assign idcode_tdo = idcode_reg;
	wire bypassed_tdo;
	reg bypass_reg;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			bypass_reg <= 1'b0;
		else if (shift_dr)
			bypass_reg <= td_i;
	assign bypassed_tdo = bypass_reg;
	always @(posedge tck_i or negedge rst_ni)
		if (~rst_ni)
			latched_jtag_ir <= 4'b0010;
		else if (tms_reset)
			latched_jtag_ir <= 4'b0010;
		else if (update_ir)
			latched_jtag_ir <= jtag_ir;
	always @(latched_jtag_ir) begin
		idcode_sel = 1'b0;
		axireg_sel = 1'b0;
		bbmuxreg_sel = 1'b0;
		clkgatereg_sel = 1'b0;
		confreg_sel = 1'b0;
		testmodereg_sel = 1'b0;
		bistreg_sel = 1'b0;
		bypass_sel = 1'b0;
		case (latched_jtag_ir)
			4'b0010: idcode_sel = 1'b1;
			4'b0100: axireg_sel = 1'b1;
			4'b0101: bbmuxreg_sel = 1'b1;
			4'b0110: clkgatereg_sel = 1'b1;
			4'b0111: confreg_sel = 1'b1;
			4'b1000: testmodereg_sel = 1'b1;
			4'b1001: bistreg_sel = 1'b1;
			4'b1111: bypass_sel = 1'b1;
			default: bypass_sel = 1'b1;
		endcase
	end
	always @(shift_ir_neg or exit1_ir or instruction_tdo or latched_jtag_ir_neg or idcode_tdo or bbmuxreg_out_i or clkgatereg_out_i or confreg_out_i or testmodereg_out_i or bistreg_out_i or axireg_out_i or bypassed_tdo)
		if (shift_ir_neg)
			tdo_comb = instruction_tdo;
		else
			case (latched_jtag_ir_neg)
				4'b0010: tdo_comb = idcode_tdo;
				4'b0100: tdo_comb = axireg_out_i;
				4'b0101: tdo_comb = bbmuxreg_out_i;
				4'b0110: tdo_comb = clkgatereg_out_i;
				4'b0111: tdo_comb = confreg_out_i;
				4'b1000: tdo_comb = testmodereg_out_i;
				4'b1001: tdo_comb = bistreg_out_i;
				default: tdo_comb = bypassed_tdo;
			endcase
	always @(negedge tck_i) td_o <= tdo_comb;
	always @(negedge tck_i) begin
		shift_ir_neg <= shift_ir;
		latched_jtag_ir_neg <= latched_jtag_ir;
	end
endmodule
module soc_interconnect (
	clk,
	rst_n,
	test_en_i,
	L2_D_o,
	L2_A_o,
	L2_CEN_o,
	L2_WEN_o,
	L2_BE_o,
	L2_ID_o,
	L2_Q_i,
	FC_DATA_req_i,
	FC_DATA_add_i,
	FC_DATA_wen_i,
	FC_DATA_wdata_i,
	FC_DATA_be_i,
	FC_DATA_aux_i,
	FC_DATA_gnt_o,
	FC_DATA_r_aux_o,
	FC_DATA_r_valid_o,
	FC_DATA_r_rdata_o,
	FC_DATA_r_opc_o,
	FC_INSTR_req_i,
	FC_INSTR_add_i,
	FC_INSTR_wen_i,
	FC_INSTR_wdata_i,
	FC_INSTR_be_i,
	FC_INSTR_aux_i,
	FC_INSTR_gnt_o,
	FC_INSTR_r_aux_o,
	FC_INSTR_r_valid_o,
	FC_INSTR_r_rdata_o,
	FC_INSTR_r_opc_o,
	UDMA_TX_req_i,
	UDMA_TX_add_i,
	UDMA_TX_wen_i,
	UDMA_TX_wdata_i,
	UDMA_TX_be_i,
	UDMA_TX_aux_i,
	UDMA_TX_gnt_o,
	UDMA_TX_r_aux_o,
	UDMA_TX_r_valid_o,
	UDMA_TX_r_rdata_o,
	UDMA_TX_r_opc_o,
	UDMA_RX_req_i,
	UDMA_RX_add_i,
	UDMA_RX_wen_i,
	UDMA_RX_wdata_i,
	UDMA_RX_be_i,
	UDMA_RX_aux_i,
	UDMA_RX_gnt_o,
	UDMA_RX_r_aux_o,
	UDMA_RX_r_valid_o,
	UDMA_RX_r_rdata_o,
	UDMA_RX_r_opc_o,
	DBG_RX_req_i,
	DBG_RX_add_i,
	DBG_RX_wen_i,
	DBG_RX_wdata_i,
	DBG_RX_be_i,
	DBG_RX_aux_i,
	DBG_RX_gnt_o,
	DBG_RX_r_aux_o,
	DBG_RX_r_valid_o,
	DBG_RX_r_rdata_o,
	DBG_RX_r_opc_o,
	HWPE_req_i,
	HWPE_add_i,
	HWPE_wen_i,
	HWPE_wdata_i,
	HWPE_be_i,
	HWPE_aux_i,
	HWPE_gnt_o,
	HWPE_r_aux_o,
	HWPE_r_valid_o,
	HWPE_r_rdata_o,
	HWPE_r_opc_o,
	AXI_Slave_aw_addr_i,
	AXI_Slave_aw_prot_i,
	AXI_Slave_aw_region_i,
	AXI_Slave_aw_len_i,
	AXI_Slave_aw_size_i,
	AXI_Slave_aw_burst_i,
	AXI_Slave_aw_lock_i,
	AXI_Slave_aw_cache_i,
	AXI_Slave_aw_qos_i,
	AXI_Slave_aw_id_i,
	AXI_Slave_aw_user_i,
	AXI_Slave_aw_valid_i,
	AXI_Slave_aw_ready_o,
	AXI_Slave_ar_addr_i,
	AXI_Slave_ar_prot_i,
	AXI_Slave_ar_region_i,
	AXI_Slave_ar_len_i,
	AXI_Slave_ar_size_i,
	AXI_Slave_ar_burst_i,
	AXI_Slave_ar_lock_i,
	AXI_Slave_ar_cache_i,
	AXI_Slave_ar_qos_i,
	AXI_Slave_ar_id_i,
	AXI_Slave_ar_user_i,
	AXI_Slave_ar_valid_i,
	AXI_Slave_ar_ready_o,
	AXI_Slave_w_user_i,
	AXI_Slave_w_data_i,
	AXI_Slave_w_strb_i,
	AXI_Slave_w_last_i,
	AXI_Slave_w_valid_i,
	AXI_Slave_w_ready_o,
	AXI_Slave_b_id_o,
	AXI_Slave_b_resp_o,
	AXI_Slave_b_user_o,
	AXI_Slave_b_valid_o,
	AXI_Slave_b_ready_i,
	AXI_Slave_r_id_o,
	AXI_Slave_r_user_o,
	AXI_Slave_r_data_o,
	AXI_Slave_r_resp_o,
	AXI_Slave_r_last_o,
	AXI_Slave_r_valid_o,
	AXI_Slave_r_ready_i,
	APB_PADDR_o,
	APB_PWDATA_o,
	APB_PWRITE_o,
	APB_PSEL_o,
	APB_PENABLE_o,
	APB_PRDATA_i,
	APB_PREADY_i,
	APB_PSLVERR_i,
	AXI_Master_aw_id_o,
	AXI_Master_aw_addr_o,
	AXI_Master_aw_len_o,
	AXI_Master_aw_size_o,
	AXI_Master_aw_burst_o,
	AXI_Master_aw_lock_o,
	AXI_Master_aw_cache_o,
	AXI_Master_aw_prot_o,
	AXI_Master_aw_region_o,
	AXI_Master_aw_user_o,
	AXI_Master_aw_qos_o,
	AXI_Master_aw_valid_o,
	AXI_Master_aw_ready_i,
	AXI_Master_w_data_o,
	AXI_Master_w_strb_o,
	AXI_Master_w_last_o,
	AXI_Master_w_user_o,
	AXI_Master_w_valid_o,
	AXI_Master_w_ready_i,
	AXI_Master_b_id_i,
	AXI_Master_b_resp_i,
	AXI_Master_b_valid_i,
	AXI_Master_b_user_i,
	AXI_Master_b_ready_o,
	AXI_Master_ar_id_o,
	AXI_Master_ar_addr_o,
	AXI_Master_ar_len_o,
	AXI_Master_ar_size_o,
	AXI_Master_ar_burst_o,
	AXI_Master_ar_lock_o,
	AXI_Master_ar_cache_o,
	AXI_Master_ar_prot_o,
	AXI_Master_ar_region_o,
	AXI_Master_ar_user_o,
	AXI_Master_ar_qos_o,
	AXI_Master_ar_valid_o,
	AXI_Master_ar_ready_i,
	AXI_Master_r_id_i,
	AXI_Master_r_data_i,
	AXI_Master_r_resp_i,
	AXI_Master_r_last_i,
	AXI_Master_r_user_i,
	AXI_Master_r_valid_i,
	AXI_Master_r_ready_o,
	rom_csn_o,
	rom_add_o,
	rom_rdata_i,
	L2_pri_D_o,
	L2_pri_A_o,
	L2_pri_CEN_o,
	L2_pri_WEN_o,
	L2_pri_BE_o,
	L2_pri_Q_i
);
	reg _sv2v_0;
	parameter USE_AXI = 1;
	parameter ADDR_WIDTH = 32;
	parameter N_HWPE_PORTS = 4;
	parameter N_MASTER_32 = 5 + N_HWPE_PORTS;
	parameter N_MASTER_AXI_64 = 1;
	parameter DATA_WIDTH = 32;
	parameter BE_WIDTH = DATA_WIDTH / 8;
	parameter ID_WIDTH = N_MASTER_32 + (N_MASTER_AXI_64 * 4);
	parameter AUX_WIDTH = 8;
	parameter N_L2_BANKS = 4;
	parameter N_L2_BANKS_PRI = 2;
	parameter ADDR_L2_WIDTH = 12;
	parameter ADDR_L2_PRI_WIDTH = 12;
	parameter ROM_ADDR_WIDTH = 10;
	parameter AXI_32_ID_WIDTH = 12;
	parameter AXI_32_USER_WIDTH = 6;
	parameter AXI_ADDR_WIDTH = 32;
	parameter AXI_DATA_WIDTH = 64;
	parameter AXI_STRB_WIDTH = 8;
	parameter AXI_USER_WIDTH = 6;
	parameter AXI_ID_WIDTH = 7;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	output reg [(N_L2_BANKS * DATA_WIDTH) - 1:0] L2_D_o;
	output reg [(N_L2_BANKS * ADDR_L2_WIDTH) - 1:0] L2_A_o;
	output reg [N_L2_BANKS - 1:0] L2_CEN_o;
	output reg [N_L2_BANKS - 1:0] L2_WEN_o;
	output reg [(N_L2_BANKS * BE_WIDTH) - 1:0] L2_BE_o;
	output reg [(N_L2_BANKS * 32) - 1:0] L2_ID_o;
	input wire [(N_L2_BANKS * DATA_WIDTH) - 1:0] L2_Q_i;
	input wire FC_DATA_req_i;
	input wire [ADDR_WIDTH - 1:0] FC_DATA_add_i;
	input wire FC_DATA_wen_i;
	input wire [DATA_WIDTH - 1:0] FC_DATA_wdata_i;
	input wire [BE_WIDTH - 1:0] FC_DATA_be_i;
	input wire [AUX_WIDTH - 1:0] FC_DATA_aux_i;
	output wire FC_DATA_gnt_o;
	output wire [AUX_WIDTH - 1:0] FC_DATA_r_aux_o;
	output wire FC_DATA_r_valid_o;
	output wire [DATA_WIDTH - 1:0] FC_DATA_r_rdata_o;
	output wire FC_DATA_r_opc_o;
	input wire FC_INSTR_req_i;
	input wire [ADDR_WIDTH - 1:0] FC_INSTR_add_i;
	input wire FC_INSTR_wen_i;
	input wire [DATA_WIDTH - 1:0] FC_INSTR_wdata_i;
	input wire [BE_WIDTH - 1:0] FC_INSTR_be_i;
	input wire [AUX_WIDTH - 1:0] FC_INSTR_aux_i;
	output wire FC_INSTR_gnt_o;
	output wire [AUX_WIDTH - 1:0] FC_INSTR_r_aux_o;
	output wire FC_INSTR_r_valid_o;
	output wire [DATA_WIDTH - 1:0] FC_INSTR_r_rdata_o;
	output wire FC_INSTR_r_opc_o;
	input wire UDMA_TX_req_i;
	input wire [ADDR_WIDTH - 1:0] UDMA_TX_add_i;
	input wire UDMA_TX_wen_i;
	input wire [DATA_WIDTH - 1:0] UDMA_TX_wdata_i;
	input wire [BE_WIDTH - 1:0] UDMA_TX_be_i;
	input wire [AUX_WIDTH - 1:0] UDMA_TX_aux_i;
	output wire UDMA_TX_gnt_o;
	output wire [AUX_WIDTH - 1:0] UDMA_TX_r_aux_o;
	output wire UDMA_TX_r_valid_o;
	output wire [DATA_WIDTH - 1:0] UDMA_TX_r_rdata_o;
	output wire UDMA_TX_r_opc_o;
	input wire UDMA_RX_req_i;
	input wire [ADDR_WIDTH - 1:0] UDMA_RX_add_i;
	input wire UDMA_RX_wen_i;
	input wire [DATA_WIDTH - 1:0] UDMA_RX_wdata_i;
	input wire [BE_WIDTH - 1:0] UDMA_RX_be_i;
	input wire [AUX_WIDTH - 1:0] UDMA_RX_aux_i;
	output wire UDMA_RX_gnt_o;
	output wire [AUX_WIDTH - 1:0] UDMA_RX_r_aux_o;
	output wire UDMA_RX_r_valid_o;
	output wire [DATA_WIDTH - 1:0] UDMA_RX_r_rdata_o;
	output wire UDMA_RX_r_opc_o;
	input wire DBG_RX_req_i;
	input wire [ADDR_WIDTH - 1:0] DBG_RX_add_i;
	input wire DBG_RX_wen_i;
	input wire [DATA_WIDTH - 1:0] DBG_RX_wdata_i;
	input wire [BE_WIDTH - 1:0] DBG_RX_be_i;
	input wire [AUX_WIDTH - 1:0] DBG_RX_aux_i;
	output wire DBG_RX_gnt_o;
	output wire [AUX_WIDTH - 1:0] DBG_RX_r_aux_o;
	output wire DBG_RX_r_valid_o;
	output wire [DATA_WIDTH - 1:0] DBG_RX_r_rdata_o;
	output wire DBG_RX_r_opc_o;
	input wire [N_HWPE_PORTS - 1:0] HWPE_req_i;
	input wire [(N_HWPE_PORTS * ADDR_WIDTH) - 1:0] HWPE_add_i;
	input wire [N_HWPE_PORTS - 1:0] HWPE_wen_i;
	input wire [(N_HWPE_PORTS * DATA_WIDTH) - 1:0] HWPE_wdata_i;
	input wire [(N_HWPE_PORTS * BE_WIDTH) - 1:0] HWPE_be_i;
	input wire [(N_HWPE_PORTS * AUX_WIDTH) - 1:0] HWPE_aux_i;
	output wire [N_HWPE_PORTS - 1:0] HWPE_gnt_o;
	output wire [(N_HWPE_PORTS * AUX_WIDTH) - 1:0] HWPE_r_aux_o;
	output wire [N_HWPE_PORTS - 1:0] HWPE_r_valid_o;
	output wire [(N_HWPE_PORTS * DATA_WIDTH) - 1:0] HWPE_r_rdata_o;
	output wire [N_HWPE_PORTS - 1:0] HWPE_r_opc_o;
	input wire [AXI_ADDR_WIDTH - 1:0] AXI_Slave_aw_addr_i;
	input wire [2:0] AXI_Slave_aw_prot_i;
	input wire [3:0] AXI_Slave_aw_region_i;
	input wire [7:0] AXI_Slave_aw_len_i;
	input wire [2:0] AXI_Slave_aw_size_i;
	input wire [1:0] AXI_Slave_aw_burst_i;
	input wire AXI_Slave_aw_lock_i;
	input wire [3:0] AXI_Slave_aw_cache_i;
	input wire [3:0] AXI_Slave_aw_qos_i;
	input wire [AXI_ID_WIDTH - 1:0] AXI_Slave_aw_id_i;
	input wire [AXI_USER_WIDTH - 1:0] AXI_Slave_aw_user_i;
	input wire AXI_Slave_aw_valid_i;
	output wire AXI_Slave_aw_ready_o;
	input wire [AXI_ADDR_WIDTH - 1:0] AXI_Slave_ar_addr_i;
	input wire [2:0] AXI_Slave_ar_prot_i;
	input wire [3:0] AXI_Slave_ar_region_i;
	input wire [7:0] AXI_Slave_ar_len_i;
	input wire [2:0] AXI_Slave_ar_size_i;
	input wire [1:0] AXI_Slave_ar_burst_i;
	input wire AXI_Slave_ar_lock_i;
	input wire [3:0] AXI_Slave_ar_cache_i;
	input wire [3:0] AXI_Slave_ar_qos_i;
	input wire [AXI_ID_WIDTH - 1:0] AXI_Slave_ar_id_i;
	input wire [AXI_USER_WIDTH - 1:0] AXI_Slave_ar_user_i;
	input wire AXI_Slave_ar_valid_i;
	output wire AXI_Slave_ar_ready_o;
	input wire [AXI_USER_WIDTH - 1:0] AXI_Slave_w_user_i;
	input wire [AXI_DATA_WIDTH - 1:0] AXI_Slave_w_data_i;
	input wire [AXI_STRB_WIDTH - 1:0] AXI_Slave_w_strb_i;
	input wire AXI_Slave_w_last_i;
	input wire AXI_Slave_w_valid_i;
	output wire AXI_Slave_w_ready_o;
	output wire [AXI_ID_WIDTH - 1:0] AXI_Slave_b_id_o;
	output wire [1:0] AXI_Slave_b_resp_o;
	output wire [AXI_USER_WIDTH - 1:0] AXI_Slave_b_user_o;
	output wire AXI_Slave_b_valid_o;
	input wire AXI_Slave_b_ready_i;
	output wire [AXI_ID_WIDTH - 1:0] AXI_Slave_r_id_o;
	output wire [AXI_USER_WIDTH - 1:0] AXI_Slave_r_user_o;
	output wire [AXI_DATA_WIDTH - 1:0] AXI_Slave_r_data_o;
	output wire [1:0] AXI_Slave_r_resp_o;
	output wire AXI_Slave_r_last_o;
	output wire AXI_Slave_r_valid_o;
	input wire AXI_Slave_r_ready_i;
	output wire [ADDR_WIDTH - 1:0] APB_PADDR_o;
	output wire [DATA_WIDTH - 1:0] APB_PWDATA_o;
	output wire APB_PWRITE_o;
	output wire APB_PSEL_o;
	output wire APB_PENABLE_o;
	input wire [DATA_WIDTH - 1:0] APB_PRDATA_i;
	input wire APB_PREADY_i;
	input wire APB_PSLVERR_i;
	output wire [AXI_32_ID_WIDTH - 1:0] AXI_Master_aw_id_o;
	output wire [ADDR_WIDTH - 1:0] AXI_Master_aw_addr_o;
	output wire [7:0] AXI_Master_aw_len_o;
	output wire [2:0] AXI_Master_aw_size_o;
	output wire [1:0] AXI_Master_aw_burst_o;
	output wire AXI_Master_aw_lock_o;
	output wire [3:0] AXI_Master_aw_cache_o;
	output wire [2:0] AXI_Master_aw_prot_o;
	output wire [3:0] AXI_Master_aw_region_o;
	output wire [AXI_32_USER_WIDTH - 1:0] AXI_Master_aw_user_o;
	output wire [3:0] AXI_Master_aw_qos_o;
	output wire AXI_Master_aw_valid_o;
	input wire AXI_Master_aw_ready_i;
	output wire [DATA_WIDTH - 1:0] AXI_Master_w_data_o;
	output wire [BE_WIDTH - 1:0] AXI_Master_w_strb_o;
	output wire AXI_Master_w_last_o;
	output wire [AXI_32_USER_WIDTH - 1:0] AXI_Master_w_user_o;
	output wire AXI_Master_w_valid_o;
	input wire AXI_Master_w_ready_i;
	input wire [AXI_32_ID_WIDTH - 1:0] AXI_Master_b_id_i;
	input wire [1:0] AXI_Master_b_resp_i;
	input wire AXI_Master_b_valid_i;
	input wire [AXI_32_USER_WIDTH - 1:0] AXI_Master_b_user_i;
	output wire AXI_Master_b_ready_o;
	output wire [AXI_32_ID_WIDTH - 1:0] AXI_Master_ar_id_o;
	output wire [ADDR_WIDTH - 1:0] AXI_Master_ar_addr_o;
	output wire [7:0] AXI_Master_ar_len_o;
	output wire [2:0] AXI_Master_ar_size_o;
	output wire [1:0] AXI_Master_ar_burst_o;
	output wire AXI_Master_ar_lock_o;
	output wire [3:0] AXI_Master_ar_cache_o;
	output wire [2:0] AXI_Master_ar_prot_o;
	output wire [3:0] AXI_Master_ar_region_o;
	output wire [AXI_32_USER_WIDTH - 1:0] AXI_Master_ar_user_o;
	output wire [3:0] AXI_Master_ar_qos_o;
	output wire AXI_Master_ar_valid_o;
	input wire AXI_Master_ar_ready_i;
	input wire [AXI_32_ID_WIDTH - 1:0] AXI_Master_r_id_i;
	input wire [DATA_WIDTH - 1:0] AXI_Master_r_data_i;
	input wire [1:0] AXI_Master_r_resp_i;
	input wire AXI_Master_r_last_i;
	input wire [AXI_32_USER_WIDTH - 1:0] AXI_Master_r_user_i;
	input wire AXI_Master_r_valid_i;
	output wire AXI_Master_r_ready_o;
	output wire rom_csn_o;
	output wire [ROM_ADDR_WIDTH - 1:0] rom_add_o;
	input wire [DATA_WIDTH - 1:0] rom_rdata_i;
	output wire [(N_L2_BANKS_PRI * DATA_WIDTH) - 1:0] L2_pri_D_o;
	output wire [(N_L2_BANKS_PRI * ADDR_L2_PRI_WIDTH) - 1:0] L2_pri_A_o;
	output wire [N_L2_BANKS_PRI - 1:0] L2_pri_CEN_o;
	output wire [N_L2_BANKS_PRI - 1:0] L2_pri_WEN_o;
	output wire [(N_L2_BANKS_PRI * BE_WIDTH) - 1:0] L2_pri_BE_o;
	input wire [(N_L2_BANKS_PRI * DATA_WIDTH) - 1:0] L2_pri_Q_i;
	localparam N_CH0 = N_MASTER_32;
	localparam N_CH1 = N_MASTER_AXI_64 * 4;
	localparam N_CH0_BRIDGE = N_CH0;
	localparam N_CH1_BRIDGE = N_CH1;
	localparam PER_ID_WIDTH = N_CH0_BRIDGE + N_CH1_BRIDGE;
	localparam N_PERIPHS = 3 + N_L2_BANKS_PRI;
	localparam L2_OFFSET_PRI = 15'h1000;
	localparam [(N_PERIPHS * ADDR_WIDTH) - 1:0] PER_START_ADDR = 160'h1c0080001c0000001a000000100000001a100000;
	localparam [(N_PERIPHS * ADDR_WIDTH) - 1:0] PER_END_ADDR = 160'h1c0100001c0080001a040000104000001a200000;
	localparam [ADDR_WIDTH - 1:0] TCDM_START_ADDR = 32'h1c010000;
	localparam [ADDR_WIDTH - 1:0] TCDM_END_ADDR = 32'h1c082000;
	wire [N_MASTER_32 - 1:0] FC_data_req_INT_32;
	wire [(N_MASTER_32 * ADDR_WIDTH) - 1:0] FC_data_add_INT_32;
	reg [ADDR_WIDTH - 1:0] FC_DATA_add_int;
	wire [N_MASTER_32 - 1:0] FC_data_wen_INT_32;
	wire [(N_MASTER_32 * DATA_WIDTH) - 1:0] FC_data_wdata_INT_32;
	wire [(N_MASTER_32 * BE_WIDTH) - 1:0] FC_data_be_INT_32;
	wire [(N_MASTER_32 * AUX_WIDTH) - 1:0] FC_data_aux_INT_32;
	wire [N_MASTER_32 - 1:0] FC_data_gnt_INT_32;
	wire [(N_MASTER_32 * AUX_WIDTH) - 1:0] FC_data_r_aux_INT_32;
	wire [N_MASTER_32 - 1:0] FC_data_r_valid_INT_32;
	wire [(N_MASTER_32 * DATA_WIDTH) - 1:0] FC_data_r_rdata_INT_32;
	wire [N_MASTER_32 - 1:0] FC_data_r_opc_INT_32;
	wire [(N_MASTER_AXI_64 * 4) - 1:0] AXI_data_req_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * ADDR_WIDTH) - 1:0] AXI_data_add_INT_64;
	wire [(N_MASTER_AXI_64 * 4) - 1:0] AXI_data_wen_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * DATA_WIDTH) - 1:0] AXI_data_wdata_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * BE_WIDTH) - 1:0] AXI_data_be_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * AUX_WIDTH) - 1:0] AXI_data_aux_INT_64;
	wire [(N_MASTER_AXI_64 * 4) - 1:0] AXI_data_gnt_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * AUX_WIDTH) - 1:0] AXI_data_r_aux_INT_64;
	wire [(N_MASTER_AXI_64 * 4) - 1:0] AXI_data_r_valid_INT_64;
	wire [((N_MASTER_AXI_64 * 4) * DATA_WIDTH) - 1:0] AXI_data_r_rdata_INT_64;
	wire [(N_MASTER_AXI_64 * 4) - 1:0] AXI_data_r_opc_INT_64;
	wire [(N_CH0 + N_CH1) - 1:0] PER_data_req_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * ADDR_WIDTH) - 1:0] PER_data_add_DEM_2_L2_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] PER_data_wen_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] PER_data_wdata_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * BE_WIDTH) - 1:0] PER_data_be_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * AUX_WIDTH) - 1:0] PER_data_aux_DEM_2_L2_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] PER_data_gnt_DEM_2_L2_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] PER_data_r_valid_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] PER_data_r_rdata_DEM_2_L2_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] PER_data_r_opc_DEM_2_L2_XBAR;
	wire [((N_CH0 + N_CH1) * AUX_WIDTH) - 1:0] PER_data_r_aux_DEM_2_L2_XBAR;
	wire [N_PERIPHS - 1:0] PER_data_req_TO_BRIDGE;
	wire [(N_PERIPHS * ADDR_WIDTH) - 1:0] PER_data_add_TO_BRIDGE;
	wire [N_PERIPHS - 1:0] PER_data_wen_TO_BRIDGE;
	wire [(N_PERIPHS * DATA_WIDTH) - 1:0] PER_data_wdata_TO_BRIDGE;
	wire [(N_PERIPHS * BE_WIDTH) - 1:0] PER_data_be_TO_BRIDGE;
	wire [(N_PERIPHS * PER_ID_WIDTH) - 1:0] PER_data_ID_TO_BRIDGE;
	wire [(N_PERIPHS * AUX_WIDTH) - 1:0] PER_data_aux_TO_BRIDGE;
	wire [N_PERIPHS - 1:0] PER_data_gnt_TO_BRIDGE;
	wire [(N_PERIPHS * DATA_WIDTH) - 1:0] PER_data_r_rdata_TO_BRIDGE;
	reg [N_PERIPHS - 1:0] PER_data_r_valid_TO_BRIDGE;
	reg [(N_PERIPHS * PER_ID_WIDTH) - 1:0] PER_data_r_ID_TO_BRIDGE;
	reg [N_PERIPHS - 1:0] PER_data_r_opc_TO_BRIDGE;
	reg [(N_PERIPHS * AUX_WIDTH) - 1:0] PER_data_r_aux_TO_BRIDGE;
	wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] TCDM_data_wdata_DEM_TO_XBAR;
	wire [((N_CH0 + N_CH1) * ADDR_WIDTH) - 1:0] TCDM_data_add_DEM_TO_XBAR;
	wire [((N_CH0 + N_CH1) * (ADDR_L2_WIDTH + $clog2(N_L2_BANKS))) - 1:0] TCDM_data_add_DEM_TO_XBAR_resized;
	wire [(N_CH0 + N_CH1) - 1:0] TCDM_data_req_DEM_TO_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] TCDM_data_wen_DEM_TO_XBAR;
	wire [((N_CH0 + N_CH1) * BE_WIDTH) - 1:0] TCDM_data_be_DEM_TO_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] TCDM_data_gnt_DEM_TO_XBAR;
	wire [((N_CH0 + N_CH1) * DATA_WIDTH) - 1:0] TCDM_data_r_rdata_DEM_TO_XBAR;
	wire [(N_CH0 + N_CH1) - 1:0] TCDM_data_r_valid_DEM_TO_XBAR;
	wire [(N_L2_BANKS * DATA_WIDTH) - 1:0] TCDM_data_wdata_TO_MEM;
	wire [(N_L2_BANKS * ADDR_L2_WIDTH) - 1:0] TCDM_data_add_TO_MEM;
	wire [N_L2_BANKS - 1:0] TCDM_data_req_TO_MEM;
	wire [N_L2_BANKS - 1:0] TCDM_data_wen_TO_MEM;
	wire [(N_L2_BANKS * BE_WIDTH) - 1:0] TCDM_data_be_TO_MEM;
	wire [(N_L2_BANKS * ID_WIDTH) - 1:0] TCDM_data_ID_TO_MEM;
	reg [(N_L2_BANKS * DATA_WIDTH) - 1:0] TCDM_data_rdata_TO_MEM;
	reg [N_L2_BANKS - 1:0] TCDM_data_rvalid_TO_MEM;
	reg [(N_L2_BANKS * ID_WIDTH) - 1:0] TCDM_data_rID_TO_MEM;
	assign rom_csn_o = ~PER_data_req_TO_BRIDGE[2];
	assign rom_add_o = PER_data_add_TO_BRIDGE[2 * ADDR_WIDTH+:ADDR_WIDTH];
	assign PER_data_r_rdata_TO_BRIDGE[2 * DATA_WIDTH+:DATA_WIDTH] = rom_rdata_i;
	assign PER_data_gnt_TO_BRIDGE[2] = 1'b1;
	always @(posedge clk or negedge rst_n) begin : proc_
		if (~rst_n)
			PER_data_r_valid_TO_BRIDGE[2] = 1'sb0;
		else begin
			PER_data_r_ID_TO_BRIDGE[2 * PER_ID_WIDTH+:PER_ID_WIDTH] = PER_data_ID_TO_BRIDGE[2 * PER_ID_WIDTH+:PER_ID_WIDTH];
			PER_data_r_valid_TO_BRIDGE[2] = PER_data_req_TO_BRIDGE[2];
			PER_data_r_aux_TO_BRIDGE[2 * AUX_WIDTH+:AUX_WIDTH] = PER_data_aux_TO_BRIDGE[2 * AUX_WIDTH+:AUX_WIDTH];
			PER_data_r_opc_TO_BRIDGE[2] = 1'b0;
		end
	end
	genvar _gv_k_7;
	generate
		for (_gv_k_7 = 0; _gv_k_7 < N_L2_BANKS_PRI; _gv_k_7 = _gv_k_7 + 1) begin : genblk1
			localparam k = _gv_k_7;
			assign L2_pri_D_o[k * DATA_WIDTH+:DATA_WIDTH] = PER_data_wdata_TO_BRIDGE[(k + 3) * DATA_WIDTH+:DATA_WIDTH];
			assign L2_pri_A_o[k * ADDR_L2_PRI_WIDTH+:ADDR_L2_PRI_WIDTH] = PER_data_add_TO_BRIDGE[((k + 3) * ADDR_WIDTH) + ((ADDR_L2_PRI_WIDTH + 1) >= 2 ? ADDR_L2_PRI_WIDTH + 1 : ((ADDR_L2_PRI_WIDTH + 1) + ((ADDR_L2_PRI_WIDTH + 1) >= 2 ? ADDR_L2_PRI_WIDTH + 0 : 3 - (ADDR_L2_PRI_WIDTH + 1))) - 1)-:((ADDR_L2_PRI_WIDTH + 1) >= 2 ? ADDR_L2_PRI_WIDTH + 0 : 3 - (ADDR_L2_PRI_WIDTH + 1))];
			assign L2_pri_CEN_o[k] = ~PER_data_req_TO_BRIDGE[k + 3];
			assign L2_pri_WEN_o[k] = PER_data_wen_TO_BRIDGE[k + 3];
			assign L2_pri_BE_o[k * BE_WIDTH+:BE_WIDTH] = PER_data_be_TO_BRIDGE[(k + 3) * BE_WIDTH+:BE_WIDTH];
			assign PER_data_r_rdata_TO_BRIDGE[(k + 3) * DATA_WIDTH+:DATA_WIDTH] = L2_pri_Q_i[k * DATA_WIDTH+:DATA_WIDTH];
			assign PER_data_gnt_TO_BRIDGE[k + 3] = 1'b1;
			wire [1:1] sv2v_tmp_6DDEF;
			assign sv2v_tmp_6DDEF = 1'sb0;
			always @(*) PER_data_r_opc_TO_BRIDGE[k + 3] = sv2v_tmp_6DDEF;
			always @(posedge clk or negedge rst_n) begin : proc_L2_CH_pri_rvalid_gen
				if (~rst_n) begin
					PER_data_r_valid_TO_BRIDGE[k + 3] = 1'sb0;
					PER_data_r_ID_TO_BRIDGE[(k + 3) * PER_ID_WIDTH+:PER_ID_WIDTH] = 1'sb0;
					PER_data_r_aux_TO_BRIDGE[(k + 3) * AUX_WIDTH+:AUX_WIDTH] = 1'sb0;
				end
				else begin
					PER_data_r_valid_TO_BRIDGE[k + 3] = PER_data_req_TO_BRIDGE[k + 3];
					PER_data_r_ID_TO_BRIDGE[(k + 3) * PER_ID_WIDTH+:PER_ID_WIDTH] = PER_data_ID_TO_BRIDGE[(k + 3) * PER_ID_WIDTH+:PER_ID_WIDTH];
					PER_data_r_aux_TO_BRIDGE[(k + 3) * AUX_WIDTH+:AUX_WIDTH] = PER_data_aux_TO_BRIDGE[(k + 3) * AUX_WIDTH+:AUX_WIDTH];
				end
			end
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		FC_DATA_add_int = FC_DATA_add_i;
		if (FC_DATA_add_i[31:20] == 12'h000)
			FC_DATA_add_int[31:20] = 12'h1c0;
	end
	assign FC_data_req_INT_32 = {FC_INSTR_req_i, UDMA_TX_req_i, UDMA_RX_req_i, DBG_RX_req_i, FC_DATA_req_i};
	assign FC_data_add_INT_32 = {FC_INSTR_add_i, UDMA_TX_add_i, UDMA_RX_add_i, DBG_RX_add_i, FC_DATA_add_int};
	assign FC_data_wen_INT_32 = {FC_INSTR_wen_i, UDMA_TX_wen_i, UDMA_RX_wen_i, DBG_RX_wen_i, FC_DATA_wen_i};
	assign FC_data_wdata_INT_32 = {FC_INSTR_wdata_i, UDMA_TX_wdata_i, UDMA_RX_wdata_i, DBG_RX_wdata_i, FC_DATA_wdata_i};
	assign FC_data_be_INT_32 = {FC_INSTR_be_i, UDMA_TX_be_i, UDMA_RX_be_i, DBG_RX_be_i, FC_DATA_be_i};
	assign FC_data_aux_INT_32 = {FC_INSTR_aux_i, UDMA_TX_aux_i, UDMA_RX_aux_i, DBG_RX_aux_i, FC_DATA_aux_i};
	assign {FC_INSTR_gnt_o, UDMA_TX_gnt_o, UDMA_RX_gnt_o, DBG_RX_gnt_o, FC_DATA_gnt_o} = FC_data_gnt_INT_32;
	assign {FC_INSTR_r_aux_o, UDMA_TX_r_aux_o, UDMA_RX_r_aux_o, DBG_RX_r_aux_o, FC_DATA_r_aux_o} = FC_data_r_aux_INT_32;
	assign {FC_INSTR_r_valid_o, UDMA_TX_r_valid_o, UDMA_RX_r_valid_o, DBG_RX_r_valid_o, FC_DATA_r_valid_o} = FC_data_r_valid_INT_32;
	assign {FC_INSTR_r_rdata_o, UDMA_TX_r_rdata_o, UDMA_RX_r_rdata_o, DBG_RX_r_rdata_o, FC_DATA_r_rdata_o} = FC_data_r_rdata_INT_32;
	assign {FC_INSTR_r_opc_o, UDMA_TX_r_opc_o, UDMA_RX_r_opc_o, DBG_RX_r_opc_o, FC_DATA_r_opc_o} = FC_data_r_opc_INT_32;
	assign TCDM_data_req_DEM_TO_XBAR[N_CH0 - 1:N_CH0 - N_HWPE_PORTS] = HWPE_req_i;
	assign TCDM_data_add_DEM_TO_XBAR[ADDR_WIDTH * (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? N_CH0 - 1 : ((N_CH0 - 1) + ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)) - 1) - (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1) - 1))+:ADDR_WIDTH * ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)] = HWPE_add_i;
	assign TCDM_data_wen_DEM_TO_XBAR[N_CH0 - 1:N_CH0 - N_HWPE_PORTS] = HWPE_wen_i;
	assign TCDM_data_wdata_DEM_TO_XBAR[DATA_WIDTH * (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? N_CH0 - 1 : ((N_CH0 - 1) + ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)) - 1) - (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1) - 1))+:DATA_WIDTH * ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)] = HWPE_wdata_i;
	assign TCDM_data_be_DEM_TO_XBAR[BE_WIDTH * (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? N_CH0 - 1 : ((N_CH0 - 1) + ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)) - 1) - (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1) - 1))+:BE_WIDTH * ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)] = HWPE_be_i;
	assign HWPE_gnt_o = TCDM_data_gnt_DEM_TO_XBAR[N_CH0 - 1:N_CH0 - N_HWPE_PORTS];
	assign HWPE_r_valid_o = TCDM_data_r_valid_DEM_TO_XBAR[N_CH0 - 1:N_CH0 - N_HWPE_PORTS];
	assign HWPE_r_rdata_o = TCDM_data_r_rdata_DEM_TO_XBAR[DATA_WIDTH * (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? N_CH0 - 1 : ((N_CH0 - 1) + ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)) - 1) - (((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1) - 1))+:DATA_WIDTH * ((N_CH0 - 1) >= (N_CH0 - N_HWPE_PORTS) ? ((N_CH0 - 1) - (N_CH0 - N_HWPE_PORTS)) + 1 : ((N_CH0 - N_HWPE_PORTS) - (N_CH0 - 1)) + 1)];
	genvar _gv_j_14;
	generate
		for (_gv_j_14 = 0; _gv_j_14 < (N_CH0 + N_CH1); _gv_j_14 = _gv_j_14 + 1) begin : genblk2
			localparam j = _gv_j_14;
			assign TCDM_data_add_DEM_TO_XBAR_resized[j * (ADDR_L2_WIDTH + $clog2(N_L2_BANKS))+:ADDR_L2_WIDTH + $clog2(N_L2_BANKS)] = TCDM_data_add_DEM_TO_XBAR[(j * ADDR_WIDTH) + (((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1) >= 2 ? (ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1 : (((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1) + (((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1) >= 2 ? (ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 0 : 3 - ((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1))) - 1)-:(((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1) >= 2 ? (ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 0 : 3 - ((ADDR_L2_WIDTH + $clog2(N_L2_BANKS)) + 1))];
		end
	endgenerate
	XBAR_L2 #(
		.N_CH0(N_CH0),
		.N_CH1(N_CH1),
		.N_SLAVE(N_L2_BANKS),
		.ID_WIDTH(N_CH0 + N_CH1),
		.ADDR_IN_WIDTH(ADDR_L2_WIDTH + $clog2(N_L2_BANKS)),
		.DATA_WIDTH(DATA_WIDTH),
		.BE_WIDTH(BE_WIDTH),
		.ADDR_MEM_WIDTH(ADDR_L2_WIDTH)
	) XBAR_L2_i(
		.data_req_i(TCDM_data_req_DEM_TO_XBAR),
		.data_add_i(TCDM_data_add_DEM_TO_XBAR_resized),
		.data_wen_i(TCDM_data_wen_DEM_TO_XBAR),
		.data_wdata_i(TCDM_data_wdata_DEM_TO_XBAR),
		.data_be_i(TCDM_data_be_DEM_TO_XBAR),
		.data_gnt_o(TCDM_data_gnt_DEM_TO_XBAR),
		.data_r_valid_o(TCDM_data_r_valid_DEM_TO_XBAR),
		.data_r_rdata_o(TCDM_data_r_rdata_DEM_TO_XBAR),
		.data_req_o(TCDM_data_req_TO_MEM),
		.data_add_o(TCDM_data_add_TO_MEM),
		.data_wen_o(TCDM_data_wen_TO_MEM),
		.data_wdata_o(TCDM_data_wdata_TO_MEM),
		.data_be_o(TCDM_data_be_TO_MEM),
		.data_ID_o(TCDM_data_ID_TO_MEM),
		.data_r_rdata_i(TCDM_data_rdata_TO_MEM),
		.data_r_valid_i(TCDM_data_rvalid_TO_MEM),
		.data_r_ID_i(TCDM_data_rID_TO_MEM),
		.clk(clk),
		.rst_n(rst_n)
	);
	XBAR_BRIDGE #(
		.N_CH0(N_CH0_BRIDGE),
		.N_CH1(N_CH1_BRIDGE),
		.N_SLAVE(N_PERIPHS),
		.ID_WIDTH(PER_ID_WIDTH),
		.AUX_WIDTH(AUX_WIDTH),
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.BE_WIDTH(BE_WIDTH)
	) XBAR_BRIDGE_i(
		.data_req_i(PER_data_req_DEM_2_L2_XBAR),
		.data_add_i(PER_data_add_DEM_2_L2_XBAR),
		.data_wen_i(PER_data_wen_DEM_2_L2_XBAR),
		.data_wdata_i(PER_data_wdata_DEM_2_L2_XBAR),
		.data_be_i(PER_data_be_DEM_2_L2_XBAR),
		.data_aux_i(PER_data_aux_DEM_2_L2_XBAR),
		.data_gnt_o(PER_data_gnt_DEM_2_L2_XBAR),
		.data_r_valid_o(PER_data_r_valid_DEM_2_L2_XBAR),
		.data_r_rdata_o(PER_data_r_rdata_DEM_2_L2_XBAR),
		.data_r_opc_o(PER_data_r_opc_DEM_2_L2_XBAR),
		.data_r_aux_o(PER_data_r_aux_DEM_2_L2_XBAR),
		.data_req_o(PER_data_req_TO_BRIDGE),
		.data_add_o(PER_data_add_TO_BRIDGE),
		.data_wen_o(PER_data_wen_TO_BRIDGE),
		.data_wdata_o(PER_data_wdata_TO_BRIDGE),
		.data_be_o(PER_data_be_TO_BRIDGE),
		.data_ID_o(PER_data_ID_TO_BRIDGE),
		.data_aux_o(PER_data_aux_TO_BRIDGE),
		.data_gnt_i(PER_data_gnt_TO_BRIDGE),
		.data_r_rdata_i(PER_data_r_rdata_TO_BRIDGE),
		.data_r_valid_i(PER_data_r_valid_TO_BRIDGE),
		.data_r_ID_i(PER_data_r_ID_TO_BRIDGE),
		.data_r_opc_i(PER_data_r_opc_TO_BRIDGE),
		.data_r_aux_i(PER_data_r_aux_TO_BRIDGE),
		.clk(clk),
		.rst_n(rst_n),
		.START_ADDR(PER_START_ADDR),
		.END_ADDR(PER_END_ADDR)
	);
	genvar _gv_i_12;
	generate
		for (_gv_i_12 = 0; _gv_i_12 < (N_CH0 - N_HWPE_PORTS); _gv_i_12 = _gv_i_12 + 1) begin : FC_DEMUX_32
			localparam i = _gv_i_12;
			l2_tcdm_demux #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.AUX_WIDTH(AUX_WIDTH),
				.N_PERIPHS(N_PERIPHS)
			) DEMUX_MASTER_32(
				.clk(clk),
				.rst_n(rst_n),
				.test_en_i(test_en_i),
				.data_req_i(FC_data_req_INT_32[i]),
				.data_add_i(FC_data_add_INT_32[i * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_i(FC_data_wen_INT_32[i]),
				.data_wdata_i(FC_data_wdata_INT_32[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_i(FC_data_be_INT_32[i * BE_WIDTH+:BE_WIDTH]),
				.data_aux_i(FC_data_aux_INT_32[i * AUX_WIDTH+:AUX_WIDTH]),
				.data_gnt_o(FC_data_gnt_INT_32[i]),
				.data_r_aux_o(FC_data_r_aux_INT_32[i * AUX_WIDTH+:AUX_WIDTH]),
				.data_r_valid_o(FC_data_r_valid_INT_32[i]),
				.data_r_rdata_o(FC_data_r_rdata_INT_32[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_r_opc_o(FC_data_r_opc_INT_32[i]),
				.data_req_o_TDCM(TCDM_data_req_DEM_TO_XBAR[i]),
				.data_add_o_TDCM(TCDM_data_add_DEM_TO_XBAR[i * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_o_TDCM(TCDM_data_wen_DEM_TO_XBAR[i]),
				.data_wdata_o_TDCM(TCDM_data_wdata_DEM_TO_XBAR[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_o_TDCM(TCDM_data_be_DEM_TO_XBAR[i * BE_WIDTH+:BE_WIDTH]),
				.data_gnt_i_TDCM(TCDM_data_gnt_DEM_TO_XBAR[i]),
				.data_r_valid_i_TDCM(TCDM_data_r_valid_DEM_TO_XBAR[i]),
				.data_r_rdata_i_TDCM(TCDM_data_r_rdata_DEM_TO_XBAR[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_req_o_PER(PER_data_req_DEM_2_L2_XBAR[i]),
				.data_add_o_PER(PER_data_add_DEM_2_L2_XBAR[i * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_o_PER(PER_data_wen_DEM_2_L2_XBAR[i]),
				.data_wdata_o_PER(PER_data_wdata_DEM_2_L2_XBAR[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_o_PER(PER_data_be_DEM_2_L2_XBAR[i * BE_WIDTH+:BE_WIDTH]),
				.data_aux_o_PER(PER_data_aux_DEM_2_L2_XBAR[i * AUX_WIDTH+:AUX_WIDTH]),
				.data_gnt_i_PER(PER_data_gnt_DEM_2_L2_XBAR[i]),
				.data_r_valid_i_PER(PER_data_r_valid_DEM_2_L2_XBAR[i]),
				.data_r_rdata_i_PER(PER_data_r_rdata_DEM_2_L2_XBAR[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_r_opc_i_PER(PER_data_r_opc_DEM_2_L2_XBAR[i]),
				.data_r_aux_i_PER(PER_data_r_aux_DEM_2_L2_XBAR[i * AUX_WIDTH+:AUX_WIDTH]),
				.PER_START_ADDR(PER_START_ADDR),
				.PER_END_ADDR(PER_END_ADDR),
				.TCDM_START_ADDR(TCDM_START_ADDR),
				.TCDM_END_ADDR(TCDM_END_ADDR)
			);
		end
		for (_gv_i_12 = 0; _gv_i_12 < N_CH1; _gv_i_12 = _gv_i_12 + 1) begin : FC_DEMUX_64
			localparam i = _gv_i_12;
			l2_tcdm_demux #(
				.ADDR_WIDTH(ADDR_WIDTH),
				.DATA_WIDTH(DATA_WIDTH),
				.BE_WIDTH(BE_WIDTH),
				.AUX_WIDTH(AUX_WIDTH),
				.N_PERIPHS(N_PERIPHS)
			) DEMUX_AXI64(
				.clk(clk),
				.rst_n(rst_n),
				.test_en_i(test_en_i),
				.data_req_i(AXI_data_req_INT_64[i]),
				.data_add_i(AXI_data_add_INT_64[i * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_i(AXI_data_wen_INT_64[i]),
				.data_wdata_i(AXI_data_wdata_INT_64[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_i(AXI_data_be_INT_64[i * BE_WIDTH+:BE_WIDTH]),
				.data_aux_i(AXI_data_aux_INT_64[i * AUX_WIDTH+:AUX_WIDTH]),
				.data_gnt_o(AXI_data_gnt_INT_64[i]),
				.data_r_aux_o(AXI_data_r_aux_INT_64[i * AUX_WIDTH+:AUX_WIDTH]),
				.data_r_valid_o(AXI_data_r_valid_INT_64[i]),
				.data_r_rdata_o(AXI_data_r_rdata_INT_64[i * DATA_WIDTH+:DATA_WIDTH]),
				.data_r_opc_o(AXI_data_r_opc_INT_64[i]),
				.data_req_o_TDCM(TCDM_data_req_DEM_TO_XBAR[N_CH0 + i]),
				.data_add_o_TDCM(TCDM_data_add_DEM_TO_XBAR[(N_CH0 + i) * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_o_TDCM(TCDM_data_wen_DEM_TO_XBAR[N_CH0 + i]),
				.data_wdata_o_TDCM(TCDM_data_wdata_DEM_TO_XBAR[(N_CH0 + i) * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_o_TDCM(TCDM_data_be_DEM_TO_XBAR[(N_CH0 + i) * BE_WIDTH+:BE_WIDTH]),
				.data_gnt_i_TDCM(TCDM_data_gnt_DEM_TO_XBAR[N_CH0 + i]),
				.data_r_valid_i_TDCM(TCDM_data_r_valid_DEM_TO_XBAR[N_CH0 + i]),
				.data_r_rdata_i_TDCM(TCDM_data_r_rdata_DEM_TO_XBAR[(N_CH0 + i) * DATA_WIDTH+:DATA_WIDTH]),
				.data_req_o_PER(PER_data_req_DEM_2_L2_XBAR[N_CH0 + i]),
				.data_add_o_PER(PER_data_add_DEM_2_L2_XBAR[(N_CH0 + i) * ADDR_WIDTH+:ADDR_WIDTH]),
				.data_wen_o_PER(PER_data_wen_DEM_2_L2_XBAR[N_CH0 + i]),
				.data_wdata_o_PER(PER_data_wdata_DEM_2_L2_XBAR[(N_CH0 + i) * DATA_WIDTH+:DATA_WIDTH]),
				.data_be_o_PER(PER_data_be_DEM_2_L2_XBAR[(N_CH0 + i) * BE_WIDTH+:BE_WIDTH]),
				.data_aux_o_PER(PER_data_aux_DEM_2_L2_XBAR[(N_CH0 + i) * AUX_WIDTH+:AUX_WIDTH]),
				.data_gnt_i_PER(PER_data_gnt_DEM_2_L2_XBAR[N_CH0 + i]),
				.data_r_valid_i_PER(PER_data_r_valid_DEM_2_L2_XBAR[N_CH0 + i]),
				.data_r_rdata_i_PER(PER_data_r_rdata_DEM_2_L2_XBAR[(N_CH0 + i) * DATA_WIDTH+:DATA_WIDTH]),
				.data_r_opc_i_PER(PER_data_r_opc_DEM_2_L2_XBAR[N_CH0 + i]),
				.data_r_aux_i_PER(PER_data_r_aux_DEM_2_L2_XBAR[(N_CH0 + i) * AUX_WIDTH+:AUX_WIDTH]),
				.PER_START_ADDR(PER_START_ADDR),
				.PER_END_ADDR(PER_END_ADDR),
				.TCDM_START_ADDR(TCDM_START_ADDR),
				.TCDM_END_ADDR(TCDM_END_ADDR)
			);
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_1
			reg [31:0] i;
			for (i = 0; i < N_L2_BANKS; i = i + 1)
				begin
					L2_D_o[i * DATA_WIDTH+:DATA_WIDTH] = TCDM_data_wdata_TO_MEM[i * DATA_WIDTH+:DATA_WIDTH];
					L2_A_o[i * ADDR_L2_WIDTH+:ADDR_L2_WIDTH] = TCDM_data_add_TO_MEM[i * ADDR_L2_WIDTH+:ADDR_L2_WIDTH] - L2_OFFSET_PRI;
					L2_CEN_o[i] = ~TCDM_data_req_TO_MEM[i];
					L2_WEN_o[i] = TCDM_data_wen_TO_MEM[i];
					L2_BE_o[i * BE_WIDTH+:BE_WIDTH] = TCDM_data_be_TO_MEM[i * BE_WIDTH+:BE_WIDTH];
					L2_ID_o[i * 32+:32] = TCDM_data_ID_TO_MEM[i * ID_WIDTH+:ID_WIDTH];
					TCDM_data_rdata_TO_MEM[i * DATA_WIDTH+:DATA_WIDTH] = L2_Q_i[i * DATA_WIDTH+:DATA_WIDTH];
				end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin : sv2v_autoblock_2
			reg [31:0] i;
			for (i = 0; i < N_L2_BANKS; i = i + 1)
				begin
					TCDM_data_rID_TO_MEM[i * ID_WIDTH+:ID_WIDTH] <= 1'sb0;
					TCDM_data_rvalid_TO_MEM[i] <= 1'sb0;
				end
		end
		else begin : sv2v_autoblock_3
			reg [31:0] i;
			for (i = 0; i < N_L2_BANKS; i = i + 1)
				if (TCDM_data_req_TO_MEM[i]) begin
					TCDM_data_rID_TO_MEM[i * ID_WIDTH+:ID_WIDTH] <= TCDM_data_ID_TO_MEM[i * ID_WIDTH+:ID_WIDTH];
					TCDM_data_rvalid_TO_MEM[i] <= 1'b1;
				end
				else
					TCDM_data_rvalid_TO_MEM[i] <= 1'b0;
		end
	wire [1:1] sv2v_tmp_lint_2_apb_i_data_r_valid_o;
	always @(*) PER_data_r_valid_TO_BRIDGE[0] = sv2v_tmp_lint_2_apb_i_data_r_valid_o;
	wire [1:1] sv2v_tmp_lint_2_apb_i_data_r_opc_o;
	always @(*) PER_data_r_opc_TO_BRIDGE[0] = sv2v_tmp_lint_2_apb_i_data_r_opc_o;
	wire [AUX_WIDTH * 1:1] sv2v_tmp_lint_2_apb_i_data_r_aux_o;
	always @(*) PER_data_r_aux_TO_BRIDGE[0+:AUX_WIDTH] = sv2v_tmp_lint_2_apb_i_data_r_aux_o;
	wire [PER_ID_WIDTH * 1:1] sv2v_tmp_lint_2_apb_i_data_r_ID_o;
	always @(*) PER_data_r_ID_TO_BRIDGE[0+:PER_ID_WIDTH] = sv2v_tmp_lint_2_apb_i_data_r_ID_o;
	lint_2_apb #(
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.BE_WIDTH(BE_WIDTH),
		.ID_WIDTH(PER_ID_WIDTH),
		.AUX_WIDTH(AUX_WIDTH)
	) lint_2_apb_i(
		.clk(clk),
		.rst_n(rst_n),
		.data_req_i(PER_data_req_TO_BRIDGE[0]),
		.data_add_i(PER_data_add_TO_BRIDGE[0+:ADDR_WIDTH]),
		.data_wen_i(PER_data_wen_TO_BRIDGE[0]),
		.data_wdata_i(PER_data_wdata_TO_BRIDGE[0+:DATA_WIDTH]),
		.data_be_i(PER_data_be_TO_BRIDGE[0+:BE_WIDTH]),
		.data_aux_i(PER_data_aux_TO_BRIDGE[0+:AUX_WIDTH]),
		.data_ID_i(PER_data_ID_TO_BRIDGE[0+:PER_ID_WIDTH]),
		.data_gnt_o(PER_data_gnt_TO_BRIDGE[0]),
		.data_r_valid_o(sv2v_tmp_lint_2_apb_i_data_r_valid_o),
		.data_r_rdata_o(PER_data_r_rdata_TO_BRIDGE[0+:DATA_WIDTH]),
		.data_r_opc_o(sv2v_tmp_lint_2_apb_i_data_r_opc_o),
		.data_r_aux_o(sv2v_tmp_lint_2_apb_i_data_r_aux_o),
		.data_r_ID_o(sv2v_tmp_lint_2_apb_i_data_r_ID_o),
		.master_PADDR(APB_PADDR_o),
		.master_PWDATA(APB_PWDATA_o),
		.master_PWRITE(APB_PWRITE_o),
		.master_PSEL(APB_PSEL_o),
		.master_PENABLE(APB_PENABLE_o),
		.master_PRDATA(APB_PRDATA_i),
		.master_PREADY(APB_PREADY_i),
		.master_PSLVERR(APB_PSLVERR_i)
	);
	wire [1:1] sv2v_tmp_i_lint_2_axi_data_rvalid_o;
	always @(*) PER_data_r_valid_TO_BRIDGE[1] = sv2v_tmp_i_lint_2_axi_data_rvalid_o;
	wire [1:1] sv2v_tmp_i_lint_2_axi_data_ropc_o;
	always @(*) PER_data_r_opc_TO_BRIDGE[1] = sv2v_tmp_i_lint_2_axi_data_ropc_o;
	wire [AUX_WIDTH * 1:1] sv2v_tmp_i_lint_2_axi_data_raux_o;
	always @(*) PER_data_r_aux_TO_BRIDGE[AUX_WIDTH+:AUX_WIDTH] = sv2v_tmp_i_lint_2_axi_data_raux_o;
	wire [PER_ID_WIDTH * 1:1] sv2v_tmp_i_lint_2_axi_data_rID_o;
	always @(*) PER_data_r_ID_TO_BRIDGE[PER_ID_WIDTH+:PER_ID_WIDTH] = sv2v_tmp_i_lint_2_axi_data_rID_o;
	lint_2_axi #(
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.BE_WIDTH(BE_WIDTH),
		.ID_WIDTH(PER_ID_WIDTH),
		.USER_WIDTH(AXI_32_USER_WIDTH),
		.AUX_WIDTH(AUX_WIDTH),
		.AXI_ID_WIDTH(AXI_32_ID_WIDTH),
		.REGISTERED_GRANT("FALSE")
	) i_lint_2_axi(
		.clk_i(clk),
		.rst_ni(rst_n),
		.data_req_i(PER_data_req_TO_BRIDGE[1]),
		.data_addr_i(PER_data_add_TO_BRIDGE[ADDR_WIDTH+:ADDR_WIDTH]),
		.data_we_i(~PER_data_wen_TO_BRIDGE[1]),
		.data_wdata_i(PER_data_wdata_TO_BRIDGE[DATA_WIDTH+:DATA_WIDTH]),
		.data_be_i(PER_data_be_TO_BRIDGE[BE_WIDTH+:BE_WIDTH]),
		.data_aux_i(PER_data_aux_TO_BRIDGE[AUX_WIDTH+:AUX_WIDTH]),
		.data_ID_i(PER_data_ID_TO_BRIDGE[PER_ID_WIDTH+:PER_ID_WIDTH]),
		.data_gnt_o(PER_data_gnt_TO_BRIDGE[1]),
		.data_rvalid_o(sv2v_tmp_i_lint_2_axi_data_rvalid_o),
		.data_rdata_o(PER_data_r_rdata_TO_BRIDGE[DATA_WIDTH+:DATA_WIDTH]),
		.data_ropc_o(sv2v_tmp_i_lint_2_axi_data_ropc_o),
		.data_raux_o(sv2v_tmp_i_lint_2_axi_data_raux_o),
		.data_rID_o(sv2v_tmp_i_lint_2_axi_data_rID_o),
		.aw_id_o(AXI_Master_aw_id_o),
		.aw_addr_o(AXI_Master_aw_addr_o),
		.aw_len_o(AXI_Master_aw_len_o),
		.aw_size_o(AXI_Master_aw_size_o),
		.aw_burst_o(AXI_Master_aw_burst_o),
		.aw_lock_o(AXI_Master_aw_lock_o),
		.aw_cache_o(AXI_Master_aw_cache_o),
		.aw_prot_o(AXI_Master_aw_prot_o),
		.aw_region_o(AXI_Master_aw_region_o),
		.aw_user_o(AXI_Master_aw_user_o),
		.aw_qos_o(AXI_Master_aw_qos_o),
		.aw_valid_o(AXI_Master_aw_valid_o),
		.aw_ready_i(AXI_Master_aw_ready_i),
		.w_data_o(AXI_Master_w_data_o),
		.w_strb_o(AXI_Master_w_strb_o),
		.w_last_o(AXI_Master_w_last_o),
		.w_user_o(AXI_Master_w_user_o),
		.w_valid_o(AXI_Master_w_valid_o),
		.w_ready_i(AXI_Master_w_ready_i),
		.b_id_i(AXI_Master_b_id_i),
		.b_resp_i(AXI_Master_b_resp_i),
		.b_valid_i(AXI_Master_b_valid_i),
		.b_user_i(AXI_Master_b_user_i),
		.b_ready_o(AXI_Master_b_ready_o),
		.ar_id_o(AXI_Master_ar_id_o),
		.ar_addr_o(AXI_Master_ar_addr_o),
		.ar_len_o(AXI_Master_ar_len_o),
		.ar_size_o(AXI_Master_ar_size_o),
		.ar_burst_o(AXI_Master_ar_burst_o),
		.ar_lock_o(AXI_Master_ar_lock_o),
		.ar_cache_o(AXI_Master_ar_cache_o),
		.ar_prot_o(AXI_Master_ar_prot_o),
		.ar_region_o(AXI_Master_ar_region_o),
		.ar_user_o(AXI_Master_ar_user_o),
		.ar_qos_o(AXI_Master_ar_qos_o),
		.ar_valid_o(AXI_Master_ar_valid_o),
		.ar_ready_i(AXI_Master_ar_ready_i),
		.r_id_i(AXI_Master_r_id_i),
		.r_data_i(AXI_Master_r_data_i),
		.r_resp_i(AXI_Master_r_resp_i),
		.r_last_i(AXI_Master_r_last_i),
		.r_user_i(AXI_Master_r_user_i),
		.r_valid_i(AXI_Master_r_valid_i),
		.r_ready_o(AXI_Master_r_ready_o)
	);
	axi64_2_lint32 #(
		.AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
		.AXI_DATA_WIDTH(AXI_DATA_WIDTH),
		.AXI_STRB_WIDTH(AXI_STRB_WIDTH),
		.AXI_USER_WIDTH(AXI_USER_WIDTH),
		.AXI_ID_WIDTH(AXI_ID_WIDTH),
		.BUFF_DEPTH_SLICES(4),
		.DATA_WIDTH(DATA_WIDTH),
		.BE_WIDTH(BE_WIDTH),
		.ADDR_WIDTH(ADDR_WIDTH),
		.AUX_WIDTH(AUX_WIDTH)
	) axi64_2_lint32_i(
		.clk(clk),
		.rst_n(rst_n),
		.test_en_i(test_en_i),
		.AW_ADDR_i(AXI_Slave_aw_addr_i),
		.AW_PROT_i(AXI_Slave_aw_prot_i),
		.AW_REGION_i(AXI_Slave_aw_region_i),
		.AW_LEN_i(AXI_Slave_aw_len_i),
		.AW_SIZE_i(AXI_Slave_aw_size_i),
		.AW_BURST_i(AXI_Slave_aw_burst_i),
		.AW_LOCK_i(AXI_Slave_aw_lock_i),
		.AW_CACHE_i(AXI_Slave_aw_cache_i),
		.AW_QOS_i(AXI_Slave_aw_qos_i),
		.AW_ID_i(AXI_Slave_aw_id_i),
		.AW_USER_i(AXI_Slave_aw_user_i),
		.AW_VALID_i(AXI_Slave_aw_valid_i),
		.AW_READY_o(AXI_Slave_aw_ready_o),
		.AR_ADDR_i(AXI_Slave_ar_addr_i),
		.AR_PROT_i(AXI_Slave_ar_prot_i),
		.AR_REGION_i(AXI_Slave_ar_region_i),
		.AR_LEN_i(AXI_Slave_ar_len_i),
		.AR_SIZE_i(AXI_Slave_ar_size_i),
		.AR_BURST_i(AXI_Slave_ar_burst_i),
		.AR_LOCK_i(AXI_Slave_ar_lock_i),
		.AR_CACHE_i(AXI_Slave_ar_cache_i),
		.AR_QOS_i(AXI_Slave_ar_qos_i),
		.AR_ID_i(AXI_Slave_ar_id_i),
		.AR_USER_i(AXI_Slave_ar_user_i),
		.AR_VALID_i(AXI_Slave_ar_valid_i),
		.AR_READY_o(AXI_Slave_ar_ready_o),
		.W_USER_i(AXI_Slave_w_user_i),
		.W_DATA_i(AXI_Slave_w_data_i),
		.W_STRB_i(AXI_Slave_w_strb_i),
		.W_LAST_i(AXI_Slave_w_last_i),
		.W_VALID_i(AXI_Slave_w_valid_i),
		.W_READY_o(AXI_Slave_w_ready_o),
		.B_ID_o(AXI_Slave_b_id_o),
		.B_RESP_o(AXI_Slave_b_resp_o),
		.B_USER_o(AXI_Slave_b_user_o),
		.B_VALID_o(AXI_Slave_b_valid_o),
		.B_READY_i(AXI_Slave_b_ready_i),
		.R_ID_o(AXI_Slave_r_id_o),
		.R_USER_o(AXI_Slave_r_user_o),
		.R_DATA_o(AXI_Slave_r_data_o),
		.R_RESP_o(AXI_Slave_r_resp_o),
		.R_LAST_o(AXI_Slave_r_last_o),
		.R_VALID_o(AXI_Slave_r_valid_o),
		.R_READY_i(AXI_Slave_r_ready_i),
		.data_W_req_o(AXI_data_req_INT_64[1:0]),
		.data_W_gnt_i(AXI_data_gnt_INT_64[1:0]),
		.data_W_wdata_o(AXI_data_wdata_INT_64[0+:DATA_WIDTH * 2]),
		.data_W_add_o(AXI_data_add_INT_64[0+:ADDR_WIDTH * 2]),
		.data_W_wen_o(AXI_data_wen_INT_64[1:0]),
		.data_W_be_o(AXI_data_be_INT_64[0+:BE_WIDTH * 2]),
		.data_W_aux_o(AXI_data_aux_INT_64[0+:AUX_WIDTH * 2]),
		.data_W_r_valid_i(AXI_data_r_valid_INT_64[1:0]),
		.data_W_r_rdata_i(AXI_data_r_rdata_INT_64[0+:DATA_WIDTH * 2]),
		.data_W_r_opc_i(AXI_data_r_opc_INT_64[1:0]),
		.data_W_r_aux_i(AXI_data_r_aux_INT_64[0+:AUX_WIDTH * 2]),
		.data_R_req_o(AXI_data_req_INT_64[3:2]),
		.data_R_gnt_i(AXI_data_gnt_INT_64[3:2]),
		.data_R_wdata_o(AXI_data_wdata_INT_64[DATA_WIDTH * 2+:DATA_WIDTH * 2]),
		.data_R_add_o(AXI_data_add_INT_64[ADDR_WIDTH * 2+:ADDR_WIDTH * 2]),
		.data_R_wen_o(AXI_data_wen_INT_64[3:2]),
		.data_R_be_o(AXI_data_be_INT_64[BE_WIDTH * 2+:BE_WIDTH * 2]),
		.data_R_aux_o(AXI_data_aux_INT_64[AUX_WIDTH * 2+:AUX_WIDTH * 2]),
		.data_R_r_valid_i(AXI_data_r_valid_INT_64[3:2]),
		.data_R_r_rdata_i(AXI_data_r_rdata_INT_64[DATA_WIDTH * 2+:DATA_WIDTH * 2]),
		.data_R_r_opc_i(AXI_data_r_opc_INT_64[3:2]),
		.data_R_r_aux_i(AXI_data_r_aux_INT_64[AUX_WIDTH * 2+:AUX_WIDTH * 2])
	);
	initial _sv2v_0 = 0;
endmodule
module riscv_L0_buffer (
	clk,
	rst_n,
	prefetch_i,
	prefetch_addr_i,
	branch_i,
	branch_addr_i,
	hwlp_i,
	hwlp_addr_i,
	fetch_gnt_o,
	fetch_valid_o,
	valid_o,
	rdata_o,
	addr_o,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	busy_o
);
	reg _sv2v_0;
	parameter RDATA_IN_WIDTH = 128;
	input wire clk;
	input wire rst_n;
	input wire prefetch_i;
	input wire [31:0] prefetch_addr_i;
	input wire branch_i;
	input wire [31:0] branch_addr_i;
	input wire hwlp_i;
	input wire [31:0] hwlp_addr_i;
	output wire fetch_gnt_o;
	output reg fetch_valid_o;
	output wire valid_o;
	output wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] rdata_o;
	output wire [31:0] addr_o;
	output reg instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] instr_rdata_i;
	output wire busy_o;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [127:0] L0_buffer;
	reg [31:0] addr_q;
	reg [31:0] instr_addr_int;
	reg valid;
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		valid = 1'b0;
		instr_req_o = 1'b0;
		instr_addr_int = 1'sb0;
		fetch_valid_o = 1'b0;
		case (CS)
			3'd0: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if ((branch_i | hwlp_i) | prefetch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd2: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = addr_q;
				if (branch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
				else begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd3: begin
				valid = instr_rvalid_i;
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if (branch_i) begin
					if (instr_rvalid_i) begin
						fetch_valid_o = 1'b1;
						instr_req_o = 1'b1;
						if (instr_gnt_i)
							NS = 3'd3;
						else
							NS = 3'd2;
					end
					else
						NS = 3'd4;
				end
				else if (instr_rvalid_i) begin
					fetch_valid_o = 1'b1;
					if (prefetch_i | hwlp_i) begin
						instr_req_o = 1'b1;
						if (instr_gnt_i)
							NS = 3'd3;
						else
							NS = 3'd2;
					end
					else
						NS = 3'd1;
				end
			end
			3'd1: begin
				valid = 1'b1;
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if ((branch_i | hwlp_i) | prefetch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd4: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else
					instr_addr_int = addr_q;
				if (instr_rvalid_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			default: NS = 3'd0;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			CS <= 3'd0;
			L0_buffer <= 1'sb0;
			addr_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (instr_rvalid_i)
				L0_buffer <= instr_rdata_i;
			if ((branch_i | hwlp_i) | prefetch_i)
				addr_q <= instr_addr_int;
		end
	assign instr_addr_o = {instr_addr_int[31:4], 4'b0000};
	assign rdata_o = (instr_rvalid_i ? instr_rdata_i : L0_buffer);
	assign addr_o = addr_q;
	assign valid_o = valid & ~branch_i;
	assign busy_o = ((CS != 3'd0) && (CS != 3'd1)) || instr_req_o;
	assign fetch_gnt_o = instr_gnt_i;
	initial _sv2v_0 = 0;
endmodule
module riscv_alu (
	clk,
	rst_n,
	operator_i,
	operand_a_i,
	operand_b_i,
	operand_c_i,
	vector_mode_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	result_o,
	comparison_result_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	parameter SHARED_INT_DIV = 0;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	input wire [6:0] operator_i;
	input wire [31:0] operand_a_i;
	input wire [31:0] operand_b_i;
	input wire [31:0] operand_c_i;
	input wire [1:0] vector_mode_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	output reg [31:0] result_o;
	output wire comparison_result_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [31:0] operand_a_rev;
	wire [31:0] operand_a_neg;
	wire [31:0] operand_a_neg_rev;
	assign operand_a_neg = ~operand_a_i;
	genvar _gv_k_8;
	generate
		for (_gv_k_8 = 0; _gv_k_8 < 32; _gv_k_8 = _gv_k_8 + 1) begin : genblk1
			localparam k = _gv_k_8;
			assign operand_a_rev[k] = operand_a_i[31 - k];
		end
	endgenerate
	genvar _gv_m_1;
	generate
		for (_gv_m_1 = 0; _gv_m_1 < 32; _gv_m_1 = _gv_m_1 + 1) begin : genblk2
			localparam m = _gv_m_1;
			assign operand_a_neg_rev[m] = operand_a_neg[31 - m];
		end
	endgenerate
	wire [31:0] operand_b_neg;
	assign operand_b_neg = ~operand_b_i;
	wire [5:0] div_shift;
	wire div_valid;
	wire [31:0] bmask;
	wire adder_op_b_negate;
	wire [31:0] adder_op_a;
	wire [31:0] adder_op_b;
	reg [35:0] adder_in_a;
	reg [35:0] adder_in_b;
	wire [31:0] adder_result;
	wire [36:0] adder_result_expanded;
	localparam riscv_defines_ALU_SUB = 7'b0011001;
	localparam riscv_defines_ALU_SUBR = 7'b0011101;
	localparam riscv_defines_ALU_SUBU = 7'b0011011;
	localparam riscv_defines_ALU_SUBUR = 7'b0011111;
	assign adder_op_b_negate = (((operator_i == riscv_defines_ALU_SUB) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_SUBU)) || (operator_i == riscv_defines_ALU_SUBUR);
	localparam riscv_defines_ALU_ABS = 7'b0010100;
	assign adder_op_a = (operator_i == riscv_defines_ALU_ABS ? operand_a_neg : operand_a_i);
	assign adder_op_b = (adder_op_b_negate ? operand_b_neg : operand_b_i);
	localparam riscv_defines_ALU_CLIP = 7'b0010110;
	localparam riscv_defines_VEC_MODE16 = 2'b10;
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		adder_in_a[0] = 1'b1;
		adder_in_a[8:1] = adder_op_a[7:0];
		adder_in_a[9] = 1'b1;
		adder_in_a[17:10] = adder_op_a[15:8];
		adder_in_a[18] = 1'b1;
		adder_in_a[26:19] = adder_op_a[23:16];
		adder_in_a[27] = 1'b1;
		adder_in_a[35:28] = adder_op_a[31:24];
		adder_in_b[0] = 1'b0;
		adder_in_b[8:1] = adder_op_b[7:0];
		adder_in_b[9] = 1'b0;
		adder_in_b[17:10] = adder_op_b[15:8];
		adder_in_b[18] = 1'b0;
		adder_in_b[26:19] = adder_op_b[23:16];
		adder_in_b[27] = 1'b0;
		adder_in_b[35:28] = adder_op_b[31:24];
		if (adder_op_b_negate || ((operator_i == riscv_defines_ALU_ABS) || (operator_i == riscv_defines_ALU_CLIP))) begin
			adder_in_b[0] = 1'b1;
			case (vector_mode_i)
				riscv_defines_VEC_MODE16: adder_in_b[18] = 1'b1;
				riscv_defines_VEC_MODE8: begin
					adder_in_b[9] = 1'b1;
					adder_in_b[18] = 1'b1;
					adder_in_b[27] = 1'b1;
				end
			endcase
		end
		else
			case (vector_mode_i)
				riscv_defines_VEC_MODE16: adder_in_a[18] = 1'b0;
				riscv_defines_VEC_MODE8: begin
					adder_in_a[9] = 1'b0;
					adder_in_a[18] = 1'b0;
					adder_in_a[27] = 1'b0;
				end
			endcase
	end
	assign adder_result_expanded = $signed(adder_in_a) + $signed(adder_in_b);
	assign adder_result = {adder_result_expanded[35:28], adder_result_expanded[26:19], adder_result_expanded[17:10], adder_result_expanded[8:1]};
	wire [31:0] adder_round_value;
	wire [31:0] adder_round_result;
	localparam riscv_defines_ALU_ADDR = 7'b0011100;
	localparam riscv_defines_ALU_ADDUR = 7'b0011110;
	assign adder_round_value = ((((operator_i == riscv_defines_ALU_ADDR) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_ADDUR)) || (operator_i == riscv_defines_ALU_SUBUR) ? {1'b0, bmask[31:1]} : {32 {1'sb0}});
	assign adder_round_result = adder_result + adder_round_value;
	wire shift_left;
	wire shift_use_round;
	wire shift_arithmetic;
	reg [31:0] shift_amt_left;
	wire [31:0] shift_amt;
	wire [31:0] shift_amt_int;
	wire [31:0] shift_amt_norm;
	wire [31:0] shift_op_a;
	wire [31:0] shift_result;
	reg [31:0] shift_right_result;
	wire [31:0] shift_left_result;
	assign shift_amt = (div_valid ? div_shift : operand_b_i);
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				shift_amt_left[15:0] = shift_amt[31:16];
				shift_amt_left[31:16] = shift_amt[15:0];
			end
			riscv_defines_VEC_MODE8: begin
				shift_amt_left[7:0] = shift_amt[31:24];
				shift_amt_left[15:8] = shift_amt[23:16];
				shift_amt_left[23:16] = shift_amt[15:8];
				shift_amt_left[31:24] = shift_amt[7:0];
			end
			default: shift_amt_left[31:0] = shift_amt[31:0];
		endcase
	end
	localparam riscv_defines_ALU_BINS = 7'b0101010;
	localparam riscv_defines_ALU_CLB = 7'b0110101;
	localparam riscv_defines_ALU_DIV = 7'b0110001;
	localparam riscv_defines_ALU_DIVU = 7'b0110000;
	localparam riscv_defines_ALU_FL1 = 7'b0110111;
	localparam riscv_defines_ALU_REM = 7'b0110011;
	localparam riscv_defines_ALU_REMU = 7'b0110010;
	localparam riscv_defines_ALU_SLL = 7'b0100111;
	assign shift_left = (((((((operator_i == riscv_defines_ALU_SLL) || (operator_i == riscv_defines_ALU_BINS)) || (operator_i == riscv_defines_ALU_FL1)) || (operator_i == riscv_defines_ALU_CLB)) || (operator_i == riscv_defines_ALU_DIV)) || (operator_i == riscv_defines_ALU_DIVU)) || (operator_i == riscv_defines_ALU_REM)) || (operator_i == riscv_defines_ALU_REMU);
	localparam riscv_defines_ALU_ADD = 7'b0011000;
	localparam riscv_defines_ALU_ADDU = 7'b0011010;
	assign shift_use_round = (((((((operator_i == riscv_defines_ALU_ADD) || (operator_i == riscv_defines_ALU_SUB)) || (operator_i == riscv_defines_ALU_ADDR)) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_ADDU)) || (operator_i == riscv_defines_ALU_SUBU)) || (operator_i == riscv_defines_ALU_ADDUR)) || (operator_i == riscv_defines_ALU_SUBUR);
	localparam riscv_defines_ALU_BEXT = 7'b0101000;
	localparam riscv_defines_ALU_SRA = 7'b0100100;
	assign shift_arithmetic = (((((operator_i == riscv_defines_ALU_SRA) || (operator_i == riscv_defines_ALU_BEXT)) || (operator_i == riscv_defines_ALU_ADD)) || (operator_i == riscv_defines_ALU_SUB)) || (operator_i == riscv_defines_ALU_ADDR)) || (operator_i == riscv_defines_ALU_SUBR);
	assign shift_op_a = (shift_left ? operand_a_rev : (shift_use_round ? adder_round_result : operand_a_i));
	assign shift_amt_int = (shift_use_round ? shift_amt_norm : (shift_left ? shift_amt_left : shift_amt));
	assign shift_amt_norm = {4 {3'b000, bmask_b_i}};
	wire [63:0] shift_op_a_32;
	localparam riscv_defines_ALU_ROR = 7'b0100110;
	assign shift_op_a_32 = (operator_i == riscv_defines_ALU_ROR ? {shift_op_a, shift_op_a} : $signed({{32 {shift_arithmetic & shift_op_a[31]}}, shift_op_a}));
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				shift_right_result[31:16] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:16]}) >>> shift_amt_int[19:16];
				shift_right_result[15:0] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:0]}) >>> shift_amt_int[3:0];
			end
			riscv_defines_VEC_MODE8: begin
				shift_right_result[31:24] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:24]}) >>> shift_amt_int[26:24];
				shift_right_result[23:16] = $signed({shift_arithmetic & shift_op_a[23], shift_op_a[23:16]}) >>> shift_amt_int[18:16];
				shift_right_result[15:8] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:8]}) >>> shift_amt_int[10:8];
				shift_right_result[7:0] = $signed({shift_arithmetic & shift_op_a[7], shift_op_a[7:0]}) >>> shift_amt_int[2:0];
			end
			default: shift_right_result = shift_op_a_32 >> shift_amt_int[4:0];
		endcase
	end
	genvar _gv_j_15;
	generate
		for (_gv_j_15 = 0; _gv_j_15 < 32; _gv_j_15 = _gv_j_15 + 1) begin : genblk3
			localparam j = _gv_j_15;
			assign shift_left_result[j] = shift_right_result[31 - j];
		end
	endgenerate
	assign shift_result = (shift_left ? shift_left_result : shift_right_result);
	reg [3:0] is_equal;
	reg [3:0] is_greater;
	wire [3:0] f_is_greater;
	reg [3:0] cmp_signed;
	wire [3:0] is_equal_vec;
	wire [3:0] is_greater_vec;
	localparam riscv_defines_ALU_CLIPU = 7'b0010111;
	localparam riscv_defines_ALU_FLE = 7'b1000101;
	localparam riscv_defines_ALU_FLT = 7'b1000100;
	localparam riscv_defines_ALU_FMAX = 7'b1000110;
	localparam riscv_defines_ALU_FMIN = 7'b1000111;
	localparam riscv_defines_ALU_GES = 7'b0001010;
	localparam riscv_defines_ALU_GTS = 7'b0001000;
	localparam riscv_defines_ALU_LES = 7'b0000100;
	localparam riscv_defines_ALU_LTS = 7'b0000000;
	localparam riscv_defines_ALU_MAX = 7'b0010010;
	localparam riscv_defines_ALU_MIN = 7'b0010000;
	localparam riscv_defines_ALU_SLETS = 7'b0000110;
	localparam riscv_defines_ALU_SLTS = 7'b0000010;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_signed = 4'b0000;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES, riscv_defines_ALU_SLTS, riscv_defines_ALU_SLETS, riscv_defines_ALU_MIN, riscv_defines_ALU_MAX, riscv_defines_ALU_ABS, riscv_defines_ALU_CLIP, riscv_defines_ALU_CLIPU, riscv_defines_ALU_FLE, riscv_defines_ALU_FLT, riscv_defines_ALU_FMAX, riscv_defines_ALU_FMIN:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: cmp_signed[3:0] = 4'b1111;
					riscv_defines_VEC_MODE16: cmp_signed[3:0] = 4'b1010;
					default: cmp_signed[3:0] = 4'b1000;
				endcase
			default:
				;
		endcase
	end
	genvar _gv_i_13;
	generate
		for (_gv_i_13 = 0; _gv_i_13 < 4; _gv_i_13 = _gv_i_13 + 1) begin : genblk4
			localparam i = _gv_i_13;
			assign is_equal_vec[i] = operand_a_i[(8 * i) + 7:8 * i] == operand_b_i[(8 * i) + 7:i * 8];
			assign is_greater_vec[i] = $signed({operand_a_i[(8 * i) + 7] & cmp_signed[i], operand_a_i[(8 * i) + 7:8 * i]}) > $signed({operand_b_i[(8 * i) + 7] & cmp_signed[i], operand_b_i[(8 * i) + 7:i * 8]});
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		is_equal[3:0] = {4 {((is_equal_vec[3] & is_equal_vec[2]) & is_equal_vec[1]) & is_equal_vec[0]}};
		is_greater[3:0] = {4 {is_greater_vec[3] | (is_equal_vec[3] & (is_greater_vec[2] | (is_equal_vec[2] & (is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])))))}};
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				is_equal[1:0] = {2 {is_equal_vec[0] & is_equal_vec[1]}};
				is_equal[3:2] = {2 {is_equal_vec[2] & is_equal_vec[3]}};
				is_greater[1:0] = {2 {is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])}};
				is_greater[3:2] = {2 {is_greater_vec[3] | (is_equal_vec[3] & is_greater_vec[2])}};
			end
			riscv_defines_VEC_MODE8: begin
				is_equal[3:0] = is_equal_vec[3:0];
				is_greater[3:0] = is_greater_vec[3:0];
			end
			default:
				;
		endcase
	end
	assign f_is_greater[3:0] = {4 {is_greater[3] ^ ((operand_a_i[31] & operand_b_i[31]) & !is_equal[3])}};
	reg [3:0] cmp_result;
	wire f_is_qnan;
	wire f_is_snan;
	reg [3:0] f_is_nan;
	localparam riscv_defines_ALU_EQ = 7'b0001100;
	localparam riscv_defines_ALU_FEQ = 7'b1000011;
	localparam riscv_defines_ALU_GEU = 7'b0001011;
	localparam riscv_defines_ALU_GTU = 7'b0001001;
	localparam riscv_defines_ALU_LEU = 7'b0000101;
	localparam riscv_defines_ALU_LTU = 7'b0000001;
	localparam riscv_defines_ALU_NE = 7'b0001101;
	localparam riscv_defines_ALU_SLETU = 7'b0000111;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_result = is_equal;
		f_is_nan = {4 {f_is_qnan | f_is_snan}};
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_EQ: cmp_result = is_equal;
			riscv_defines_ALU_NE: cmp_result = ~is_equal;
			riscv_defines_ALU_GTS, riscv_defines_ALU_GTU: cmp_result = is_greater;
			riscv_defines_ALU_GES, riscv_defines_ALU_GEU: cmp_result = is_greater | is_equal;
			riscv_defines_ALU_LTS, riscv_defines_ALU_SLTS, riscv_defines_ALU_LTU, riscv_defines_ALU_SLTU: cmp_result = ~(is_greater | is_equal);
			riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU, riscv_defines_ALU_LES, riscv_defines_ALU_LEU: cmp_result = ~is_greater;
			riscv_defines_ALU_FEQ: cmp_result = is_equal & ~f_is_nan;
			riscv_defines_ALU_FLE: cmp_result = ~f_is_greater & ~f_is_nan;
			riscv_defines_ALU_FLT: cmp_result = ~(f_is_greater | is_equal) & ~f_is_nan;
			default:
				;
		endcase
	end
	assign comparison_result_o = cmp_result[3];
	wire [31:0] result_minmax;
	wire [31:0] fp_canonical_nan;
	wire [3:0] sel_minmax;
	wire do_min;
	wire minmax_is_fp_special;
	wire [31:0] minmax_b;
	assign minmax_b = (operator_i == riscv_defines_ALU_ABS ? adder_result : operand_b_i);
	localparam riscv_defines_ALU_MINU = 7'b0010001;
	assign do_min = ((((operator_i == riscv_defines_ALU_MIN) || (operator_i == riscv_defines_ALU_MINU)) || (operator_i == riscv_defines_ALU_CLIP)) || (operator_i == riscv_defines_ALU_CLIPU)) || (operator_i == riscv_defines_ALU_FMIN);
	assign sel_minmax[3:0] = ((operator_i == riscv_defines_ALU_FMIN) || (operator_i == riscv_defines_ALU_FMAX) ? f_is_greater : is_greater) ^ {4 {do_min}};
	assign result_minmax[31:24] = (sel_minmax[3] == 1'b1 ? operand_a_i[31:24] : minmax_b[31:24]);
	assign result_minmax[23:16] = (sel_minmax[2] == 1'b1 ? operand_a_i[23:16] : minmax_b[23:16]);
	assign result_minmax[15:8] = (sel_minmax[1] == 1'b1 ? operand_a_i[15:8] : minmax_b[15:8]);
	assign result_minmax[7:0] = (sel_minmax[0] == 1'b1 ? operand_a_i[7:0] : minmax_b[7:0]);
	wire [31:0] fclass_result;
	generate
		if (FPU == 1) begin : genblk5
			wire [7:0] fclass_exponent;
			wire [22:0] fclass_mantiassa;
			wire fclass_ninf;
			wire fclass_pinf;
			wire fclass_normal;
			wire fclass_subnormal;
			wire fclass_nzero;
			wire fclass_pzero;
			wire fclass_is_negative;
			wire fclass_snan_a;
			wire fclass_qnan_a;
			wire fclass_snan_b;
			wire fclass_qnan_b;
			assign fclass_exponent = operand_a_i[30:23];
			assign fclass_mantiassa = operand_a_i[22:0];
			assign fclass_is_negative = operand_a_i[31];
			assign fclass_ninf = operand_a_i == 32'hff800000;
			assign fclass_pinf = operand_a_i == 32'h7f800000;
			assign fclass_normal = (fclass_exponent != 0) && (fclass_exponent != 255);
			assign fclass_subnormal = (fclass_exponent == 0) && (fclass_mantiassa != 0);
			assign fclass_nzero = operand_a_i == 32'h80000000;
			assign fclass_pzero = operand_a_i == 32'h00000000;
			assign fclass_snan_a = operand_a_i[30:0] == 32'h7fa00000;
			assign fclass_qnan_a = operand_a_i[30:0] == 32'h7fc00000;
			assign fclass_snan_b = operand_b_i[30:0] == 32'h7fa00000;
			assign fclass_qnan_b = operand_b_i[30:0] == 32'h7fc00000;
			assign fclass_result[31:0] = {{22 {1'b0}}, fclass_qnan_a, fclass_snan_a, fclass_pinf, fclass_normal && !fclass_is_negative, fclass_subnormal && !fclass_is_negative, fclass_pzero, fclass_nzero, fclass_subnormal && fclass_is_negative, fclass_normal && fclass_is_negative, fclass_ninf};
			assign f_is_qnan = fclass_qnan_a | fclass_qnan_b;
			assign f_is_snan = fclass_snan_a | fclass_snan_b;
			assign minmax_is_fp_special = ((operator_i == riscv_defines_ALU_FMIN) || (operator_i == riscv_defines_ALU_FMAX)) & (f_is_snan | f_is_qnan);
			assign fp_canonical_nan = 32'h7fc00000;
		end
		else begin : genblk5
			assign minmax_is_fp_special = 1'sb0;
			assign f_is_qnan = 1'sb0;
			assign f_is_snan = 1'sb0;
			assign fclass_result = 1'sb0;
			assign fp_canonical_nan = 1'sb0;
		end
	endgenerate
	reg [31:0] f_sign_inject_result;
	localparam riscv_defines_ALU_FKEEP = 7'b1111111;
	localparam riscv_defines_ALU_FSGNJ = 7'b1000000;
	localparam riscv_defines_ALU_FSGNJN = 7'b1000001;
	localparam riscv_defines_ALU_FSGNJX = 7'b1000010;
	always @(*) begin
		if (_sv2v_0)
			;
		if (FPU == 1) begin
			f_sign_inject_result[30:0] = operand_a_i[30:0];
			f_sign_inject_result[31] = operand_a_i[31];
			(* full_case, parallel_case *)
			case (operator_i)
				riscv_defines_ALU_FKEEP: f_sign_inject_result[31] = operand_a_i[31];
				riscv_defines_ALU_FSGNJ: f_sign_inject_result[31] = operand_b_i[31];
				riscv_defines_ALU_FSGNJN: f_sign_inject_result[31] = !operand_b_i[31];
				riscv_defines_ALU_FSGNJX: f_sign_inject_result[31] = operand_a_i[31] ^ operand_b_i[31];
				default:
					;
			endcase
		end
		else
			f_sign_inject_result = 1'sb0;
	end
	wire [31:0] clip_result;
	wire clip_is_lower_neg;
	wire clip_is_lower_u;
	assign clip_is_lower_neg = adder_result_expanded[36];
	assign clip_is_lower_u = (operator_i == riscv_defines_ALU_CLIPU) && operand_a_i[31];
	assign clip_result = (is_greater ? result_minmax : (clip_is_lower_u ? {32 {1'sb0}} : (clip_is_lower_neg ? operand_b_neg : result_minmax)));
	reg [7:0] shuffle_byte_sel;
	reg [3:0] shuffle_reg_sel;
	reg [1:0] shuffle_reg1_sel;
	reg [1:0] shuffle_reg0_sel;
	reg [3:0] shuffle_through;
	wire [31:0] shuffle_r1;
	wire [31:0] shuffle_r0;
	wire [31:0] shuffle_r1_in;
	wire [31:0] shuffle_r0_in;
	wire [31:0] shuffle_result;
	wire [31:0] pack_result;
	localparam riscv_defines_ALU_EXT = 7'b0111111;
	localparam riscv_defines_ALU_EXTS = 7'b0111110;
	localparam riscv_defines_ALU_INS = 7'b0101101;
	localparam riscv_defines_ALU_PCKHI = 7'b0111001;
	localparam riscv_defines_ALU_PCKLO = 7'b0111000;
	localparam riscv_defines_ALU_SHUF2 = 7'b0111011;
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_reg_sel = 1'sb0;
		shuffle_reg1_sel = 2'b01;
		shuffle_reg0_sel = 2'b10;
		shuffle_through = 1'sb1;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_EXT, riscv_defines_ALU_EXTS: begin
				if (operator_i == riscv_defines_ALU_EXTS)
					shuffle_reg1_sel = 2'b11;
				if (vector_mode_i == riscv_defines_VEC_MODE8) begin
					shuffle_reg_sel[3:1] = 3'b111;
					shuffle_reg_sel[0] = 1'b0;
				end
				else begin
					shuffle_reg_sel[3:2] = 2'b11;
					shuffle_reg_sel[1:0] = 2'b00;
				end
			end
			riscv_defines_ALU_PCKLO: begin
				shuffle_reg1_sel = 2'b00;
				if (vector_mode_i == riscv_defines_VEC_MODE8) begin
					shuffle_through = 4'b0011;
					shuffle_reg_sel = 4'b0001;
				end
				else
					shuffle_reg_sel = 4'b0011;
			end
			riscv_defines_ALU_PCKHI: begin
				shuffle_reg1_sel = 2'b00;
				shuffle_reg_sel = 4'b0100;
				shuffle_through = 4'b1100;
			end
			riscv_defines_ALU_SHUF2:
				(* full_case, parallel_case *)
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_reg_sel[3] = ~operand_b_i[26];
						shuffle_reg_sel[2] = ~operand_b_i[18];
						shuffle_reg_sel[1] = ~operand_b_i[10];
						shuffle_reg_sel[0] = ~operand_b_i[2];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_reg_sel[3] = ~operand_b_i[17];
						shuffle_reg_sel[2] = ~operand_b_i[17];
						shuffle_reg_sel[1] = ~operand_b_i[1];
						shuffle_reg_sel[0] = ~operand_b_i[1];
					end
					default:
						;
				endcase
			riscv_defines_ALU_INS:
				(* full_case, parallel_case *)
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_reg0_sel = 2'b00;
						(* full_case, parallel_case *)
						case (imm_vec_ext_i)
							2'b00: shuffle_reg_sel[3:0] = 4'b1110;
							2'b01: shuffle_reg_sel[3:0] = 4'b1101;
							2'b10: shuffle_reg_sel[3:0] = 4'b1011;
							2'b11: shuffle_reg_sel[3:0] = 4'b0111;
							default:
								;
						endcase
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_reg0_sel = 2'b01;
						shuffle_reg_sel[3] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[2] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[1] = imm_vec_ext_i[0];
						shuffle_reg_sel[0] = imm_vec_ext_i[0];
					end
					default:
						;
				endcase
			default:
				;
		endcase
	end
	localparam riscv_defines_ALU_SHUF = 7'b0111010;
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_byte_sel = 1'sb0;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_EXTS, riscv_defines_ALU_EXT:
				(* full_case, parallel_case *)
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[4+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[2+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[0+:2] = imm_vec_ext_i[1:0];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[4+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[2+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {imm_vec_ext_i[0], 1'b0};
					end
					default:
						;
				endcase
			riscv_defines_ALU_PCKLO, riscv_defines_ALU_PCKHI:
				(* full_case, parallel_case *)
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = 2'b00;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b00;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = 2'b01;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b01;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					default:
						;
				endcase
			riscv_defines_ALU_SHUF2, riscv_defines_ALU_SHUF:
				(* full_case, parallel_case *)
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = operand_b_i[25:24];
						shuffle_byte_sel[4+:2] = operand_b_i[17:16];
						shuffle_byte_sel[2+:2] = operand_b_i[9:8];
						shuffle_byte_sel[0+:2] = operand_b_i[1:0];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {operand_b_i[16], 1'b1};
						shuffle_byte_sel[4+:2] = {operand_b_i[16], 1'b0};
						shuffle_byte_sel[2+:2] = {operand_b_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {operand_b_i[0], 1'b0};
					end
					default:
						;
				endcase
			riscv_defines_ALU_INS: begin
				shuffle_byte_sel[6+:2] = 2'b11;
				shuffle_byte_sel[4+:2] = 2'b10;
				shuffle_byte_sel[2+:2] = 2'b01;
				shuffle_byte_sel[0+:2] = 2'b00;
			end
			default:
				;
		endcase
	end
	assign shuffle_r0_in = (shuffle_reg0_sel[1] ? operand_a_i : (shuffle_reg0_sel[0] ? {2 {operand_a_i[15:0]}} : {4 {operand_a_i[7:0]}}));
	assign shuffle_r1_in = (shuffle_reg1_sel[1] ? {{8 {operand_a_i[31]}}, {8 {operand_a_i[23]}}, {8 {operand_a_i[15]}}, {8 {operand_a_i[7]}}} : (shuffle_reg1_sel[0] ? operand_c_i : operand_b_i));
	assign shuffle_r0[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r1[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_result[31:24] = (shuffle_reg_sel[3] ? shuffle_r1[31:24] : shuffle_r0[31:24]);
	assign shuffle_result[23:16] = (shuffle_reg_sel[2] ? shuffle_r1[23:16] : shuffle_r0[23:16]);
	assign shuffle_result[15:8] = (shuffle_reg_sel[1] ? shuffle_r1[15:8] : shuffle_r0[15:8]);
	assign shuffle_result[7:0] = (shuffle_reg_sel[0] ? shuffle_r1[7:0] : shuffle_r0[7:0]);
	assign pack_result[31:24] = (shuffle_through[3] ? shuffle_result[31:24] : operand_c_i[31:24]);
	assign pack_result[23:16] = (shuffle_through[2] ? shuffle_result[23:16] : operand_c_i[23:16]);
	assign pack_result[15:8] = (shuffle_through[1] ? shuffle_result[15:8] : operand_c_i[15:8]);
	assign pack_result[7:0] = (shuffle_through[0] ? shuffle_result[7:0] : operand_c_i[7:0]);
	reg [31:0] ff_input;
	wire [5:0] cnt_result;
	wire [5:0] clb_result;
	wire [4:0] ff1_result;
	wire ff_no_one;
	wire [4:0] fl1_result;
	reg [5:0] bitop_result;
	alu_popcnt alu_popcnt_i(
		.in_i(operand_a_i),
		.result_o(cnt_result)
	);
	localparam riscv_defines_ALU_FF1 = 7'b0110110;
	always @(*) begin
		if (_sv2v_0)
			;
		ff_input = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_FF1: ff_input = operand_a_i;
			riscv_defines_ALU_DIVU, riscv_defines_ALU_REMU, riscv_defines_ALU_FL1: ff_input = operand_a_rev;
			riscv_defines_ALU_DIV, riscv_defines_ALU_REM, riscv_defines_ALU_CLB:
				if (operand_a_i[31])
					ff_input = operand_a_neg_rev;
				else
					ff_input = operand_a_rev;
		endcase
	end
	alu_ff alu_ff_i(
		.in_i(ff_input),
		.first_one_o(ff1_result),
		.no_ones_o(ff_no_one)
	);
	assign fl1_result = 5'd31 - ff1_result;
	assign clb_result = ff1_result - 5'd1;
	localparam riscv_defines_ALU_CNT = 7'b0110100;
	always @(*) begin
		if (_sv2v_0)
			;
		bitop_result = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_FF1: bitop_result = (ff_no_one ? 6'd32 : {1'b0, ff1_result});
			riscv_defines_ALU_FL1: bitop_result = (ff_no_one ? 6'd32 : {1'b0, fl1_result});
			riscv_defines_ALU_CNT: bitop_result = cnt_result;
			riscv_defines_ALU_CLB:
				if (ff_no_one) begin
					if (operand_a_i[31])
						bitop_result = 6'd31;
					else
						bitop_result = 1'sb0;
				end
				else
					bitop_result = clb_result;
			default:
				;
		endcase
	end
	wire extract_is_signed;
	wire extract_sign;
	wire [31:0] bmask_first;
	wire [31:0] bmask_inv;
	wire [31:0] bextins_and;
	wire [31:0] bextins_result;
	wire [31:0] bclr_result;
	wire [31:0] bset_result;
	assign bmask_first = 32'hfffffffe << bmask_a_i;
	assign bmask = ~bmask_first << bmask_b_i;
	assign bmask_inv = ~bmask;
	assign bextins_and = (operator_i == riscv_defines_ALU_BINS ? operand_c_i : {32 {extract_sign}});
	assign extract_is_signed = operator_i == riscv_defines_ALU_BEXT;
	assign extract_sign = extract_is_signed & shift_result[bmask_a_i];
	assign bextins_result = (bmask & shift_result) | (bextins_and & bmask_inv);
	assign bclr_result = operand_a_i & bmask_inv;
	assign bset_result = operand_a_i | bmask;
	wire [31:0] result_div;
	wire div_ready;
	generate
		if (SHARED_INT_DIV == 1) begin : genblk6
			assign result_div = 1'sb0;
			assign div_ready = 1'sb1;
			assign div_valid = 1'sb0;
		end
		else begin : int_div
			wire div_signed;
			wire div_op_a_signed;
			wire div_op_b_signed;
			wire [5:0] div_shift_int;
			assign div_signed = operator_i[0];
			assign div_op_a_signed = operand_a_i[31] & div_signed;
			assign div_op_b_signed = operand_b_i[31] & div_signed;
			assign div_shift_int = (ff_no_one ? 6'd31 : clb_result);
			assign div_shift = div_shift_int + (div_op_a_signed ? 6'd0 : 6'd1);
			assign div_valid = (((operator_i == riscv_defines_ALU_DIV) || (operator_i == riscv_defines_ALU_DIVU)) || (operator_i == riscv_defines_ALU_REM)) || (operator_i == riscv_defines_ALU_REMU);
			riscv_alu_div div_i(
				.Clk_CI(clk),
				.Rst_RBI(rst_n),
				.OpA_DI(operand_b_i),
				.OpB_DI(shift_left_result),
				.OpBShift_DI(div_shift),
				.OpBIsZero_SI(cnt_result == 0),
				.OpBSign_SI(div_op_a_signed),
				.OpCode_SI(operator_i[1:0]),
				.Res_DO(result_div),
				.InVld_SI(div_valid),
				.OutRdy_SI(ex_ready_i),
				.OutVld_SO(div_ready)
			);
		end
	endgenerate
	localparam riscv_defines_ALU_AND = 7'b0010101;
	localparam riscv_defines_ALU_BCLR = 7'b0101011;
	localparam riscv_defines_ALU_BEXTU = 7'b0101001;
	localparam riscv_defines_ALU_BSET = 7'b0101100;
	localparam riscv_defines_ALU_FCLASS = 7'b1001000;
	localparam riscv_defines_ALU_MAXU = 7'b0010011;
	localparam riscv_defines_ALU_OR = 7'b0101110;
	localparam riscv_defines_ALU_SRL = 7'b0100101;
	localparam riscv_defines_ALU_XOR = 7'b0101111;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_AND: result_o = operand_a_i & operand_b_i;
			riscv_defines_ALU_OR: result_o = operand_a_i | operand_b_i;
			riscv_defines_ALU_XOR: result_o = operand_a_i ^ operand_b_i;
			riscv_defines_ALU_ADD, riscv_defines_ALU_ADDR, riscv_defines_ALU_ADDU, riscv_defines_ALU_ADDUR, riscv_defines_ALU_SUB, riscv_defines_ALU_SUBR, riscv_defines_ALU_SUBU, riscv_defines_ALU_SUBUR, riscv_defines_ALU_SLL, riscv_defines_ALU_SRL, riscv_defines_ALU_SRA, riscv_defines_ALU_ROR: result_o = shift_result;
			riscv_defines_ALU_BINS, riscv_defines_ALU_BEXT, riscv_defines_ALU_BEXTU: result_o = bextins_result;
			riscv_defines_ALU_BCLR: result_o = bclr_result;
			riscv_defines_ALU_BSET: result_o = bset_result;
			riscv_defines_ALU_SHUF, riscv_defines_ALU_SHUF2, riscv_defines_ALU_PCKLO, riscv_defines_ALU_PCKHI, riscv_defines_ALU_EXT, riscv_defines_ALU_EXTS, riscv_defines_ALU_INS: result_o = pack_result;
			riscv_defines_ALU_MIN, riscv_defines_ALU_MINU, riscv_defines_ALU_MAX, riscv_defines_ALU_MAXU, riscv_defines_ALU_ABS, riscv_defines_ALU_FMIN, riscv_defines_ALU_FMAX: result_o = (minmax_is_fp_special ? fp_canonical_nan : result_minmax);
			riscv_defines_ALU_CLIP, riscv_defines_ALU_CLIPU: result_o = clip_result;
			riscv_defines_ALU_EQ, riscv_defines_ALU_NE, riscv_defines_ALU_GTU, riscv_defines_ALU_GEU, riscv_defines_ALU_LTU, riscv_defines_ALU_LEU, riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES: begin
				result_o[31:24] = {8 {cmp_result[3]}};
				result_o[23:16] = {8 {cmp_result[2]}};
				result_o[15:8] = {8 {cmp_result[1]}};
				result_o[7:0] = {8 {cmp_result[0]}};
			end
			riscv_defines_ALU_FEQ, riscv_defines_ALU_FLT, riscv_defines_ALU_FLE, riscv_defines_ALU_SLTS, riscv_defines_ALU_SLTU, riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU: result_o = {31'b0000000000000000000000000000000, comparison_result_o};
			riscv_defines_ALU_FF1, riscv_defines_ALU_FL1, riscv_defines_ALU_CLB, riscv_defines_ALU_CNT: result_o = {26'h0000000, bitop_result[5:0]};
			riscv_defines_ALU_DIV, riscv_defines_ALU_DIVU, riscv_defines_ALU_REM, riscv_defines_ALU_REMU: result_o = result_div;
			riscv_defines_ALU_FCLASS: result_o = fclass_result;
			riscv_defines_ALU_FSGNJ, riscv_defines_ALU_FSGNJN, riscv_defines_ALU_FSGNJX, riscv_defines_ALU_FKEEP: result_o = f_sign_inject_result;
			default:
				;
		endcase
	end
	assign ready_o = div_ready;
	initial _sv2v_0 = 0;
endmodule
module alu_ff (
	in_i,
	first_one_o,
	no_ones_o
);
	parameter LEN = 32;
	input wire [LEN - 1:0] in_i;
	output wire [$clog2(LEN) - 1:0] first_one_o;
	output wire no_ones_o;
	localparam NUM_LEVELS = $clog2(LEN);
	wire [(LEN * NUM_LEVELS) - 1:0] index_lut;
	wire [(2 ** NUM_LEVELS) - 1:0] sel_nodes;
	wire [((2 ** NUM_LEVELS) * NUM_LEVELS) - 1:0] index_nodes;
	genvar _gv_j_16;
	generate
		for (_gv_j_16 = 0; _gv_j_16 < LEN; _gv_j_16 = _gv_j_16 + 1) begin : genblk1
			localparam j = _gv_j_16;
			assign index_lut[j * NUM_LEVELS+:NUM_LEVELS] = $unsigned(j);
		end
	endgenerate
	genvar _gv_k_9;
	genvar _gv_l_1;
	genvar _gv_level_1;
	generate
		for (_gv_level_1 = 0; _gv_level_1 < NUM_LEVELS; _gv_level_1 = _gv_level_1 + 1) begin : genblk2
			localparam level = _gv_level_1;
			if (level < (NUM_LEVELS - 1)) begin : genblk1
				for (_gv_l_1 = 0; _gv_l_1 < (2 ** level); _gv_l_1 = _gv_l_1 + 1) begin : genblk1
					localparam l = _gv_l_1;
					assign sel_nodes[((2 ** level) - 1) + l] = sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] | sel_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) + 1];
					assign index_nodes[(((2 ** level) - 1) + l) * NUM_LEVELS+:NUM_LEVELS] = (sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] == 1'b1 ? index_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) * NUM_LEVELS+:NUM_LEVELS] : index_nodes[((((2 ** (level + 1)) - 1) + (l * 2)) + 1) * NUM_LEVELS+:NUM_LEVELS]);
				end
			end
			if (level == (NUM_LEVELS - 1)) begin : genblk2
				for (_gv_k_9 = 0; _gv_k_9 < (2 ** level); _gv_k_9 = _gv_k_9 + 1) begin : genblk1
					localparam k = _gv_k_9;
					if ((k * 2) < (LEN - 1)) begin : genblk1
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2] | in_i[(k * 2) + 1];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = (in_i[k * 2] == 1'b1 ? index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS] : index_lut[((k * 2) + 1) * NUM_LEVELS+:NUM_LEVELS]);
					end
					if ((k * 2) == (LEN - 1)) begin : genblk2
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS];
					end
					if ((k * 2) > (LEN - 1)) begin : genblk3
						assign sel_nodes[((2 ** level) - 1) + k] = 1'b0;
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = 1'sb0;
					end
				end
			end
		end
	endgenerate
	assign first_one_o = index_nodes[0+:NUM_LEVELS];
	assign no_ones_o = ~sel_nodes[0];
endmodule
module alu_popcnt (
	in_i,
	result_o
);
	input wire [31:0] in_i;
	output wire [5:0] result_o;
	wire [31:0] cnt_l1;
	wire [23:0] cnt_l2;
	wire [15:0] cnt_l3;
	wire [9:0] cnt_l4;
	genvar _gv_l_2;
	genvar _gv_m_2;
	genvar _gv_n_1;
	genvar _gv_p_1;
	generate
		for (_gv_l_2 = 0; _gv_l_2 < 16; _gv_l_2 = _gv_l_2 + 1) begin : genblk1
			localparam l = _gv_l_2;
			assign cnt_l1[l * 2+:2] = {1'b0, in_i[2 * l]} + {1'b0, in_i[(2 * l) + 1]};
		end
		for (_gv_m_2 = 0; _gv_m_2 < 8; _gv_m_2 = _gv_m_2 + 1) begin : genblk2
			localparam m = _gv_m_2;
			assign cnt_l2[m * 3+:3] = {1'b0, cnt_l1[(2 * m) * 2+:2]} + {1'b0, cnt_l1[((2 * m) + 1) * 2+:2]};
		end
		for (_gv_n_1 = 0; _gv_n_1 < 4; _gv_n_1 = _gv_n_1 + 1) begin : genblk3
			localparam n = _gv_n_1;
			assign cnt_l3[n * 4+:4] = {1'b0, cnt_l2[(2 * n) * 3+:3]} + {1'b0, cnt_l2[((2 * n) + 1) * 3+:3]};
		end
		for (_gv_p_1 = 0; _gv_p_1 < 2; _gv_p_1 = _gv_p_1 + 1) begin : genblk4
			localparam p = _gv_p_1;
			assign cnt_l4[p * 5+:5] = {1'b0, cnt_l3[(2 * p) * 4+:4]} + {1'b0, cnt_l3[((2 * p) + 1) * 4+:4]};
		end
	endgenerate
	assign result_o = {1'b0, cnt_l4[0+:5]} + {1'b0, cnt_l4[5+:5]};
endmodule
module riscv_alu_basic (
	clk,
	rst_n,
	operator_i,
	operand_a_i,
	operand_b_i,
	operand_c_i,
	vector_mode_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	result_o,
	comparison_result_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	input wire [6:0] operator_i;
	input wire [31:0] operand_a_i;
	input wire [31:0] operand_b_i;
	input wire [31:0] operand_c_i;
	input wire [1:0] vector_mode_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	output reg [31:0] result_o;
	output wire comparison_result_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [31:0] operand_a_rev;
	wire [31:0] operand_a_neg;
	wire [31:0] operand_a_neg_rev;
	assign operand_a_neg = ~operand_a_i;
	genvar _gv_k_10;
	generate
		for (_gv_k_10 = 0; _gv_k_10 < 32; _gv_k_10 = _gv_k_10 + 1) begin : genblk1
			localparam k = _gv_k_10;
			assign operand_a_rev[k] = operand_a_i[31 - k];
		end
	endgenerate
	genvar _gv_m_3;
	generate
		for (_gv_m_3 = 0; _gv_m_3 < 32; _gv_m_3 = _gv_m_3 + 1) begin : genblk2
			localparam m = _gv_m_3;
			assign operand_a_neg_rev[m] = operand_a_neg[31 - m];
		end
	endgenerate
	wire [31:0] operand_b_neg;
	assign operand_b_neg = ~operand_b_i;
	wire [31:0] bmask;
	wire adder_op_b_negate;
	wire [31:0] adder_op_a;
	wire [31:0] adder_op_b;
	wire [35:0] adder_in_a;
	wire [35:0] adder_in_b;
	wire [31:0] adder_result;
	wire [35:0] adder_result_expanded;
	localparam riscv_defines_ALU_SUB = 7'b0011001;
	localparam riscv_defines_ALU_SUBR = 7'b0011101;
	localparam riscv_defines_ALU_SUBU = 7'b0011011;
	assign adder_op_b_negate = (((operator_i == riscv_defines_ALU_SUB) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_SUBU)) || (operator_i == riscv_defines_ALU_SUBR);
	localparam riscv_defines_ALU_ABS = 7'b0010100;
	assign adder_op_a = (operator_i == riscv_defines_ALU_ABS ? operand_a_neg : operand_a_i);
	assign adder_op_b = (adder_op_b_negate ? operand_b_neg : operand_b_i);
	assign adder_result = (adder_op_a + adder_op_b) + adder_op_b_negate;
	wire shift_left;
	wire shift_arithmetic;
	wire [31:0] shift_amt_left;
	wire [31:0] shift_amt;
	wire [31:0] shift_amt_int;
	wire [31:0] shift_op_a;
	wire [32:0] shift_op_a_ext;
	wire [31:0] shift_result;
	wire [31:0] shift_right_result;
	wire [31:0] shift_left_result;
	assign shift_amt = operand_b_i;
	assign shift_amt_left[31:0] = shift_amt[31:0];
	localparam riscv_defines_ALU_SLL = 7'b0100111;
	assign shift_left = operator_i == riscv_defines_ALU_SLL;
	localparam riscv_defines_ALU_SRA = 7'b0100100;
	assign shift_arithmetic = operator_i == riscv_defines_ALU_SRA;
	assign shift_op_a = (shift_left ? operand_a_rev : operand_a_i);
	assign shift_amt_int = (shift_left ? shift_amt_left : shift_amt);
	assign shift_op_a_ext = (shift_arithmetic ? {shift_op_a[31], shift_op_a} : {1'b0, shift_op_a});
	assign shift_right_result = $signed(shift_op_a_ext) >>> shift_amt_int[4:0];
	genvar _gv_j_17;
	generate
		for (_gv_j_17 = 0; _gv_j_17 < 32; _gv_j_17 = _gv_j_17 + 1) begin : genblk3
			localparam j = _gv_j_17;
			assign shift_left_result[j] = shift_right_result[31 - j];
		end
	endgenerate
	assign shift_result = (shift_left ? shift_left_result : shift_right_result);
	reg [3:0] is_equal;
	reg [3:0] is_greater;
	reg [3:0] cmp_signed;
	wire [3:0] is_equal_vec;
	wire [3:0] is_greater_vec;
	localparam riscv_defines_ALU_CLIP = 7'b0010110;
	localparam riscv_defines_ALU_CLIPU = 7'b0010111;
	localparam riscv_defines_ALU_GES = 7'b0001010;
	localparam riscv_defines_ALU_GTS = 7'b0001000;
	localparam riscv_defines_ALU_LES = 7'b0000100;
	localparam riscv_defines_ALU_LTS = 7'b0000000;
	localparam riscv_defines_ALU_MAX = 7'b0010010;
	localparam riscv_defines_ALU_MIN = 7'b0010000;
	localparam riscv_defines_ALU_SLETS = 7'b0000110;
	localparam riscv_defines_ALU_SLTS = 7'b0000010;
	localparam riscv_defines_VEC_MODE16 = 2'b10;
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_signed = 4'b0000;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES, riscv_defines_ALU_SLTS, riscv_defines_ALU_SLETS, riscv_defines_ALU_MIN, riscv_defines_ALU_MAX, riscv_defines_ALU_ABS, riscv_defines_ALU_CLIP, riscv_defines_ALU_CLIPU:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: cmp_signed[3:0] = 4'b1111;
					riscv_defines_VEC_MODE16: cmp_signed[3:0] = 4'b1010;
					default: cmp_signed[3:0] = 4'b1000;
				endcase
			default:
				;
		endcase
	end
	genvar _gv_i_14;
	generate
		for (_gv_i_14 = 0; _gv_i_14 < 4; _gv_i_14 = _gv_i_14 + 1) begin : genblk4
			localparam i = _gv_i_14;
			assign is_equal_vec[i] = operand_a_i[(8 * i) + 7:8 * i] == operand_b_i[(8 * i) + 7:i * 8];
			assign is_greater_vec[i] = $signed({operand_a_i[(8 * i) + 7] & cmp_signed[i], operand_a_i[(8 * i) + 7:8 * i]}) > $signed({operand_b_i[(8 * i) + 7] & cmp_signed[i], operand_b_i[(8 * i) + 7:i * 8]});
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		is_equal[3:0] = {4 {((is_equal_vec[3] & is_equal_vec[2]) & is_equal_vec[1]) & is_equal_vec[0]}};
		is_greater[3:0] = {4 {is_greater_vec[3] | (is_equal_vec[3] & (is_greater_vec[2] | (is_equal_vec[2] & (is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])))))}};
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				is_equal[1:0] = {2 {is_equal_vec[0] & is_equal_vec[1]}};
				is_equal[3:2] = {2 {is_equal_vec[2] & is_equal_vec[3]}};
				is_greater[1:0] = {2 {is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])}};
				is_greater[3:2] = {2 {is_greater_vec[3] | (is_equal_vec[3] & is_greater_vec[2])}};
			end
			riscv_defines_VEC_MODE8: begin
				is_equal[3:0] = is_equal_vec[3:0];
				is_greater[3:0] = is_greater_vec[3:0];
			end
			default:
				;
		endcase
	end
	reg [3:0] cmp_result;
	localparam riscv_defines_ALU_EQ = 7'b0001100;
	localparam riscv_defines_ALU_GEU = 7'b0001011;
	localparam riscv_defines_ALU_GTU = 7'b0001001;
	localparam riscv_defines_ALU_LEU = 7'b0000101;
	localparam riscv_defines_ALU_LTU = 7'b0000001;
	localparam riscv_defines_ALU_NE = 7'b0001101;
	localparam riscv_defines_ALU_SLETU = 7'b0000111;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_result = is_equal;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_EQ: cmp_result = is_equal;
			riscv_defines_ALU_NE: cmp_result = ~is_equal;
			riscv_defines_ALU_GTS, riscv_defines_ALU_GTU: cmp_result = is_greater;
			riscv_defines_ALU_GES, riscv_defines_ALU_GEU: cmp_result = is_greater | is_equal;
			riscv_defines_ALU_LTS, riscv_defines_ALU_SLTS, riscv_defines_ALU_LTU, riscv_defines_ALU_SLTU: cmp_result = ~(is_greater | is_equal);
			riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU, riscv_defines_ALU_LES, riscv_defines_ALU_LEU: cmp_result = ~is_greater;
			default:
				;
		endcase
	end
	assign comparison_result_o = cmp_result[3];
	localparam riscv_defines_ALU_ADD = 7'b0011000;
	localparam riscv_defines_ALU_AND = 7'b0010101;
	localparam riscv_defines_ALU_OR = 7'b0101110;
	localparam riscv_defines_ALU_SRL = 7'b0100101;
	localparam riscv_defines_ALU_XOR = 7'b0101111;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sbx;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_ALU_AND: result_o = operand_a_i & operand_b_i;
			riscv_defines_ALU_OR: result_o = operand_a_i | operand_b_i;
			riscv_defines_ALU_XOR: result_o = operand_a_i ^ operand_b_i;
			riscv_defines_ALU_ADD, riscv_defines_ALU_SUB: result_o = adder_result;
			riscv_defines_ALU_SLL, riscv_defines_ALU_SRL, riscv_defines_ALU_SRA: result_o = shift_result;
			riscv_defines_ALU_EQ, riscv_defines_ALU_NE, riscv_defines_ALU_GTU, riscv_defines_ALU_GEU, riscv_defines_ALU_LTU, riscv_defines_ALU_LEU, riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES: begin
				result_o[31:24] = {8 {cmp_result[3]}};
				result_o[23:16] = {8 {cmp_result[2]}};
				result_o[15:8] = {8 {cmp_result[1]}};
				result_o[7:0] = {8 {cmp_result[0]}};
			end
			riscv_defines_ALU_SLTS, riscv_defines_ALU_SLTU, riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU: result_o = {31'b0000000000000000000000000000000, comparison_result_o};
			default:
				$display("Warning [%0t] hackatdac18-2018-soc/ips/riscv/riscv_alu_basic.sv:313:9 - riscv_alu_basic.<unnamed_block>.<unnamed_block>\n msg: ", $time, "instruction not supported in basic alu");
		endcase
	end
	assign ready_o = 1'b1;
	initial _sv2v_0 = 0;
endmodule
module riscv_alu_div (
	Clk_CI,
	Rst_RBI,
	OpA_DI,
	OpB_DI,
	OpBShift_DI,
	OpBIsZero_SI,
	OpBSign_SI,
	OpCode_SI,
	InVld_SI,
	OutRdy_SI,
	OutVld_SO,
	Res_DO
);
	reg _sv2v_0;
	parameter C_WIDTH = 32;
	parameter C_LOG_WIDTH = 6;
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire [C_WIDTH - 1:0] OpA_DI;
	input wire [C_WIDTH - 1:0] OpB_DI;
	input wire [C_LOG_WIDTH - 1:0] OpBShift_DI;
	input wire OpBIsZero_SI;
	input wire OpBSign_SI;
	input wire [1:0] OpCode_SI;
	input wire InVld_SI;
	input wire OutRdy_SI;
	output reg OutVld_SO;
	output wire [C_WIDTH - 1:0] Res_DO;
	reg [C_WIDTH - 1:0] ResReg_DP;
	wire [C_WIDTH - 1:0] ResReg_DN;
	wire [C_WIDTH - 1:0] ResReg_DP_rev;
	reg [C_WIDTH - 1:0] AReg_DP;
	wire [C_WIDTH - 1:0] AReg_DN;
	reg [C_WIDTH - 1:0] BReg_DP;
	wire [C_WIDTH - 1:0] BReg_DN;
	wire RemSel_SN;
	reg RemSel_SP;
	wire CompInv_SN;
	reg CompInv_SP;
	wire ResInv_SN;
	reg ResInv_SP;
	wire [C_WIDTH - 1:0] AddMux_D;
	wire [C_WIDTH - 1:0] AddOut_D;
	wire [C_WIDTH - 1:0] AddTmp_D;
	wire [C_WIDTH - 1:0] BMux_D;
	wire [C_WIDTH - 1:0] OutMux_D;
	reg [C_LOG_WIDTH - 1:0] Cnt_DP;
	wire [C_LOG_WIDTH - 1:0] Cnt_DN;
	wire CntZero_S;
	reg ARegEn_S;
	reg BRegEn_S;
	reg ResRegEn_S;
	wire ABComp_S;
	wire PmSel_S;
	reg LoadEn_S;
	reg [1:0] State_SN;
	reg [1:0] State_SP;
	assign PmSel_S = LoadEn_S & ~(OpCode_SI[0] & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI));
	assign AddMux_D = (LoadEn_S ? OpA_DI : BReg_DP);
	assign BMux_D = (LoadEn_S ? OpB_DI : {CompInv_SP, BReg_DP[C_WIDTH - 1:1]});
	function automatic [C_WIDTH - 1:0] _sv2v_strm_4AED9;
		input reg [(0 + C_WIDTH) - 1:0] inp;
		reg [(0 + C_WIDTH) - 1:0] _sv2v_strm_5EA55_inp;
		reg [(0 + C_WIDTH) - 1:0] _sv2v_strm_5EA55_out;
		integer _sv2v_strm_5EA55_idx;
		begin
			_sv2v_strm_5EA55_inp = {inp};
			for (_sv2v_strm_5EA55_idx = 0; _sv2v_strm_5EA55_idx <= ((0 + C_WIDTH) - 1); _sv2v_strm_5EA55_idx = _sv2v_strm_5EA55_idx + 1)
				_sv2v_strm_5EA55_out[((0 + C_WIDTH) - 1) - _sv2v_strm_5EA55_idx-:1] = _sv2v_strm_5EA55_inp[_sv2v_strm_5EA55_idx+:1];
			_sv2v_strm_4AED9 = ((0 + C_WIDTH) <= C_WIDTH ? _sv2v_strm_5EA55_out << (C_WIDTH - (0 + C_WIDTH)) : _sv2v_strm_5EA55_out >> ((0 + C_WIDTH) - C_WIDTH));
		end
	endfunction
	assign ResReg_DP_rev = _sv2v_strm_4AED9({ResReg_DP});
	assign OutMux_D = (RemSel_SP ? AReg_DP : ResReg_DP_rev);
	assign Res_DO = (ResInv_SP ? -$signed(OutMux_D) : OutMux_D);
	assign ABComp_S = ((AReg_DP == BReg_DP) | ((AReg_DP > BReg_DP) ^ CompInv_SP)) & (|AReg_DP | OpBIsZero_SI);
	assign AddTmp_D = (LoadEn_S ? 0 : AReg_DP);
	assign AddOut_D = (PmSel_S ? AddTmp_D + AddMux_D : AddTmp_D - $signed(AddMux_D));
	assign Cnt_DN = (LoadEn_S ? OpBShift_DI : (~CntZero_S ? Cnt_DP - 1 : Cnt_DP));
	assign CntZero_S = ~(|Cnt_DP);
	always @(*) begin : p_fsm
		if (_sv2v_0)
			;
		State_SN = State_SP;
		OutVld_SO = 1'b0;
		LoadEn_S = 1'b0;
		ARegEn_S = 1'b0;
		BRegEn_S = 1'b0;
		ResRegEn_S = 1'b0;
		case (State_SP)
			2'd0: begin
				OutVld_SO = 1'b1;
				if (InVld_SI) begin
					OutVld_SO = 1'b0;
					ARegEn_S = 1'b1;
					BRegEn_S = 1'b1;
					LoadEn_S = 1'b1;
					State_SN = 2'd1;
				end
			end
			2'd1: begin
				ARegEn_S = ABComp_S;
				BRegEn_S = 1'b1;
				ResRegEn_S = 1'b1;
				if (CntZero_S)
					State_SN = 2'd2;
			end
			2'd2: begin
				OutVld_SO = 1'b1;
				if (OutRdy_SI)
					State_SN = 2'd0;
			end
			default:
				;
		endcase
	end
	assign RemSel_SN = (LoadEn_S ? OpCode_SI[1] : RemSel_SP);
	assign CompInv_SN = (LoadEn_S ? OpBSign_SI : CompInv_SP);
	assign ResInv_SN = (LoadEn_S ? ((~OpBIsZero_SI | OpCode_SI[1]) & OpCode_SI[0]) & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI) : ResInv_SP);
	assign AReg_DN = (ARegEn_S ? AddOut_D : AReg_DP);
	assign BReg_DN = (BRegEn_S ? BMux_D : BReg_DP);
	assign ResReg_DN = (LoadEn_S ? {C_WIDTH {1'sb0}} : (ResRegEn_S ? {ABComp_S, ResReg_DP[C_WIDTH - 1:1]} : ResReg_DP));
	always @(posedge Clk_CI or negedge Rst_RBI) begin : p_regs
		if (~Rst_RBI) begin
			State_SP <= 2'd0;
			AReg_DP <= 1'sb0;
			BReg_DP <= 1'sb0;
			ResReg_DP <= 1'sb0;
			Cnt_DP <= 1'sb0;
			RemSel_SP <= 1'b0;
			CompInv_SP <= 1'b0;
			ResInv_SP <= 1'b0;
		end
		else begin
			State_SP <= State_SN;
			AReg_DP <= AReg_DN;
			BReg_DP <= BReg_DN;
			ResReg_DP <= ResReg_DN;
			Cnt_DP <= Cnt_DN;
			RemSel_SP <= RemSel_SN;
			CompInv_SP <= CompInv_SN;
			ResInv_SP <= ResInv_SN;
		end
	end
	initial begin : p_assertions
		
	end
	initial _sv2v_0 = 0;
endmodule
module riscv_apu_disp (
	clk_i,
	rst_ni,
	enable_i,
	apu_lat_i,
	apu_waddr_i,
	apu_waddr_o,
	apu_multicycle_o,
	apu_singlecycle_o,
	active_o,
	stall_o,
	read_regs_i,
	read_regs_valid_i,
	read_dep_o,
	write_regs_i,
	write_regs_valid_i,
	write_dep_o,
	perf_type_o,
	perf_cont_o,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_valid_i
);
	reg _sv2v_0;
	input wire clk_i;
	input wire rst_ni;
	input wire enable_i;
	input wire [1:0] apu_lat_i;
	input wire [5:0] apu_waddr_i;
	output reg [5:0] apu_waddr_o;
	output wire apu_multicycle_o;
	output wire apu_singlecycle_o;
	output wire active_o;
	output wire stall_o;
	input wire [17:0] read_regs_i;
	input wire [2:0] read_regs_valid_i;
	output wire read_dep_o;
	input wire [11:0] write_regs_i;
	input wire [1:0] write_regs_valid_i;
	output wire write_dep_o;
	output wire perf_type_o;
	output wire perf_cont_o;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	input wire apu_master_valid_i;
	wire [5:0] addr_req;
	reg [5:0] addr_inflight;
	reg [5:0] addr_waiting;
	reg [5:0] addr_inflight_dn;
	reg [5:0] addr_waiting_dn;
	wire valid_req;
	reg valid_inflight;
	reg valid_waiting;
	reg valid_inflight_dn;
	reg valid_waiting_dn;
	wire returned_req;
	wire returned_inflight;
	wire returned_waiting;
	wire req_accepted;
	wire active;
	reg [1:0] apu_lat;
	wire [2:0] read_deps_req;
	wire [2:0] read_deps_inflight;
	wire [2:0] read_deps_waiting;
	wire [1:0] write_deps_req;
	wire [1:0] write_deps_inflight;
	wire [1:0] write_deps_waiting;
	wire read_dep_req;
	wire read_dep_inflight;
	wire read_dep_waiting;
	wire write_dep_req;
	wire write_dep_inflight;
	wire write_dep_waiting;
	wire stall_full;
	wire stall_type;
	wire stall_nack;
	assign valid_req = enable_i & !(stall_full | stall_type);
	assign addr_req = apu_waddr_i;
	assign req_accepted = valid_req & apu_master_gnt_i;
	assign returned_req = ((valid_req & apu_master_valid_i) & !valid_inflight) & !valid_waiting;
	assign returned_inflight = (valid_inflight & apu_master_valid_i) & !valid_waiting;
	assign returned_waiting = valid_waiting & apu_master_valid_i;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			valid_inflight <= 1'b0;
			valid_waiting <= 1'b0;
			addr_inflight <= 1'sb0;
			addr_waiting <= 1'sb0;
		end
		else begin
			valid_inflight <= valid_inflight_dn;
			valid_waiting <= valid_waiting_dn;
			addr_inflight <= addr_inflight_dn;
			addr_waiting <= addr_waiting_dn;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		valid_inflight_dn = valid_inflight;
		valid_waiting_dn = valid_waiting;
		addr_inflight_dn = addr_inflight;
		addr_waiting_dn = addr_waiting;
		if (req_accepted & !returned_req) begin
			valid_inflight_dn = 1'b1;
			addr_inflight_dn = addr_req;
			if (valid_inflight & !returned_inflight) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
			if (returned_waiting) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
		end
		else if (returned_inflight) begin
			valid_inflight_dn = 1'sb0;
			valid_waiting_dn = 1'sb0;
			addr_inflight_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
		else if (returned_waiting) begin
			valid_waiting_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
	end
	assign active = valid_inflight | valid_waiting;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			apu_lat <= 1'sb0;
		else if (valid_req)
			apu_lat <= apu_lat_i;
	genvar _gv_i_15;
	generate
		for (_gv_i_15 = 0; _gv_i_15 < 3; _gv_i_15 = _gv_i_15 + 1) begin : genblk1
			localparam i = _gv_i_15;
			assign read_deps_req[i] = (read_regs_i[i * 6+:6] == addr_req) & read_regs_valid_i[i];
			assign read_deps_inflight[i] = (read_regs_i[i * 6+:6] == addr_inflight) & read_regs_valid_i[i];
			assign read_deps_waiting[i] = (read_regs_i[i * 6+:6] == addr_waiting) & read_regs_valid_i[i];
		end
	endgenerate
	genvar _gv_i_16;
	generate
		for (_gv_i_16 = 0; _gv_i_16 < 2; _gv_i_16 = _gv_i_16 + 1) begin : genblk2
			localparam i = _gv_i_16;
			assign write_deps_req[i] = (write_regs_i[i * 6+:6] == addr_req) & write_regs_valid_i[i];
			assign write_deps_inflight[i] = (write_regs_i[i * 6+:6] == addr_inflight) & write_regs_valid_i[i];
			assign write_deps_waiting[i] = (write_regs_i[i * 6+:6] == addr_waiting) & write_regs_valid_i[i];
		end
	endgenerate
	assign read_dep_req = (|read_deps_req & valid_req) & !returned_req;
	assign read_dep_inflight = (|read_deps_inflight & valid_inflight) & !returned_inflight;
	assign read_dep_waiting = (|read_deps_waiting & valid_waiting) & !returned_waiting;
	assign write_dep_req = (|write_deps_req & valid_req) & !returned_req;
	assign write_dep_inflight = (|write_deps_inflight & valid_inflight) & !returned_inflight;
	assign write_dep_waiting = (|write_deps_waiting & valid_waiting) & !returned_waiting;
	assign read_dep_o = (read_dep_req | read_dep_inflight) | read_dep_waiting;
	assign write_dep_o = (write_dep_req | write_dep_inflight) | write_dep_waiting;
	assign stall_full = valid_inflight & valid_waiting;
	assign stall_type = (enable_i & active) & (((apu_lat_i == 2'h1) | ((apu_lat_i == 2'h2) & (apu_lat == 2'h3))) | (apu_lat_i == 2'h3));
	assign stall_nack = valid_req & !apu_master_gnt_i;
	assign stall_o = (stall_full | stall_type) | stall_nack;
	assign apu_master_req_o = valid_req;
	assign apu_master_ready_o = 1'b1;
	always @(*) begin
		if (_sv2v_0)
			;
		apu_waddr_o = 1'sb0;
		if (returned_req)
			apu_waddr_o = addr_req;
		if (returned_inflight)
			apu_waddr_o = addr_inflight;
		if (returned_waiting)
			apu_waddr_o = addr_waiting;
	end
	assign active_o = active;
	assign perf_type_o = stall_type;
	assign perf_cont_o = stall_nack;
	assign apu_multicycle_o = apu_lat == 2'h3;
	assign apu_singlecycle_o = ~(valid_inflight | valid_waiting);
	initial _sv2v_0 = 0;
endmodule
module riscv_compressed_decoder (
	instr_i,
	instr_o,
	is_compressed_o,
	illegal_instr_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	input wire [31:0] instr_i;
	output reg [31:0] instr_o;
	output wire is_compressed_o;
	output reg illegal_instr_o;
	localparam riscv_defines_OPCODE_BRANCH = 7'h63;
	localparam riscv_defines_OPCODE_JAL = 7'h6f;
	localparam riscv_defines_OPCODE_JALR = 7'h67;
	localparam riscv_defines_OPCODE_LOAD = 7'h03;
	localparam riscv_defines_OPCODE_LOAD_FP = 7'h07;
	localparam riscv_defines_OPCODE_LUI = 7'h37;
	localparam riscv_defines_OPCODE_OP = 7'h33;
	localparam riscv_defines_OPCODE_OPIMM = 7'h13;
	localparam riscv_defines_OPCODE_STORE = 7'h23;
	localparam riscv_defines_OPCODE_STORE_FP = 7'h27;
	always @(*) begin
		if (_sv2v_0)
			;
		illegal_instr_o = 1'b0;
		instr_o = 1'sb0;
		(* full_case, parallel_case *)
		case (instr_i[1:0])
			2'b00:
				(* full_case, parallel_case *)
				case (instr_i[15:13])
					3'b000: begin
						instr_o = {2'b00, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 12'h041, instr_i[4:2], riscv_defines_OPCODE_OPIMM};
						if (instr_i[12:5] == 8'b00000000)
							illegal_instr_o = 1'b1;
					end
					3'b001:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[6:5], instr_i[12:10], 5'b00001, instr_i[9:7], 5'b01101, instr_i[4:2], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], riscv_defines_OPCODE_LOAD};
					3'b011:
						if (FPU == 1)
							instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b101:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[6:5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b011, instr_i[11:10], 3'b000, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, riscv_defines_OPCODE_STORE};
					3'b111:
						if (FPU == 1)
							instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					default: illegal_instr_o = 1'b1;
				endcase
			2'b01:
				(* full_case, parallel_case *)
				case (instr_i[15:13])
					3'b000: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
					3'b001, 3'b101: instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0000, ~instr_i[15], riscv_defines_OPCODE_JAL};
					3'b010: begin
						instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 8'b00000000, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
					end
					3'b011: begin
						instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], riscv_defines_OPCODE_LUI};
						if (instr_i[11:7] == 5'h02)
							instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 17'h00202, riscv_defines_OPCODE_OPIMM};
						else if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
						if ({instr_i[12], instr_i[6:2]} == 6'b000000)
							illegal_instr_o = 1'b1;
					end
					3'b100:
						(* full_case, parallel_case *)
						case (instr_i[11:10])
							2'b00, 2'b01: begin
								instr_o = {1'b0, instr_i[10], 5'b00000, instr_i[6:2], 2'b01, instr_i[9:7], 5'b10101, instr_i[9:7], riscv_defines_OPCODE_OPIMM};
								if (instr_i[12] == 1'b1)
									illegal_instr_o = 1'b1;
								if (instr_i[6:2] == 5'b00000)
									illegal_instr_o = 1'b1;
							end
							2'b10: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], riscv_defines_OPCODE_OPIMM};
							2'b11:
								(* full_case, parallel_case *)
								case ({instr_i[12], instr_i[6:5]})
									3'b000: instr_o = {9'b010000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b00001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b001: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b10001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b010: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b011: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b100, 3'b101, 3'b110, 3'b111: illegal_instr_o = 1'b1;
								endcase
						endcase
					3'b110, 3'b111: instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 7'b0000001, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], riscv_defines_OPCODE_BRANCH};
				endcase
			2'b10:
				(* full_case, parallel_case *)
				case (instr_i[15:13])
					3'b000: begin
						instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
						if ((instr_i[12] == 1'b1) || (instr_i[6:2] == 5'b00000))
							illegal_instr_o = 1'b1;
					end
					3'b001:
						if (FPU == 1)
							instr_o = {3'b000, instr_i[4:2], instr_i[12], instr_i[6:5], 11'h013, instr_i[11:7], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: begin
						instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], riscv_defines_OPCODE_LOAD};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
					end
					3'b011:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b100:
						if (instr_i[12] == 1'b0) begin
							instr_o = {7'b0000000, instr_i[6:2], 8'b00000000, instr_i[11:7], riscv_defines_OPCODE_OP};
							if (instr_i[6:2] == 5'b00000)
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000000, riscv_defines_OPCODE_JALR};
						end
						else begin
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], riscv_defines_OPCODE_OP};
							if (instr_i[11:7] == 5'b00000) begin
								instr_o = 32'h00100073;
								if (instr_i[6:2] != 5'b00000)
									illegal_instr_o = 1'b1;
							end
							else if (instr_i[6:2] == 5'b00000)
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000001, riscv_defines_OPCODE_JALR};
						end
					3'b101:
						if (FPU == 1)
							instr_o = {3'b000, instr_i[9:7], instr_i[12], instr_i[6:2], 8'h13, instr_i[11:10], 3'b000, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, riscv_defines_OPCODE_STORE};
					3'b111:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
				endcase
			default: instr_o = instr_i;
		endcase
	end
	assign is_compressed_o = instr_i[1:0] != 2'b11;
	initial _sv2v_0 = 0;
endmodule
module riscv_controller (
	clk,
	rst_n,
	fetch_enable_i,
	ctrl_busy_o,
	first_fetch_o,
	is_decoding_o,
	deassert_we_o,
	illegal_insn_i,
	ecall_insn_i,
	mret_insn_i,
	uret_insn_i,
	pipe_flush_i,
	ebrk_insn_i,
	csr_status_i,
	instr_valid_i,
	instr_req_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	data_req_ex_i,
	data_misaligned_i,
	data_load_event_i,
	mult_multicycle_i,
	apu_en_i,
	apu_read_dep_i,
	apu_write_dep_i,
	apu_stall_o,
	branch_taken_ex_i,
	jump_in_id_i,
	jump_in_dec_i,
	irq_i,
	irq_req_ctrl_i,
	irq_sec_ctrl_i,
	irq_id_ctrl_i,
	m_IE_i,
	u_IE_i,
	current_priv_lvl_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	exc_ack_o,
	exc_kill_o,
	csr_save_if_o,
	csr_save_id_o,
	csr_cause_o,
	csr_irq_sec_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_save_cause_o,
	dbg_req_i,
	dbg_ack_o,
	dbg_stall_i,
	dbg_jump_req_i,
	dbg_settings_i,
	dbg_trap_o,
	regfile_alu_waddr_id_i,
	regfile_we_ex_i,
	regfile_waddr_ex_i,
	regfile_we_wb_i,
	regfile_alu_we_fw_i,
	operand_a_fw_mux_sel_o,
	operand_b_fw_mux_sel_o,
	operand_c_fw_mux_sel_o,
	reg_d_ex_is_reg_a_i,
	reg_d_ex_is_reg_b_i,
	reg_d_ex_is_reg_c_i,
	reg_d_wb_is_reg_a_i,
	reg_d_wb_is_reg_b_i,
	reg_d_wb_is_reg_c_i,
	reg_d_alu_is_reg_a_i,
	reg_d_alu_is_reg_b_i,
	reg_d_alu_is_reg_c_i,
	halt_if_o,
	halt_id_o,
	misaligned_stall_o,
	jr_stall_o,
	load_stall_o,
	id_ready_i,
	ex_valid_i,
	wb_ready_i,
	perf_jump_o,
	perf_jr_stall_o,
	perf_ld_stall_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	input wire fetch_enable_i;
	output reg ctrl_busy_o;
	output reg first_fetch_o;
	output reg is_decoding_o;
	output reg deassert_we_o;
	input wire illegal_insn_i;
	input wire ecall_insn_i;
	input wire mret_insn_i;
	input wire uret_insn_i;
	input wire pipe_flush_i;
	input wire ebrk_insn_i;
	input wire csr_status_i;
	input wire instr_valid_i;
	output reg instr_req_o;
	output reg pc_set_o;
	output reg [2:0] pc_mux_o;
	output reg [1:0] exc_pc_mux_o;
	output reg trap_addr_mux_o;
	input wire data_req_ex_i;
	input wire data_misaligned_i;
	input wire data_load_event_i;
	input wire mult_multicycle_i;
	input wire apu_en_i;
	input wire apu_read_dep_i;
	input wire apu_write_dep_i;
	output wire apu_stall_o;
	input wire branch_taken_ex_i;
	input wire [1:0] jump_in_id_i;
	input wire [1:0] jump_in_dec_i;
	input wire irq_i;
	input wire irq_req_ctrl_i;
	input wire irq_sec_ctrl_i;
	input wire [4:0] irq_id_ctrl_i;
	input wire m_IE_i;
	input wire u_IE_i;
	input wire [1:0] current_priv_lvl_i;
	output reg irq_ack_o;
	output reg [4:0] irq_id_o;
	output reg [5:0] exc_cause_o;
	output reg exc_ack_o;
	output reg exc_kill_o;
	output reg csr_save_if_o;
	output reg csr_save_id_o;
	output reg [5:0] csr_cause_o;
	output reg csr_irq_sec_o;
	output reg csr_restore_mret_id_o;
	output reg csr_restore_uret_id_o;
	output reg csr_save_cause_o;
	input wire dbg_req_i;
	output reg dbg_ack_o;
	input wire dbg_stall_i;
	input wire dbg_jump_req_i;
	localparam riscv_defines_DBG_SETS_W = 6;
	input wire [5:0] dbg_settings_i;
	output reg dbg_trap_o;
	input wire [5:0] regfile_alu_waddr_id_i;
	input wire regfile_we_ex_i;
	input wire [5:0] regfile_waddr_ex_i;
	input wire regfile_we_wb_i;
	input wire regfile_alu_we_fw_i;
	output reg [1:0] operand_a_fw_mux_sel_o;
	output reg [1:0] operand_b_fw_mux_sel_o;
	output reg [1:0] operand_c_fw_mux_sel_o;
	input wire reg_d_ex_is_reg_a_i;
	input wire reg_d_ex_is_reg_b_i;
	input wire reg_d_ex_is_reg_c_i;
	input wire reg_d_wb_is_reg_a_i;
	input wire reg_d_wb_is_reg_b_i;
	input wire reg_d_wb_is_reg_c_i;
	input wire reg_d_alu_is_reg_a_i;
	input wire reg_d_alu_is_reg_b_i;
	input wire reg_d_alu_is_reg_c_i;
	output reg halt_if_o;
	output reg halt_id_o;
	output wire misaligned_stall_o;
	output reg jr_stall_o;
	output reg load_stall_o;
	input wire id_ready_i;
	input wire ex_valid_i;
	input wire wb_ready_i;
	output wire perf_jump_o;
	output wire perf_jr_stall_o;
	output wire perf_ld_stall_o;
	reg [4:0] ctrl_fsm_cs;
	reg [4:0] ctrl_fsm_ns;
	reg jump_done;
	reg jump_done_q;
	reg jump_in_dec;
	reg branch_in_id;
	reg boot_done;
	reg boot_done_q;
	reg irq_enable_int;
	always @(negedge clk)
		if (is_decoding_o && illegal_insn_i)
			$display("%t: Illegal instruction (core %0d) at PC 0x%h:", $time, riscv_core.core_id_i, riscv_id_stage.pc_id_i);
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_BRANCH_JAL = 2'b01;
	localparam riscv_defines_BRANCH_JALR = 2'b10;
	localparam riscv_defines_DBG_SETS_EBRK = 1;
	localparam riscv_defines_DBG_SETS_ECALL = 4;
	localparam riscv_defines_DBG_SETS_EILL = 3;
	localparam riscv_defines_DBG_SETS_SSTE = 0;
	localparam riscv_defines_EXC_CAUSE_BREAKPOINT = 6'h03;
	localparam riscv_defines_EXC_CAUSE_ECALL_MMODE = 6'h0b;
	localparam riscv_defines_EXC_CAUSE_ECALL_UMODE = 6'h08;
	localparam riscv_defines_EXC_CAUSE_ILLEGAL_INSN = 6'h02;
	localparam riscv_defines_EXC_PC_ECALL = 2'b01;
	localparam riscv_defines_EXC_PC_ILLINSN = 2'b00;
	localparam riscv_defines_EXC_PC_IRQ = 2'b11;
	localparam riscv_defines_PC_BOOT = 3'b000;
	localparam riscv_defines_PC_BRANCH = 3'b011;
	localparam riscv_defines_PC_DBG_NPC = 3'b111;
	localparam riscv_defines_PC_ERET = 3'b101;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	localparam riscv_defines_PC_JUMP = 3'b010;
	localparam riscv_defines_TRAP_MACHINE = 1'b0;
	localparam riscv_defines_TRAP_USER = 1'b1;
	always @(*) begin
		if (_sv2v_0)
			;
		instr_req_o = 1'b1;
		exc_ack_o = 1'b0;
		exc_kill_o = 1'b0;
		csr_save_if_o = 1'b0;
		csr_save_id_o = 1'b0;
		csr_restore_mret_id_o = 1'b0;
		csr_restore_uret_id_o = 1'b0;
		csr_save_cause_o = 1'b0;
		exc_cause_o = 1'sb0;
		exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
		trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
		csr_cause_o = 1'sb0;
		csr_irq_sec_o = 1'b0;
		pc_mux_o = riscv_defines_PC_BOOT;
		pc_set_o = 1'b0;
		jump_done = jump_done_q;
		ctrl_fsm_ns = ctrl_fsm_cs;
		ctrl_busy_o = 1'b1;
		first_fetch_o = 1'b0;
		halt_if_o = 1'b0;
		halt_id_o = 1'b0;
		dbg_ack_o = 1'b0;
		irq_ack_o = 1'b0;
		irq_id_o = irq_id_ctrl_i;
		boot_done = 1'b0;
		jump_in_dec = (jump_in_dec_i == riscv_defines_BRANCH_JALR) || (jump_in_dec_i == riscv_defines_BRANCH_JAL);
		branch_in_id = jump_in_id_i == riscv_defines_BRANCH_COND;
		irq_enable_int = ((u_IE_i | irq_sec_ctrl_i) & (current_priv_lvl_i == 2'b00)) | (m_IE_i & (current_priv_lvl_i == 2'b11));
		dbg_trap_o = 1'b0;
		(* full_case, parallel_case *)
		case (ctrl_fsm_cs)
			5'd0: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				if (fetch_enable_i == 1'b1)
					ctrl_fsm_ns = 5'd1;
				else if (dbg_req_i)
					ctrl_fsm_ns = 5'd12;
			end
			5'd1: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b1;
				pc_mux_o = riscv_defines_PC_BOOT;
				pc_set_o = 1'b1;
				boot_done = 1'b1;
				ctrl_fsm_ns = 5'd4;
			end
			5'd3: begin
				is_decoding_o = 1'b0;
				ctrl_busy_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				ctrl_fsm_ns = 5'd2;
			end
			5'd2: begin
				is_decoding_o = 1'b0;
				ctrl_busy_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
				if (dbg_req_i)
					ctrl_fsm_ns = 5'd13;
				else if (irq_i)
					ctrl_fsm_ns = 5'd4;
			end
			5'd4: begin
				is_decoding_o = 1'b0;
				first_fetch_o = 1'b1;
				if ((id_ready_i == 1'b1) && (dbg_stall_i == 1'b0))
					ctrl_fsm_ns = 5'd5;
				if (irq_req_ctrl_i & irq_enable_int) begin
					ctrl_fsm_ns = 5'd7;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
				end
			end
			5'd5:
				if (branch_taken_ex_i) begin
					is_decoding_o = 1'b0;
					pc_mux_o = riscv_defines_PC_BRANCH;
					pc_set_o = 1'b1;
					dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					if (dbg_req_i)
						ctrl_fsm_ns = 5'd12;
				end
				else if (instr_valid_i) begin
					is_decoding_o = 1'b1;
					(* full_case, parallel_case *)
					case (1'b1)
						irq_req_ctrl_i & irq_enable_int: begin
							halt_if_o = 1'b1;
							halt_id_o = 1'b1;
							ctrl_fsm_ns = 5'd8;
						end
						default: begin
							exc_kill_o = (irq_req_ctrl_i ? 1'b1 : 1'b0);
							(* full_case, parallel_case *)
							case (1'b1)
								jump_in_dec: begin
									pc_mux_o = riscv_defines_PC_JUMP;
									if (~jr_stall_o && ~jump_done_q) begin
										pc_set_o = 1'b1;
										jump_done = 1'b1;
									end
									dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
								end
								((((mret_insn_i | uret_insn_i) | ecall_insn_i) | pipe_flush_i) | ebrk_insn_i) | illegal_insn_i: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b1;
									ctrl_fsm_ns = 5'd10;
								end
								csr_status_i: begin
									halt_if_o = 1'b1;
									ctrl_fsm_ns = (id_ready_i ? 5'd10 : 5'd5);
								end
								data_load_event_i: begin
									ctrl_fsm_ns = (id_ready_i ? 5'd9 : 5'd5);
									halt_if_o = 1'b1;
									dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
								end
								default: dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
							endcase
							if (dbg_req_i) begin
								halt_if_o = 1'b1;
								if (id_ready_i)
									(* full_case, parallel_case *)
									case (1'b1)
										branch_in_id: ctrl_fsm_ns = 5'd16;
										default: ctrl_fsm_ns = 5'd12;
									endcase
							end
						end
					endcase
				end
				else
					is_decoding_o = 1'b0;
			5'd16: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				if (branch_taken_ex_i) begin
					pc_mux_o = riscv_defines_PC_BRANCH;
					pc_set_o = 1'b1;
				end
				ctrl_fsm_ns = 5'd12;
			end
			5'd12: begin
				is_decoding_o = 1'b0;
				dbg_ack_o = 1'b1;
				halt_if_o = 1'b1;
				ctrl_fsm_ns = 5'd15;
			end
			5'd13: begin
				is_decoding_o = 1'b0;
				dbg_ack_o = 1'b1;
				halt_if_o = 1'b1;
				ctrl_fsm_ns = 5'd15;
			end
			5'd14: begin
				is_decoding_o = 1'b0;
				dbg_ack_o = 1'b1;
				halt_if_o = 1'b1;
				ctrl_fsm_ns = 5'd17;
			end
			5'd17: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				if (dbg_jump_req_i) begin
					pc_mux_o = riscv_defines_PC_DBG_NPC;
					pc_set_o = 1'b1;
					ctrl_fsm_ns = 5'd15;
				end
				if (dbg_stall_i == 1'b0)
					ctrl_fsm_ns = 5'd9;
			end
			5'd15: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				if (dbg_jump_req_i) begin
					pc_mux_o = riscv_defines_PC_DBG_NPC;
					pc_set_o = 1'b1;
					ctrl_fsm_ns = 5'd15;
				end
				if (dbg_stall_i == 1'b0)
					ctrl_fsm_ns = (boot_done_q ? 5'd5 : 5'd0);
			end
			5'd10: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (ex_valid_i)
					ctrl_fsm_ns = 5'd11;
			end
			5'd8: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (irq_req_ctrl_i & irq_enable_int)
					ctrl_fsm_ns = 5'd6;
				else
					ctrl_fsm_ns = 5'd5;
			end
			5'd9: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (id_ready_i)
					ctrl_fsm_ns = 5'd8;
				else if (dbg_req_i)
					ctrl_fsm_ns = 5'd14;
				else
					ctrl_fsm_ns = 5'd9;
			end
			5'd6: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
				exc_cause_o = {1'b0, irq_id_ctrl_i};
				csr_irq_sec_o = irq_sec_ctrl_i;
				csr_save_cause_o = 1'b1;
				csr_cause_o = {1'b1, irq_id_ctrl_i};
				csr_save_id_o = 1'b1;
				if (irq_sec_ctrl_i)
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
				else
					trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINE);
				irq_ack_o = 1'b1;
				exc_ack_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
			end
			5'd7: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
				exc_cause_o = {1'b0, irq_id_ctrl_i};
				csr_irq_sec_o = irq_sec_ctrl_i;
				csr_save_cause_o = 1'b1;
				csr_cause_o = {1'b1, irq_id_ctrl_i};
				csr_save_if_o = 1'b1;
				if (irq_sec_ctrl_i)
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
				else
					trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINE);
				irq_ack_o = 1'b1;
				exc_ack_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
			end
			5'd11: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				(* full_case, parallel_case *)
				case (1'b1)
					ecall_insn_i: begin
						pc_mux_o = riscv_defines_PC_EXCEPTION;
						pc_set_o = 1'b1;
						csr_save_id_o = 1'b1;
						csr_save_cause_o = 1'b1;
						trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
						exc_pc_mux_o = riscv_defines_EXC_PC_ECALL;
						exc_cause_o = riscv_defines_EXC_CAUSE_ECALL_MMODE;
						csr_cause_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_EXC_CAUSE_ECALL_UMODE : riscv_defines_EXC_CAUSE_ECALL_MMODE);
						dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_ECALL] | dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					end
					illegal_insn_i: begin
						pc_mux_o = riscv_defines_PC_EXCEPTION;
						pc_set_o = 1'b1;
						csr_save_id_o = 1'b1;
						csr_save_cause_o = 1'b1;
						trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
						exc_pc_mux_o = riscv_defines_EXC_PC_ILLINSN;
						exc_cause_o = riscv_defines_EXC_CAUSE_ILLEGAL_INSN;
						csr_cause_o = riscv_defines_EXC_CAUSE_ILLEGAL_INSN;
						dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_EILL] | dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					end
					mret_insn_i: begin
						pc_mux_o = riscv_defines_PC_ERET;
						pc_set_o = 1'b1;
						csr_restore_mret_id_o = 1'b1;
						dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					end
					uret_insn_i: begin
						pc_mux_o = riscv_defines_PC_ERET;
						pc_set_o = 1'b1;
						csr_restore_uret_id_o = 1'b1;
						dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					end
					ebrk_insn_i: begin
						dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_EBRK] | dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
						exc_cause_o = riscv_defines_EXC_CAUSE_BREAKPOINT;
					end
					csr_status_i: dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					pipe_flush_i: dbg_trap_o = dbg_settings_i[riscv_defines_DBG_SETS_SSTE];
					default:
						;
				endcase
				if (~pipe_flush_i) begin
					if (dbg_req_i)
						ctrl_fsm_ns = 5'd12;
					else
						ctrl_fsm_ns = 5'd5;
				end
				else if (dbg_req_i)
					ctrl_fsm_ns = 5'd13;
				else
					ctrl_fsm_ns = 5'd3;
			end
			default: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				ctrl_fsm_ns = 5'd0;
			end
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		load_stall_o = 1'b0;
		jr_stall_o = 1'b0;
		deassert_we_o = 1'b0;
		if (~is_decoding_o)
			deassert_we_o = 1'b1;
		if (illegal_insn_i)
			deassert_we_o = 1'b1;
		if ((((data_req_ex_i == 1'b1) && (regfile_we_ex_i == 1'b1)) || ((wb_ready_i == 1'b0) && (regfile_we_wb_i == 1'b1))) && ((((reg_d_ex_is_reg_a_i == 1'b1) || (reg_d_ex_is_reg_b_i == 1'b1)) || (reg_d_ex_is_reg_c_i == 1'b1)) || (regfile_waddr_ex_i == regfile_alu_waddr_id_i))) begin
			deassert_we_o = 1'b1;
			load_stall_o = 1'b1;
		end
		if ((jump_in_dec_i == riscv_defines_BRANCH_JALR) && ((((regfile_we_wb_i == 1'b1) && (reg_d_wb_is_reg_a_i == 1'b1)) || ((regfile_we_ex_i == 1'b1) && (reg_d_ex_is_reg_a_i == 1'b1))) || ((regfile_alu_we_fw_i == 1'b1) && (reg_d_alu_is_reg_a_i == 1'b1)))) begin
			jr_stall_o = 1'b1;
			deassert_we_o = 1'b1;
		end
	end
	assign misaligned_stall_o = data_misaligned_i;
	assign apu_stall_o = apu_read_dep_i | (apu_write_dep_i & ~apu_en_i);
	localparam riscv_defines_SEL_FW_EX = 2'b01;
	localparam riscv_defines_SEL_FW_WB = 2'b10;
	localparam riscv_defines_SEL_REGFILE = 2'b00;
	always @(*) begin
		if (_sv2v_0)
			;
		operand_a_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		operand_b_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		operand_c_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		if (regfile_we_wb_i == 1'b1) begin
			if (reg_d_wb_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
			if (reg_d_wb_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
			if (reg_d_wb_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
		end
		if (regfile_alu_we_fw_i == 1'b1) begin
			if (reg_d_alu_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			if (reg_d_alu_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			if (reg_d_alu_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
		end
		if (data_misaligned_i) begin
			operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			operand_b_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		end
		else if (mult_multicycle_i)
			operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
	end
	always @(posedge clk or negedge rst_n) begin : UPDATE_REGS
		if (rst_n == 1'b0) begin
			ctrl_fsm_cs <= 5'd0;
			jump_done_q <= 1'b0;
			boot_done_q <= 1'b0;
		end
		else begin
			ctrl_fsm_cs <= ctrl_fsm_ns;
			boot_done_q <= boot_done | (~boot_done & boot_done_q);
			jump_done_q <= jump_done & ~id_ready_i;
		end
	end
	assign perf_jump_o = (jump_in_id_i == riscv_defines_BRANCH_JAL) || (jump_in_id_i == riscv_defines_BRANCH_JALR);
	assign perf_jr_stall_o = jr_stall_o;
	assign perf_ld_stall_o = load_stall_o;
	initial _sv2v_0 = 0;
endmodule
module riscv_core (
	clk_i,
	rst_ni,
	clock_en_i,
	test_en_i,
	fregfile_disable_i,
	boot_addr_i,
	core_id_i,
	cluster_id_i,
	instr_req_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_addr_o,
	instr_rdata_i,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_we_o,
	data_be_o,
	data_addr_o,
	data_wdata_o,
	data_rdata_i,
	data_err_i,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_operands_o,
	apu_master_op_o,
	apu_master_type_o,
	apu_master_flags_o,
	apu_master_valid_i,
	apu_master_result_i,
	apu_master_flags_i,
	irq_i,
	irq_id_i,
	irq_ack_o,
	irq_id_o,
	irq_sec_i,
	sec_lvl_o,
	debug_req_i,
	debug_gnt_o,
	debug_rvalid_o,
	debug_addr_i,
	debug_we_i,
	debug_wdata_i,
	debug_rdata_o,
	debug_halted_o,
	debug_halt_i,
	debug_resume_i,
	fetch_enable_i,
	core_busy_o,
	ext_perf_counters_i
);
	parameter N_EXT_PERF_COUNTERS = 0;
	parameter INSTR_RDATA_WIDTH = 32;
	parameter PULP_SECURE = 0;
	parameter PULP_CLUSTER = 1;
	parameter FPU = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk_i;
	input wire rst_ni;
	input wire clock_en_i;
	input wire test_en_i;
	input wire fregfile_disable_i;
	input wire [31:0] boot_addr_i;
	input wire [3:0] core_id_i;
	input wire [5:0] cluster_id_i;
	output wire instr_req_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	output wire [31:0] instr_addr_o;
	input wire [INSTR_RDATA_WIDTH - 1:0] instr_rdata_i;
	output wire data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_addr_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	input wire data_err_i;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	output wire [(APU_NARGS_CPU * 32) - 1:0] apu_master_operands_o;
	output wire [APU_WOP_CPU - 1:0] apu_master_op_o;
	output wire [WAPUTYPE - 1:0] apu_master_type_o;
	output wire [APU_NDSFLAGS_CPU - 1:0] apu_master_flags_o;
	input wire apu_master_valid_i;
	input wire [31:0] apu_master_result_i;
	input wire [APU_NUSFLAGS_CPU - 1:0] apu_master_flags_i;
	input wire irq_i;
	input wire [4:0] irq_id_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	input wire irq_sec_i;
	output wire sec_lvl_o;
	input wire debug_req_i;
	output wire debug_gnt_o;
	output wire debug_rvalid_o;
	input wire [14:0] debug_addr_i;
	input wire debug_we_i;
	input wire [31:0] debug_wdata_i;
	output wire [31:0] debug_rdata_o;
	output wire debug_halted_o;
	input wire debug_halt_i;
	input wire debug_resume_i;
	input wire fetch_enable_i;
	output wire core_busy_o;
	input wire [N_EXT_PERF_COUNTERS - 1:0] ext_perf_counters_i;
	localparam N_HWLP = 2;
	localparam N_HWLP_BITS = 1;
	localparam APU = (((SHARED_DSP_MULT == 1) | (SHARED_INT_DIV == 1)) | (FPU == 1) ? 1 : 0);
	wire is_hwlp_id;
	wire [1:0] hwlp_dec_cnt_id;
	wire instr_valid_id;
	wire [31:0] instr_rdata_id;
	wire is_compressed_id;
	wire illegal_c_insn_id;
	wire [31:0] pc_if;
	wire [31:0] pc_id;
	wire clear_instr_valid;
	wire pc_set;
	wire [2:0] pc_mux_id;
	wire [1:0] exc_pc_mux_id;
	wire [5:0] exc_cause;
	wire trap_addr_mux;
	wire lsu_load_err;
	wire lsu_store_err;
	wire is_decoding;
	wire useincr_addr_ex;
	wire data_misaligned;
	wire mult_multicycle;
	wire [31:0] jump_target_id;
	wire [31:0] jump_target_ex;
	wire branch_in_ex;
	wire branch_decision;
	wire ctrl_busy;
	wire if_busy;
	wire lsu_busy;
	wire apu_busy;
	wire [31:0] pc_ex;
	wire alu_en_ex;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	wire [6:0] alu_operator_ex;
	wire [31:0] alu_operand_a_ex;
	wire [31:0] alu_operand_b_ex;
	wire [31:0] alu_operand_c_ex;
	wire [4:0] bmask_a_ex;
	wire [4:0] bmask_b_ex;
	wire [1:0] imm_vec_ext_ex;
	wire [1:0] alu_vec_mode_ex;
	wire [2:0] mult_operator_ex;
	wire [31:0] mult_operand_a_ex;
	wire [31:0] mult_operand_b_ex;
	wire [31:0] mult_operand_c_ex;
	wire mult_en_ex;
	wire mult_sel_subword_ex;
	wire [1:0] mult_signed_mode_ex;
	wire [4:0] mult_imm_ex;
	wire [31:0] mult_dot_op_a_ex;
	wire [31:0] mult_dot_op_b_ex;
	wire [31:0] mult_dot_op_c_ex;
	wire [1:0] mult_dot_signed_ex;
	localparam riscv_defines_C_CMD = 4;
	wire [3:0] fpu_op_ex;
	localparam riscv_defines_C_PC = 5;
	wire [4:0] fprec_csr;
	localparam riscv_defines_C_RM = 3;
	wire [2:0] frm_csr;
	localparam riscv_defines_C_FFLAG = 5;
	wire [4:0] fflags;
	wire [4:0] fflags_csr;
	wire fflags_we;
	wire apu_en_ex;
	wire [WAPUTYPE - 1:0] apu_type_ex;
	wire [APU_NDSFLAGS_CPU - 1:0] apu_flags_ex;
	wire [APU_WOP_CPU - 1:0] apu_op_ex;
	wire [1:0] apu_lat_ex;
	wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_ex;
	wire [5:0] apu_waddr_ex;
	wire [17:0] apu_read_regs;
	wire [2:0] apu_read_regs_valid;
	wire apu_read_dep;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire apu_write_dep;
	wire perf_apu_type;
	wire perf_apu_cont;
	wire perf_apu_dep;
	wire perf_apu_wb;
	wire [5:0] regfile_waddr_ex;
	wire regfile_we_ex;
	wire [5:0] regfile_waddr_fw_wb_o;
	wire regfile_we_wb;
	wire [31:0] regfile_wdata;
	wire [5:0] regfile_alu_waddr_ex;
	wire regfile_alu_we_ex;
	wire [5:0] regfile_alu_waddr_fw;
	wire regfile_alu_we_fw;
	wire [31:0] regfile_alu_wdata_fw;
	wire csr_access_ex;
	wire [1:0] csr_op_ex;
	wire [23:0] mtvec;
	wire [23:0] utvec;
	wire csr_access;
	wire [1:0] csr_op;
	wire [11:0] csr_addr;
	wire [11:0] csr_addr_int;
	wire [31:0] csr_rdata;
	wire [31:0] csr_wdata;
	wire [1:0] current_priv_lvl;
	wire data_we_ex;
	wire [1:0] data_type_ex;
	wire data_sign_ext_ex;
	wire [1:0] data_reg_offset_ex;
	wire data_req_ex;
	wire data_load_event_ex;
	wire data_misaligned_ex;
	wire [31:0] lsu_rdata;
	wire halt_if;
	wire id_ready;
	wire ex_ready;
	wire id_valid;
	wire ex_valid;
	wire wb_valid;
	wire lsu_ready_ex;
	wire lsu_ready_wb;
	wire apu_ready_wb;
	wire instr_req_int;
	wire m_irq_enable;
	wire u_irq_enable;
	wire csr_irq_sec;
	wire [31:0] epc;
	wire csr_save_cause;
	wire csr_save_if;
	wire csr_save_id;
	wire [5:0] csr_cause;
	wire csr_restore_mret_id;
	wire csr_restore_uret_id;
	wire [63:0] hwlp_start;
	wire [63:0] hwlp_end;
	wire [63:0] hwlp_cnt;
	wire [0:0] csr_hwlp_regid;
	wire [2:0] csr_hwlp_we;
	wire [31:0] csr_hwlp_data;
	localparam riscv_defines_DBG_SETS_W = 6;
	wire [5:0] dbg_settings;
	wire dbg_req;
	wire dbg_ack;
	wire dbg_stall;
	wire dbg_trap;
	wire dbg_reg_rreq;
	wire [5:0] dbg_reg_raddr;
	wire [31:0] dbg_reg_rdata;
	wire dbg_reg_wreq;
	wire [5:0] dbg_reg_waddr;
	wire [31:0] dbg_reg_wdata;
	wire dbg_csr_req;
	wire [11:0] dbg_csr_addr;
	wire dbg_csr_we;
	wire [31:0] dbg_csr_wdata;
	wire [31:0] dbg_jump_addr;
	wire dbg_jump_req;
	wire perf_imiss;
	wire perf_jump;
	wire perf_jr_stall;
	wire perf_ld_stall;
	wire core_ctrl_firstfetch;
	wire core_busy_int;
	reg core_busy_q;
	wire is_interrupt;
	localparam riscv_defines_EXC_PC_IRQ = 2'b11;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	assign is_interrupt = (pc_mux_id == riscv_defines_PC_EXCEPTION) && (exc_pc_mux_id == riscv_defines_EXC_PC_IRQ);
	generate
		if (SHARED_FP == 1) begin : genblk1
			assign apu_master_type_o = apu_type_ex;
			assign apu_master_flags_o = apu_flags_ex;
			assign fflags_csr = apu_master_flags_i;
		end
		else begin : genblk1
			assign apu_master_type_o = 1'sb0;
			assign apu_master_flags_o = 1'sb0;
			assign fflags_csr = fflags;
		end
	endgenerate
	wire clk;
	wire clock_en;
	wire dbg_busy;
	wire sleeping;
	assign dbg_busy = (((dbg_req | dbg_csr_req) | dbg_jump_req) | dbg_reg_wreq) | debug_req_i;
	assign core_busy_o = (core_ctrl_firstfetch ? 1'b1 : core_busy_q);
	assign core_busy_int = ((PULP_CLUSTER & data_load_event_ex) & data_req_o ? if_busy | apu_busy : ((if_busy | ctrl_busy) | lsu_busy) | apu_busy);
	assign clock_en = (PULP_CLUSTER ? (clock_en_i | core_busy_o) | dbg_busy : (irq_i | core_busy_o) | dbg_busy);
	assign sleeping = ~core_busy_o;
	always @(posedge clk_i or negedge rst_ni)
		if (rst_ni == 1'b0)
			core_busy_q <= 1'b0;
		else
			core_busy_q <= core_busy_int;
	cluster_clock_gating core_clock_gate_i(
		.clk_i(clk_i),
		.en_i(clock_en),
		.test_en_i(test_en_i),
		.clk_o(clk)
	);
	riscv_if_stage #(
		.N_HWLP(N_HWLP),
		.RDATA_WIDTH(INSTR_RDATA_WIDTH),
		.FPU(FPU)
	) if_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.boot_addr_i(boot_addr_i[31:8]),
		.m_trap_base_addr_i(mtvec),
		.u_trap_base_addr_i(utvec),
		.trap_addr_mux_i(trap_addr_mux),
		.req_i(instr_req_int),
		.instr_req_o(instr_req_o),
		.instr_addr_o(instr_addr_o),
		.instr_gnt_i(instr_gnt_i),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_rdata_i(instr_rdata_i),
		.hwlp_dec_cnt_id_o(hwlp_dec_cnt_id),
		.is_hwlp_id_o(is_hwlp_id),
		.instr_valid_id_o(instr_valid_id),
		.instr_rdata_id_o(instr_rdata_id),
		.is_compressed_id_o(is_compressed_id),
		.illegal_c_insn_id_o(illegal_c_insn_id),
		.pc_if_o(pc_if),
		.pc_id_o(pc_id),
		.clear_instr_valid_i(clear_instr_valid),
		.pc_set_i(pc_set),
		.exception_pc_reg_i(epc),
		.pc_mux_i(pc_mux_id),
		.exc_pc_mux_i(exc_pc_mux_id),
		.exc_vec_pc_mux_i(exc_cause[4:0]),
		.hwlp_start_i(hwlp_start),
		.hwlp_end_i(hwlp_end),
		.hwlp_cnt_i(hwlp_cnt),
		.dbg_jump_addr_i(dbg_jump_addr),
		.dbg_jump_req_i(dbg_jump_req),
		.jump_target_id_i(jump_target_id),
		.jump_target_ex_i(jump_target_ex),
		.halt_if_i(halt_if),
		.id_ready_i(id_ready),
		.if_busy_o(if_busy),
		.perf_imiss_o(perf_imiss)
	);
	riscv_id_stage #(
		.N_HWLP(N_HWLP),
		.PULP_SECURE(PULP_SECURE),
		.FPU(FPU),
		.APU(APU),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.SHARED_FP_DIVSQRT(SHARED_FP_DIVSQRT),
		.WAPUTYPE(WAPUTYPE),
		.APU_NARGS_CPU(APU_NARGS_CPU),
		.APU_WOP_CPU(APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU)
	) id_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.test_en_i(test_en_i),
		.fregfile_disable_i(fregfile_disable_i),
		.fetch_enable_i(fetch_enable_i),
		.ctrl_busy_o(ctrl_busy),
		.core_ctrl_firstfetch_o(core_ctrl_firstfetch),
		.is_decoding_o(is_decoding),
		.hwlp_dec_cnt_i(hwlp_dec_cnt_id),
		.is_hwlp_i(is_hwlp_id),
		.instr_valid_i(instr_valid_id),
		.instr_rdata_i(instr_rdata_id),
		.instr_req_o(instr_req_int),
		.branch_in_ex_o(branch_in_ex),
		.branch_decision_i(branch_decision),
		.jump_target_o(jump_target_id),
		.clear_instr_valid_o(clear_instr_valid),
		.pc_set_o(pc_set),
		.pc_mux_o(pc_mux_id),
		.exc_pc_mux_o(exc_pc_mux_id),
		.exc_cause_o(exc_cause),
		.trap_addr_mux_o(trap_addr_mux),
		.illegal_c_insn_i(illegal_c_insn_id),
		.is_compressed_i(is_compressed_id),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.halt_if_o(halt_if),
		.id_ready_o(id_ready),
		.ex_ready_i(ex_ready),
		.wb_ready_i(lsu_ready_wb),
		.id_valid_o(id_valid),
		.ex_valid_i(ex_valid),
		.pc_ex_o(pc_ex),
		.alu_en_ex_o(alu_en_ex),
		.alu_operator_ex_o(alu_operator_ex),
		.alu_operand_a_ex_o(alu_operand_a_ex),
		.alu_operand_b_ex_o(alu_operand_b_ex),
		.alu_operand_c_ex_o(alu_operand_c_ex),
		.bmask_a_ex_o(bmask_a_ex),
		.bmask_b_ex_o(bmask_b_ex),
		.imm_vec_ext_ex_o(imm_vec_ext_ex),
		.alu_vec_mode_ex_o(alu_vec_mode_ex),
		.regfile_waddr_ex_o(regfile_waddr_ex),
		.regfile_we_ex_o(regfile_we_ex),
		.regfile_alu_we_ex_o(regfile_alu_we_ex),
		.regfile_alu_waddr_ex_o(regfile_alu_waddr_ex),
		.mult_operator_ex_o(mult_operator_ex),
		.mult_en_ex_o(mult_en_ex),
		.mult_sel_subword_ex_o(mult_sel_subword_ex),
		.mult_signed_mode_ex_o(mult_signed_mode_ex),
		.mult_operand_a_ex_o(mult_operand_a_ex),
		.mult_operand_b_ex_o(mult_operand_b_ex),
		.mult_operand_c_ex_o(mult_operand_c_ex),
		.mult_imm_ex_o(mult_imm_ex),
		.mult_dot_op_a_ex_o(mult_dot_op_a_ex),
		.mult_dot_op_b_ex_o(mult_dot_op_b_ex),
		.mult_dot_op_c_ex_o(mult_dot_op_c_ex),
		.mult_dot_signed_ex_o(mult_dot_signed_ex),
		.fpu_op_ex_o(fpu_op_ex),
		.apu_en_ex_o(apu_en_ex),
		.apu_type_ex_o(apu_type_ex),
		.apu_op_ex_o(apu_op_ex),
		.apu_lat_ex_o(apu_lat_ex),
		.apu_operands_ex_o(apu_operands_ex),
		.apu_flags_ex_o(apu_flags_ex),
		.apu_waddr_ex_o(apu_waddr_ex),
		.apu_read_regs_o(apu_read_regs),
		.apu_read_regs_valid_o(apu_read_regs_valid),
		.apu_read_dep_i(apu_read_dep),
		.apu_write_regs_o(apu_write_regs),
		.apu_write_regs_valid_o(apu_write_regs_valid),
		.apu_write_dep_i(apu_write_dep),
		.apu_perf_dep_o(perf_apu_dep),
		.apu_busy_i(apu_busy),
		.frm_i(frm_csr),
		.csr_access_ex_o(csr_access_ex),
		.csr_op_ex_o(csr_op_ex),
		.current_priv_lvl_i(current_priv_lvl),
		.csr_irq_sec_o(csr_irq_sec),
		.csr_cause_o(csr_cause),
		.csr_save_if_o(csr_save_if),
		.csr_save_id_o(csr_save_id),
		.csr_restore_mret_id_o(csr_restore_mret_id),
		.csr_restore_uret_id_o(csr_restore_uret_id),
		.csr_save_cause_o(csr_save_cause),
		.hwlp_start_o(hwlp_start),
		.hwlp_end_o(hwlp_end),
		.hwlp_cnt_o(hwlp_cnt),
		.csr_hwlp_regid_i(csr_hwlp_regid),
		.csr_hwlp_we_i(csr_hwlp_we),
		.csr_hwlp_data_i(csr_hwlp_data),
		.data_req_ex_o(data_req_ex),
		.data_we_ex_o(data_we_ex),
		.data_type_ex_o(data_type_ex),
		.data_sign_ext_ex_o(data_sign_ext_ex),
		.data_reg_offset_ex_o(data_reg_offset_ex),
		.data_load_event_ex_o(data_load_event_ex),
		.data_misaligned_ex_o(data_misaligned_ex),
		.prepost_useincr_ex_o(useincr_addr_ex),
		.data_misaligned_i(data_misaligned),
		.irq_i(irq_i),
		.irq_sec_i((PULP_SECURE ? irq_sec_i : 1'b0)),
		.irq_id_i(irq_id_i),
		.m_irq_enable_i(m_irq_enable),
		.u_irq_enable_i(u_irq_enable),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.lsu_load_err_i(lsu_load_err),
		.lsu_store_err_i(lsu_store_err),
		.dbg_settings_i(dbg_settings),
		.dbg_req_i(dbg_req),
		.dbg_ack_o(dbg_ack),
		.dbg_stall_i(dbg_stall),
		.dbg_trap_o(dbg_trap),
		.dbg_reg_rreq_i(dbg_reg_rreq),
		.dbg_reg_raddr_i(dbg_reg_raddr),
		.dbg_reg_rdata_o(dbg_reg_rdata),
		.dbg_reg_wreq_i(dbg_reg_wreq),
		.dbg_reg_waddr_i(dbg_reg_waddr),
		.dbg_reg_wdata_i(dbg_reg_wdata),
		.dbg_jump_req_i(dbg_jump_req),
		.regfile_waddr_wb_i(regfile_waddr_fw_wb_o),
		.regfile_we_wb_i(regfile_we_wb),
		.regfile_wdata_wb_i(regfile_wdata),
		.regfile_alu_waddr_fw_i(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_i(regfile_alu_we_fw),
		.regfile_alu_wdata_fw_i(regfile_alu_wdata_fw),
		.mult_multicycle_i(mult_multicycle),
		.perf_jump_o(perf_jump),
		.perf_jr_stall_o(perf_jr_stall),
		.perf_ld_stall_o(perf_ld_stall)
	);
	riscv_ex_stage #(
		.FPU(FPU),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.APU_NARGS_CPU(APU_NARGS_CPU),
		.APU_WOP_CPU(APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU)
	) ex_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.alu_en_i(alu_en_ex),
		.alu_operator_i(alu_operator_ex),
		.alu_operand_a_i(alu_operand_a_ex),
		.alu_operand_b_i(alu_operand_b_ex),
		.alu_operand_c_i(alu_operand_c_ex),
		.bmask_a_i(bmask_a_ex),
		.bmask_b_i(bmask_b_ex),
		.imm_vec_ext_i(imm_vec_ext_ex),
		.alu_vec_mode_i(alu_vec_mode_ex),
		.mult_operator_i(mult_operator_ex),
		.mult_operand_a_i(mult_operand_a_ex),
		.mult_operand_b_i(mult_operand_b_ex),
		.mult_operand_c_i(mult_operand_c_ex),
		.mult_en_i(mult_en_ex),
		.mult_sel_subword_i(mult_sel_subword_ex),
		.mult_signed_mode_i(mult_signed_mode_ex),
		.mult_imm_i(mult_imm_ex),
		.mult_dot_op_a_i(mult_dot_op_a_ex),
		.mult_dot_op_b_i(mult_dot_op_b_ex),
		.mult_dot_op_c_i(mult_dot_op_c_ex),
		.mult_dot_signed_i(mult_dot_signed_ex),
		.mult_multicycle_o(mult_multicycle),
		.fpu_op_i(fpu_op_ex),
		.fpu_prec_i(fprec_csr),
		.fpu_fflags_o(fflags),
		.fpu_fflags_we_o(fflags_we),
		.apu_en_i(apu_en_ex),
		.apu_op_i(apu_op_ex),
		.apu_lat_i(apu_lat_ex),
		.apu_operands_i(apu_operands_ex),
		.apu_waddr_i(apu_waddr_ex),
		.apu_flags_i(apu_flags_ex),
		.apu_read_regs_i(apu_read_regs),
		.apu_read_regs_valid_i(apu_read_regs_valid),
		.apu_read_dep_o(apu_read_dep),
		.apu_write_regs_i(apu_write_regs),
		.apu_write_regs_valid_i(apu_write_regs_valid),
		.apu_write_dep_o(apu_write_dep),
		.apu_perf_type_o(perf_apu_type),
		.apu_perf_cont_o(perf_apu_cont),
		.apu_perf_wb_o(perf_apu_wb),
		.apu_ready_wb_o(apu_ready_wb),
		.apu_busy_o(apu_busy),
		.apu_master_req_o(apu_master_req_o),
		.apu_master_ready_o(apu_master_ready_o),
		.apu_master_gnt_i(apu_master_gnt_i),
		.apu_master_operands_o(apu_master_operands_o),
		.apu_master_op_o(apu_master_op_o),
		.apu_master_valid_i(apu_master_valid_i),
		.apu_master_result_i(apu_master_result_i),
		.lsu_en_i(data_req_ex),
		.lsu_rdata_i(lsu_rdata),
		.csr_access_i(csr_access_ex),
		.csr_rdata_i(csr_rdata),
		.branch_in_ex_i(branch_in_ex),
		.regfile_alu_waddr_i(regfile_alu_waddr_ex),
		.regfile_alu_we_i(regfile_alu_we_ex),
		.regfile_waddr_i(regfile_waddr_ex),
		.regfile_we_i(regfile_we_ex),
		.regfile_waddr_wb_o(regfile_waddr_fw_wb_o),
		.regfile_we_wb_o(regfile_we_wb),
		.regfile_wdata_wb_o(regfile_wdata),
		.jump_target_o(jump_target_ex),
		.branch_decision_o(branch_decision),
		.regfile_alu_waddr_fw_o(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_o(regfile_alu_we_fw),
		.regfile_alu_wdata_fw_o(regfile_alu_wdata_fw),
		.lsu_ready_ex_i(lsu_ready_ex),
		.ex_ready_o(ex_ready),
		.ex_valid_o(ex_valid),
		.wb_ready_i(lsu_ready_wb)
	);
	riscv_load_store_unit load_store_unit_i(
		.clk(clk),
		.rst_n(rst_ni),
		.data_req_o(data_req_o),
		.data_gnt_i(data_gnt_i),
		.data_rvalid_i(data_rvalid_i),
		.data_err_i(data_err_i),
		.data_addr_o(data_addr_o),
		.data_we_o(data_we_o),
		.data_be_o(data_be_o),
		.data_wdata_o(data_wdata_o),
		.data_rdata_i(data_rdata_i),
		.data_we_ex_i(data_we_ex),
		.data_type_ex_i(data_type_ex),
		.data_wdata_ex_i(alu_operand_c_ex),
		.data_reg_offset_ex_i(data_reg_offset_ex),
		.data_sign_ext_ex_i(data_sign_ext_ex),
		.data_rdata_ex_o(lsu_rdata),
		.data_req_ex_i(data_req_ex),
		.operand_a_ex_i(alu_operand_a_ex),
		.operand_b_ex_i(alu_operand_b_ex),
		.addr_useincr_ex_i(useincr_addr_ex),
		.data_misaligned_ex_i(data_misaligned_ex),
		.data_misaligned_o(data_misaligned),
		.load_err_o(lsu_load_err),
		.store_err_o(lsu_store_err),
		.lsu_ready_ex_o(lsu_ready_ex),
		.lsu_ready_wb_o(lsu_ready_wb),
		.ex_valid_i(ex_valid),
		.busy_o(lsu_busy)
	);
	assign wb_valid = lsu_ready_wb & apu_ready_wb;
	riscv_cs_registers #(
		.N_EXT_CNT(N_EXT_PERF_COUNTERS),
		.FPU(FPU),
		.APU(APU),
		.PULP_SECURE(PULP_SECURE)
	) cs_registers_i(
		.clk(clk),
		.rst_n(rst_ni),
		.core_id_i(core_id_i),
		.cluster_id_i(cluster_id_i),
		.mtvec_o(mtvec),
		.utvec_o(utvec),
		.boot_addr_i(boot_addr_i[31:8]),
		.csr_access_i(csr_access),
		.csr_addr_i(csr_addr),
		.csr_wdata_i(csr_wdata),
		.csr_op_i(csr_op),
		.csr_rdata_o(csr_rdata),
		.frm_o(frm_csr),
		.fprec_o(fprec_csr),
		.fflags_i(fflags_csr),
		.fflags_we_i(fflags_we),
		.m_irq_enable_o(m_irq_enable),
		.u_irq_enable_o(u_irq_enable),
		.csr_irq_sec_i(csr_irq_sec),
		.sec_lvl_o(sec_lvl_o),
		.epc_o(epc),
		.priv_lvl_o(current_priv_lvl),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.csr_save_if_i(csr_save_if),
		.csr_save_id_i(csr_save_id),
		.csr_restore_mret_i(csr_restore_mret_id),
		.csr_restore_uret_i(csr_restore_uret_id),
		.csr_cause_i(csr_cause),
		.csr_save_cause_i(csr_save_cause),
		.hwlp_start_i(hwlp_start),
		.hwlp_end_i(hwlp_end),
		.hwlp_cnt_i(hwlp_cnt),
		.hwlp_regid_o(csr_hwlp_regid),
		.hwlp_we_o(csr_hwlp_we),
		.hwlp_data_o(csr_hwlp_data),
		.id_valid_i(id_valid),
		.is_compressed_i(is_compressed_id),
		.is_decoding_i(is_decoding),
		.imiss_i(perf_imiss),
		.pc_set_i(pc_set),
		.jump_i(perf_jump),
		.branch_i(branch_in_ex),
		.branch_taken_i(branch_decision),
		.ld_stall_i(perf_ld_stall),
		.jr_stall_i(perf_jr_stall),
		.apu_typeconflict_i(perf_apu_type),
		.apu_contention_i(perf_apu_cont),
		.apu_dep_i(perf_apu_dep),
		.apu_wb_i(perf_apu_wb),
		.mem_load_i((data_req_o & data_gnt_i) & ~data_we_o),
		.mem_store_i((data_req_o & data_gnt_i) & data_we_o),
		.ext_counters_i(ext_perf_counters_i)
	);
	assign csr_access = (dbg_csr_req == 1'b0 ? csr_access_ex : 1'b1);
	assign csr_addr = (dbg_csr_req == 1'b0 ? csr_addr_int : dbg_csr_addr);
	assign csr_wdata = (dbg_csr_req == 1'b0 ? alu_operand_a_ex : dbg_csr_wdata);
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_WRITE = 2'b01;
	assign csr_op = (dbg_csr_req == 1'b0 ? csr_op_ex : (dbg_csr_we == 1'b1 ? riscv_defines_CSR_OP_WRITE : riscv_defines_CSR_OP_NONE));
	assign csr_addr_int = (csr_access_ex ? alu_operand_b_ex[11:0] : {12 {1'sb0}});
	riscv_debug_unit debug_unit_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.debug_req_i(debug_req_i),
		.debug_gnt_o(debug_gnt_o),
		.debug_rvalid_o(debug_rvalid_o),
		.debug_addr_i(debug_addr_i),
		.debug_we_i(debug_we_i),
		.debug_wdata_i(debug_wdata_i),
		.debug_rdata_o(debug_rdata_o),
		.debug_halt_i(debug_halt_i),
		.debug_resume_i(debug_resume_i),
		.debug_halted_o(debug_halted_o),
		.settings_o(dbg_settings),
		.trap_i(dbg_trap),
		.exc_cause_i(exc_cause),
		.stall_o(dbg_stall),
		.dbg_req_o(dbg_req),
		.dbg_ack_i(dbg_ack),
		.regfile_rreq_o(dbg_reg_rreq),
		.regfile_raddr_o(dbg_reg_raddr),
		.regfile_rdata_i(dbg_reg_rdata),
		.regfile_wreq_o(dbg_reg_wreq),
		.regfile_waddr_o(dbg_reg_waddr),
		.regfile_wdata_o(dbg_reg_wdata),
		.csr_req_o(dbg_csr_req),
		.csr_addr_o(dbg_csr_addr),
		.csr_we_o(dbg_csr_we),
		.csr_wdata_o(dbg_csr_wdata),
		.csr_rdata_i(csr_rdata),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.pc_ex_i(pc_ex),
		.data_load_event_i(data_load_event_ex),
		.instr_valid_id_i(instr_valid_id),
		.sleeping_i(sleeping),
		.branch_in_ex_i(branch_in_ex),
		.branch_taken_i(branch_decision),
		.jump_addr_o(dbg_jump_addr),
		.jump_req_o(dbg_jump_req)
	);
	riscv_tracer riscv_tracer_i(
		.clk(clk_i),
		.rst_n(rst_ni),
		.fetch_enable(fetch_enable_i),
		.core_id(core_id_i),
		.cluster_id(cluster_id_i),
		.pc(id_stage_i.pc_id_i),
		.instr(id_stage_i.instr),
		.compressed(id_stage_i.is_compressed_i),
		.id_valid(id_stage_i.id_valid_o),
		.is_decoding(id_stage_i.is_decoding_o),
		.pipe_flush(id_stage_i.controller_i.pipe_flush_i),
		.mret(id_stage_i.controller_i.mret_insn_i),
		.uret(id_stage_i.controller_i.uret_insn_i),
		.ecall(id_stage_i.controller_i.ecall_insn_i),
		.ebreak(id_stage_i.controller_i.ebrk_insn_i),
		.rs1_value(id_stage_i.operand_a_fw_id),
		.rs2_value(id_stage_i.operand_b_fw_id),
		.rs3_value(id_stage_i.alu_operand_c),
		.rs2_value_vec(id_stage_i.alu_operand_b),
		.rs1_is_fp(id_stage_i.regfile_fp_a),
		.rs2_is_fp(id_stage_i.regfile_fp_b),
		.rs3_is_fp(id_stage_i.regfile_fp_c),
		.rd_is_fp(id_stage_i.regfile_fp_d),
		.ex_valid(ex_valid),
		.ex_reg_addr(regfile_alu_waddr_fw),
		.ex_reg_we(regfile_alu_we_fw),
		.ex_reg_wdata(regfile_alu_wdata_fw),
		.ex_data_addr(data_addr_o),
		.ex_data_req(data_req_o),
		.ex_data_gnt(data_gnt_i),
		.ex_data_we(data_we_o),
		.ex_data_wdata(data_wdata_o),
		.wb_bypass(ex_stage_i.branch_in_ex_i),
		.wb_valid(wb_valid),
		.wb_reg_addr(regfile_waddr_fw_wb_o),
		.wb_reg_we(regfile_we_wb),
		.wb_reg_wdata(regfile_wdata),
		.imm_u_type(id_stage_i.imm_u_type),
		.imm_uj_type(id_stage_i.imm_uj_type),
		.imm_i_type(id_stage_i.imm_i_type),
		.imm_iz_type(id_stage_i.imm_iz_type[11:0]),
		.imm_z_type(id_stage_i.imm_z_type),
		.imm_s_type(id_stage_i.imm_s_type),
		.imm_sb_type(id_stage_i.imm_sb_type),
		.imm_s2_type(id_stage_i.imm_s2_type),
		.imm_s3_type(id_stage_i.imm_s3_type),
		.imm_vs_type(id_stage_i.imm_vs_type),
		.imm_vu_type(id_stage_i.imm_vu_type),
		.imm_shuffle_type(id_stage_i.imm_shuffle_type),
		.imm_clip_type(id_stage_i.instr_rdata_i[11:7])
	);
endmodule
module riscv_cs_registers (
	clk,
	rst_n,
	core_id_i,
	cluster_id_i,
	mtvec_o,
	utvec_o,
	boot_addr_i,
	csr_access_i,
	csr_addr_i,
	csr_wdata_i,
	csr_op_i,
	csr_rdata_o,
	frm_o,
	fprec_o,
	fflags_i,
	fflags_we_i,
	m_irq_enable_o,
	u_irq_enable_o,
	csr_irq_sec_i,
	sec_lvl_o,
	epc_o,
	priv_lvl_o,
	pc_if_i,
	pc_id_i,
	csr_save_if_i,
	csr_save_id_i,
	csr_restore_mret_i,
	csr_restore_uret_i,
	csr_cause_i,
	csr_save_cause_i,
	hwlp_start_i,
	hwlp_end_i,
	hwlp_cnt_i,
	hwlp_data_o,
	hwlp_regid_o,
	hwlp_we_o,
	id_valid_i,
	is_compressed_i,
	is_decoding_i,
	imiss_i,
	pc_set_i,
	jump_i,
	branch_i,
	branch_taken_i,
	ld_stall_i,
	jr_stall_i,
	apu_typeconflict_i,
	apu_contention_i,
	apu_dep_i,
	apu_wb_i,
	mem_load_i,
	mem_store_i,
	ext_counters_i
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter N_HWLP_BITS = $clog2(N_HWLP);
	parameter N_EXT_CNT = 0;
	parameter APU = 0;
	parameter FPU = 0;
	parameter PULP_SECURE = 0;
	input wire clk;
	input wire rst_n;
	input wire [3:0] core_id_i;
	input wire [5:0] cluster_id_i;
	output wire [23:0] mtvec_o;
	output wire [23:0] utvec_o;
	input wire [23:0] boot_addr_i;
	input wire csr_access_i;
	input wire [11:0] csr_addr_i;
	input wire [31:0] csr_wdata_i;
	input wire [1:0] csr_op_i;
	output reg [31:0] csr_rdata_o;
	output wire [2:0] frm_o;
	localparam riscv_defines_C_PC = 5;
	output wire [4:0] fprec_o;
	localparam riscv_defines_C_FFLAG = 5;
	input wire [4:0] fflags_i;
	input wire fflags_we_i;
	output wire m_irq_enable_o;
	output wire u_irq_enable_o;
	input wire csr_irq_sec_i;
	output wire sec_lvl_o;
	output reg [31:0] epc_o;
	output wire [1:0] priv_lvl_o;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	input wire csr_save_if_i;
	input wire csr_save_id_i;
	input wire csr_restore_mret_i;
	input wire csr_restore_uret_i;
	input wire [5:0] csr_cause_i;
	input wire csr_save_cause_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_start_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_end_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_cnt_i;
	output wire [31:0] hwlp_data_o;
	output reg [N_HWLP_BITS - 1:0] hwlp_regid_o;
	output reg [2:0] hwlp_we_o;
	input wire id_valid_i;
	input wire is_compressed_i;
	input wire is_decoding_i;
	input wire imiss_i;
	input wire pc_set_i;
	input wire jump_i;
	input wire branch_i;
	input wire branch_taken_i;
	input wire ld_stall_i;
	input wire jr_stall_i;
	input wire apu_typeconflict_i;
	input wire apu_contention_i;
	input wire apu_dep_i;
	input wire apu_wb_i;
	input wire mem_load_i;
	input wire mem_store_i;
	input wire [N_EXT_CNT - 1:0] ext_counters_i;
	localparam N_APU_CNT = (APU == 1 ? 4 : 0);
	localparam N_PERF_COUNTERS = (12 + N_EXT_CNT) + N_APU_CNT;
	localparam PERF_EXT_ID = 11;
	localparam PERF_APU_ID = 12 + N_EXT_CNT;
	localparam PULP_SEC = 1;
	localparam N_PERF_REGS = N_PERF_COUNTERS;
	reg [31:0] csr_wdata_int;
	reg [31:0] csr_rdata_int;
	reg csr_we_int;
	localparam riscv_defines_C_RM = 3;
	reg [2:0] frm_q;
	reg [2:0] frm_n;
	reg [4:0] fflags_q;
	reg [4:0] fflags_n;
	reg [4:0] fprec_q;
	reg [4:0] fprec_n;
	reg [31:0] mepc_q;
	reg [31:0] mepc_n;
	reg [31:0] uepc_q;
	reg [31:0] uepc_n;
	reg [31:0] exception_pc;
	reg [5:0] mstatus_q;
	reg [5:0] mstatus_n;
	reg [5:0] mcause_q;
	reg [5:0] mcause_n;
	reg [5:0] ucause_q;
	reg [5:0] ucause_n;
	reg [23:0] mtvec_n;
	wire [23:0] mtvec_q;
	reg [23:0] mtvec_reg_q;
	reg [23:0] utvec_n;
	reg [23:0] utvec_q;
	wire is_irq;
	reg [1:0] priv_lvl_n;
	reg [1:0] priv_lvl_q;
	wire [1:0] priv_lvl_reg_q;
	reg id_valid_q;
	wire [N_PERF_COUNTERS - 1:0] PCCR_in;
	reg [N_PERF_COUNTERS - 1:0] PCCR_inc;
	reg [N_PERF_COUNTERS - 1:0] PCCR_inc_q;
	reg [(N_PERF_REGS * 32) - 1:0] PCCR_q;
	reg [(N_PERF_REGS * 32) - 1:0] PCCR_n;
	reg [1:0] PCMR_n;
	reg [1:0] PCMR_q;
	reg [N_PERF_COUNTERS - 1:0] PCER_n;
	reg [N_PERF_COUNTERS - 1:0] PCER_q;
	reg [31:0] perf_rdata;
	reg [4:0] pccr_index;
	reg pccr_all_sel;
	reg is_pccr;
	reg is_pcer;
	reg is_pcmr;
	assign is_irq = csr_cause_i[5];
	generate
		if (1) begin : genblk1
			always @(*) begin
				if (_sv2v_0)
					;
				case (csr_addr_i)
					12'h001: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fflags_q} : {32 {1'sb0}});
					12'h002: csr_rdata_int = (FPU == 1 ? {29'b00000000000000000000000000000, frm_q} : {32 {1'sb0}});
					12'h003: csr_rdata_int = (FPU == 1 ? {24'b000000000000000000000000, frm_q, fflags_q} : {32 {1'sb0}});
					12'h006: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fprec_q} : {32 {1'sb0}});
					12'h300: csr_rdata_int = {19'b0000000000000000000, mstatus_q[1-:2], 3'b000, mstatus_q[2], 2'h0, mstatus_q[3], mstatus_q[4], 2'h0, mstatus_q[5]};
					12'h305: csr_rdata_int = {mtvec_q, 8'h00};
					12'h341: csr_rdata_int = mepc_q;
					12'h342: csr_rdata_int = {mcause_q[5], 26'b00000000000000000000000000, mcause_q[4:0]};
					12'hf14: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'h7b0: csr_rdata_int = hwlp_start_i[0+:32];
					12'h7b1: csr_rdata_int = hwlp_end_i[0+:32];
					12'h7b2: csr_rdata_int = hwlp_cnt_i[0+:32];
					12'h7b4: csr_rdata_int = hwlp_start_i[32+:32];
					12'h7b5: csr_rdata_int = hwlp_end_i[32+:32];
					12'h7b6: csr_rdata_int = hwlp_cnt_i[32+:32];
					12'h000: csr_rdata_int = {27'b000000000000000000000000000, mstatus_q[3], 3'h0, mstatus_q[5]};
					12'h005: csr_rdata_int = {utvec_q, 8'h00};
					12'h014: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'h041: csr_rdata_int = uepc_q;
					12'h042: csr_rdata_int = {ucause_q[5], 26'h0000000, ucause_q[4:0]};
					12'hc10: csr_rdata_int = {30'h00000000, priv_lvl_q};
					default: csr_rdata_int = 1'sb0;
				endcase
			end
		end
	endgenerate
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	generate
		if (PULP_SECURE == 1) begin : genblk2
			always @(*) begin
				if (_sv2v_0)
					;
				fflags_n = fflags_q;
				frm_n = frm_q;
				fprec_n = fprec_q;
				epc_o = mepc_q;
				mepc_n = mepc_q;
				uepc_n = uepc_q;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				ucause_n = ucause_q;
				hwlp_we_o = 1'sb0;
				hwlp_regid_o = 1'sb0;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = mtvec_q;
				utvec_n = utvec_q;
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
				end
				case (csr_addr_i)
					12'h001:
						if (csr_we_int)
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h002:
						if (csr_we_int)
							frm_n = (FPU == 1 ? csr_wdata_int[2:0] : {3 {1'sb0}});
					12'h003:
						if (csr_we_int) begin
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
							frm_n = (FPU == 1 ? csr_wdata_int[7:riscv_defines_C_FFLAG] : {3 {1'sb0}});
						end
					12'h006:
						if (csr_we_int)
							fprec_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h300:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], csr_wdata_int[3], csr_wdata_int[4], csr_wdata_int[7], csr_wdata_int[12:11]};
					12'h305:
						if (csr_we_int)
							mtvec_n = csr_wdata_int[31:8];
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
					12'h7b0:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b0;
						end
					12'h7b1:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b0;
						end
					12'h7b2:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b0;
						end
					12'h7b4:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b1;
						end
					12'h7b5:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b1;
						end
					12'h7b6:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b1;
						end
					12'h000:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], mstatus_q[4], csr_wdata_int[4], mstatus_q[2], sv2v_cast_2(mstatus_q[1-:2])};
					12'h005:
						if (csr_we_int)
							utvec_n = csr_wdata_int[31:8];
					12'h041:
						if (csr_we_int)
							uepc_n = csr_wdata_int;
					12'h042:
						if (csr_we_int)
							ucause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
				endcase
				(* full_case, parallel_case *)
				case (1'b1)
					csr_save_cause_i: begin
						(* full_case, parallel_case *)
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							default:
								;
						endcase
						(* full_case, parallel_case *)
						case (priv_lvl_q)
							2'b00:
								if (~is_irq) begin
									priv_lvl_n = 2'b11;
									mstatus_n[2] = mstatus_q[5];
									mstatus_n[4] = 1'b0;
									mstatus_n[1-:2] = 2'b00;
									mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
								else if (~csr_irq_sec_i) begin
									priv_lvl_n = 2'b00;
									mstatus_n[3] = mstatus_q[5];
									mstatus_n[5] = 1'b0;
									uepc_n = exception_pc;
									ucause_n = csr_cause_i;
								end
								else begin
									priv_lvl_n = 2'b11;
									mstatus_n[2] = mstatus_q[5];
									mstatus_n[4] = 1'b0;
									mstatus_n[1-:2] = 2'b00;
									mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
							2'b11: begin
								priv_lvl_n = 2'b11;
								mstatus_n[2] = mstatus_q[4];
								mstatus_n[4] = 1'b0;
								mstatus_n[1-:2] = 2'b11;
								mepc_n = exception_pc;
								mcause_n = csr_cause_i;
							end
							default:
								;
						endcase
					end
					csr_restore_uret_i: begin
						mstatus_n[5] = mstatus_q[3];
						priv_lvl_n = 2'b00;
						mstatus_n[3] = 1'b1;
						epc_o = uepc_q;
					end
					csr_restore_mret_i: begin
						(* full_case, parallel_case *)
						case (mstatus_q[1-:2])
							2'b00: begin
								mstatus_n[5] = mstatus_q[2];
								priv_lvl_n = 2'b00;
								mstatus_n[2] = 1'b1;
								mstatus_n[1-:2] = 2'b00;
							end
							2'b11: begin
								mstatus_n[4] = mstatus_q[2];
								priv_lvl_n = 2'b11;
								mstatus_n[2] = 1'b1;
								mstatus_n[1-:2] = 2'b00;
							end
							default:
								;
						endcase
						epc_o = mepc_q;
					end
					default:
						;
				endcase
			end
		end
		else begin : genblk2
			always @(*) begin
				if (_sv2v_0)
					;
				fflags_n = fflags_q;
				frm_n = frm_q;
				fprec_n = fprec_q;
				epc_o = mepc_q;
				mepc_n = mepc_q;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				hwlp_we_o = 1'sb0;
				hwlp_regid_o = 1'sb0;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = mtvec_q;
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
				end
				case (csr_addr_i)
					12'h001:
						if (csr_we_int)
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h002:
						if (csr_we_int)
							frm_n = (FPU == 1 ? csr_wdata_int[2:0] : {3 {1'sb0}});
					12'h003:
						if (csr_we_int) begin
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
							frm_n = (FPU == 1 ? csr_wdata_int[7:riscv_defines_C_FFLAG] : {3 {1'sb0}});
						end
					12'h006:
						if (csr_we_int)
							fprec_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h300:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], csr_wdata_int[3], csr_wdata_int[4], csr_wdata_int[7], csr_wdata_int[12:11]};
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
					12'h7b0:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b0;
						end
					12'h7b1:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b0;
						end
					12'h7b2:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b0;
						end
					12'h7b4:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b1;
						end
					12'h7b5:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b1;
						end
					12'h7b6:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b1;
						end
				endcase
				(* full_case, parallel_case *)
				case (1'b1)
					csr_save_cause_i: begin
						(* full_case, parallel_case *)
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							default:
								;
						endcase
						priv_lvl_n = 2'b11;
						mstatus_n[2] = mstatus_q[4];
						mstatus_n[4] = 1'b0;
						mstatus_n[1-:2] = 2'b11;
						mepc_n = exception_pc;
						mcause_n = csr_cause_i;
					end
					csr_restore_mret_i: begin
						mstatus_n[4] = mstatus_q[2];
						priv_lvl_n = 2'b11;
						mstatus_n[2] = 1'b1;
						mstatus_n[1-:2] = 2'b11;
						epc_o = mepc_q;
					end
					default:
						;
				endcase
			end
		end
	endgenerate
	assign hwlp_data_o = csr_wdata_int;
	localparam riscv_defines_CSR_OP_CLEAR = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_SET = 2'b10;
	localparam riscv_defines_CSR_OP_WRITE = 2'b01;
	always @(*) begin
		if (_sv2v_0)
			;
		csr_wdata_int = csr_wdata_i;
		csr_we_int = 1'b1;
		(* full_case, parallel_case *)
		case (csr_op_i)
			riscv_defines_CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
			riscv_defines_CSR_OP_SET: csr_wdata_int = csr_wdata_i | csr_rdata_o;
			riscv_defines_CSR_OP_CLEAR: csr_wdata_int = ~csr_wdata_i & csr_rdata_o;
			riscv_defines_CSR_OP_NONE: begin
				csr_wdata_int = csr_wdata_i;
				csr_we_int = 1'b0;
			end
			default:
				;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		csr_rdata_o = csr_rdata_int;
		if ((is_pccr || is_pcer) || is_pcmr)
			csr_rdata_o = perf_rdata;
	end
	assign m_irq_enable_o = mstatus_q[4] & (priv_lvl_q == 2'b11);
	assign u_irq_enable_o = mstatus_q[5] & (priv_lvl_q == 2'b00);
	assign priv_lvl_o = priv_lvl_q;
	assign sec_lvl_o = priv_lvl_q[0];
	assign frm_o = (FPU == 1 ? frm_q : {3 {1'sb0}});
	assign fprec_o = (FPU == 1 ? fprec_q : {5 {1'sb0}});
	assign mtvec_o = mtvec_q;
	assign utvec_o = utvec_q;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			if (FPU == 1) begin
				frm_q <= 1'sb0;
				fflags_q <= 1'sb0;
				fprec_q <= 1'sb0;
			end
			if (PULP_SECURE == 1) begin
				uepc_q <= 1'sb0;
				ucause_q <= 1'sb0;
				mtvec_reg_q <= 1'sb0;
				utvec_q <= 1'sb0;
			end
			priv_lvl_q <= 2'b11;
			mstatus_q <= 6'b000011;
			mepc_q <= 1'sb0;
			mcause_q <= 1'sb0;
		end
		else begin
			if (FPU == 1) begin
				frm_q <= frm_n;
				fflags_q <= fflags_n;
				fprec_q <= fprec_n;
			end
			if (PULP_SECURE == 1) begin
				mstatus_q <= mstatus_n;
				uepc_q <= uepc_n;
				ucause_q <= ucause_n;
				priv_lvl_q <= priv_lvl_n;
				utvec_q <= utvec_n;
				mtvec_reg_q <= mtvec_n;
			end
			else begin
				mstatus_q <= {1'b0, mstatus_n[4], 1'b0, mstatus_n[2], 2'b11};
				priv_lvl_q <= 2'b11;
			end
			mepc_q <= mepc_n;
			mcause_q <= mcause_n;
		end
	assign mtvec_q = (PULP_SECURE ? mtvec_reg_q : boot_addr_i);
	assign PCCR_in[0] = 1'b1;
	assign PCCR_in[1] = id_valid_i & is_decoding_i;
	assign PCCR_in[2] = ld_stall_i & id_valid_q;
	assign PCCR_in[3] = jr_stall_i & id_valid_q;
	assign PCCR_in[4] = imiss_i & ~pc_set_i;
	assign PCCR_in[5] = mem_load_i;
	assign PCCR_in[6] = mem_store_i;
	assign PCCR_in[7] = jump_i & id_valid_q;
	assign PCCR_in[8] = branch_i & id_valid_q;
	assign PCCR_in[9] = (branch_i & branch_taken_i) & id_valid_q;
	assign PCCR_in[10] = (id_valid_i & is_decoding_i) & is_compressed_i;
	generate
		if (APU == 1) begin : genblk3
			assign PCCR_in[PERF_APU_ID] = apu_typeconflict_i & ~apu_dep_i;
			assign PCCR_in[PERF_APU_ID + 1] = apu_contention_i;
			assign PCCR_in[PERF_APU_ID + 2] = apu_dep_i & ~apu_contention_i;
			assign PCCR_in[PERF_APU_ID + 3] = apu_wb_i;
		end
	endgenerate
	genvar _gv_i_17;
	generate
		for (_gv_i_17 = 0; _gv_i_17 < N_EXT_CNT; _gv_i_17 = _gv_i_17 + 1) begin : genblk4
			localparam i = _gv_i_17;
			assign PCCR_in[PERF_EXT_ID + i] = ext_counters_i[i];
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		is_pccr = 1'b0;
		is_pcmr = 1'b0;
		is_pcer = 1'b0;
		pccr_all_sel = 1'b0;
		pccr_index = 1'sb0;
		perf_rdata = 1'sb0;
		if (csr_access_i) begin
			(* full_case, parallel_case *)
			case (csr_addr_i)
				12'h7a0: begin
					is_pcer = 1'b1;
					perf_rdata[N_PERF_COUNTERS - 1:0] = PCER_q;
				end
				12'h7a1: begin
					is_pcmr = 1'b1;
					perf_rdata[1:0] = PCMR_q;
				end
				12'h79f: begin
					is_pccr = 1'b1;
					pccr_all_sel = 1'b1;
				end
				default:
					;
			endcase
			if (csr_addr_i[11:5] == 7'b0111100) begin
				is_pccr = 1'b1;
				pccr_index = csr_addr_i[4:0];
				perf_rdata = (csr_addr_i[4:0] < N_PERF_COUNTERS ? PCCR_q[csr_addr_i[4:0] * 32+:32] : {32 {1'sb0}});
			end
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < N_PERF_COUNTERS; i = i + 1)
				begin : PERF_CNT_INC
					PCCR_inc[i] = (PCCR_in[i] & PCER_q[i]) & PCMR_q[0];
					PCCR_n[i * 32+:32] = PCCR_q[i * 32+:32];
					if ((PCCR_inc_q[i] == 1'b1) && ((PCCR_q[i * 32+:32] != 32'hffffffff) || (PCMR_q[1] == 1'b0)))
						PCCR_n[i * 32+:32] = PCCR_q[i * 32+:32] + 1;
					if ((is_pccr == 1'b1) && ((pccr_all_sel == 1'b1) || (pccr_index == i)))
						(* full_case, parallel_case *)
						case (csr_op_i)
							riscv_defines_CSR_OP_NONE:
								;
							riscv_defines_CSR_OP_WRITE: PCCR_n[i * 32+:32] = csr_wdata_i;
							riscv_defines_CSR_OP_SET: PCCR_n[i * 32+:32] = csr_wdata_i | PCCR_q[i * 32+:32];
							riscv_defines_CSR_OP_CLEAR: PCCR_n[i * 32+:32] = csr_wdata_i & ~PCCR_q[i * 32+:32];
						endcase
				end
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		PCMR_n = PCMR_q;
		PCER_n = PCER_q;
		if (is_pcmr)
			(* full_case, parallel_case *)
			case (csr_op_i)
				riscv_defines_CSR_OP_NONE:
					;
				riscv_defines_CSR_OP_WRITE: PCMR_n = csr_wdata_i[1:0];
				riscv_defines_CSR_OP_SET: PCMR_n = csr_wdata_i[1:0] | PCMR_q;
				riscv_defines_CSR_OP_CLEAR: PCMR_n = csr_wdata_i[1:0] & ~PCMR_q;
			endcase
		if (is_pcer)
			(* full_case, parallel_case *)
			case (csr_op_i)
				riscv_defines_CSR_OP_NONE:
					;
				riscv_defines_CSR_OP_WRITE: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0];
				riscv_defines_CSR_OP_SET: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0] | PCER_q;
				riscv_defines_CSR_OP_CLEAR: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0] & ~PCER_q;
			endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			id_valid_q <= 1'b0;
			PCER_q <= 1'sb0;
			PCMR_q <= 2'h3;
			begin : sv2v_autoblock_2
				reg signed [31:0] i;
				for (i = 0; i < N_PERF_REGS; i = i + 1)
					begin
						PCCR_q[i * 32+:32] <= 1'sb0;
						PCCR_inc_q[i] <= 1'sb0;
					end
			end
		end
		else begin
			id_valid_q <= id_valid_i;
			PCER_q <= PCER_n;
			PCMR_q <= PCMR_n;
			begin : sv2v_autoblock_3
				reg signed [31:0] i;
				for (i = 0; i < N_PERF_REGS; i = i + 1)
					begin
						PCCR_q[i * 32+:32] <= PCCR_n[i * 32+:32];
						PCCR_inc_q[i] <= PCCR_inc[i];
					end
			end
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_debug_unit (
	clk,
	rst_n,
	debug_req_i,
	debug_gnt_o,
	debug_rvalid_o,
	debug_addr_i,
	debug_we_i,
	debug_wdata_i,
	debug_rdata_o,
	debug_halted_o,
	debug_halt_i,
	debug_resume_i,
	settings_o,
	trap_i,
	exc_cause_i,
	stall_o,
	dbg_req_o,
	dbg_ack_i,
	regfile_rreq_o,
	regfile_raddr_o,
	regfile_rdata_i,
	regfile_wreq_o,
	regfile_waddr_o,
	regfile_wdata_o,
	csr_req_o,
	csr_addr_o,
	csr_we_o,
	csr_wdata_o,
	csr_rdata_i,
	pc_if_i,
	pc_id_i,
	pc_ex_i,
	data_load_event_i,
	instr_valid_id_i,
	sleeping_i,
	branch_in_ex_i,
	branch_taken_i,
	jump_req_o,
	jump_addr_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire debug_req_i;
	output reg debug_gnt_o;
	output reg debug_rvalid_o;
	input wire [14:0] debug_addr_i;
	input wire debug_we_i;
	input wire [31:0] debug_wdata_i;
	output reg [31:0] debug_rdata_o;
	output reg debug_halted_o;
	input wire debug_halt_i;
	input wire debug_resume_i;
	localparam riscv_defines_DBG_SETS_W = 6;
	output wire [5:0] settings_o;
	input wire trap_i;
	input wire [5:0] exc_cause_i;
	output reg stall_o;
	output reg dbg_req_o;
	input wire dbg_ack_i;
	output wire regfile_rreq_o;
	output wire [5:0] regfile_raddr_o;
	input wire [31:0] regfile_rdata_i;
	output wire regfile_wreq_o;
	output wire [5:0] regfile_waddr_o;
	output wire [31:0] regfile_wdata_o;
	output wire csr_req_o;
	output wire [11:0] csr_addr_o;
	output reg csr_we_o;
	output wire [31:0] csr_wdata_o;
	input wire [31:0] csr_rdata_i;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	input wire [31:0] pc_ex_i;
	input wire data_load_event_i;
	input wire instr_valid_id_i;
	input wire sleeping_i;
	input wire branch_in_ex_i;
	input wire branch_taken_i;
	output wire jump_req_o;
	output wire [31:0] jump_addr_o;
	reg [2:0] rdata_sel_q;
	reg [2:0] rdata_sel_n;
	reg [0:0] state_q;
	reg [0:0] state_n;
	reg [5:0] settings_q;
	reg [5:0] settings_n;
	reg [14:0] addr_q;
	reg [31:0] wdata_q;
	reg regfile_rreq_q;
	reg regfile_rreq_n;
	reg regfile_fp_sel_q;
	reg regfile_fp_sel_n;
	reg jump_req_q;
	reg jump_req_n;
	reg csr_req_q;
	reg csr_req_n;
	reg regfile_wreq;
	reg regfile_fp_wr;
	reg [1:0] stall_cs;
	reg [1:0] stall_ns;
	reg [31:0] dbg_rdata;
	reg dbg_resume;
	reg dbg_halt;
	reg [5:0] dbg_cause_q;
	reg [5:0] dbg_cause_n;
	reg dbg_ssth_q;
	reg dbg_ssth_n;
	reg ssth_clear;
	reg [1:0] pc_tracking_fsm_cs;
	reg [1:0] pc_tracking_fsm_ns;
	reg [31:0] ppc_int;
	reg [31:0] npc_int;
	localparam riscv_defines_DBG_SETS_EBRK = 1;
	localparam riscv_defines_DBG_SETS_ECALL = 4;
	localparam riscv_defines_DBG_SETS_EILL = 3;
	localparam riscv_defines_DBG_SETS_ELSU = 2;
	localparam riscv_defines_DBG_SETS_SSTE = 0;
	always @(*) begin
		if (_sv2v_0)
			;
		rdata_sel_n = 3'd0;
		state_n = 1'd0;
		debug_gnt_o = 1'b0;
		regfile_rreq_n = 1'b0;
		regfile_wreq = 1'b0;
		csr_req_n = 1'b0;
		csr_we_o = 1'b0;
		jump_req_n = 1'b0;
		dbg_resume = 1'b0;
		dbg_halt = 1'b0;
		settings_n = settings_q;
		ssth_clear = 1'b0;
		regfile_fp_sel_n = 1'b0;
		regfile_fp_wr = 1'b0;
		if (debug_req_i) begin
			if (debug_we_i) begin
				if (debug_addr_i[14]) begin
					if (state_q == 1'd0) begin
						debug_gnt_o = 1'b0;
						state_n = 1'd1;
						if (debug_halted_o)
							csr_req_n = 1'b1;
					end
					else begin
						debug_gnt_o = 1'b1;
						state_n = 1'd0;
						csr_we_o = 1'b1;
					end
				end
				else
					(* full_case, parallel_case *)
					case (debug_addr_i[13:8])
						6'b000000: begin
							debug_gnt_o = 1'b1;
							(* full_case, parallel_case *)
							case (debug_addr_i[6:2])
								5'b00000: begin
									if (debug_wdata_i[16]) begin
										if (~debug_halted_o)
											dbg_halt = 1'b1;
									end
									else if (debug_halted_o)
										dbg_resume = 1'b1;
									settings_n[riscv_defines_DBG_SETS_SSTE] = debug_wdata_i[0];
								end
								5'b00001: ssth_clear = debug_wdata_i[0];
								5'b00010: begin
									settings_n[riscv_defines_DBG_SETS_ECALL] = debug_wdata_i[11];
									settings_n[riscv_defines_DBG_SETS_ELSU] = debug_wdata_i[7] | debug_wdata_i[5];
									settings_n[riscv_defines_DBG_SETS_EBRK] = debug_wdata_i[3];
									settings_n[riscv_defines_DBG_SETS_EILL] = debug_wdata_i[2];
								end
								default:
									;
							endcase
						end
						6'b100000: begin
							debug_gnt_o = 1'b1;
							if (debug_halted_o)
								(* full_case, parallel_case *)
								case (debug_addr_i[6:2])
									5'b00000: jump_req_n = 1'b1;
									default:
										;
								endcase
						end
						6'b000100: begin
							debug_gnt_o = 1'b1;
							if (debug_halted_o)
								regfile_wreq = 1'b1;
						end
						6'b000101: begin
							debug_gnt_o = 1'b1;
							if (debug_halted_o) begin
								regfile_wreq = 1'b1;
								regfile_fp_wr = 1'b1;
							end
						end
						default: debug_gnt_o = 1'b1;
					endcase
			end
			else if (debug_addr_i[14]) begin
				debug_gnt_o = 1'b1;
				if (debug_halted_o) begin
					csr_req_n = 1'b1;
					rdata_sel_n = 3'd1;
				end
			end
			else
				(* full_case, parallel_case *)
				case (debug_addr_i[13:8])
					6'b000000: begin
						debug_gnt_o = 1'b1;
						rdata_sel_n = 3'd3;
					end
					6'b100000: begin
						debug_gnt_o = 1'b1;
						rdata_sel_n = 3'd4;
					end
					6'b000100: begin
						debug_gnt_o = 1'b1;
						if (debug_halted_o) begin
							regfile_rreq_n = 1'b1;
							rdata_sel_n = 3'd2;
						end
					end
					6'b000101: begin
						debug_gnt_o = 1'b1;
						if (debug_halted_o) begin
							regfile_rreq_n = 1'b1;
							regfile_fp_sel_n = 1'b1;
							rdata_sel_n = 3'd2;
						end
					end
					default: debug_gnt_o = 1'b1;
				endcase
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		dbg_rdata = 1'sb0;
		case (rdata_sel_q)
			3'd3:
				(* full_case, parallel_case *)
				case (addr_q[6:2])
					5'h00: dbg_rdata[31:0] = {15'b000000000000000, debug_halted_o, 15'b000000000000000, settings_q[riscv_defines_DBG_SETS_SSTE]};
					5'h01: dbg_rdata[31:0] = {15'b000000000000000, sleeping_i, 15'b000000000000000, dbg_ssth_q};
					5'h02: begin
						dbg_rdata[31:16] = 1'sb0;
						dbg_rdata[15:12] = 1'sb0;
						dbg_rdata[11] = settings_q[riscv_defines_DBG_SETS_ECALL];
						dbg_rdata[10:8] = 1'sb0;
						dbg_rdata[7] = settings_q[riscv_defines_DBG_SETS_ELSU];
						dbg_rdata[6] = 1'b0;
						dbg_rdata[5] = settings_q[riscv_defines_DBG_SETS_ELSU];
						dbg_rdata[4] = 1'b0;
						dbg_rdata[3] = settings_q[riscv_defines_DBG_SETS_EBRK];
						dbg_rdata[2] = settings_q[riscv_defines_DBG_SETS_EILL];
						dbg_rdata[1:0] = 1'sb0;
					end
					5'h03: dbg_rdata = {dbg_cause_q[5], 26'b00000000000000000000000000, dbg_cause_q[4:0]};
					5'h10: dbg_rdata = 1'sb0;
					5'h12: dbg_rdata = 1'sb0;
					5'h14: dbg_rdata = 1'sb0;
					5'h16: dbg_rdata = 1'sb0;
					5'h18: dbg_rdata = 1'sb0;
					5'h1a: dbg_rdata = 1'sb0;
					5'h1c: dbg_rdata = 1'sb0;
					5'h1e: dbg_rdata = 1'sb0;
					default:
						;
				endcase
			3'd4:
				(* full_case, parallel_case *)
				case (addr_q[2:2])
					1'b0: dbg_rdata = npc_int;
					1'b1: dbg_rdata = ppc_int;
					default:
						;
				endcase
			default:
				;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		debug_rdata_o = 1'sb0;
		case (rdata_sel_q)
			3'd1: debug_rdata_o = csr_rdata_i;
			3'd2: debug_rdata_o = regfile_rdata_i;
			3'd3: debug_rdata_o = dbg_rdata;
			3'd4: debug_rdata_o = dbg_rdata;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n)
			debug_rvalid_o <= 1'b0;
		else
			debug_rvalid_o <= debug_gnt_o;
	localparam riscv_defines_DBG_CAUSE_HALT = 6'h1f;
	always @(*) begin
		if (_sv2v_0)
			;
		stall_ns = stall_cs;
		dbg_req_o = 1'b0;
		stall_o = 1'b0;
		debug_halted_o = 1'b0;
		dbg_cause_n = dbg_cause_q;
		dbg_ssth_n = dbg_ssth_q;
		case (stall_cs)
			2'd0: begin
				dbg_ssth_n = 1'b0;
				if ((dbg_halt | debug_halt_i) | trap_i) begin
					dbg_req_o = 1'b1;
					stall_ns = 2'd1;
					if (trap_i) begin
						if (settings_q[riscv_defines_DBG_SETS_SSTE])
							dbg_ssth_n = 1'b1;
						dbg_cause_n = exc_cause_i;
					end
					else
						dbg_cause_n = riscv_defines_DBG_CAUSE_HALT;
				end
			end
			2'd1: begin
				dbg_req_o = 1'b1;
				if (dbg_ack_i)
					stall_ns = 2'd2;
				if (dbg_resume | debug_resume_i)
					stall_ns = 2'd0;
			end
			2'd2: begin
				stall_o = 1'b1;
				debug_halted_o = 1'b1;
				if (dbg_resume | debug_resume_i) begin
					stall_ns = 2'd0;
					stall_o = 1'b0;
				end
			end
		endcase
		if (ssth_clear)
			dbg_ssth_n = 1'b0;
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			stall_cs <= 2'd0;
			dbg_cause_q <= riscv_defines_DBG_CAUSE_HALT;
			dbg_ssth_q <= 1'b0;
		end
		else begin
			stall_cs <= stall_ns;
			dbg_cause_q <= dbg_cause_n;
			dbg_ssth_q <= dbg_ssth_n;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		pc_tracking_fsm_ns = pc_tracking_fsm_cs;
		ppc_int = pc_id_i;
		npc_int = pc_if_i;
		(* full_case, parallel_case *)
		case (pc_tracking_fsm_cs)
			2'd0: begin
				ppc_int = pc_id_i;
				npc_int = pc_if_i;
			end
			2'd1: begin
				ppc_int = pc_ex_i;
				npc_int = pc_if_i;
			end
			2'd2: begin
				ppc_int = pc_ex_i;
				npc_int = pc_id_i;
				if (jump_req_o)
					pc_tracking_fsm_ns = 2'd1;
			end
			default: pc_tracking_fsm_ns = 2'd0;
		endcase
		if (dbg_ack_i) begin
			pc_tracking_fsm_ns = 2'd0;
			if (branch_in_ex_i) begin
				if (branch_taken_i)
					pc_tracking_fsm_ns = 2'd1;
				else
					pc_tracking_fsm_ns = 2'd2;
			end
			else if (data_load_event_i) begin
				if (instr_valid_id_i)
					pc_tracking_fsm_ns = 2'd2;
				else
					pc_tracking_fsm_ns = 2'd1;
			end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			pc_tracking_fsm_cs <= 2'd0;
			addr_q <= 1'sb0;
			wdata_q <= 1'sb0;
			state_q <= 1'd0;
			rdata_sel_q <= 3'd0;
			regfile_rreq_q <= 1'b0;
			regfile_fp_sel_q <= 1'b0;
			csr_req_q <= 1'b0;
			jump_req_q <= 1'b0;
			settings_q <= 1'b0;
		end
		else begin
			pc_tracking_fsm_cs <= pc_tracking_fsm_ns;
			settings_q <= settings_n;
			if (debug_req_i) begin
				addr_q <= debug_addr_i;
				wdata_q <= debug_wdata_i;
				state_q <= state_n;
			end
			if (debug_req_i | debug_rvalid_o) begin
				regfile_rreq_q <= regfile_rreq_n;
				regfile_fp_sel_q <= regfile_fp_sel_n;
				csr_req_q <= csr_req_n;
				jump_req_q <= jump_req_n;
				rdata_sel_q <= rdata_sel_n;
			end
		end
	assign regfile_rreq_o = regfile_rreq_q;
	assign regfile_raddr_o = {regfile_fp_sel_q, addr_q[6:2]};
	assign regfile_wreq_o = regfile_wreq;
	assign regfile_waddr_o = {regfile_fp_wr, debug_addr_i[6:2]};
	assign regfile_wdata_o = debug_wdata_i;
	assign csr_req_o = csr_req_q;
	assign csr_addr_o = addr_q[13:2];
	assign csr_wdata_o = wdata_q;
	assign jump_req_o = jump_req_q;
	assign jump_addr_o = wdata_q;
	assign settings_o = settings_q;
	initial _sv2v_0 = 0;
endmodule
module riscv_decoder (
	deassert_we_i,
	data_misaligned_i,
	mult_multicycle_i,
	illegal_insn_o,
	ebrk_insn_o,
	mret_insn_o,
	uret_insn_o,
	ecall_insn_o,
	pipe_flush_o,
	rega_used_o,
	regb_used_o,
	regc_used_o,
	reg_fp_a_o,
	reg_fp_b_o,
	reg_fp_c_o,
	reg_fp_d_o,
	bmask_a_mux_o,
	bmask_b_mux_o,
	alu_bmask_a_mux_sel_o,
	alu_bmask_b_mux_sel_o,
	instr_rdata_i,
	illegal_c_insn_i,
	alu_en_o,
	alu_operator_o,
	alu_op_a_mux_sel_o,
	alu_op_b_mux_sel_o,
	alu_op_c_mux_sel_o,
	alu_vec_mode_o,
	scalar_replication_o,
	imm_a_mux_sel_o,
	imm_b_mux_sel_o,
	regc_mux_o,
	mult_operator_o,
	mult_int_en_o,
	mult_dot_en_o,
	mult_imm_mux_o,
	mult_sel_subword_o,
	mult_signed_mode_o,
	mult_dot_signed_o,
	fpu_op_o,
	apu_en_o,
	apu_type_o,
	apu_op_o,
	apu_lat_o,
	apu_flags_src_o,
	fp_rnd_mode_o,
	regfile_mem_we_o,
	regfile_alu_we_o,
	regfile_alu_waddr_sel_o,
	csr_access_o,
	csr_status_o,
	csr_op_o,
	current_priv_lvl_i,
	data_req_o,
	data_we_o,
	prepost_useincr_o,
	data_type_o,
	data_sign_extension_o,
	data_reg_offset_o,
	data_load_event_o,
	hwloop_we_o,
	hwloop_target_mux_sel_o,
	hwloop_start_mux_sel_o,
	hwloop_cnt_mux_sel_o,
	jump_in_dec_o,
	jump_in_id_o,
	jump_target_mux_sel_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	parameter PULP_SECURE = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_WOP_CPU = 6;
	input wire deassert_we_i;
	input wire data_misaligned_i;
	input wire mult_multicycle_i;
	output reg illegal_insn_o;
	output reg ebrk_insn_o;
	output reg mret_insn_o;
	output reg uret_insn_o;
	output reg ecall_insn_o;
	output reg pipe_flush_o;
	output reg rega_used_o;
	output reg regb_used_o;
	output reg regc_used_o;
	output reg reg_fp_a_o;
	output reg reg_fp_b_o;
	output reg reg_fp_c_o;
	output reg reg_fp_d_o;
	output reg [0:0] bmask_a_mux_o;
	output reg [1:0] bmask_b_mux_o;
	output reg alu_bmask_a_mux_sel_o;
	output reg alu_bmask_b_mux_sel_o;
	input wire [31:0] instr_rdata_i;
	input wire illegal_c_insn_i;
	output reg alu_en_o;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_o;
	output reg [2:0] alu_op_a_mux_sel_o;
	output reg [2:0] alu_op_b_mux_sel_o;
	output reg [1:0] alu_op_c_mux_sel_o;
	output reg [1:0] alu_vec_mode_o;
	output reg scalar_replication_o;
	output reg [0:0] imm_a_mux_sel_o;
	output reg [3:0] imm_b_mux_sel_o;
	output reg [1:0] regc_mux_o;
	output reg [2:0] mult_operator_o;
	output reg mult_int_en_o;
	output reg mult_dot_en_o;
	output reg [0:0] mult_imm_mux_o;
	output reg mult_sel_subword_o;
	output reg [1:0] mult_signed_mode_o;
	output reg [1:0] mult_dot_signed_o;
	localparam riscv_defines_C_CMD = 4;
	output reg [3:0] fpu_op_o;
	output wire apu_en_o;
	output reg [WAPUTYPE - 1:0] apu_type_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_o;
	output reg [1:0] apu_lat_o;
	output reg [WAPUTYPE - 1:0] apu_flags_src_o;
	output reg [2:0] fp_rnd_mode_o;
	output wire regfile_mem_we_o;
	output wire regfile_alu_we_o;
	output reg regfile_alu_waddr_sel_o;
	output reg csr_access_o;
	output reg csr_status_o;
	output wire [1:0] csr_op_o;
	input wire [1:0] current_priv_lvl_i;
	output wire data_req_o;
	output reg data_we_o;
	output reg prepost_useincr_o;
	output reg [1:0] data_type_o;
	output reg data_sign_extension_o;
	output reg [1:0] data_reg_offset_o;
	output reg data_load_event_o;
	output wire [2:0] hwloop_we_o;
	output reg hwloop_target_mux_sel_o;
	output reg hwloop_start_mux_sel_o;
	output reg hwloop_cnt_mux_sel_o;
	output wire [1:0] jump_in_dec_o;
	output wire [1:0] jump_in_id_o;
	output reg [1:0] jump_target_mux_sel_o;
	localparam apu_core_package_SHARED_INT_MULT = 0;
	localparam APUTYPE_FP = (SHARED_FP ? (SHARED_DSP_MULT + apu_core_package_SHARED_INT_MULT) + SHARED_INT_DIV : 0);
	localparam APUTYPE_DSP_MULT = (SHARED_DSP_MULT ? 0 : 0);
	localparam APUTYPE_INT_MULT = (apu_core_package_SHARED_INT_MULT ? SHARED_DSP_MULT : 0);
	localparam APUTYPE_INT_DIV = (SHARED_INT_DIV ? SHARED_DSP_MULT + apu_core_package_SHARED_INT_MULT : 0);
	localparam APUTYPE_ADDSUB = (SHARED_FP ? APUTYPE_FP : 0);
	localparam APUTYPE_MULT = (SHARED_FP ? APUTYPE_FP + 1 : 0);
	localparam APUTYPE_CAST = (SHARED_FP ? APUTYPE_FP + 2 : 0);
	localparam APUTYPE_MAC = (SHARED_FP ? APUTYPE_FP + 3 : 0);
	localparam APUTYPE_DIV = (SHARED_FP_DIVSQRT == 1 ? APUTYPE_FP + 4 : 0);
	localparam APUTYPE_SQRT = (SHARED_FP_DIVSQRT == 1 ? APUTYPE_FP + 5 : 0);
	localparam APUTYPE_DIVSQRT = (SHARED_FP_DIVSQRT == 2 ? APUTYPE_FP + 4 : 0);
	reg regfile_mem_we;
	reg regfile_alu_we;
	reg data_req;
	reg [2:0] hwloop_we;
	reg csr_illegal;
	reg [1:0] jump_in_id;
	reg [1:0] csr_op;
	reg apu_en;
	localparam apu_core_package_APU_FLAGS_DSP_MULT = 0;
	localparam apu_core_package_APU_FLAGS_FP = 2;
	localparam apu_core_package_APU_FLAGS_INT_MULT = 1;
	localparam apu_core_package_PIPE_REG_ADDSUB = 1;
	localparam apu_core_package_PIPE_REG_CAST = 1;
	localparam apu_core_package_PIPE_REG_DSP_MULT = 1;
	localparam apu_core_package_PIPE_REG_MAC = 2;
	localparam apu_core_package_PIPE_REG_MULT = 1;
	localparam riscv_defines_ALU_ABS = 7'b0010100;
	localparam riscv_defines_ALU_ADD = 7'b0011000;
	localparam riscv_defines_ALU_ADDR = 7'b0011100;
	localparam riscv_defines_ALU_ADDU = 7'b0011010;
	localparam riscv_defines_ALU_ADDUR = 7'b0011110;
	localparam riscv_defines_ALU_AND = 7'b0010101;
	localparam riscv_defines_ALU_BCLR = 7'b0101011;
	localparam riscv_defines_ALU_BEXT = 7'b0101000;
	localparam riscv_defines_ALU_BEXTU = 7'b0101001;
	localparam riscv_defines_ALU_BINS = 7'b0101010;
	localparam riscv_defines_ALU_BSET = 7'b0101100;
	localparam riscv_defines_ALU_CLB = 7'b0110101;
	localparam riscv_defines_ALU_CLIP = 7'b0010110;
	localparam riscv_defines_ALU_CLIPU = 7'b0010111;
	localparam riscv_defines_ALU_CNT = 7'b0110100;
	localparam riscv_defines_ALU_DIV = 7'b0110001;
	localparam riscv_defines_ALU_DIVU = 7'b0110000;
	localparam riscv_defines_ALU_EQ = 7'b0001100;
	localparam riscv_defines_ALU_EXT = 7'b0111111;
	localparam riscv_defines_ALU_EXTS = 7'b0111110;
	localparam riscv_defines_ALU_FCLASS = 7'b1001000;
	localparam riscv_defines_ALU_FEQ = 7'b1000011;
	localparam riscv_defines_ALU_FF1 = 7'b0110110;
	localparam riscv_defines_ALU_FKEEP = 7'b1111111;
	localparam riscv_defines_ALU_FL1 = 7'b0110111;
	localparam riscv_defines_ALU_FLE = 7'b1000101;
	localparam riscv_defines_ALU_FLT = 7'b1000100;
	localparam riscv_defines_ALU_FMAX = 7'b1000110;
	localparam riscv_defines_ALU_FMIN = 7'b1000111;
	localparam riscv_defines_ALU_FSGNJ = 7'b1000000;
	localparam riscv_defines_ALU_FSGNJN = 7'b1000001;
	localparam riscv_defines_ALU_FSGNJX = 7'b1000010;
	localparam riscv_defines_ALU_GES = 7'b0001010;
	localparam riscv_defines_ALU_GEU = 7'b0001011;
	localparam riscv_defines_ALU_GTS = 7'b0001000;
	localparam riscv_defines_ALU_GTU = 7'b0001001;
	localparam riscv_defines_ALU_INS = 7'b0101101;
	localparam riscv_defines_ALU_LES = 7'b0000100;
	localparam riscv_defines_ALU_LEU = 7'b0000101;
	localparam riscv_defines_ALU_LTS = 7'b0000000;
	localparam riscv_defines_ALU_LTU = 7'b0000001;
	localparam riscv_defines_ALU_MAX = 7'b0010010;
	localparam riscv_defines_ALU_MAXU = 7'b0010011;
	localparam riscv_defines_ALU_MIN = 7'b0010000;
	localparam riscv_defines_ALU_MINU = 7'b0010001;
	localparam riscv_defines_ALU_NE = 7'b0001101;
	localparam riscv_defines_ALU_OR = 7'b0101110;
	localparam riscv_defines_ALU_PCKHI = 7'b0111001;
	localparam riscv_defines_ALU_PCKLO = 7'b0111000;
	localparam riscv_defines_ALU_REM = 7'b0110011;
	localparam riscv_defines_ALU_REMU = 7'b0110010;
	localparam riscv_defines_ALU_ROR = 7'b0100110;
	localparam riscv_defines_ALU_SHUF = 7'b0111010;
	localparam riscv_defines_ALU_SHUF2 = 7'b0111011;
	localparam riscv_defines_ALU_SLETS = 7'b0000110;
	localparam riscv_defines_ALU_SLETU = 7'b0000111;
	localparam riscv_defines_ALU_SLL = 7'b0100111;
	localparam riscv_defines_ALU_SLTS = 7'b0000010;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	localparam riscv_defines_ALU_SRA = 7'b0100100;
	localparam riscv_defines_ALU_SRL = 7'b0100101;
	localparam riscv_defines_ALU_SUB = 7'b0011001;
	localparam riscv_defines_ALU_SUBR = 7'b0011101;
	localparam riscv_defines_ALU_SUBU = 7'b0011011;
	localparam riscv_defines_ALU_SUBUR = 7'b0011111;
	localparam riscv_defines_ALU_XOR = 7'b0101111;
	localparam riscv_defines_BMASK_A_IMM = 1'b1;
	localparam riscv_defines_BMASK_A_REG = 1'b0;
	localparam riscv_defines_BMASK_A_S3 = 1'b1;
	localparam riscv_defines_BMASK_A_ZERO = 1'b0;
	localparam riscv_defines_BMASK_B_IMM = 1'b1;
	localparam riscv_defines_BMASK_B_ONE = 2'b11;
	localparam riscv_defines_BMASK_B_REG = 1'b0;
	localparam riscv_defines_BMASK_B_S2 = 2'b00;
	localparam riscv_defines_BMASK_B_S3 = 2'b01;
	localparam riscv_defines_BMASK_B_ZERO = 2'b10;
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_BRANCH_JAL = 2'b01;
	localparam riscv_defines_BRANCH_JALR = 2'b10;
	localparam riscv_defines_BRANCH_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_CLEAR = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_SET = 2'b10;
	localparam riscv_defines_CSR_OP_WRITE = 2'b01;
	localparam riscv_defines_C_FPU_ADD_CMD = 4'h0;
	localparam riscv_defines_C_FPU_DIV_CMD = 4'h3;
	localparam riscv_defines_C_FPU_F2I_CMD = 4'h5;
	localparam riscv_defines_C_FPU_FMADD_CMD = 4'h8;
	localparam riscv_defines_C_FPU_FMSUB_CMD = 4'h9;
	localparam riscv_defines_C_FPU_FNMADD_CMD = 4'ha;
	localparam riscv_defines_C_FPU_FNMSUB_CMD = 4'hb;
	localparam riscv_defines_C_FPU_I2F_CMD = 4'h4;
	localparam riscv_defines_C_FPU_MUL_CMD = 4'h2;
	localparam riscv_defines_C_FPU_SQRT_CMD = 4'h6;
	localparam riscv_defines_C_FPU_SUB_CMD = 4'h1;
	localparam riscv_defines_IMMA_Z = 1'b0;
	localparam riscv_defines_IMMA_ZERO = 1'b1;
	localparam riscv_defines_IMMB_BI = 4'b1011;
	localparam riscv_defines_IMMB_CLIP = 4'b1001;
	localparam riscv_defines_IMMB_I = 4'b0000;
	localparam riscv_defines_IMMB_PCINCR = 4'b0011;
	localparam riscv_defines_IMMB_S = 4'b0001;
	localparam riscv_defines_IMMB_S2 = 4'b0100;
	localparam riscv_defines_IMMB_SHUF = 4'b1000;
	localparam riscv_defines_IMMB_U = 4'b0010;
	localparam riscv_defines_IMMB_VS = 4'b0110;
	localparam riscv_defines_IMMB_VU = 4'b0111;
	localparam riscv_defines_JT_COND = 2'b11;
	localparam riscv_defines_JT_JAL = 2'b01;
	localparam riscv_defines_JT_JALR = 2'b10;
	localparam riscv_defines_MIMM_S3 = 1'b1;
	localparam riscv_defines_MIMM_ZERO = 1'b0;
	localparam riscv_defines_MUL_DOT16 = 3'b101;
	localparam riscv_defines_MUL_DOT8 = 3'b100;
	localparam riscv_defines_MUL_H = 3'b110;
	localparam riscv_defines_MUL_I = 3'b010;
	localparam riscv_defines_MUL_IR = 3'b011;
	localparam riscv_defines_MUL_MAC32 = 3'b000;
	localparam riscv_defines_MUL_MSU32 = 3'b001;
	localparam riscv_defines_OPCODE_AUIPC = 7'h17;
	localparam riscv_defines_OPCODE_BRANCH = 7'h63;
	localparam riscv_defines_OPCODE_HWLOOP = 7'h7b;
	localparam riscv_defines_OPCODE_JAL = 7'h6f;
	localparam riscv_defines_OPCODE_JALR = 7'h67;
	localparam riscv_defines_OPCODE_LOAD = 7'h03;
	localparam riscv_defines_OPCODE_LOAD_FP = 7'h07;
	localparam riscv_defines_OPCODE_LOAD_POST = 7'h0b;
	localparam riscv_defines_OPCODE_LUI = 7'h37;
	localparam riscv_defines_OPCODE_OP = 7'h33;
	localparam riscv_defines_OPCODE_OPIMM = 7'h13;
	localparam riscv_defines_OPCODE_OP_FMADD = 7'h43;
	localparam riscv_defines_OPCODE_OP_FMSUB = 7'h47;
	localparam riscv_defines_OPCODE_OP_FNMADD = 7'h4f;
	localparam riscv_defines_OPCODE_OP_FNMSUB = 7'h4b;
	localparam riscv_defines_OPCODE_OP_FP = 7'h53;
	localparam riscv_defines_OPCODE_PULP_OP = 7'h5b;
	localparam riscv_defines_OPCODE_STORE = 7'h23;
	localparam riscv_defines_OPCODE_STORE_FP = 7'h27;
	localparam riscv_defines_OPCODE_STORE_POST = 7'h2b;
	localparam riscv_defines_OPCODE_SYSTEM = 7'h73;
	localparam riscv_defines_OPCODE_VECOP = 7'h57;
	localparam riscv_defines_OP_A_CURRPC = 3'b001;
	localparam riscv_defines_OP_A_IMM = 3'b010;
	localparam riscv_defines_OP_A_REGA_OR_FWD = 3'b000;
	localparam riscv_defines_OP_A_REGB_OR_FWD = 3'b011;
	localparam riscv_defines_OP_A_REGC_OR_FWD = 3'b100;
	localparam riscv_defines_OP_B_BMASK = 3'b100;
	localparam riscv_defines_OP_B_IMM = 3'b010;
	localparam riscv_defines_OP_B_REGA_OR_FWD = 3'b011;
	localparam riscv_defines_OP_B_REGB_OR_FWD = 3'b000;
	localparam riscv_defines_OP_B_REGC_OR_FWD = 3'b001;
	localparam riscv_defines_OP_C_JT = 2'b10;
	localparam riscv_defines_OP_C_REGB_OR_FWD = 2'b01;
	localparam riscv_defines_OP_C_REGC_OR_FWD = 2'b00;
	localparam riscv_defines_REGC_RD = 2'b01;
	localparam riscv_defines_REGC_S1 = 2'b10;
	localparam riscv_defines_REGC_S4 = 2'b00;
	localparam riscv_defines_REGC_ZERO = 2'b11;
	localparam riscv_defines_VEC_MODE16 = 2'b10;
	localparam riscv_defines_VEC_MODE32 = 2'b00;
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		jump_in_id = riscv_defines_BRANCH_NONE;
		jump_target_mux_sel_o = riscv_defines_JT_JAL;
		alu_en_o = 1'b1;
		alu_operator_o = riscv_defines_ALU_SLTU;
		alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
		alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
		alu_op_c_mux_sel_o = riscv_defines_OP_C_REGC_OR_FWD;
		alu_vec_mode_o = riscv_defines_VEC_MODE32;
		scalar_replication_o = 1'b0;
		regc_mux_o = riscv_defines_REGC_ZERO;
		imm_a_mux_sel_o = riscv_defines_IMMA_ZERO;
		imm_b_mux_sel_o = riscv_defines_IMMB_I;
		mult_operator_o = riscv_defines_MUL_I;
		mult_int_en_o = 1'b0;
		mult_dot_en_o = 1'b0;
		mult_imm_mux_o = riscv_defines_MIMM_ZERO;
		mult_signed_mode_o = 2'b00;
		mult_sel_subword_o = 1'b0;
		mult_dot_signed_o = 2'b00;
		apu_en = 1'b0;
		apu_type_o = 1'sb0;
		apu_op_o = 1'sb0;
		apu_lat_o = 1'sb0;
		apu_flags_src_o = 1'sb0;
		fp_rnd_mode_o = 1'sb0;
		fpu_op_o = 1'sb0;
		regfile_mem_we = 1'b0;
		regfile_alu_we = 1'b0;
		regfile_alu_waddr_sel_o = 1'b1;
		prepost_useincr_o = 1'b1;
		hwloop_we = 3'b000;
		hwloop_target_mux_sel_o = 1'b0;
		hwloop_start_mux_sel_o = 1'b0;
		hwloop_cnt_mux_sel_o = 1'b0;
		csr_access_o = 1'b0;
		csr_status_o = 1'b0;
		csr_illegal = 1'b0;
		csr_op = riscv_defines_CSR_OP_NONE;
		mret_insn_o = 1'b0;
		uret_insn_o = 1'b0;
		data_we_o = 1'b0;
		data_type_o = 2'b00;
		data_sign_extension_o = 1'b0;
		data_reg_offset_o = 2'b00;
		data_req = 1'b0;
		data_load_event_o = 1'b0;
		illegal_insn_o = 1'b0;
		ebrk_insn_o = 1'b0;
		ecall_insn_o = 1'b0;
		pipe_flush_o = 1'b0;
		rega_used_o = 1'b0;
		regb_used_o = 1'b0;
		regc_used_o = 1'b0;
		reg_fp_a_o = 1'b0;
		reg_fp_b_o = 1'b0;
		reg_fp_c_o = 1'b0;
		reg_fp_d_o = 1'b0;
		bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
		bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
		alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_IMM;
		alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_IMM;
		(* full_case, parallel_case *)
		case (instr_rdata_i[6:0])
			riscv_defines_OPCODE_JAL: begin
				jump_target_mux_sel_o = riscv_defines_JT_JAL;
				jump_in_id = riscv_defines_BRANCH_JAL;
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_JALR: begin
				jump_target_mux_sel_o = riscv_defines_JT_JALR;
				jump_in_id = riscv_defines_BRANCH_JALR;
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				if (instr_rdata_i[14:12] != 3'b000) begin
					jump_in_id = riscv_defines_BRANCH_NONE;
					regfile_alu_we = 1'b0;
					illegal_insn_o = 1'b1;
				end
			end
			riscv_defines_OPCODE_BRANCH: begin
				jump_target_mux_sel_o = riscv_defines_JT_COND;
				jump_in_id = riscv_defines_BRANCH_COND;
				alu_op_c_mux_sel_o = riscv_defines_OP_C_JT;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				(* full_case, parallel_case *)
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = riscv_defines_ALU_EQ;
					3'b001: alu_operator_o = riscv_defines_ALU_NE;
					3'b100: alu_operator_o = riscv_defines_ALU_LTS;
					3'b101: alu_operator_o = riscv_defines_ALU_GES;
					3'b110: alu_operator_o = riscv_defines_ALU_LTU;
					3'b111: alu_operator_o = riscv_defines_ALU_GEU;
					3'b010: begin
						alu_operator_o = riscv_defines_ALU_EQ;
						regb_used_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
						imm_b_mux_sel_o = riscv_defines_IMMB_BI;
					end
					3'b011: begin
						alu_operator_o = riscv_defines_ALU_NE;
						regb_used_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
						imm_b_mux_sel_o = riscv_defines_IMMB_BI;
					end
				endcase
			end
			riscv_defines_OPCODE_STORE, riscv_defines_OPCODE_STORE_POST: begin
				data_req = 1'b1;
				data_we_o = 1'b1;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				alu_operator_o = riscv_defines_ALU_ADD;
				alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
				if (instr_rdata_i[6:0] == riscv_defines_OPCODE_STORE_POST) begin
					prepost_useincr_o = 1'b0;
					regfile_alu_waddr_sel_o = 1'b0;
					regfile_alu_we = 1'b1;
				end
				if (instr_rdata_i[14] == 1'b0) begin
					imm_b_mux_sel_o = riscv_defines_IMMB_S;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				end
				else begin
					regc_used_o = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
					regc_mux_o = riscv_defines_REGC_RD;
				end
				(* full_case, parallel_case *)
				case (instr_rdata_i[13:12])
					2'b00: data_type_o = 2'b10;
					2'b01: data_type_o = 2'b01;
					2'b10: data_type_o = 2'b00;
					default: begin
						data_req = 1'b0;
						data_we_o = 1'b0;
						illegal_insn_o = 1'b1;
					end
				endcase
			end
			riscv_defines_OPCODE_LOAD, riscv_defines_OPCODE_LOAD_POST: begin
				data_req = 1'b1;
				regfile_mem_we = 1'b1;
				rega_used_o = 1'b1;
				data_type_o = 2'b00;
				alu_operator_o = riscv_defines_ALU_ADD;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_I;
				if (instr_rdata_i[6:0] == riscv_defines_OPCODE_LOAD_POST) begin
					prepost_useincr_o = 1'b0;
					regfile_alu_waddr_sel_o = 1'b0;
					regfile_alu_we = 1'b1;
				end
				data_sign_extension_o = ~instr_rdata_i[14];
				(* full_case, parallel_case *)
				case (instr_rdata_i[13:12])
					2'b00: data_type_o = 2'b10;
					2'b01: data_type_o = 2'b01;
					2'b10: data_type_o = 2'b00;
					default: data_type_o = 2'b00;
				endcase
				if (instr_rdata_i[14:12] == 3'b111) begin
					regb_used_o = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
					data_sign_extension_o = ~instr_rdata_i[30];
					(* full_case, parallel_case *)
					case (instr_rdata_i[31:25])
						7'b0000000, 7'b0100000: data_type_o = 2'b10;
						7'b0001000, 7'b0101000: data_type_o = 2'b01;
						7'b0010000: data_type_o = 2'b00;
						default: illegal_insn_o = 1'b1;
					endcase
				end
				if (instr_rdata_i[14:12] == 3'b110)
					data_load_event_o = 1'b1;
				if (instr_rdata_i[14:12] == 3'b011)
					illegal_insn_o = 1'b1;
			end
			riscv_defines_OPCODE_LUI: begin
				alu_op_a_mux_sel_o = riscv_defines_OP_A_IMM;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_a_mux_sel_o = riscv_defines_IMMA_ZERO;
				imm_b_mux_sel_o = riscv_defines_IMMB_U;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_AUIPC: begin
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_U;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_OPIMM: begin
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_I;
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				(* full_case, parallel_case *)
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = riscv_defines_ALU_ADD;
					3'b010: alu_operator_o = riscv_defines_ALU_SLTS;
					3'b011: alu_operator_o = riscv_defines_ALU_SLTU;
					3'b100: alu_operator_o = riscv_defines_ALU_XOR;
					3'b110: alu_operator_o = riscv_defines_ALU_OR;
					3'b111: alu_operator_o = riscv_defines_ALU_AND;
					3'b001: begin
						alu_operator_o = riscv_defines_ALU_SLL;
						if (instr_rdata_i[31:25] != 7'b0000000)
							illegal_insn_o = 1'b1;
					end
					3'b101:
						if (instr_rdata_i[31:25] == 7'b0000000)
							alu_operator_o = riscv_defines_ALU_SRL;
						else if (instr_rdata_i[31:25] == 7'b0100000)
							alu_operator_o = riscv_defines_ALU_SRA;
						else
							illegal_insn_o = 1'b1;
				endcase
			end
			riscv_defines_OPCODE_OP: begin
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				if (instr_rdata_i[31]) begin
					bmask_a_mux_o = riscv_defines_BMASK_A_S3;
					bmask_b_mux_o = riscv_defines_BMASK_B_S2;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					(* full_case, parallel_case *)
					case (instr_rdata_i[14:12])
						3'b000: begin
							alu_operator_o = riscv_defines_ALU_BEXT;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
							if (~instr_rdata_i[30]) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								regb_used_o = 1'b1;
							end
						end
						3'b001: begin
							alu_operator_o = riscv_defines_ALU_BEXTU;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
							if (~instr_rdata_i[30]) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								regb_used_o = 1'b1;
							end
						end
						3'b010: begin
							alu_operator_o = riscv_defines_ALU_BINS;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							if (~instr_rdata_i[30]) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
								regb_used_o = 1'b1;
							end
						end
						3'b011: begin
							alu_operator_o = riscv_defines_ALU_BCLR;
							if (~instr_rdata_i[30]) begin
								regb_used_o = 1'b1;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							end
						end
						3'b100: begin
							alu_operator_o = riscv_defines_ALU_BSET;
							if (~instr_rdata_i[30]) begin
								regb_used_o = 1'b1;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							end
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else begin
					if (~instr_rdata_i[28])
						regb_used_o = 1'b1;
					(* full_case, parallel_case *)
					case ({instr_rdata_i[30:25], instr_rdata_i[14:12]})
						9'b000000000: alu_operator_o = riscv_defines_ALU_ADD;
						9'b100000000: alu_operator_o = riscv_defines_ALU_SUB;
						9'b000000010: alu_operator_o = riscv_defines_ALU_SLTS;
						9'b000000011: alu_operator_o = riscv_defines_ALU_SLTU;
						9'b000000100: alu_operator_o = riscv_defines_ALU_XOR;
						9'b000000110: alu_operator_o = riscv_defines_ALU_OR;
						9'b000000111: alu_operator_o = riscv_defines_ALU_AND;
						9'b000000001: alu_operator_o = riscv_defines_ALU_SLL;
						9'b000000101: alu_operator_o = riscv_defines_ALU_SRL;
						9'b100000101: alu_operator_o = riscv_defines_ALU_SRA;
						9'b000001000: begin
							alu_en_o = 1'b0;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_MAC32;
							regc_mux_o = riscv_defines_REGC_ZERO;
						end
						9'b000001001: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b11;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
						end
						9'b000001010: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b01;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
						end
						9'b000001011: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b00;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
						end
						9'b000001100: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_DIV;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001101: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_DIVU;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001110: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_REM;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001111: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_REMU;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b100001000: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_MAC32;
							if (apu_core_package_SHARED_INT_MULT) begin
								mult_int_en_o = 1'b0;
								mult_dot_en_o = 1'b0;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
								apu_op_o = mult_operator_o;
								apu_type_o = APUTYPE_INT_MULT;
								apu_lat_o = 2'h1;
							end
						end
						9'b100001001: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							mult_int_en_o = 1'b1;
							mult_operator_o = riscv_defines_MUL_MSU32;
							if (apu_core_package_SHARED_INT_MULT) begin
								mult_int_en_o = 1'b0;
								mult_dot_en_o = 1'b0;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
								apu_op_o = mult_operator_o;
								apu_type_o = APUTYPE_INT_MULT;
								apu_lat_o = 2'h1;
							end
						end
						9'b000010010: alu_operator_o = riscv_defines_ALU_SLETS;
						9'b000010011: alu_operator_o = riscv_defines_ALU_SLETU;
						9'b000010100: alu_operator_o = riscv_defines_ALU_MIN;
						9'b000010101: alu_operator_o = riscv_defines_ALU_MINU;
						9'b000010110: alu_operator_o = riscv_defines_ALU_MAX;
						9'b000010111: alu_operator_o = riscv_defines_ALU_MAXU;
						9'b000100101: alu_operator_o = riscv_defines_ALU_ROR;
						9'b001000000: alu_operator_o = riscv_defines_ALU_FF1;
						9'b001000001: alu_operator_o = riscv_defines_ALU_FL1;
						9'b001000010: alu_operator_o = riscv_defines_ALU_CLB;
						9'b001000011: alu_operator_o = riscv_defines_ALU_CNT;
						9'b001000100: begin
							alu_operator_o = riscv_defines_ALU_EXTS;
							alu_vec_mode_o = riscv_defines_VEC_MODE16;
						end
						9'b001000101: begin
							alu_operator_o = riscv_defines_ALU_EXT;
							alu_vec_mode_o = riscv_defines_VEC_MODE16;
						end
						9'b001000110: begin
							alu_operator_o = riscv_defines_ALU_EXTS;
							alu_vec_mode_o = riscv_defines_VEC_MODE8;
						end
						9'b001000111: begin
							alu_operator_o = riscv_defines_ALU_EXT;
							alu_vec_mode_o = riscv_defines_VEC_MODE8;
						end
						9'b000010000: alu_operator_o = riscv_defines_ALU_ABS;
						9'b001010001: begin
							alu_operator_o = riscv_defines_ALU_CLIP;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							imm_b_mux_sel_o = riscv_defines_IMMB_CLIP;
						end
						9'b001010010: begin
							alu_operator_o = riscv_defines_ALU_CLIPU;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							imm_b_mux_sel_o = riscv_defines_IMMB_CLIP;
						end
						9'b001010101: begin
							alu_operator_o = riscv_defines_ALU_CLIP;
							regb_used_o = 1'b1;
						end
						9'b001010110: begin
							alu_operator_o = riscv_defines_ALU_CLIPU;
							regb_used_o = 1'b1;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
			end
			riscv_defines_OPCODE_OP_FP:
				if (FPU == 1) begin
					fp_rnd_mode_o = instr_rdata_i[14:12];
					if (instr_rdata_i[26:25] == 2'b00)
						case (instr_rdata_i[31:27])
							5'h00: begin
								apu_type_o = APUTYPE_ADDSUB;
								apu_op_o = 2'b00;
								fpu_op_o = riscv_defines_C_FPU_ADD_CMD;
								apu_lat_o = 2'h2;
								if (FPU == 1) begin
									apu_en = 1'b1;
									alu_en_o = 1'b0;
									apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									reg_fp_a_o = 1'b1;
									reg_fp_b_o = 1'b1;
									reg_fp_d_o = 1'b1;
								end
							end
							5'h01: begin
								apu_type_o = APUTYPE_ADDSUB;
								apu_op_o = 2'b01;
								fpu_op_o = riscv_defines_C_FPU_SUB_CMD;
								apu_lat_o = 2'h2;
								if (FPU == 1) begin
									apu_en = 1'b1;
									alu_en_o = 1'b0;
									apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									reg_fp_a_o = 1'b1;
									reg_fp_b_o = 1'b1;
									reg_fp_d_o = 1'b1;
								end
							end
							5'h02: begin
								apu_type_o = APUTYPE_MULT;
								fpu_op_o = riscv_defines_C_FPU_MUL_CMD;
								apu_lat_o = 2'h2;
								if (FPU == 1) begin
									apu_en = 1'b1;
									alu_en_o = 1'b0;
									apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									reg_fp_a_o = 1'b1;
									reg_fp_b_o = 1'b1;
									reg_fp_d_o = 1'b1;
								end
							end
							5'h03:
								if (SHARED_FP_DIVSQRT == 1) begin
									apu_type_o = APUTYPE_DIV;
									apu_lat_o = 2'h3;
									if (FPU == 1) begin
										apu_en = 1'b1;
										alu_en_o = 1'b0;
										apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
										rega_used_o = 1'b1;
										regb_used_o = 1'b1;
										reg_fp_a_o = 1'b1;
										reg_fp_b_o = 1'b1;
										reg_fp_d_o = 1'b1;
									end
								end
								else if (SHARED_FP_DIVSQRT == 2) begin
									apu_type_o = APUTYPE_DIVSQRT;
									apu_lat_o = 2'h3;
									apu_op_o = 1'b0;
									fpu_op_o = riscv_defines_C_FPU_DIV_CMD;
									if (FPU == 1) begin
										apu_en = 1'b1;
										alu_en_o = 1'b0;
										apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
										rega_used_o = 1'b1;
										regb_used_o = 1'b1;
										reg_fp_a_o = 1'b1;
										reg_fp_b_o = 1'b1;
										reg_fp_d_o = 1'b1;
									end
								end
								else
									illegal_insn_o = 1'b1;
							5'h0b:
								if (SHARED_FP_DIVSQRT == 1) begin
									apu_type_o = APUTYPE_SQRT;
									apu_lat_o = 2'h3;
									if (FPU == 1) begin
										apu_en = 1'b1;
										alu_en_o = 1'b0;
										apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
										rega_used_o = 1'b1;
										regb_used_o = 1'b1;
										reg_fp_a_o = 1'b1;
										reg_fp_b_o = 1'b1;
										reg_fp_d_o = 1'b1;
									end
								end
								else if (SHARED_FP_DIVSQRT == 2) begin
									apu_type_o = APUTYPE_DIVSQRT;
									apu_lat_o = 2'h3;
									apu_op_o = 1'b1;
									fpu_op_o = riscv_defines_C_FPU_SQRT_CMD;
									if (FPU == 1) begin
										apu_en = 1'b1;
										alu_en_o = 1'b0;
										apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
										rega_used_o = 1'b1;
										regb_used_o = 1'b1;
										reg_fp_a_o = 1'b1;
										reg_fp_b_o = 1'b1;
										reg_fp_d_o = 1'b1;
									end
								end
								else
									illegal_insn_o = 1'b1;
							5'h04: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								reg_fp_b_o = 1'b1;
								reg_fp_d_o = 1'b1;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FSGNJ;
									3'h1: alu_operator_o = riscv_defines_ALU_FSGNJN;
									3'h2: alu_operator_o = riscv_defines_ALU_FSGNJX;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							5'h05: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								reg_fp_b_o = 1'b1;
								reg_fp_d_o = 1'b1;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FMIN;
									3'h1: alu_operator_o = riscv_defines_ALU_FMAX;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							5'h08: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b0;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								reg_fp_d_o = 1'b1;
								alu_operator_o = riscv_defines_ALU_FKEEP;
							end
							5'h14: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								reg_fp_b_o = 1'b1;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FLE;
									3'h1: alu_operator_o = riscv_defines_ALU_FLT;
									3'h2: alu_operator_o = riscv_defines_ALU_FEQ;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							5'h18: begin
								rega_used_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
								apu_type_o = APUTYPE_CAST;
								apu_op_o = 2'b01;
								apu_lat_o = 2'h2;
								fpu_op_o = riscv_defines_C_FPU_F2I_CMD;
							end
							5'h1a: begin
								rega_used_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_d_o = 1'b1;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
								apu_type_o = APUTYPE_CAST;
								apu_op_o = 2'b00;
								apu_lat_o = 2'h2;
								fpu_op_o = riscv_defines_C_FPU_I2F_CMD;
							end
							5'h1e: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b1;
								alu_operator_o = riscv_defines_ALU_ADD;
								regfile_alu_we = 1'b1;
								reg_fp_d_o = 1'b1;
							end
							5'h1c:
								case (instr_rdata_i[14:12])
									3'h0: begin
										rega_used_o = 1'b1;
										regb_used_o = 1'b1;
										alu_operator_o = riscv_defines_ALU_ADD;
										regfile_alu_we = 1'b1;
										reg_fp_a_o = 1'b1;
									end
									3'h1: begin
										rega_used_o = 1'b1;
										regfile_alu_we = 1'b1;
										reg_fp_a_o = 1'b1;
										alu_operator_o = riscv_defines_ALU_FCLASS;
									end
									default: illegal_insn_o = 1'b1;
								endcase
							default: illegal_insn_o = 1'b1;
						endcase
					else if (instr_rdata_i[26:25] == 2'b01)
						case (instr_rdata_i[31:27])
							5'h08: begin
								rega_used_o = 1'b1;
								regb_used_o = 1'b0;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b1;
								reg_fp_d_o = 1'b1;
								alu_operator_o = riscv_defines_ALU_FKEEP;
							end
							default: illegal_insn_o = 1'b1;
						endcase
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_OP_FMADD:
				if (FPU == 1) begin
					fp_rnd_mode_o = instr_rdata_i[14:12];
					if (instr_rdata_i[26:25] == 2'b00) begin
						apu_type_o = APUTYPE_MAC;
						apu_lat_o = 2'h3;
						apu_op_o = 2'b00;
						fpu_op_o = riscv_defines_C_FPU_FMADD_CMD;
						if (FPU == 1) begin
							apu_en = 1'b1;
							alu_en_o = 1'b0;
							apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							reg_fp_a_o = 1'b1;
							reg_fp_b_o = 1'b1;
							reg_fp_c_o = 1'b1;
							reg_fp_d_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_S4;
						end
					end
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_OP_FMSUB:
				if (FPU == 1) begin
					fp_rnd_mode_o = instr_rdata_i[14:12];
					if (instr_rdata_i[26:25] == 2'b00) begin
						apu_type_o = APUTYPE_MAC;
						apu_lat_o = 2'h3;
						apu_op_o = 2'b01;
						fpu_op_o = riscv_defines_C_FPU_FMSUB_CMD;
						if (FPU == 1) begin
							apu_en = 1'b1;
							alu_en_o = 1'b0;
							apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							reg_fp_a_o = 1'b1;
							reg_fp_b_o = 1'b1;
							reg_fp_c_o = 1'b1;
							reg_fp_d_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_S4;
						end
					end
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_OP_FNMADD:
				if (FPU == 1) begin
					fp_rnd_mode_o = instr_rdata_i[14:12];
					if (instr_rdata_i[26:25] == 2'b00) begin
						apu_type_o = APUTYPE_MAC;
						apu_lat_o = 2'h3;
						apu_op_o = 2'b11;
						fpu_op_o = riscv_defines_C_FPU_FNMADD_CMD;
						if (FPU == 1) begin
							apu_en = 1'b1;
							alu_en_o = 1'b0;
							apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							reg_fp_a_o = 1'b1;
							reg_fp_b_o = 1'b1;
							reg_fp_c_o = 1'b1;
							reg_fp_d_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_S4;
						end
					end
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_OP_FNMSUB:
				if (FPU == 1) begin
					fp_rnd_mode_o = instr_rdata_i[14:12];
					if (instr_rdata_i[26:25] == 2'b00) begin
						apu_type_o = APUTYPE_MAC;
						apu_lat_o = 2'h3;
						apu_op_o = 2'b10;
						fpu_op_o = riscv_defines_C_FPU_FNMSUB_CMD;
						if (FPU == 1) begin
							apu_en = 1'b1;
							alu_en_o = 1'b0;
							apu_flags_src_o = apu_core_package_APU_FLAGS_FP;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							reg_fp_a_o = 1'b1;
							reg_fp_b_o = 1'b1;
							reg_fp_c_o = 1'b1;
							reg_fp_d_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_S4;
						end
					end
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_STORE_FP:
				if (FPU == 1)
					case (instr_rdata_i[14:12])
						3'b010, 3'b011: begin
							data_req = 1'b1;
							data_we_o = 1'b1;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							alu_operator_o = riscv_defines_ALU_ADD;
							reg_fp_b_o = 1'b1;
							imm_b_mux_sel_o = riscv_defines_IMMB_S;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							data_type_o = 2'b00;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_LOAD_FP:
				if (FPU == 1)
					case (instr_rdata_i[14:12])
						3'b010, 3'b011: begin
							data_req = 1'b1;
							regfile_mem_we = 1'b1;
							reg_fp_d_o = 1'b1;
							rega_used_o = 1'b1;
							data_type_o = 2'b00;
							alu_operator_o = riscv_defines_ALU_ADD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							imm_b_mux_sel_o = riscv_defines_IMMB_I;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_PULP_OP: begin
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				case (instr_rdata_i[13:12])
					2'b00: begin
						alu_en_o = 1'b0;
						mult_sel_subword_o = instr_rdata_i[30];
						mult_signed_mode_o = {2 {instr_rdata_i[31]}};
						mult_imm_mux_o = riscv_defines_MIMM_S3;
						regc_mux_o = riscv_defines_REGC_ZERO;
						mult_int_en_o = 1'b1;
						if (instr_rdata_i[14])
							mult_operator_o = riscv_defines_MUL_IR;
						else
							mult_operator_o = riscv_defines_MUL_I;
						if (apu_core_package_SHARED_INT_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
							apu_op_o = mult_operator_o;
							apu_type_o = APUTYPE_INT_MULT;
							apu_lat_o = 2'h1;
						end
					end
					2'b01: begin
						alu_en_o = 1'b0;
						mult_sel_subword_o = instr_rdata_i[30];
						mult_signed_mode_o = {2 {instr_rdata_i[31]}};
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						mult_imm_mux_o = riscv_defines_MIMM_S3;
						mult_int_en_o = 1'b1;
						if (instr_rdata_i[14])
							mult_operator_o = riscv_defines_MUL_IR;
						else
							mult_operator_o = riscv_defines_MUL_I;
						if (apu_core_package_SHARED_INT_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
							apu_op_o = mult_operator_o;
							apu_type_o = APUTYPE_INT_MULT;
							apu_lat_o = 2'h1;
						end
					end
					2'b10: begin
						case ({instr_rdata_i[31], instr_rdata_i[14]})
							2'b00: alu_operator_o = riscv_defines_ALU_ADD;
							2'b01: alu_operator_o = riscv_defines_ALU_ADDR;
							2'b10: alu_operator_o = riscv_defines_ALU_ADDU;
							2'b11: alu_operator_o = riscv_defines_ALU_ADDUR;
						endcase
						bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
						bmask_b_mux_o = riscv_defines_BMASK_B_S3;
						if (instr_rdata_i[30]) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGC_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
						end
					end
					2'b11: begin
						case ({instr_rdata_i[31], instr_rdata_i[14]})
							2'b00: alu_operator_o = riscv_defines_ALU_SUB;
							2'b01: alu_operator_o = riscv_defines_ALU_SUBR;
							2'b10: alu_operator_o = riscv_defines_ALU_SUBU;
							2'b11: alu_operator_o = riscv_defines_ALU_SUBUR;
						endcase
						bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
						bmask_b_mux_o = riscv_defines_BMASK_B_S3;
						if (instr_rdata_i[30]) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGC_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
						end
					end
				endcase
			end
			riscv_defines_OPCODE_VECOP: begin
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				imm_b_mux_sel_o = riscv_defines_IMMB_VS;
				if (instr_rdata_i[12]) begin
					alu_vec_mode_o = riscv_defines_VEC_MODE8;
					mult_operator_o = riscv_defines_MUL_DOT8;
				end
				else begin
					alu_vec_mode_o = riscv_defines_VEC_MODE16;
					mult_operator_o = riscv_defines_MUL_DOT16;
				end
				if (instr_rdata_i[14]) begin
					scalar_replication_o = 1'b1;
					if (instr_rdata_i[13])
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					else
						regb_used_o = 1'b1;
				end
				else
					regb_used_o = 1'b1;
				(* full_case, parallel_case *)
				case (instr_rdata_i[31:26])
					6'b000000: begin
						alu_operator_o = riscv_defines_ALU_ADD;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000010: begin
						alu_operator_o = riscv_defines_ALU_SUB;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000100: begin
						alu_operator_o = riscv_defines_ALU_ADD;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
						bmask_b_mux_o = riscv_defines_BMASK_B_ONE;
					end
					6'b000110: begin
						alu_operator_o = riscv_defines_ALU_ADDU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
						bmask_b_mux_o = riscv_defines_BMASK_B_ONE;
					end
					6'b001000: begin
						alu_operator_o = riscv_defines_ALU_MIN;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001010: begin
						alu_operator_o = riscv_defines_ALU_MINU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b001100: begin
						alu_operator_o = riscv_defines_ALU_MAX;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001110: begin
						alu_operator_o = riscv_defines_ALU_MAXU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010000: begin
						alu_operator_o = riscv_defines_ALU_SRL;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010010: begin
						alu_operator_o = riscv_defines_ALU_SRA;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010100: begin
						alu_operator_o = riscv_defines_ALU_SLL;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010110: begin
						alu_operator_o = riscv_defines_ALU_OR;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011000: begin
						alu_operator_o = riscv_defines_ALU_XOR;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011010: begin
						alu_operator_o = riscv_defines_ALU_AND;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011100: begin
						alu_operator_o = riscv_defines_ALU_ABS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b111010, 6'b111100, 6'b111110, 6'b110000: begin
						alu_operator_o = riscv_defines_ALU_SHUF;
						imm_b_mux_sel_o = riscv_defines_IMMB_SHUF;
						regb_used_o = 1'b1;
						scalar_replication_o = 1'b0;
					end
					6'b110010: begin
						alu_operator_o = riscv_defines_ALU_SHUF2;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						scalar_replication_o = 1'b0;
					end
					6'b110100: begin
						alu_operator_o = riscv_defines_ALU_PCKLO;
						regb_used_o = 1'b1;
					end
					6'b110110: begin
						alu_operator_o = riscv_defines_ALU_PCKHI;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
					end
					6'b111000: begin
						alu_operator_o = riscv_defines_ALU_PCKLO;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
					end
					6'b011110: alu_operator_o = riscv_defines_ALU_EXTS;
					6'b100100: alu_operator_o = riscv_defines_ALU_EXT;
					6'b101100: begin
						alu_operator_o = riscv_defines_ALU_INS;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
					end
					6'b100000: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b00;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b100010: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b01;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b100110: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b11;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101000: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b00;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101010: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b01;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101110: begin
						alu_en_o = 1'b0;
						mult_dot_en_o = 1'b1;
						mult_dot_signed_o = 2'b11;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						if (SHARED_DSP_MULT) begin
							mult_int_en_o = 1'b0;
							mult_dot_en_o = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b000001: begin
						alu_operator_o = riscv_defines_ALU_EQ;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000011: begin
						alu_operator_o = riscv_defines_ALU_NE;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000101: begin
						alu_operator_o = riscv_defines_ALU_GTS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000111: begin
						alu_operator_o = riscv_defines_ALU_GES;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001001: begin
						alu_operator_o = riscv_defines_ALU_LTS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001011: begin
						alu_operator_o = riscv_defines_ALU_LES;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001101: begin
						alu_operator_o = riscv_defines_ALU_GTU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b001111: begin
						alu_operator_o = riscv_defines_ALU_GEU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010001: begin
						alu_operator_o = riscv_defines_ALU_LTU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010011: begin
						alu_operator_o = riscv_defines_ALU_LEU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					default: illegal_insn_o = 1'b1;
				endcase
			end
			riscv_defines_OPCODE_SYSTEM:
				if (instr_rdata_i[14:12] == 3'b000)
					(* full_case, parallel_case *)
					case (instr_rdata_i[31:20])
						12'h000: ecall_insn_o = 1'b1;
						12'h001: ebrk_insn_o = 1'b1;
						12'h302: begin
							illegal_insn_o = (PULP_SECURE ? current_priv_lvl_i != 2'b11 : 1'b0);
							mret_insn_o = ~illegal_insn_o;
						end
						12'h002: uret_insn_o = (PULP_SECURE ? 1'b1 : 1'b0);
						12'h105: pipe_flush_o = 1'b1;
						default: illegal_insn_o = 1'b1;
					endcase
				else begin
					csr_access_o = 1'b1;
					regfile_alu_we = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					imm_a_mux_sel_o = riscv_defines_IMMA_Z;
					imm_b_mux_sel_o = riscv_defines_IMMB_I;
					if (instr_rdata_i[14] == 1'b1)
						alu_op_a_mux_sel_o = riscv_defines_OP_A_IMM;
					else begin
						rega_used_o = 1'b1;
						alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
					end
					(* full_case, parallel_case *)
					case (instr_rdata_i[13:12])
						2'b01: csr_op = riscv_defines_CSR_OP_WRITE;
						2'b10: csr_op = riscv_defines_CSR_OP_SET;
						2'b11: csr_op = riscv_defines_CSR_OP_CLEAR;
						default: csr_illegal = 1'b1;
					endcase
					if (instr_rdata_i[29:28] > current_priv_lvl_i)
						csr_illegal = 1'b1;
					if (~csr_illegal) begin
						if ((instr_rdata_i[31:20] == 12'h300) || (instr_rdata_i[31:20] == 12'h000))
							csr_status_o = 1'b1;
					end
					illegal_insn_o = csr_illegal;
				end
			riscv_defines_OPCODE_HWLOOP: begin
				hwloop_target_mux_sel_o = 1'b0;
				(* full_case, parallel_case *)
				case (instr_rdata_i[14:12])
					3'b000: begin
						hwloop_we[0] = 1'b1;
						hwloop_start_mux_sel_o = 1'b0;
					end
					3'b001: hwloop_we[1] = 1'b1;
					3'b010: begin
						hwloop_we[2] = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b1;
						rega_used_o = 1'b1;
					end
					3'b011: begin
						hwloop_we[2] = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b0;
					end
					3'b100: begin
						hwloop_we = 3'b111;
						hwloop_start_mux_sel_o = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b1;
						rega_used_o = 1'b1;
					end
					3'b101: begin
						hwloop_we = 3'b111;
						hwloop_target_mux_sel_o = 1'b1;
						hwloop_start_mux_sel_o = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b0;
					end
					default: illegal_insn_o = 1'b1;
				endcase
			end
			default: illegal_insn_o = 1'b1;
		endcase
		if (illegal_c_insn_i)
			illegal_insn_o = 1'b1;
		if (data_misaligned_i == 1'b1) begin
			alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
			alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
			imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
			regfile_alu_we = 1'b0;
			prepost_useincr_o = 1'b1;
			scalar_replication_o = 1'b0;
		end
		else if (mult_multicycle_i)
			alu_op_c_mux_sel_o = riscv_defines_OP_C_REGC_OR_FWD;
	end
	assign apu_en_o = (deassert_we_i ? 1'b0 : apu_en);
	assign regfile_mem_we_o = (deassert_we_i ? 1'b0 : regfile_mem_we);
	assign regfile_alu_we_o = (deassert_we_i ? 1'b0 : regfile_alu_we);
	assign data_req_o = (deassert_we_i ? 1'b0 : data_req);
	assign hwloop_we_o = (deassert_we_i ? 3'b000 : hwloop_we);
	assign csr_op_o = (deassert_we_i ? riscv_defines_CSR_OP_NONE : csr_op);
	assign jump_in_id_o = (deassert_we_i ? riscv_defines_BRANCH_NONE : jump_in_id);
	assign jump_in_dec_o = jump_in_id;
	initial _sv2v_0 = 0;
endmodule
module riscv_ex_stage (
	clk,
	rst_n,
	alu_operator_i,
	alu_operand_a_i,
	alu_operand_b_i,
	alu_operand_c_i,
	alu_en_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	alu_vec_mode_i,
	mult_operator_i,
	mult_operand_a_i,
	mult_operand_b_i,
	mult_operand_c_i,
	mult_en_i,
	mult_sel_subword_i,
	mult_signed_mode_i,
	mult_imm_i,
	mult_dot_op_a_i,
	mult_dot_op_b_i,
	mult_dot_op_c_i,
	mult_dot_signed_i,
	mult_multicycle_o,
	fpu_op_i,
	fpu_prec_i,
	fpu_fflags_o,
	fpu_fflags_we_o,
	apu_en_i,
	apu_op_i,
	apu_lat_i,
	apu_operands_i,
	apu_waddr_i,
	apu_flags_i,
	apu_read_regs_i,
	apu_read_regs_valid_i,
	apu_read_dep_o,
	apu_write_regs_i,
	apu_write_regs_valid_i,
	apu_write_dep_o,
	apu_perf_type_o,
	apu_perf_cont_o,
	apu_perf_wb_o,
	apu_busy_o,
	apu_ready_wb_o,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_operands_o,
	apu_master_op_o,
	apu_master_valid_i,
	apu_master_result_i,
	lsu_en_i,
	lsu_rdata_i,
	branch_in_ex_i,
	regfile_alu_waddr_i,
	regfile_alu_we_i,
	regfile_we_i,
	regfile_waddr_i,
	csr_access_i,
	csr_rdata_i,
	regfile_waddr_wb_o,
	regfile_we_wb_o,
	regfile_wdata_wb_o,
	regfile_alu_waddr_fw_o,
	regfile_alu_we_fw_o,
	regfile_alu_wdata_fw_o,
	jump_target_o,
	branch_decision_o,
	lsu_ready_ex_i,
	ex_ready_o,
	ex_valid_o,
	wb_ready_i
);
	reg _sv2v_0;
	parameter FPU = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk;
	input wire rst_n;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	input wire [6:0] alu_operator_i;
	input wire [31:0] alu_operand_a_i;
	input wire [31:0] alu_operand_b_i;
	input wire [31:0] alu_operand_c_i;
	input wire alu_en_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	input wire [1:0] alu_vec_mode_i;
	input wire [2:0] mult_operator_i;
	input wire [31:0] mult_operand_a_i;
	input wire [31:0] mult_operand_b_i;
	input wire [31:0] mult_operand_c_i;
	input wire mult_en_i;
	input wire mult_sel_subword_i;
	input wire [1:0] mult_signed_mode_i;
	input wire [4:0] mult_imm_i;
	input wire [31:0] mult_dot_op_a_i;
	input wire [31:0] mult_dot_op_b_i;
	input wire [31:0] mult_dot_op_c_i;
	input wire [1:0] mult_dot_signed_i;
	output wire mult_multicycle_o;
	localparam riscv_defines_C_CMD = 4;
	input wire [3:0] fpu_op_i;
	localparam riscv_defines_C_PC = 5;
	input wire [4:0] fpu_prec_i;
	localparam riscv_defines_C_FFLAG = 5;
	output wire [4:0] fpu_fflags_o;
	output wire fpu_fflags_we_o;
	input wire apu_en_i;
	input wire [APU_WOP_CPU - 1:0] apu_op_i;
	input wire [1:0] apu_lat_i;
	input wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_i;
	input wire [5:0] apu_waddr_i;
	input wire [APU_NDSFLAGS_CPU - 1:0] apu_flags_i;
	input wire [17:0] apu_read_regs_i;
	input wire [2:0] apu_read_regs_valid_i;
	output wire apu_read_dep_o;
	input wire [11:0] apu_write_regs_i;
	input wire [1:0] apu_write_regs_valid_i;
	output wire apu_write_dep_o;
	output wire apu_perf_type_o;
	output wire apu_perf_cont_o;
	output wire apu_perf_wb_o;
	output wire apu_busy_o;
	output wire apu_ready_wb_o;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	output wire [(APU_NARGS_CPU * 32) - 1:0] apu_master_operands_o;
	output wire [APU_WOP_CPU - 1:0] apu_master_op_o;
	input wire apu_master_valid_i;
	input wire [31:0] apu_master_result_i;
	input wire lsu_en_i;
	input wire [31:0] lsu_rdata_i;
	input wire branch_in_ex_i;
	input wire [5:0] regfile_alu_waddr_i;
	input wire regfile_alu_we_i;
	input wire regfile_we_i;
	input wire [5:0] regfile_waddr_i;
	input wire csr_access_i;
	input wire [31:0] csr_rdata_i;
	output reg [5:0] regfile_waddr_wb_o;
	output reg regfile_we_wb_o;
	output reg [31:0] regfile_wdata_wb_o;
	output reg [5:0] regfile_alu_waddr_fw_o;
	output reg regfile_alu_we_fw_o;
	output reg [31:0] regfile_alu_wdata_fw_o;
	output wire [31:0] jump_target_o;
	output wire branch_decision_o;
	input wire lsu_ready_ex_i;
	output wire ex_ready_o;
	output wire ex_valid_o;
	input wire wb_ready_i;
	wire [31:0] alu_result;
	wire [31:0] mult_result;
	wire alu_cmp_result;
	reg regfile_we_lsu;
	reg [5:0] regfile_waddr_lsu;
	reg wb_contention;
	reg wb_contention_lsu;
	wire alu_ready;
	wire mult_ready;
	wire fpu_busy;
	wire apu_valid;
	wire [5:0] apu_waddr;
	wire [31:0] apu_result;
	wire apu_stall;
	wire apu_active;
	wire apu_singlecycle;
	wire apu_multicycle;
	wire apu_req;
	wire apu_ready;
	wire apu_gnt;
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_alu_wdata_fw_o = 1'sb0;
		regfile_alu_waddr_fw_o = 1'sb0;
		regfile_alu_we_fw_o = 1'sb0;
		wb_contention = 1'b0;
		if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
			regfile_alu_we_fw_o = 1'b1;
			regfile_alu_waddr_fw_o = apu_waddr;
			regfile_alu_wdata_fw_o = apu_result;
			if (regfile_alu_we_i & ~apu_en_i)
				wb_contention = 1'b1;
		end
		else begin
			regfile_alu_we_fw_o = regfile_alu_we_i & ~apu_en_i;
			regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
			if (alu_en_i)
				regfile_alu_wdata_fw_o = alu_result;
			if (mult_en_i)
				regfile_alu_wdata_fw_o = mult_result;
			if (csr_access_i)
				regfile_alu_wdata_fw_o = csr_rdata_i;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_we_wb_o = 1'b0;
		regfile_waddr_wb_o = regfile_waddr_lsu;
		regfile_wdata_wb_o = lsu_rdata_i;
		wb_contention_lsu = 1'b0;
		if (regfile_we_lsu) begin
			regfile_we_wb_o = wb_ready_i;
			if (apu_valid & (!apu_singlecycle & !apu_multicycle))
				wb_contention_lsu = 1'b1;
		end
		else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
			regfile_we_wb_o = 1'b1;
			regfile_waddr_wb_o = apu_waddr;
			regfile_wdata_wb_o = apu_result;
		end
	end
	assign branch_decision_o = alu_cmp_result;
	assign jump_target_o = alu_operand_c_i;
	riscv_alu #(
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.FPU(FPU)
	) alu_i(
		.clk(clk),
		.rst_n(rst_n),
		.operator_i(alu_operator_i),
		.operand_a_i(alu_operand_a_i),
		.operand_b_i(alu_operand_b_i),
		.operand_c_i(alu_operand_c_i),
		.vector_mode_i(alu_vec_mode_i),
		.bmask_a_i(bmask_a_i),
		.bmask_b_i(bmask_b_i),
		.imm_vec_ext_i(imm_vec_ext_i),
		.result_o(alu_result),
		.comparison_result_o(alu_cmp_result),
		.ready_o(alu_ready),
		.ex_ready_i(ex_ready_o)
	);
	riscv_mult #(.SHARED_DSP_MULT(SHARED_DSP_MULT)) mult_i(
		.clk(clk),
		.rst_n(rst_n),
		.enable_i(mult_en_i),
		.operator_i(mult_operator_i),
		.short_subword_i(mult_sel_subword_i),
		.short_signed_i(mult_signed_mode_i),
		.op_a_i(mult_operand_a_i),
		.op_b_i(mult_operand_b_i),
		.op_c_i(mult_operand_c_i),
		.imm_i(mult_imm_i),
		.dot_op_a_i(mult_dot_op_a_i),
		.dot_op_b_i(mult_dot_op_b_i),
		.dot_op_c_i(mult_dot_op_c_i),
		.dot_signed_i(mult_dot_signed_i),
		.result_o(mult_result),
		.multicycle_o(mult_multicycle_o),
		.ready_o(mult_ready),
		.ex_ready_i(ex_ready_o)
	);
	generate
		if (FPU == 1) begin : genblk1
			riscv_apu_disp apu_disp_i(
				.clk_i(clk),
				.rst_ni(rst_n),
				.enable_i(apu_en_i),
				.apu_lat_i(apu_lat_i),
				.apu_waddr_i(apu_waddr_i),
				.apu_waddr_o(apu_waddr),
				.apu_multicycle_o(apu_multicycle),
				.apu_singlecycle_o(apu_singlecycle),
				.active_o(apu_active),
				.stall_o(apu_stall),
				.read_regs_i(apu_read_regs_i),
				.read_regs_valid_i(apu_read_regs_valid_i),
				.read_dep_o(apu_read_dep_o),
				.write_regs_i(apu_write_regs_i),
				.write_regs_valid_i(apu_write_regs_valid_i),
				.write_dep_o(apu_write_dep_o),
				.perf_type_o(apu_perf_type_o),
				.perf_cont_o(apu_perf_cont_o),
				.apu_master_req_o(apu_req),
				.apu_master_ready_o(apu_ready),
				.apu_master_gnt_i(apu_gnt),
				.apu_master_valid_i(apu_valid)
			);
			assign apu_perf_wb_o = wb_contention | wb_contention_lsu;
			assign apu_ready_wb_o = ~((apu_active | apu_en_i) | apu_stall) | apu_valid;
			if (SHARED_FP == 1) begin : genblk1
				assign apu_master_req_o = apu_req;
				assign apu_master_ready_o = apu_ready;
				assign apu_gnt = apu_master_gnt_i;
				assign apu_valid = apu_master_valid_i;
				assign apu_master_operands_o = apu_operands_i;
				assign apu_master_op_o = apu_op_i;
				assign apu_result = apu_master_result_i;
				assign fpu_fflags_we_o = apu_valid;
			end
			else begin : genblk1
				fpu_private fpu_i(
					.clk_i(clk),
					.rst_ni(rst_n),
					.fpu_en_i(apu_req),
					.operand_a_i(apu_operands_i[0+:32]),
					.operand_b_i(apu_operands_i[32+:32]),
					.operand_c_i(apu_operands_i[64+:32]),
					.rm_i(apu_flags_i[2:0]),
					.fpu_op_i(fpu_op_i),
					.prec_i(fpu_prec_i),
					.result_o(apu_result),
					.valid_o(apu_valid),
					.flags_o(fpu_fflags_o),
					.divsqrt_busy_o(fpu_busy)
				);
				assign fpu_fflags_we_o = apu_valid;
				assign apu_master_req_o = 1'sb0;
				assign apu_master_ready_o = 1'b1;
				assign apu_master_operands_o[0+:32] = 1'sb0;
				assign apu_master_operands_o[32+:32] = 1'sb0;
				assign apu_master_operands_o[64+:32] = 1'sb0;
				assign apu_master_op_o = 1'sb0;
				assign apu_gnt = 1'b1;
			end
		end
		else begin : genblk1
			assign apu_master_req_o = 1'sb0;
			assign apu_master_ready_o = 1'b1;
			assign apu_master_operands_o[0+:32] = 1'sb0;
			assign apu_master_operands_o[32+:32] = 1'sb0;
			assign apu_master_operands_o[64+:32] = 1'sb0;
			assign apu_master_op_o = 1'sb0;
			assign apu_valid = 1'b0;
			assign apu_waddr = 6'b000000;
			assign apu_stall = 1'b0;
			assign apu_active = 1'b0;
			assign apu_ready_wb_o = 1'b1;
			assign apu_perf_wb_o = 1'b0;
			assign apu_perf_cont_o = 1'b0;
			assign apu_perf_type_o = 1'b0;
			assign apu_singlecycle = 1'b0;
			assign apu_multicycle = 1'b0;
			assign apu_read_dep_o = 1'b0;
			assign apu_write_dep_o = 1'b0;
			assign fpu_fflags_we_o = 1'b0;
			assign fpu_fflags_o = 1'sb0;
		end
	endgenerate
	assign apu_busy_o = apu_active;
	always @(posedge clk or negedge rst_n) begin : EX_WB_Pipeline_Register
		if (~rst_n) begin
			regfile_waddr_lsu <= 1'sb0;
			regfile_we_lsu <= 1'b0;
		end
		else if (ex_valid_o) begin
			regfile_we_lsu <= regfile_we_i;
			if (regfile_we_i)
				regfile_waddr_lsu <= regfile_waddr_i;
		end
		else if (wb_ready_i)
			regfile_we_lsu <= 1'b0;
	end
	assign ex_ready_o = (((((~apu_stall & alu_ready) & mult_ready) & lsu_ready_ex_i) & wb_ready_i) & ~wb_contention) | branch_in_ex_i;
	assign ex_valid_o = ((((apu_valid | alu_en_i) | mult_en_i) | csr_access_i) | lsu_en_i) & (((alu_ready & mult_ready) & lsu_ready_ex_i) & wb_ready_i);
	initial _sv2v_0 = 0;
endmodule
module riscv_fetch_fifo (
	clk,
	rst_n,
	clear_i,
	in_addr_i,
	in_rdata_i,
	in_valid_i,
	in_ready_o,
	in_replace2_i,
	in_is_hwlp_i,
	out_valid_o,
	out_ready_i,
	out_rdata_o,
	out_addr_o,
	unaligned_is_compressed_o,
	out_valid_stored_o,
	out_is_hwlp_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire clear_i;
	input wire [31:0] in_addr_i;
	input wire [31:0] in_rdata_i;
	input wire in_valid_i;
	output wire in_ready_o;
	input wire in_replace2_i;
	input wire in_is_hwlp_i;
	output reg out_valid_o;
	input wire out_ready_i;
	output reg [31:0] out_rdata_o;
	output wire [31:0] out_addr_o;
	output wire unaligned_is_compressed_o;
	output reg out_valid_stored_o;
	output wire out_is_hwlp_o;
	localparam DEPTH = 4;
	reg [127:0] addr_n;
	reg [127:0] addr_int;
	reg [127:0] addr_Q;
	reg [127:0] rdata_n;
	reg [127:0] rdata_int;
	reg [127:0] rdata_Q;
	reg [0:3] valid_n;
	reg [0:3] valid_int;
	reg [0:3] valid_Q;
	reg [0:1] is_hwlp_n;
	reg [0:1] is_hwlp_int;
	reg [0:1] is_hwlp_Q;
	wire [31:0] addr_next;
	wire [31:0] rdata;
	wire [31:0] rdata_unaligned;
	wire valid;
	wire valid_unaligned;
	wire aligned_is_compressed;
	wire unaligned_is_compressed;
	wire aligned_is_compressed_st;
	wire unaligned_is_compressed_st;
	assign rdata = (valid_Q[0] ? rdata_Q[96+:32] : in_rdata_i);
	assign valid = (valid_Q[0] || in_valid_i) || is_hwlp_Q[1];
	assign rdata_unaligned = (valid_Q[1] ? {rdata_Q[79-:16], rdata[31:16]} : {in_rdata_i[15:0], rdata[31:16]});
	assign valid_unaligned = valid_Q[1] || (valid_Q[0] && in_valid_i);
	assign unaligned_is_compressed_o = unaligned_is_compressed;
	assign unaligned_is_compressed = rdata[17:16] != 2'b11;
	assign aligned_is_compressed = rdata[1:0] != 2'b11;
	assign unaligned_is_compressed_st = rdata_Q[113-:2] != 2'b11;
	assign aligned_is_compressed_st = rdata_Q[97-:2] != 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		if (out_addr_o[1] && ~is_hwlp_Q[1]) begin
			out_rdata_o = rdata_unaligned;
			if (unaligned_is_compressed)
				out_valid_o = valid;
			else
				out_valid_o = valid_unaligned;
		end
		else begin
			out_rdata_o = rdata;
			out_valid_o = valid;
		end
	end
	assign out_addr_o = (valid_Q[0] ? addr_Q[96+:32] : in_addr_i);
	assign out_is_hwlp_o = (valid_Q[0] ? is_hwlp_Q[0] : in_is_hwlp_i);
	always @(*) begin
		if (_sv2v_0)
			;
		out_valid_stored_o = 1'b1;
		if (out_addr_o[1] && ~is_hwlp_Q[1]) begin
			if (unaligned_is_compressed_st)
				out_valid_stored_o = 1'b1;
			else
				out_valid_stored_o = valid_Q[1];
		end
		else
			out_valid_stored_o = valid_Q[0];
	end
	assign in_ready_o = ~valid_Q[2];
	always @(*) begin : sv2v_autoblock_1
		reg [1:0] _sv2v_jump;
		_sv2v_jump = 2'b00;
		if (_sv2v_0)
			;
		addr_int = addr_Q;
		rdata_int = rdata_Q;
		valid_int = valid_Q;
		is_hwlp_int = is_hwlp_Q;
		if (in_valid_i) begin
			begin : sv2v_autoblock_2
				reg signed [31:0] j;
				begin : sv2v_autoblock_3
					reg signed [31:0] _sv2v_value_on_break;
					for (j = 0; j < DEPTH; j = j + 1)
						if (_sv2v_jump < 2'b10) begin
							_sv2v_jump = 2'b00;
							if (~valid_Q[j]) begin
								addr_int[(3 - j) * 32+:32] = in_addr_i;
								rdata_int[(3 - j) * 32+:32] = in_rdata_i;
								valid_int[j] = 1'b1;
								_sv2v_jump = 2'b10;
							end
							_sv2v_value_on_break = j;
						end
					if (!(_sv2v_jump < 2'b10))
						j = _sv2v_value_on_break;
					if (_sv2v_jump != 2'b11)
						_sv2v_jump = 2'b00;
				end
			end
			if (_sv2v_jump == 2'b00) begin
				if (in_replace2_i) begin
					if (valid_Q[0]) begin
						addr_int[64+:32] = in_addr_i;
						rdata_int[96+:32] = out_rdata_o;
						rdata_int[64+:32] = in_rdata_i;
						valid_int[1] = 1'b1;
						valid_int[2:3] = 1'sb0;
						is_hwlp_int[1] = in_is_hwlp_i;
					end
					else
						is_hwlp_int[0] = in_is_hwlp_i;
				end
			end
		end
	end
	assign addr_next = {addr_int[127-:30], 2'b00} + 32'h00000004;
	always @(*) begin
		if (_sv2v_0)
			;
		addr_n = addr_int;
		rdata_n = rdata_int;
		valid_n = valid_int;
		is_hwlp_n = is_hwlp_int;
		if (out_ready_i && out_valid_o) begin
			is_hwlp_n = {is_hwlp_int[1], 1'b0};
			if (is_hwlp_int[1]) begin
				addr_n[96+:32] = addr_int[95-:32];
				begin : sv2v_autoblock_4
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
			else if (addr_int[97]) begin
				if (unaligned_is_compressed)
					addr_n[96+:32] = {addr_next[31:2], 2'b00};
				else
					addr_n[96+:32] = {addr_next[31:2], 2'b10};
				begin : sv2v_autoblock_5
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
			else if (aligned_is_compressed)
				addr_n[96+:32] = {addr_int[127-:30], 2'b10};
			else begin
				addr_n[96+:32] = {addr_next[31:2], 2'b00};
				begin : sv2v_autoblock_6
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			addr_Q <= {DEPTH {32'b00000000000000000000000000000000}};
			rdata_Q <= {DEPTH {32'b00000000000000000000000000000000}};
			valid_Q <= 1'sb0;
			is_hwlp_Q <= 1'sb0;
		end
		else if (clear_i) begin
			valid_Q <= 1'sb0;
			is_hwlp_Q <= 1'sb0;
		end
		else begin
			addr_Q <= addr_n;
			rdata_Q <= rdata_n;
			valid_Q <= valid_n;
			is_hwlp_Q <= is_hwlp_n;
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_hwloop_controller (
	current_pc_i,
	hwlp_start_addr_i,
	hwlp_end_addr_i,
	hwlp_counter_i,
	hwlp_dec_cnt_o,
	hwlp_dec_cnt_id_i,
	hwlp_jump_o,
	hwlp_targ_addr_o
);
	reg _sv2v_0;
	parameter N_REGS = 2;
	input wire [31:0] current_pc_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_start_addr_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_end_addr_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_counter_i;
	output reg [N_REGS - 1:0] hwlp_dec_cnt_o;
	input wire [N_REGS - 1:0] hwlp_dec_cnt_id_i;
	output wire hwlp_jump_o;
	output reg [31:0] hwlp_targ_addr_o;
	reg [N_REGS - 1:0] pc_is_end_addr;
	integer j;
	genvar _gv_i_18;
	generate
		for (_gv_i_18 = 0; _gv_i_18 < N_REGS; _gv_i_18 = _gv_i_18 + 1) begin : genblk1
			localparam i = _gv_i_18;
			always @(*) begin
				pc_is_end_addr[i] = 1'b0;
				if (current_pc_i == hwlp_end_addr_i[i * 32+:32]) begin
					if (hwlp_counter_i[(i * 32) + 31-:30] != 30'h00000000)
						pc_is_end_addr[i] = 1'b1;
					else
						case (hwlp_counter_i[(i * 32) + 1-:2])
							2'b11: pc_is_end_addr[i] = 1'b1;
							2'b10: pc_is_end_addr[i] = ~hwlp_dec_cnt_id_i[i];
							2'b01, 2'b00: pc_is_end_addr[i] = 1'b0;
						endcase
				end
			end
		end
	endgenerate
	always @(*) begin : sv2v_autoblock_1
		reg [1:0] _sv2v_jump;
		_sv2v_jump = 2'b00;
		if (_sv2v_0)
			;
		hwlp_targ_addr_o = 1'sb0;
		hwlp_dec_cnt_o = 1'sb0;
		begin : sv2v_autoblock_2
			integer _sv2v_value_on_break;
			for (j = 0; j < N_REGS; j = j + 1)
				if (_sv2v_jump < 2'b10) begin
					_sv2v_jump = 2'b00;
					if (pc_is_end_addr[j]) begin
						hwlp_targ_addr_o = hwlp_start_addr_i[j * 32+:32];
						hwlp_dec_cnt_o[j] = 1'b1;
						_sv2v_jump = 2'b10;
					end
					_sv2v_value_on_break = j;
				end
			if (!(_sv2v_jump < 2'b10))
				j = _sv2v_value_on_break;
			if (_sv2v_jump != 2'b11)
				_sv2v_jump = 2'b00;
		end
	end
	assign hwlp_jump_o = |pc_is_end_addr;
	initial _sv2v_0 = 0;
endmodule
module riscv_hwloop_regs (
	clk,
	rst_n,
	hwlp_start_data_i,
	hwlp_end_data_i,
	hwlp_cnt_data_i,
	hwlp_we_i,
	hwlp_regid_i,
	valid_i,
	hwlp_dec_cnt_i,
	hwlp_start_addr_o,
	hwlp_end_addr_o,
	hwlp_counter_o
);
	parameter N_REGS = 2;
	parameter N_REG_BITS = $clog2(N_REGS);
	input wire clk;
	input wire rst_n;
	input wire [31:0] hwlp_start_data_i;
	input wire [31:0] hwlp_end_data_i;
	input wire [31:0] hwlp_cnt_data_i;
	input wire [2:0] hwlp_we_i;
	input wire [N_REG_BITS - 1:0] hwlp_regid_i;
	input wire valid_i;
	input wire [N_REGS - 1:0] hwlp_dec_cnt_i;
	output wire [(N_REGS * 32) - 1:0] hwlp_start_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_end_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_counter_o;
	reg [(N_REGS * 32) - 1:0] hwlp_start_q;
	reg [(N_REGS * 32) - 1:0] hwlp_end_q;
	reg [(N_REGS * 32) - 1:0] hwlp_counter_q;
	wire [(N_REGS * 32) - 1:0] hwlp_counter_n;
	reg [31:0] i;
	assign hwlp_start_addr_o = hwlp_start_q;
	assign hwlp_end_addr_o = hwlp_end_q;
	assign hwlp_counter_o = hwlp_counter_q;
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_START
		if (rst_n == 1'b0)
			hwlp_start_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[0] == 1'b1)
			hwlp_start_q[hwlp_regid_i * 32+:32] <= hwlp_start_data_i;
	end
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_END
		if (rst_n == 1'b0)
			hwlp_end_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[1] == 1'b1)
			hwlp_end_q[hwlp_regid_i * 32+:32] <= hwlp_end_data_i;
	end
	genvar _gv_k_11;
	generate
		for (_gv_k_11 = 0; _gv_k_11 < N_REGS; _gv_k_11 = _gv_k_11 + 1) begin : genblk1
			localparam k = _gv_k_11;
			assign hwlp_counter_n[k * 32+:32] = hwlp_counter_q[k * 32+:32] - 1;
		end
	endgenerate
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_COUNTER
		if (rst_n == 1'b0)
			hwlp_counter_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else
			for (i = 0; i < N_REGS; i = i + 1)
				if ((hwlp_we_i[2] == 1'b1) && (i == hwlp_regid_i))
					hwlp_counter_q[i * 32+:32] <= hwlp_cnt_data_i;
				else if (hwlp_dec_cnt_i[i] && valid_i)
					hwlp_counter_q[i * 32+:32] <= hwlp_counter_n[i * 32+:32];
	end
endmodule
module riscv_id_stage (
	clk,
	rst_n,
	test_en_i,
	fregfile_disable_i,
	fetch_enable_i,
	ctrl_busy_o,
	core_ctrl_firstfetch_o,
	is_decoding_o,
	hwlp_dec_cnt_i,
	is_hwlp_i,
	instr_valid_i,
	instr_rdata_i,
	instr_req_o,
	branch_in_ex_o,
	branch_decision_i,
	jump_target_o,
	clear_instr_valid_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	illegal_c_insn_i,
	is_compressed_i,
	pc_if_i,
	pc_id_i,
	halt_if_o,
	id_ready_o,
	ex_ready_i,
	wb_ready_i,
	id_valid_o,
	ex_valid_i,
	pc_ex_o,
	alu_operand_a_ex_o,
	alu_operand_b_ex_o,
	alu_operand_c_ex_o,
	bmask_a_ex_o,
	bmask_b_ex_o,
	imm_vec_ext_ex_o,
	alu_vec_mode_ex_o,
	regfile_waddr_ex_o,
	regfile_we_ex_o,
	regfile_alu_waddr_ex_o,
	regfile_alu_we_ex_o,
	alu_en_ex_o,
	alu_operator_ex_o,
	mult_operator_ex_o,
	mult_operand_a_ex_o,
	mult_operand_b_ex_o,
	mult_operand_c_ex_o,
	mult_en_ex_o,
	mult_sel_subword_ex_o,
	mult_signed_mode_ex_o,
	mult_imm_ex_o,
	mult_dot_op_a_ex_o,
	mult_dot_op_b_ex_o,
	mult_dot_op_c_ex_o,
	mult_dot_signed_ex_o,
	fpu_op_ex_o,
	apu_en_ex_o,
	apu_type_ex_o,
	apu_op_ex_o,
	apu_lat_ex_o,
	apu_operands_ex_o,
	apu_flags_ex_o,
	apu_waddr_ex_o,
	apu_read_regs_o,
	apu_read_regs_valid_o,
	apu_read_dep_i,
	apu_write_regs_o,
	apu_write_regs_valid_o,
	apu_write_dep_i,
	apu_perf_dep_o,
	apu_busy_i,
	frm_i,
	csr_access_ex_o,
	csr_op_ex_o,
	current_priv_lvl_i,
	csr_irq_sec_o,
	csr_cause_o,
	csr_save_if_o,
	csr_save_id_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_save_cause_o,
	hwlp_start_o,
	hwlp_end_o,
	hwlp_cnt_o,
	csr_hwlp_regid_i,
	csr_hwlp_we_i,
	csr_hwlp_data_i,
	data_req_ex_o,
	data_we_ex_o,
	data_type_ex_o,
	data_sign_ext_ex_o,
	data_reg_offset_ex_o,
	data_load_event_ex_o,
	data_misaligned_ex_o,
	prepost_useincr_ex_o,
	data_misaligned_i,
	irq_i,
	irq_sec_i,
	irq_id_i,
	m_irq_enable_i,
	u_irq_enable_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	lsu_load_err_i,
	lsu_store_err_i,
	dbg_settings_i,
	dbg_req_i,
	dbg_ack_o,
	dbg_stall_i,
	dbg_trap_o,
	dbg_reg_rreq_i,
	dbg_reg_raddr_i,
	dbg_reg_rdata_o,
	dbg_reg_wreq_i,
	dbg_reg_waddr_i,
	dbg_reg_wdata_i,
	dbg_jump_req_i,
	regfile_waddr_wb_i,
	regfile_we_wb_i,
	regfile_wdata_wb_i,
	regfile_alu_waddr_fw_i,
	regfile_alu_we_fw_i,
	regfile_alu_wdata_fw_i,
	mult_multicycle_i,
	perf_jump_o,
	perf_jr_stall_o,
	perf_ld_stall_o
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter N_HWLP_BITS = $clog2(N_HWLP);
	parameter PULP_SECURE = 0;
	parameter FPU = 0;
	parameter APU = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire fregfile_disable_i;
	input wire fetch_enable_i;
	output wire ctrl_busy_o;
	output wire core_ctrl_firstfetch_o;
	output wire is_decoding_o;
	input wire [N_HWLP - 1:0] hwlp_dec_cnt_i;
	input wire is_hwlp_i;
	input wire instr_valid_i;
	input wire [31:0] instr_rdata_i;
	output wire instr_req_o;
	output reg branch_in_ex_o;
	input wire branch_decision_i;
	output wire [31:0] jump_target_o;
	output wire clear_instr_valid_o;
	output wire pc_set_o;
	output wire [2:0] pc_mux_o;
	output wire [1:0] exc_pc_mux_o;
	output wire trap_addr_mux_o;
	input wire illegal_c_insn_i;
	input wire is_compressed_i;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	output wire halt_if_o;
	output wire id_ready_o;
	input wire ex_ready_i;
	input wire wb_ready_i;
	output wire id_valid_o;
	input wire ex_valid_i;
	output reg [31:0] pc_ex_o;
	output reg [31:0] alu_operand_a_ex_o;
	output reg [31:0] alu_operand_b_ex_o;
	output reg [31:0] alu_operand_c_ex_o;
	output reg [4:0] bmask_a_ex_o;
	output reg [4:0] bmask_b_ex_o;
	output reg [1:0] imm_vec_ext_ex_o;
	output reg [1:0] alu_vec_mode_ex_o;
	output reg [5:0] regfile_waddr_ex_o;
	output reg regfile_we_ex_o;
	output reg [5:0] regfile_alu_waddr_ex_o;
	output reg regfile_alu_we_ex_o;
	output reg alu_en_ex_o;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_ex_o;
	output reg [2:0] mult_operator_ex_o;
	output reg [31:0] mult_operand_a_ex_o;
	output reg [31:0] mult_operand_b_ex_o;
	output reg [31:0] mult_operand_c_ex_o;
	output reg mult_en_ex_o;
	output reg mult_sel_subword_ex_o;
	output reg [1:0] mult_signed_mode_ex_o;
	output reg [4:0] mult_imm_ex_o;
	output reg [31:0] mult_dot_op_a_ex_o;
	output reg [31:0] mult_dot_op_b_ex_o;
	output reg [31:0] mult_dot_op_c_ex_o;
	output reg [1:0] mult_dot_signed_ex_o;
	localparam riscv_defines_C_CMD = 4;
	output reg [3:0] fpu_op_ex_o;
	output reg apu_en_ex_o;
	output reg [WAPUTYPE - 1:0] apu_type_ex_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_ex_o;
	output reg [1:0] apu_lat_ex_o;
	output reg [(APU_NARGS_CPU * 32) - 1:0] apu_operands_ex_o;
	output reg [APU_NDSFLAGS_CPU - 1:0] apu_flags_ex_o;
	output reg [5:0] apu_waddr_ex_o;
	output wire [17:0] apu_read_regs_o;
	output wire [2:0] apu_read_regs_valid_o;
	input wire apu_read_dep_i;
	output wire [11:0] apu_write_regs_o;
	output wire [1:0] apu_write_regs_valid_o;
	input wire apu_write_dep_i;
	output wire apu_perf_dep_o;
	input wire apu_busy_i;
	localparam riscv_defines_C_RM = 3;
	input wire [2:0] frm_i;
	output reg csr_access_ex_o;
	output reg [1:0] csr_op_ex_o;
	input wire [1:0] current_priv_lvl_i;
	output wire csr_irq_sec_o;
	output wire [5:0] csr_cause_o;
	output wire csr_save_if_o;
	output wire csr_save_id_o;
	output wire csr_restore_mret_id_o;
	output wire csr_restore_uret_id_o;
	output wire csr_save_cause_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_start_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_end_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_cnt_o;
	input wire [N_HWLP_BITS - 1:0] csr_hwlp_regid_i;
	input wire [2:0] csr_hwlp_we_i;
	input wire [31:0] csr_hwlp_data_i;
	output reg data_req_ex_o;
	output reg data_we_ex_o;
	output reg [1:0] data_type_ex_o;
	output reg data_sign_ext_ex_o;
	output reg [1:0] data_reg_offset_ex_o;
	output reg data_load_event_ex_o;
	output reg data_misaligned_ex_o;
	output reg prepost_useincr_ex_o;
	input wire data_misaligned_i;
	input wire irq_i;
	input wire irq_sec_i;
	input wire [4:0] irq_id_i;
	input wire m_irq_enable_i;
	input wire u_irq_enable_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	output wire [5:0] exc_cause_o;
	input wire lsu_load_err_i;
	input wire lsu_store_err_i;
	localparam riscv_defines_DBG_SETS_W = 6;
	input wire [5:0] dbg_settings_i;
	input wire dbg_req_i;
	output wire dbg_ack_o;
	input wire dbg_stall_i;
	output wire dbg_trap_o;
	input wire dbg_reg_rreq_i;
	input wire [5:0] dbg_reg_raddr_i;
	output wire [31:0] dbg_reg_rdata_o;
	input wire dbg_reg_wreq_i;
	input wire [5:0] dbg_reg_waddr_i;
	input wire [31:0] dbg_reg_wdata_i;
	input wire dbg_jump_req_i;
	input wire [5:0] regfile_waddr_wb_i;
	input wire regfile_we_wb_i;
	input wire [31:0] regfile_wdata_wb_i;
	input wire [5:0] regfile_alu_waddr_fw_i;
	input wire regfile_alu_we_fw_i;
	input wire [31:0] regfile_alu_wdata_fw_i;
	input wire mult_multicycle_i;
	output wire perf_jump_o;
	output wire perf_jr_stall_o;
	output wire perf_ld_stall_o;
	wire [31:0] instr;
	wire deassert_we;
	wire illegal_insn_dec;
	wire ebrk_insn;
	wire mret_insn_dec;
	wire uret_insn_dec;
	wire ecall_insn_dec;
	wire pipe_flush_dec;
	wire rega_used_dec;
	wire regb_used_dec;
	wire regc_used_dec;
	wire branch_taken_ex;
	wire [1:0] jump_in_id;
	wire [1:0] jump_in_dec;
	wire misaligned_stall;
	wire jr_stall;
	wire load_stall;
	wire csr_apu_stall;
	wire halt_id;
	wire [31:0] imm_i_type;
	wire [31:0] imm_iz_type;
	wire [31:0] imm_s_type;
	wire [31:0] imm_sb_type;
	wire [31:0] imm_u_type;
	wire [31:0] imm_uj_type;
	wire [31:0] imm_z_type;
	wire [31:0] imm_s2_type;
	wire [31:0] imm_bi_type;
	wire [31:0] imm_s3_type;
	wire [31:0] imm_vs_type;
	wire [31:0] imm_vu_type;
	wire [31:0] imm_shuffleb_type;
	wire [31:0] imm_shuffleh_type;
	reg [31:0] imm_shuffle_type;
	wire [31:0] imm_clip_type;
	reg [31:0] imm_a;
	reg [31:0] imm_b;
	reg [31:0] jump_target;
	wire irq_req_ctrl;
	wire irq_sec_ctrl;
	wire [4:0] irq_id_ctrl;
	wire exc_ack;
	wire exc_kill;
	wire [5:0] regfile_addr_ra_id;
	wire [5:0] regfile_addr_rb_id;
	reg [5:0] regfile_addr_rc_id;
	wire regfile_fp_a;
	wire regfile_fp_b;
	wire regfile_fp_c;
	wire regfile_fp_d;
	wire [5:0] regfile_waddr_id;
	wire [5:0] regfile_alu_waddr_id;
	wire regfile_alu_we_id;
	wire [31:0] regfile_data_ra_id;
	wire [31:0] regfile_data_rb_id;
	wire [31:0] regfile_data_rc_id;
	wire alu_en;
	wire [6:0] alu_operator;
	wire [2:0] alu_op_a_mux_sel;
	wire [2:0] alu_op_b_mux_sel;
	wire [1:0] alu_op_c_mux_sel;
	wire [1:0] regc_mux;
	wire [0:0] imm_a_mux_sel;
	wire [3:0] imm_b_mux_sel;
	wire [1:0] jump_target_mux_sel;
	wire [2:0] mult_operator;
	wire mult_en;
	wire mult_int_en;
	wire mult_sel_subword;
	wire [1:0] mult_signed_mode;
	wire mult_dot_en;
	wire [1:0] mult_dot_signed;
	wire [3:0] fpu_op;
	wire apu_en;
	wire [WAPUTYPE - 1:0] apu_type;
	wire [APU_WOP_CPU - 1:0] apu_op;
	wire [1:0] apu_lat;
	wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands;
	reg [APU_NDSFLAGS_CPU - 1:0] apu_flags;
	wire [5:0] apu_waddr;
	reg [17:0] apu_read_regs;
	reg [2:0] apu_read_regs_valid;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire [WAPUTYPE - 1:0] apu_flags_src;
	wire apu_stall;
	wire [2:0] fp_rnd_mode;
	wire regfile_we_id;
	wire regfile_alu_waddr_mux_sel;
	wire data_we_id;
	wire [1:0] data_type_id;
	wire data_sign_ext_id;
	wire [1:0] data_reg_offset_id;
	wire data_req_id;
	wire data_load_event_id;
	wire [N_HWLP_BITS - 1:0] hwloop_regid;
	wire [N_HWLP_BITS - 1:0] hwloop_regid_int;
	wire [2:0] hwloop_we;
	wire [2:0] hwloop_we_int;
	wire hwloop_target_mux_sel;
	wire hwloop_start_mux_sel;
	wire hwloop_cnt_mux_sel;
	reg [31:0] hwloop_target;
	wire [31:0] hwloop_start;
	reg [31:0] hwloop_start_int;
	wire [31:0] hwloop_end;
	wire [31:0] hwloop_cnt;
	reg [31:0] hwloop_cnt_int;
	wire hwloop_valid;
	wire csr_access;
	wire [1:0] csr_op;
	wire csr_status;
	wire prepost_useincr;
	wire [1:0] operand_a_fw_mux_sel;
	wire [1:0] operand_b_fw_mux_sel;
	wire [1:0] operand_c_fw_mux_sel;
	reg [31:0] operand_a_fw_id;
	reg [31:0] operand_b_fw_id;
	reg [31:0] operand_c_fw_id;
	reg [31:0] operand_b;
	reg [31:0] operand_b_vec;
	reg [31:0] alu_operand_a;
	wire [31:0] alu_operand_b;
	reg [31:0] alu_operand_c;
	wire [0:0] bmask_a_mux;
	wire [1:0] bmask_b_mux;
	wire alu_bmask_a_mux_sel;
	wire alu_bmask_b_mux_sel;
	wire [0:0] mult_imm_mux;
	reg [4:0] bmask_a_id_imm;
	reg [4:0] bmask_b_id_imm;
	reg [4:0] bmask_a_id;
	reg [4:0] bmask_b_id;
	wire [1:0] imm_vec_ext_id;
	reg [4:0] mult_imm_id;
	wire [1:0] alu_vec_mode;
	wire scalar_replication;
	wire reg_d_ex_is_reg_a_id;
	wire reg_d_ex_is_reg_b_id;
	wire reg_d_ex_is_reg_c_id;
	wire reg_d_wb_is_reg_a_id;
	wire reg_d_wb_is_reg_b_id;
	wire reg_d_wb_is_reg_c_id;
	wire reg_d_alu_is_reg_a_id;
	wire reg_d_alu_is_reg_b_id;
	wire reg_d_alu_is_reg_c_id;
	assign instr = instr_rdata_i;
	assign imm_i_type = {{20 {instr[31]}}, instr[31:20]};
	assign imm_iz_type = {20'b00000000000000000000, instr[31:20]};
	assign imm_s_type = {{20 {instr[31]}}, instr[31:25], instr[11:7]};
	assign imm_sb_type = {{19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
	assign imm_u_type = {instr[31:12], 12'b000000000000};
	assign imm_uj_type = {{12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
	assign imm_z_type = {27'b000000000000000000000000000, instr[19:15]};
	assign imm_s2_type = {27'b000000000000000000000000000, instr[24:20]};
	assign imm_bi_type = {{27 {instr[24]}}, instr[24:20]};
	assign imm_s3_type = {27'b000000000000000000000000000, instr[29:25]};
	assign imm_vs_type = {{26 {instr[24]}}, instr[24:20], instr[25]};
	assign imm_vu_type = {26'b00000000000000000000000000, instr[24:20], instr[25]};
	assign imm_shuffleb_type = {6'b000000, instr[28:27], 6'b000000, instr[24:23], 6'b000000, instr[22:21], 6'b000000, instr[20], instr[25]};
	assign imm_shuffleh_type = {15'h0000, instr[20], 15'h0000, instr[25]};
	assign imm_clip_type = (32'h00000001 << instr[24:20]) - 1;
	assign regfile_addr_ra_id = {regfile_fp_a, instr[19:15]};
	assign regfile_addr_rb_id = {regfile_fp_b, instr[24:20]};
	localparam riscv_defines_REGC_RD = 2'b01;
	localparam riscv_defines_REGC_S1 = 2'b10;
	localparam riscv_defines_REGC_S4 = 2'b00;
	localparam riscv_defines_REGC_ZERO = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (regc_mux)
			riscv_defines_REGC_ZERO: regfile_addr_rc_id = 1'sb0;
			riscv_defines_REGC_RD: regfile_addr_rc_id = {regfile_fp_c, instr[11:7]};
			riscv_defines_REGC_S1: regfile_addr_rc_id = {regfile_fp_c, instr[19:15]};
			riscv_defines_REGC_S4: regfile_addr_rc_id = {regfile_fp_c, instr[31:27]};
			default: regfile_addr_rc_id = 1'sb0;
		endcase
	end
	assign regfile_waddr_id = {regfile_fp_d, instr[11:7]};
	assign regfile_alu_waddr_id = (regfile_alu_waddr_mux_sel ? regfile_waddr_id : regfile_addr_ra_id);
	assign reg_d_ex_is_reg_a_id = ((regfile_waddr_ex_o == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_b_id = ((regfile_waddr_ex_o == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_c_id = ((regfile_waddr_ex_o == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_a_id = ((regfile_waddr_wb_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_b_id = ((regfile_waddr_wb_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_c_id = ((regfile_waddr_wb_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_a_id = ((regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_b_id = ((regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_c_id = ((regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign clear_instr_valid_o = (id_ready_o | halt_id) | branch_taken_ex;
	assign branch_taken_ex = branch_in_ex_o & branch_decision_i;
	assign mult_en = mult_int_en | mult_dot_en;
	assign hwloop_regid_int = instr[7];
	always @(*) begin
		if (_sv2v_0)
			;
		case (hwloop_target_mux_sel)
			1'b0: hwloop_target = pc_id_i + {imm_iz_type[30:0], 1'b0};
			1'b1: hwloop_target = pc_id_i + {imm_z_type[30:0], 1'b0};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (hwloop_start_mux_sel)
			1'b0: hwloop_start_int = hwloop_target;
			1'b1: hwloop_start_int = pc_if_i;
		endcase
	end
	always @(*) begin : hwloop_cnt_mux
		if (_sv2v_0)
			;
		case (hwloop_cnt_mux_sel)
			1'b0: hwloop_cnt_int = imm_iz_type;
			1'b1: hwloop_cnt_int = operand_a_fw_id;
		endcase
	end
	assign hwloop_start = (hwloop_we_int[0] ? hwloop_start_int : csr_hwlp_data_i);
	assign hwloop_end = (hwloop_we_int[1] ? hwloop_target : csr_hwlp_data_i);
	assign hwloop_cnt = (hwloop_we_int[2] ? hwloop_cnt_int : csr_hwlp_data_i);
	assign hwloop_regid = (|hwloop_we_int ? hwloop_regid_int : csr_hwlp_regid_i);
	assign hwloop_we = (|hwloop_we_int ? hwloop_we_int : csr_hwlp_we_i);
	localparam riscv_defines_JT_COND = 2'b11;
	localparam riscv_defines_JT_JAL = 2'b01;
	localparam riscv_defines_JT_JALR = 2'b10;
	always @(*) begin : jump_target_mux
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (jump_target_mux_sel)
			riscv_defines_JT_JAL: jump_target = pc_id_i + imm_uj_type;
			riscv_defines_JT_COND: jump_target = pc_id_i + imm_sb_type;
			riscv_defines_JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
			default: jump_target = regfile_data_ra_id + imm_i_type;
		endcase
	end
	assign jump_target_o = jump_target;
	localparam riscv_defines_OP_A_CURRPC = 3'b001;
	localparam riscv_defines_OP_A_IMM = 3'b010;
	localparam riscv_defines_OP_A_REGA_OR_FWD = 3'b000;
	localparam riscv_defines_OP_A_REGB_OR_FWD = 3'b011;
	localparam riscv_defines_OP_A_REGC_OR_FWD = 3'b100;
	always @(*) begin : alu_operand_a_mux
		if (_sv2v_0)
			;
		case (alu_op_a_mux_sel)
			riscv_defines_OP_A_REGA_OR_FWD: alu_operand_a = operand_a_fw_id;
			riscv_defines_OP_A_REGB_OR_FWD: alu_operand_a = operand_b_fw_id;
			riscv_defines_OP_A_REGC_OR_FWD: alu_operand_a = operand_c_fw_id;
			riscv_defines_OP_A_CURRPC: alu_operand_a = pc_id_i;
			riscv_defines_OP_A_IMM: alu_operand_a = imm_a;
			default: alu_operand_a = operand_a_fw_id;
		endcase
	end
	localparam riscv_defines_IMMA_Z = 1'b0;
	localparam riscv_defines_IMMA_ZERO = 1'b1;
	always @(*) begin : immediate_a_mux
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (imm_a_mux_sel)
			riscv_defines_IMMA_Z: imm_a = imm_z_type;
			riscv_defines_IMMA_ZERO: imm_a = 1'sb0;
			default: imm_a = 1'sb0;
		endcase
	end
	localparam riscv_defines_SEL_FW_EX = 2'b01;
	localparam riscv_defines_SEL_FW_WB = 2'b10;
	localparam riscv_defines_SEL_REGFILE = 2'b00;
	always @(*) begin : operand_a_fw_mux
		if (_sv2v_0)
			;
		case (operand_a_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_a_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_a_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_a_fw_id = regfile_data_ra_id;
			default: operand_a_fw_id = regfile_data_ra_id;
		endcase
	end
	localparam riscv_defines_IMMB_BI = 4'b1011;
	localparam riscv_defines_IMMB_CLIP = 4'b1001;
	localparam riscv_defines_IMMB_I = 4'b0000;
	localparam riscv_defines_IMMB_PCINCR = 4'b0011;
	localparam riscv_defines_IMMB_S = 4'b0001;
	localparam riscv_defines_IMMB_S2 = 4'b0100;
	localparam riscv_defines_IMMB_S3 = 4'b0101;
	localparam riscv_defines_IMMB_SHUF = 4'b1000;
	localparam riscv_defines_IMMB_U = 4'b0010;
	localparam riscv_defines_IMMB_VS = 4'b0110;
	localparam riscv_defines_IMMB_VU = 4'b0111;
	always @(*) begin : immediate_b_mux
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (imm_b_mux_sel)
			riscv_defines_IMMB_I: imm_b = imm_i_type;
			riscv_defines_IMMB_S: imm_b = imm_s_type;
			riscv_defines_IMMB_U: imm_b = imm_u_type;
			riscv_defines_IMMB_PCINCR: imm_b = (is_compressed_i && ~data_misaligned_i ? 32'h00000002 : 32'h00000004);
			riscv_defines_IMMB_S2: imm_b = imm_s2_type;
			riscv_defines_IMMB_BI: imm_b = imm_bi_type;
			riscv_defines_IMMB_S3: imm_b = imm_s3_type;
			riscv_defines_IMMB_VS: imm_b = imm_vs_type;
			riscv_defines_IMMB_VU: imm_b = imm_vu_type;
			riscv_defines_IMMB_SHUF: imm_b = imm_shuffle_type;
			riscv_defines_IMMB_CLIP: imm_b = {1'b0, imm_clip_type[31:1]};
			default: imm_b = imm_i_type;
		endcase
	end
	localparam riscv_defines_OP_B_BMASK = 3'b100;
	localparam riscv_defines_OP_B_IMM = 3'b010;
	localparam riscv_defines_OP_B_REGA_OR_FWD = 3'b011;
	localparam riscv_defines_OP_B_REGB_OR_FWD = 3'b000;
	localparam riscv_defines_OP_B_REGC_OR_FWD = 3'b001;
	always @(*) begin : alu_operand_b_mux
		if (_sv2v_0)
			;
		case (alu_op_b_mux_sel)
			riscv_defines_OP_B_REGA_OR_FWD: operand_b = operand_a_fw_id;
			riscv_defines_OP_B_REGB_OR_FWD: operand_b = operand_b_fw_id;
			riscv_defines_OP_B_REGC_OR_FWD: operand_b = operand_c_fw_id;
			riscv_defines_OP_B_IMM: operand_b = imm_b;
			riscv_defines_OP_B_BMASK: operand_b = $unsigned(operand_b_fw_id[4:0]);
			default: operand_b = operand_b_fw_id;
		endcase
	end
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		if (alu_vec_mode == riscv_defines_VEC_MODE8) begin
			operand_b_vec = {4 {operand_b[7:0]}};
			imm_shuffle_type = imm_shuffleb_type;
		end
		else begin
			operand_b_vec = {2 {operand_b[15:0]}};
			imm_shuffle_type = imm_shuffleh_type;
		end
	end
	assign alu_operand_b = (scalar_replication == 1'b1 ? operand_b_vec : operand_b);
	always @(*) begin : operand_b_fw_mux
		if (_sv2v_0)
			;
		case (operand_b_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_b_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_b_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_b_fw_id = regfile_data_rb_id;
			default: operand_b_fw_id = regfile_data_rb_id;
		endcase
	end
	localparam riscv_defines_OP_C_JT = 2'b10;
	localparam riscv_defines_OP_C_REGB_OR_FWD = 2'b01;
	localparam riscv_defines_OP_C_REGC_OR_FWD = 2'b00;
	always @(*) begin : alu_operand_c_mux
		if (_sv2v_0)
			;
		case (alu_op_c_mux_sel)
			riscv_defines_OP_C_REGC_OR_FWD: alu_operand_c = operand_c_fw_id;
			riscv_defines_OP_C_REGB_OR_FWD: alu_operand_c = operand_b_fw_id;
			riscv_defines_OP_C_JT: alu_operand_c = jump_target;
			default: alu_operand_c = operand_c_fw_id;
		endcase
	end
	always @(*) begin : operand_c_fw_mux
		if (_sv2v_0)
			;
		case (operand_c_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_c_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_c_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_c_fw_id = regfile_data_rc_id;
			default: operand_c_fw_id = regfile_data_rc_id;
		endcase
	end
	localparam riscv_defines_BMASK_A_S3 = 1'b1;
	localparam riscv_defines_BMASK_A_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (bmask_a_mux)
			riscv_defines_BMASK_A_ZERO: bmask_a_id_imm = 1'sb0;
			riscv_defines_BMASK_A_S3: bmask_a_id_imm = imm_s3_type[4:0];
			default: bmask_a_id_imm = 1'sb0;
		endcase
	end
	localparam riscv_defines_BMASK_B_ONE = 2'b11;
	localparam riscv_defines_BMASK_B_S2 = 2'b00;
	localparam riscv_defines_BMASK_B_S3 = 2'b01;
	localparam riscv_defines_BMASK_B_ZERO = 2'b10;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (bmask_b_mux)
			riscv_defines_BMASK_B_ZERO: bmask_b_id_imm = 1'sb0;
			riscv_defines_BMASK_B_ONE: bmask_b_id_imm = 5'd1;
			riscv_defines_BMASK_B_S2: bmask_b_id_imm = imm_s2_type[4:0];
			riscv_defines_BMASK_B_S3: bmask_b_id_imm = imm_s3_type[4:0];
			default: bmask_b_id_imm = 1'sb0;
		endcase
	end
	localparam riscv_defines_BMASK_A_IMM = 1'b1;
	localparam riscv_defines_BMASK_A_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (alu_bmask_a_mux_sel)
			riscv_defines_BMASK_A_IMM: bmask_a_id = bmask_a_id_imm;
			riscv_defines_BMASK_A_REG: bmask_a_id = operand_b_fw_id[9:5];
			default: bmask_a_id = bmask_a_id_imm;
		endcase
	end
	localparam riscv_defines_BMASK_B_IMM = 1'b1;
	localparam riscv_defines_BMASK_B_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (alu_bmask_b_mux_sel)
			riscv_defines_BMASK_B_IMM: bmask_b_id = bmask_b_id_imm;
			riscv_defines_BMASK_B_REG: bmask_b_id = operand_b_fw_id[4:0];
			default: bmask_b_id = bmask_b_id_imm;
		endcase
	end
	assign imm_vec_ext_id = imm_vu_type[1:0];
	localparam riscv_defines_MIMM_S3 = 1'b1;
	localparam riscv_defines_MIMM_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		(* full_case, parallel_case *)
		case (mult_imm_mux)
			riscv_defines_MIMM_ZERO: mult_imm_id = 1'sb0;
			riscv_defines_MIMM_S3: mult_imm_id = imm_s3_type[4:0];
			default: mult_imm_id = 1'sb0;
		endcase
	end
	localparam apu_core_package_APU_FLAGS_DSP_MULT = 0;
	localparam apu_core_package_APU_FLAGS_FP = 2;
	localparam apu_core_package_APU_FLAGS_INT_MULT = 1;
	generate
		if (APU == 1) begin : apu_op_preparation
			if (APU_NARGS_CPU >= 1) begin : genblk1
				assign apu_operands[0+:32] = alu_operand_a;
			end
			if (APU_NARGS_CPU >= 2) begin : genblk2
				assign apu_operands[32+:32] = alu_operand_b;
			end
			if (APU_NARGS_CPU >= 3) begin : genblk3
				assign apu_operands[64+:32] = alu_operand_c;
			end
			assign apu_waddr = regfile_alu_waddr_id;
			always @(*) begin
				if (_sv2v_0)
					;
				(* full_case, parallel_case *)
				case (apu_flags_src)
					apu_core_package_APU_FLAGS_INT_MULT: apu_flags = {7'h00, mult_imm_id, mult_signed_mode, mult_sel_subword};
					apu_core_package_APU_FLAGS_DSP_MULT: apu_flags = {13'h0000, mult_dot_signed};
					apu_core_package_APU_FLAGS_FP:
						if (FPU == 1) begin
							if (fp_rnd_mode == 3'b111)
								apu_flags = frm_i;
							else
								apu_flags = fp_rnd_mode;
						end
						else
							apu_flags = 1'sb0;
					default: apu_flags = 1'sb0;
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				(* full_case, parallel_case *)
				case (alu_op_a_mux_sel)
					riscv_defines_OP_A_REGA_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					riscv_defines_OP_A_REGB_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					default: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				(* full_case, parallel_case *)
				case (alu_op_b_mux_sel)
					riscv_defines_OP_B_REGB_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					riscv_defines_OP_B_REGC_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					default: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				(* full_case, parallel_case *)
				case (alu_op_c_mux_sel)
					riscv_defines_OP_C_REGB_OR_FWD: begin
						apu_read_regs[12+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[2] = 1'b1;
					end
					riscv_defines_OP_C_REGC_OR_FWD: begin
						apu_read_regs[12+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[2] = 1'b1;
					end
					default: begin
						apu_read_regs[12+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[2] = 1'b0;
					end
				endcase
			end
			assign apu_write_regs[0+:6] = regfile_alu_waddr_id;
			assign apu_write_regs_valid[0] = regfile_alu_we_id;
			assign apu_write_regs[6+:6] = regfile_waddr_id;
			assign apu_write_regs_valid[1] = regfile_we_id;
			assign apu_read_regs_o = apu_read_regs;
			assign apu_read_regs_valid_o = apu_read_regs_valid;
			assign apu_write_regs_o = apu_write_regs;
			assign apu_write_regs_valid_o = apu_write_regs_valid;
		end
		else begin : genblk1
			genvar _gv_i_19;
			for (_gv_i_19 = 0; _gv_i_19 < APU_NARGS_CPU; _gv_i_19 = _gv_i_19 + 1) begin : genblk1
				localparam i = _gv_i_19;
				assign apu_operands[i * 32+:32] = 1'sb0;
			end
			assign apu_waddr = 1'sb0;
			wire [APU_NDSFLAGS_CPU:1] sv2v_tmp_98B16;
			assign sv2v_tmp_98B16 = 1'sb0;
			always @(*) apu_flags = sv2v_tmp_98B16;
			assign apu_write_regs_o = 1'sb0;
			assign apu_read_regs_o = 1'sb0;
			assign apu_write_regs_valid_o = 1'sb0;
			assign apu_read_regs_valid_o = 1'sb0;
		end
	endgenerate
	assign apu_perf_dep_o = apu_stall;
	assign csr_apu_stall = csr_access & ((apu_en_ex_o & (apu_lat_ex_o[1] == 1'b1)) | apu_busy_i);
	riscv_register_file #(
		.ADDR_WIDTH(6),
		.FPU(FPU)
	) registers_i(
		.clk(clk),
		.rst_n(rst_n),
		.test_en_i(test_en_i),
		.fregfile_disable_i(fregfile_disable_i),
		.raddr_a_i(regfile_addr_ra_id),
		.rdata_a_o(regfile_data_ra_id),
		.raddr_b_i(regfile_addr_rb_id),
		.rdata_b_o(regfile_data_rb_id),
		.raddr_c_i((dbg_reg_rreq_i == 1'b0 ? regfile_addr_rc_id : dbg_reg_raddr_i)),
		.rdata_c_o(regfile_data_rc_id),
		.waddr_a_i(regfile_waddr_wb_i),
		.wdata_a_i(regfile_wdata_wb_i),
		.we_a_i(regfile_we_wb_i),
		.waddr_b_i((dbg_reg_wreq_i == 1'b0 ? regfile_alu_waddr_fw_i : dbg_reg_waddr_i)),
		.wdata_b_i((dbg_reg_wreq_i == 1'b0 ? regfile_alu_wdata_fw_i : dbg_reg_wdata_i)),
		.we_b_i((dbg_reg_wreq_i == 1'b0 ? regfile_alu_we_fw_i : 1'b1))
	);
	assign dbg_reg_rdata_o = regfile_data_rc_id;
	riscv_decoder #(
		.FPU(FPU),
		.PULP_SECURE(PULP_SECURE),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.SHARED_FP_DIVSQRT(SHARED_FP_DIVSQRT),
		.WAPUTYPE(WAPUTYPE),
		.APU_WOP_CPU(APU_WOP_CPU)
	) decoder_i(
		.deassert_we_i(deassert_we),
		.data_misaligned_i(data_misaligned_i),
		.mult_multicycle_i(mult_multicycle_i),
		.illegal_insn_o(illegal_insn_dec),
		.ebrk_insn_o(ebrk_insn),
		.mret_insn_o(mret_insn_dec),
		.uret_insn_o(uret_insn_dec),
		.ecall_insn_o(ecall_insn_dec),
		.pipe_flush_o(pipe_flush_dec),
		.rega_used_o(rega_used_dec),
		.regb_used_o(regb_used_dec),
		.regc_used_o(regc_used_dec),
		.reg_fp_a_o(regfile_fp_a),
		.reg_fp_b_o(regfile_fp_b),
		.reg_fp_c_o(regfile_fp_c),
		.reg_fp_d_o(regfile_fp_d),
		.bmask_a_mux_o(bmask_a_mux),
		.bmask_b_mux_o(bmask_b_mux),
		.alu_bmask_a_mux_sel_o(alu_bmask_a_mux_sel),
		.alu_bmask_b_mux_sel_o(alu_bmask_b_mux_sel),
		.instr_rdata_i(instr),
		.illegal_c_insn_i(illegal_c_insn_i),
		.alu_en_o(alu_en),
		.alu_operator_o(alu_operator),
		.alu_op_a_mux_sel_o(alu_op_a_mux_sel),
		.alu_op_b_mux_sel_o(alu_op_b_mux_sel),
		.alu_op_c_mux_sel_o(alu_op_c_mux_sel),
		.alu_vec_mode_o(alu_vec_mode),
		.scalar_replication_o(scalar_replication),
		.imm_a_mux_sel_o(imm_a_mux_sel),
		.imm_b_mux_sel_o(imm_b_mux_sel),
		.regc_mux_o(regc_mux),
		.mult_operator_o(mult_operator),
		.mult_int_en_o(mult_int_en),
		.mult_sel_subword_o(mult_sel_subword),
		.mult_signed_mode_o(mult_signed_mode),
		.mult_imm_mux_o(mult_imm_mux),
		.mult_dot_en_o(mult_dot_en),
		.mult_dot_signed_o(mult_dot_signed),
		.fpu_op_o(fpu_op),
		.apu_en_o(apu_en),
		.apu_type_o(apu_type),
		.apu_op_o(apu_op),
		.apu_lat_o(apu_lat),
		.apu_flags_src_o(apu_flags_src),
		.fp_rnd_mode_o(fp_rnd_mode),
		.regfile_mem_we_o(regfile_we_id),
		.regfile_alu_we_o(regfile_alu_we_id),
		.regfile_alu_waddr_sel_o(regfile_alu_waddr_mux_sel),
		.csr_access_o(csr_access),
		.csr_status_o(csr_status),
		.csr_op_o(csr_op),
		.current_priv_lvl_i(current_priv_lvl_i),
		.data_req_o(data_req_id),
		.data_we_o(data_we_id),
		.prepost_useincr_o(prepost_useincr),
		.data_type_o(data_type_id),
		.data_sign_extension_o(data_sign_ext_id),
		.data_reg_offset_o(data_reg_offset_id),
		.data_load_event_o(data_load_event_id),
		.hwloop_we_o(hwloop_we_int),
		.hwloop_target_mux_sel_o(hwloop_target_mux_sel),
		.hwloop_start_mux_sel_o(hwloop_start_mux_sel),
		.hwloop_cnt_mux_sel_o(hwloop_cnt_mux_sel),
		.jump_in_dec_o(jump_in_dec),
		.jump_in_id_o(jump_in_id),
		.jump_target_mux_sel_o(jump_target_mux_sel)
	);
	riscv_controller #(.FPU(FPU)) controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.fetch_enable_i(fetch_enable_i),
		.ctrl_busy_o(ctrl_busy_o),
		.first_fetch_o(core_ctrl_firstfetch_o),
		.is_decoding_o(is_decoding_o),
		.deassert_we_o(deassert_we),
		.illegal_insn_i(illegal_insn_dec),
		.ecall_insn_i(ecall_insn_dec),
		.mret_insn_i(mret_insn_dec),
		.uret_insn_i(uret_insn_dec),
		.pipe_flush_i(pipe_flush_dec),
		.ebrk_insn_i(ebrk_insn),
		.csr_status_i(csr_status),
		.instr_valid_i(instr_valid_i),
		.instr_req_o(instr_req_o),
		.pc_set_o(pc_set_o),
		.pc_mux_o(pc_mux_o),
		.exc_pc_mux_o(exc_pc_mux_o),
		.exc_cause_o(exc_cause_o),
		.trap_addr_mux_o(trap_addr_mux_o),
		.data_req_ex_i(data_req_ex_o),
		.data_misaligned_i(data_misaligned_i),
		.data_load_event_i(data_load_event_id),
		.mult_multicycle_i(mult_multicycle_i),
		.apu_en_i(apu_en),
		.apu_read_dep_i(apu_read_dep_i),
		.apu_write_dep_i(apu_write_dep_i),
		.apu_stall_o(apu_stall),
		.branch_taken_ex_i(branch_taken_ex),
		.jump_in_id_i(jump_in_id),
		.jump_in_dec_i(jump_in_dec),
		.irq_i(irq_i),
		.irq_req_ctrl_i(irq_req_ctrl),
		.irq_sec_ctrl_i(irq_sec_ctrl),
		.irq_id_ctrl_i(irq_id_ctrl),
		.m_IE_i(m_irq_enable_i),
		.u_IE_i(u_irq_enable_i),
		.current_priv_lvl_i(current_priv_lvl_i),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.exc_ack_o(exc_ack),
		.exc_kill_o(exc_kill),
		.csr_save_cause_o(csr_save_cause_o),
		.csr_cause_o(csr_cause_o),
		.csr_save_if_o(csr_save_if_o),
		.csr_save_id_o(csr_save_id_o),
		.csr_restore_mret_id_o(csr_restore_mret_id_o),
		.csr_restore_uret_id_o(csr_restore_uret_id_o),
		.csr_irq_sec_o(csr_irq_sec_o),
		.dbg_req_i(dbg_req_i),
		.dbg_ack_o(dbg_ack_o),
		.dbg_stall_i(dbg_stall_i),
		.dbg_jump_req_i(dbg_jump_req_i),
		.dbg_settings_i(dbg_settings_i),
		.dbg_trap_o(dbg_trap_o),
		.regfile_alu_waddr_id_i(regfile_alu_waddr_id),
		.regfile_we_ex_i(regfile_we_ex_o),
		.regfile_waddr_ex_i(regfile_waddr_ex_o),
		.regfile_we_wb_i(regfile_we_wb_i),
		.regfile_alu_we_fw_i(regfile_alu_we_fw_i),
		.reg_d_ex_is_reg_a_i(reg_d_ex_is_reg_a_id),
		.reg_d_ex_is_reg_b_i(reg_d_ex_is_reg_b_id),
		.reg_d_ex_is_reg_c_i(reg_d_ex_is_reg_c_id),
		.reg_d_wb_is_reg_a_i(reg_d_wb_is_reg_a_id),
		.reg_d_wb_is_reg_b_i(reg_d_wb_is_reg_b_id),
		.reg_d_wb_is_reg_c_i(reg_d_wb_is_reg_c_id),
		.reg_d_alu_is_reg_a_i(reg_d_alu_is_reg_a_id),
		.reg_d_alu_is_reg_b_i(reg_d_alu_is_reg_b_id),
		.reg_d_alu_is_reg_c_i(reg_d_alu_is_reg_c_id),
		.operand_a_fw_mux_sel_o(operand_a_fw_mux_sel),
		.operand_b_fw_mux_sel_o(operand_b_fw_mux_sel),
		.operand_c_fw_mux_sel_o(operand_c_fw_mux_sel),
		.halt_if_o(halt_if_o),
		.halt_id_o(halt_id),
		.misaligned_stall_o(misaligned_stall),
		.jr_stall_o(jr_stall),
		.load_stall_o(load_stall),
		.id_ready_i(id_ready_o),
		.ex_valid_i(ex_valid_i),
		.wb_ready_i(wb_ready_i),
		.perf_jump_o(perf_jump_o),
		.perf_jr_stall_o(perf_jr_stall_o),
		.perf_ld_stall_o(perf_ld_stall_o)
	);
	riscv_int_controller #(.PULP_SECURE(PULP_SECURE)) int_controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.irq_req_ctrl_o(irq_req_ctrl),
		.irq_sec_ctrl_o(irq_sec_ctrl),
		.irq_id_ctrl_o(irq_id_ctrl),
		.ctrl_ack_i(exc_ack),
		.ctrl_kill_i(exc_kill),
		.irq_i(irq_i),
		.irq_sec_i(irq_sec_i),
		.irq_id_i(irq_id_i),
		.m_IE_i(m_irq_enable_i),
		.u_IE_i(u_irq_enable_i),
		.current_priv_lvl_i(current_priv_lvl_i)
	);
	riscv_hwloop_regs #(.N_REGS(N_HWLP)) hwloop_regs_i(
		.clk(clk),
		.rst_n(rst_n),
		.hwlp_start_data_i(hwloop_start),
		.hwlp_end_data_i(hwloop_end),
		.hwlp_cnt_data_i(hwloop_cnt),
		.hwlp_we_i(hwloop_we),
		.hwlp_regid_i(hwloop_regid),
		.valid_i(hwloop_valid),
		.hwlp_start_addr_o(hwlp_start_o),
		.hwlp_end_addr_o(hwlp_end_o),
		.hwlp_counter_o(hwlp_cnt_o),
		.hwlp_dec_cnt_i(hwlp_dec_cnt_i)
	);
	assign hwloop_valid = (instr_valid_i & clear_instr_valid_o) & is_hwlp_i;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	always @(posedge clk or negedge rst_n) begin : ID_EX_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			alu_en_ex_o <= 1'sb0;
			alu_operator_ex_o <= riscv_defines_ALU_SLTU;
			alu_operand_a_ex_o <= 1'sb0;
			alu_operand_b_ex_o <= 1'sb0;
			alu_operand_c_ex_o <= 1'sb0;
			bmask_a_ex_o <= 1'sb0;
			bmask_b_ex_o <= 1'sb0;
			imm_vec_ext_ex_o <= 1'sb0;
			alu_vec_mode_ex_o <= 1'sb0;
			mult_operator_ex_o <= 1'sb0;
			mult_operand_a_ex_o <= 1'sb0;
			mult_operand_b_ex_o <= 1'sb0;
			mult_operand_c_ex_o <= 1'sb0;
			mult_en_ex_o <= 1'b0;
			mult_sel_subword_ex_o <= 1'b0;
			mult_signed_mode_ex_o <= 2'b00;
			mult_imm_ex_o <= 1'sb0;
			mult_dot_op_a_ex_o <= 1'sb0;
			mult_dot_op_b_ex_o <= 1'sb0;
			mult_dot_op_c_ex_o <= 1'sb0;
			mult_dot_signed_ex_o <= 1'sb0;
			fpu_op_ex_o <= 1'sb0;
			apu_en_ex_o <= 1'sb0;
			apu_type_ex_o <= 1'sb0;
			apu_op_ex_o <= 1'sb0;
			apu_lat_ex_o <= 1'sb0;
			apu_operands_ex_o[0+:32] <= 1'sb0;
			apu_operands_ex_o[32+:32] <= 1'sb0;
			apu_operands_ex_o[64+:32] <= 1'sb0;
			apu_flags_ex_o <= 1'sb0;
			apu_waddr_ex_o <= 1'sb0;
			regfile_waddr_ex_o <= 6'b000000;
			regfile_we_ex_o <= 1'b0;
			regfile_alu_waddr_ex_o <= 6'b000000;
			regfile_alu_we_ex_o <= 1'b0;
			prepost_useincr_ex_o <= 1'b0;
			csr_access_ex_o <= 1'b0;
			csr_op_ex_o <= riscv_defines_CSR_OP_NONE;
			data_we_ex_o <= 1'b0;
			data_type_ex_o <= 2'b00;
			data_sign_ext_ex_o <= 1'b0;
			data_reg_offset_ex_o <= 2'b00;
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			pc_ex_o <= 1'sb0;
			branch_in_ex_o <= 1'b0;
		end
		else if (data_misaligned_i) begin
			if (ex_ready_i) begin
				if (prepost_useincr_ex_o == 1'b1)
					alu_operand_a_ex_o <= alu_operand_a;
				alu_operand_b_ex_o <= alu_operand_b;
				regfile_alu_we_ex_o <= regfile_alu_we_id;
				prepost_useincr_ex_o <= prepost_useincr;
				data_misaligned_ex_o <= 1'b1;
			end
		end
		else if (mult_multicycle_i)
			mult_operand_c_ex_o <= alu_operand_c;
		else if (id_valid_o) begin
			alu_en_ex_o <= alu_en;
			if (alu_en) begin
				alu_operator_ex_o <= alu_operator;
				alu_operand_a_ex_o <= alu_operand_a;
				alu_operand_b_ex_o <= alu_operand_b;
				alu_operand_c_ex_o <= alu_operand_c;
				bmask_a_ex_o <= bmask_a_id;
				bmask_b_ex_o <= bmask_b_id;
				imm_vec_ext_ex_o <= imm_vec_ext_id;
				alu_vec_mode_ex_o <= alu_vec_mode;
			end
			mult_en_ex_o <= mult_en;
			if (mult_int_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_sel_subword_ex_o <= mult_sel_subword;
				mult_signed_mode_ex_o <= mult_signed_mode;
				mult_operand_a_ex_o <= alu_operand_a;
				mult_operand_b_ex_o <= alu_operand_b;
				mult_operand_c_ex_o <= alu_operand_c;
				mult_imm_ex_o <= mult_imm_id;
			end
			if (mult_dot_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_dot_signed_ex_o <= mult_dot_signed;
				mult_dot_op_a_ex_o <= alu_operand_a;
				mult_dot_op_b_ex_o <= alu_operand_b;
				mult_dot_op_c_ex_o <= alu_operand_c;
			end
			apu_en_ex_o <= apu_en;
			if (apu_en) begin
				fpu_op_ex_o <= fpu_op;
				apu_type_ex_o <= apu_type;
				apu_op_ex_o <= apu_op;
				apu_lat_ex_o <= apu_lat;
				apu_operands_ex_o <= apu_operands;
				apu_flags_ex_o <= apu_flags;
				apu_waddr_ex_o <= apu_waddr;
			end
			regfile_we_ex_o <= regfile_we_id;
			if (regfile_we_id)
				regfile_waddr_ex_o <= regfile_waddr_id;
			regfile_alu_we_ex_o <= regfile_alu_we_id;
			if (regfile_alu_we_id)
				regfile_alu_waddr_ex_o <= regfile_alu_waddr_id;
			prepost_useincr_ex_o <= prepost_useincr;
			csr_access_ex_o <= csr_access;
			csr_op_ex_o <= csr_op;
			data_req_ex_o <= data_req_id;
			if (data_req_id) begin
				data_we_ex_o <= data_we_id;
				data_type_ex_o <= data_type_id;
				data_sign_ext_ex_o <= data_sign_ext_id;
				data_reg_offset_ex_o <= data_reg_offset_id;
				data_load_event_ex_o <= data_load_event_id;
			end
			else
				data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			if ((jump_in_id == riscv_defines_BRANCH_COND) || data_load_event_id)
				pc_ex_o <= pc_id_i;
			branch_in_ex_o <= jump_in_id == riscv_defines_BRANCH_COND;
		end
		else if (ex_ready_i) begin
			regfile_we_ex_o <= 1'b0;
			regfile_alu_we_ex_o <= 1'b0;
			csr_op_ex_o <= riscv_defines_CSR_OP_NONE;
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			branch_in_ex_o <= 1'b0;
			apu_en_ex_o <= 1'b0;
		end
		else if (csr_access_ex_o)
			regfile_alu_we_ex_o <= 1'b0;
	end
	assign id_ready_o = ((((~misaligned_stall & ~jr_stall) & ~load_stall) & ~apu_stall) & ~csr_apu_stall) & ex_ready_i;
	assign id_valid_o = ~halt_id & id_ready_o;
	initial _sv2v_0 = 0;
endmodule
module riscv_if_stage (
	clk,
	rst_n,
	m_trap_base_addr_i,
	u_trap_base_addr_i,
	trap_addr_mux_i,
	boot_addr_i,
	req_i,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	hwlp_dec_cnt_id_o,
	is_hwlp_id_o,
	instr_valid_id_o,
	instr_rdata_id_o,
	is_compressed_id_o,
	illegal_c_insn_id_o,
	pc_if_o,
	pc_id_o,
	clear_instr_valid_i,
	pc_set_i,
	exception_pc_reg_i,
	pc_mux_i,
	exc_pc_mux_i,
	exc_vec_pc_mux_i,
	jump_target_id_i,
	jump_target_ex_i,
	hwlp_start_i,
	hwlp_end_i,
	hwlp_cnt_i,
	dbg_jump_addr_i,
	dbg_jump_req_i,
	halt_if_i,
	id_ready_i,
	if_busy_o,
	perf_imiss_o
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter RDATA_WIDTH = 32;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	input wire [23:0] m_trap_base_addr_i;
	input wire [23:0] u_trap_base_addr_i;
	input wire trap_addr_mux_i;
	input wire [23:0] boot_addr_i;
	input wire req_i;
	output wire instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [RDATA_WIDTH - 1:0] instr_rdata_i;
	output reg [N_HWLP - 1:0] hwlp_dec_cnt_id_o;
	output wire is_hwlp_id_o;
	output reg instr_valid_id_o;
	output reg [31:0] instr_rdata_id_o;
	output reg is_compressed_id_o;
	output reg illegal_c_insn_id_o;
	output wire [31:0] pc_if_o;
	output reg [31:0] pc_id_o;
	input wire clear_instr_valid_i;
	input wire pc_set_i;
	input wire [31:0] exception_pc_reg_i;
	input wire [2:0] pc_mux_i;
	input wire [1:0] exc_pc_mux_i;
	input wire [4:0] exc_vec_pc_mux_i;
	input wire [31:0] jump_target_id_i;
	input wire [31:0] jump_target_ex_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_start_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_end_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_cnt_i;
	input wire [31:0] dbg_jump_addr_i;
	input wire dbg_jump_req_i;
	input wire halt_if_i;
	input wire id_ready_i;
	output wire if_busy_o;
	output wire perf_imiss_o;
	reg [0:0] offset_fsm_cs;
	reg [0:0] offset_fsm_ns;
	wire if_valid;
	wire if_ready;
	reg valid;
	wire prefetch_busy;
	reg branch_req;
	reg [31:0] fetch_addr_n;
	wire fetch_valid;
	reg fetch_ready;
	wire [31:0] fetch_rdata;
	wire [31:0] fetch_addr;
	reg is_hwlp_id_q;
	wire fetch_is_hwlp;
	reg [31:0] exc_pc;
	wire hwlp_jump;
	wire hwlp_branch;
	wire [31:0] hwlp_target;
	wire [N_HWLP - 1:0] hwlp_dec_cnt;
	reg [N_HWLP - 1:0] hwlp_dec_cnt_if;
	reg [23:0] trap_base_addr;
	localparam riscv_defines_EXC_OFF_ECALL = 8'h88;
	localparam riscv_defines_EXC_OFF_ILLINSN = 8'h84;
	localparam riscv_defines_EXC_PC_ECALL = 2'b01;
	localparam riscv_defines_EXC_PC_ILLINSN = 2'b00;
	localparam riscv_defines_EXC_PC_IRQ = 2'b11;
	localparam riscv_defines_TRAP_MACHINE = 1'b0;
	localparam riscv_defines_TRAP_USER = 1'b1;
	always @(*) begin : EXC_PC_MUX
		if (_sv2v_0)
			;
		exc_pc = 1'sb0;
		(* full_case, parallel_case *)
		case (trap_addr_mux_i)
			riscv_defines_TRAP_MACHINE: trap_base_addr = m_trap_base_addr_i;
			riscv_defines_TRAP_USER: trap_base_addr = u_trap_base_addr_i;
			default:
				;
		endcase
		(* full_case, parallel_case *)
		case (exc_pc_mux_i)
			riscv_defines_EXC_PC_ILLINSN: exc_pc = {trap_base_addr, riscv_defines_EXC_OFF_ILLINSN};
			riscv_defines_EXC_PC_ECALL: exc_pc = {trap_base_addr, riscv_defines_EXC_OFF_ECALL};
			riscv_defines_EXC_PC_IRQ: exc_pc = {trap_base_addr, 1'b0, exc_vec_pc_mux_i[4:0], 2'b00};
			default:
				;
		endcase
	end
	localparam riscv_defines_EXC_OFF_RST = 8'h80;
	localparam riscv_defines_PC_BOOT = 3'b000;
	localparam riscv_defines_PC_BRANCH = 3'b011;
	localparam riscv_defines_PC_DBG_NPC = 3'b111;
	localparam riscv_defines_PC_ERET = 3'b101;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	localparam riscv_defines_PC_JUMP = 3'b010;
	always @(*) begin
		if (_sv2v_0)
			;
		fetch_addr_n = 1'sb0;
		(* full_case, parallel_case *)
		case (pc_mux_i)
			riscv_defines_PC_BOOT: fetch_addr_n = {boot_addr_i, riscv_defines_EXC_OFF_RST};
			riscv_defines_PC_JUMP: fetch_addr_n = jump_target_id_i;
			riscv_defines_PC_BRANCH: fetch_addr_n = jump_target_ex_i;
			riscv_defines_PC_EXCEPTION: fetch_addr_n = exc_pc;
			riscv_defines_PC_ERET: fetch_addr_n = exception_pc_reg_i;
			riscv_defines_PC_DBG_NPC: fetch_addr_n = dbg_jump_addr_i;
			default:
				;
		endcase
	end
	generate
		if (RDATA_WIDTH == 32) begin : prefetch_32
			riscv_prefetch_buffer prefetch_buffer_i(
				.clk(clk),
				.rst_n(rst_n),
				.req_i(req_i),
				.branch_i(branch_req),
				.addr_i({fetch_addr_n[31:1], 1'b0}),
				.hwloop_i(hwlp_jump),
				.hwloop_target_i(hwlp_target),
				.hwlp_branch_o(hwlp_branch),
				.ready_i(fetch_ready),
				.valid_o(fetch_valid),
				.rdata_o(fetch_rdata),
				.addr_o(fetch_addr),
				.is_hwlp_o(fetch_is_hwlp),
				.instr_req_o(instr_req_o),
				.instr_addr_o(instr_addr_o),
				.instr_gnt_i(instr_gnt_i),
				.instr_rvalid_i(instr_rvalid_i),
				.instr_rdata_i(instr_rdata_i),
				.busy_o(prefetch_busy)
			);
		end
		else if (RDATA_WIDTH == 128) begin : prefetch_128
			riscv_prefetch_L0_buffer prefetch_buffer_i(
				.clk(clk),
				.rst_n(rst_n),
				.req_i(1'b1),
				.branch_i(branch_req),
				.addr_i({fetch_addr_n[31:1], 1'b0}),
				.hwloop_i(hwlp_jump),
				.hwloop_target_i(hwlp_target),
				.ready_i(fetch_ready),
				.valid_o(fetch_valid),
				.rdata_o(fetch_rdata),
				.addr_o(fetch_addr),
				.is_hwlp_o(fetch_is_hwlp),
				.instr_req_o(instr_req_o),
				.instr_addr_o(instr_addr_o),
				.instr_gnt_i(instr_gnt_i),
				.instr_rvalid_i(instr_rvalid_i),
				.instr_rdata_i(instr_rdata_i),
				.busy_o(prefetch_busy)
			);
		end
	endgenerate
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			offset_fsm_cs <= 1'd1;
		else
			offset_fsm_cs <= offset_fsm_ns;
	always @(*) begin
		if (_sv2v_0)
			;
		offset_fsm_ns = offset_fsm_cs;
		fetch_ready = 1'b0;
		branch_req = 1'b0;
		valid = 1'b0;
		(* full_case, parallel_case *)
		case (offset_fsm_cs)
			1'd1:
				if (req_i) begin
					branch_req = 1'b1;
					offset_fsm_ns = 1'd0;
				end
			1'd0:
				if (fetch_valid) begin
					valid = 1'b1;
					if (req_i && if_valid) begin
						fetch_ready = 1'b1;
						offset_fsm_ns = 1'd0;
					end
				end
			default: offset_fsm_ns = 1'd1;
		endcase
		if (pc_set_i) begin
			valid = 1'b0;
			branch_req = 1'b1;
			offset_fsm_ns = 1'd0;
		end
		else if (hwlp_branch)
			valid = 1'b0;
	end
	riscv_hwloop_controller #(.N_REGS(N_HWLP)) hwloop_controller_i(
		.current_pc_i(fetch_addr),
		.hwlp_jump_o(hwlp_jump),
		.hwlp_targ_addr_o(hwlp_target),
		.hwlp_start_addr_i(hwlp_start_i),
		.hwlp_end_addr_i(hwlp_end_i),
		.hwlp_counter_i(hwlp_cnt_i),
		.hwlp_dec_cnt_o(hwlp_dec_cnt),
		.hwlp_dec_cnt_id_i(hwlp_dec_cnt_id_o & {N_HWLP {is_hwlp_id_o}})
	);
	assign pc_if_o = fetch_addr;
	assign if_busy_o = prefetch_busy;
	assign perf_imiss_o = ~fetch_valid | branch_req;
	wire [31:0] instr_decompressed;
	wire illegal_c_insn;
	wire instr_compressed_int;
	riscv_compressed_decoder #(.FPU(FPU)) compressed_decoder_i(
		.instr_i(fetch_rdata),
		.instr_o(instr_decompressed),
		.is_compressed_o(instr_compressed_int),
		.illegal_instr_o(illegal_c_insn)
	);
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			hwlp_dec_cnt_if <= 1'sb0;
		else if (hwlp_jump)
			hwlp_dec_cnt_if <= hwlp_dec_cnt;
	always @(posedge clk or negedge rst_n) begin : IF_ID_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			instr_valid_id_o <= 1'b0;
			instr_rdata_id_o <= 1'sb0;
			illegal_c_insn_id_o <= 1'b0;
			is_compressed_id_o <= 1'b0;
			pc_id_o <= 1'sb0;
			is_hwlp_id_q <= 1'b0;
			hwlp_dec_cnt_id_o <= 1'sb0;
		end
		else if (if_valid) begin
			instr_valid_id_o <= 1'b1;
			instr_rdata_id_o <= instr_decompressed;
			illegal_c_insn_id_o <= illegal_c_insn;
			is_compressed_id_o <= instr_compressed_int;
			pc_id_o <= pc_if_o;
			is_hwlp_id_q <= fetch_is_hwlp;
			if (fetch_is_hwlp)
				hwlp_dec_cnt_id_o <= hwlp_dec_cnt_if;
		end
		else if (clear_instr_valid_i)
			instr_valid_id_o <= 1'b0;
	end
	assign is_hwlp_id_o = is_hwlp_id_q & instr_valid_id_o;
	assign if_ready = valid & id_ready_i;
	assign if_valid = ~halt_if_i & if_ready;
	initial _sv2v_0 = 0;
endmodule
module riscv_int_controller (
	clk,
	rst_n,
	irq_req_ctrl_o,
	irq_sec_ctrl_o,
	irq_id_ctrl_o,
	ctrl_ack_i,
	ctrl_kill_i,
	irq_i,
	irq_sec_i,
	irq_id_i,
	m_IE_i,
	u_IE_i,
	current_priv_lvl_i
);
	parameter PULP_SECURE = 0;
	input wire clk;
	input wire rst_n;
	output wire irq_req_ctrl_o;
	output wire irq_sec_ctrl_o;
	output wire [4:0] irq_id_ctrl_o;
	input wire ctrl_ack_i;
	input wire ctrl_kill_i;
	input wire irq_i;
	input wire irq_sec_i;
	input wire [4:0] irq_id_i;
	input wire m_IE_i;
	input wire u_IE_i;
	input wire [1:0] current_priv_lvl_i;
	localparam PULP_SEC = 1;
	reg [1:0] exc_ctrl_cs;
	wire [1:0] exc_ctrl_ns;
	wire irq_enable_ext;
	reg [4:0] irq_id_q;
	reg irq_sec_q;
	generate
		if (PULP_SEC) begin : genblk1
			assign irq_enable_ext = ((u_IE_i | irq_sec_i) & (current_priv_lvl_i == 2'b00)) | (m_IE_i & (current_priv_lvl_i == 2'b11));
		end
		else begin : genblk1
			assign irq_enable_ext = m_IE_i;
		end
	endgenerate
	assign irq_req_ctrl_o = exc_ctrl_cs == 2'd1;
	assign irq_sec_ctrl_o = irq_sec_q;
	assign irq_id_ctrl_o = irq_id_q;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			irq_id_q <= 1'sb0;
			irq_sec_q <= 1'b0;
			exc_ctrl_cs <= 2'd0;
		end
		else
			(* full_case, parallel_case *)
			case (exc_ctrl_cs)
				2'd0:
					if (irq_enable_ext & irq_i) begin
						exc_ctrl_cs <= 2'd1;
						irq_id_q <= irq_id_i;
						irq_sec_q <= irq_sec_i;
					end
				2'd1:
					(* full_case, parallel_case *)
					case (1'b1)
						ctrl_ack_i: exc_ctrl_cs <= 2'd2;
						ctrl_kill_i: exc_ctrl_cs <= 2'd0;
						default: exc_ctrl_cs <= 2'd1;
					endcase
				2'd2: begin
					irq_sec_q <= 1'b0;
					exc_ctrl_cs <= 2'd0;
				end
			endcase
endmodule
module riscv_load_store_unit (
	clk,
	rst_n,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_err_i,
	data_addr_o,
	data_we_o,
	data_be_o,
	data_wdata_o,
	data_rdata_i,
	data_we_ex_i,
	data_type_ex_i,
	data_wdata_ex_i,
	data_reg_offset_ex_i,
	data_sign_ext_ex_i,
	data_rdata_ex_o,
	data_req_ex_i,
	operand_a_ex_i,
	operand_b_ex_i,
	addr_useincr_ex_i,
	data_misaligned_ex_i,
	data_misaligned_o,
	load_err_o,
	store_err_o,
	lsu_ready_ex_o,
	lsu_ready_wb_o,
	ex_valid_i,
	busy_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	output reg data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	input wire data_err_i;
	output wire [31:0] data_addr_o;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	input wire data_we_ex_i;
	input wire [1:0] data_type_ex_i;
	input wire [31:0] data_wdata_ex_i;
	input wire [1:0] data_reg_offset_ex_i;
	input wire data_sign_ext_ex_i;
	output wire [31:0] data_rdata_ex_o;
	input wire data_req_ex_i;
	input wire [31:0] operand_a_ex_i;
	input wire [31:0] operand_b_ex_i;
	input wire addr_useincr_ex_i;
	input wire data_misaligned_ex_i;
	output reg data_misaligned_o;
	output wire load_err_o;
	output wire store_err_o;
	output reg lsu_ready_ex_o;
	output reg lsu_ready_wb_o;
	input wire ex_valid_i;
	output wire busy_o;
	wire [31:0] data_addr_int;
	reg [1:0] data_type_q;
	reg [1:0] rdata_offset_q;
	reg data_sign_ext_q;
	reg data_we_q;
	wire [1:0] wdata_offset;
	reg [3:0] data_be;
	reg [31:0] data_wdata;
	wire misaligned_st;
	reg [1:0] CS;
	reg [1:0] NS;
	reg [31:0] rdata_q;
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_ex_i)
			2'b00:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b1111;
						2'b01: data_be = 4'b1110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0000;
						2'b01: data_be = 4'b0001;
						2'b10: data_be = 4'b0011;
						2'b11: data_be = 4'b0111;
					endcase
			2'b01:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0011;
						2'b01: data_be = 4'b0110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					data_be = 4'b0001;
			2'b10, 2'b11:
				case (data_addr_int[1:0])
					2'b00: data_be = 4'b0001;
					2'b01: data_be = 4'b0010;
					2'b10: data_be = 4'b0100;
					2'b11: data_be = 4'b1000;
				endcase
		endcase
	end
	assign wdata_offset = data_addr_int[1:0] - data_reg_offset_ex_i[1:0];
	always @(*) begin
		if (_sv2v_0)
			;
		case (wdata_offset)
			2'b00: data_wdata = data_wdata_ex_i[31:0];
			2'b01: data_wdata = {data_wdata_ex_i[23:0], data_wdata_ex_i[31:24]};
			2'b10: data_wdata = {data_wdata_ex_i[15:0], data_wdata_ex_i[31:16]};
			2'b11: data_wdata = {data_wdata_ex_i[7:0], data_wdata_ex_i[31:8]};
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			data_type_q <= 1'sb0;
			rdata_offset_q <= 1'sb0;
			data_sign_ext_q <= 1'sb0;
			data_we_q <= 1'b0;
		end
		else if (data_gnt_i == 1'b1) begin
			data_type_q <= data_type_ex_i;
			rdata_offset_q <= data_addr_int[1:0];
			data_sign_ext_q <= data_sign_ext_ex_i;
			data_we_q <= data_we_ex_i;
		end
	reg [31:0] data_rdata_ext;
	reg [31:0] rdata_w_ext;
	reg [31:0] rdata_h_ext;
	reg [31:0] rdata_b_ext;
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00: rdata_w_ext = data_rdata_i[31:0];
			2'b01: rdata_w_ext = {data_rdata_i[7:0], rdata_q[31:8]};
			2'b10: rdata_w_ext = {data_rdata_i[15:0], rdata_q[31:16]};
			2'b11: rdata_w_ext = {data_rdata_i[23:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 1'b0)
					rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
				else
					rdata_h_ext = {{16 {data_rdata_i[15]}}, data_rdata_i[15:0]};
			2'b01:
				if (data_sign_ext_q == 1'b0)
					rdata_h_ext = {16'h0000, data_rdata_i[23:8]};
				else
					rdata_h_ext = {{16 {data_rdata_i[23]}}, data_rdata_i[23:8]};
			2'b10:
				if (data_sign_ext_q == 1'b0)
					rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
				else
					rdata_h_ext = {{16 {data_rdata_i[31]}}, data_rdata_i[31:16]};
			2'b11:
				if (data_sign_ext_q == 1'b0)
					rdata_h_ext = {16'h0000, data_rdata_i[7:0], rdata_q[31:24]};
				else
					rdata_h_ext = {{16 {data_rdata_i[7]}}, data_rdata_i[7:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 1'b0)
					rdata_b_ext = {24'h000000, data_rdata_i[7:0]};
				else
					rdata_b_ext = {{24 {data_rdata_i[7]}}, data_rdata_i[7:0]};
			2'b01:
				if (data_sign_ext_q == 1'b0)
					rdata_b_ext = {24'h000000, data_rdata_i[15:8]};
				else
					rdata_b_ext = {{24 {data_rdata_i[15]}}, data_rdata_i[15:8]};
			2'b10:
				if (data_sign_ext_q == 1'b0)
					rdata_b_ext = {24'h000000, data_rdata_i[23:16]};
				else
					rdata_b_ext = {{24 {data_rdata_i[23]}}, data_rdata_i[23:16]};
			2'b11:
				if (data_sign_ext_q == 1'b0)
					rdata_b_ext = {24'h000000, data_rdata_i[31:24]};
				else
					rdata_b_ext = {{24 {data_rdata_i[31]}}, data_rdata_i[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_q)
			2'b00: data_rdata_ext = rdata_w_ext;
			2'b01: data_rdata_ext = rdata_h_ext;
			2'b10, 2'b11: data_rdata_ext = rdata_b_ext;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			CS <= 2'd0;
			rdata_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (data_rvalid_i && ~data_we_q) begin
				if ((data_misaligned_ex_i == 1'b1) || (data_misaligned_o == 1'b1))
					rdata_q <= data_rdata_i;
				else
					rdata_q <= data_rdata_ext;
			end
		end
	assign data_rdata_ex_o = (data_rvalid_i == 1'b1 ? data_rdata_ext : rdata_q);
	assign data_addr_o = data_addr_int;
	assign data_wdata_o = data_wdata;
	assign data_we_o = data_we_ex_i;
	assign data_be_o = data_be;
	assign misaligned_st = data_misaligned_ex_i;
	assign load_err_o = (data_gnt_i && data_err_i) && ~data_we_o;
	assign store_err_o = (data_gnt_i && data_err_i) && data_we_o;
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		data_req_o = 1'b0;
		lsu_ready_ex_o = 1'b1;
		lsu_ready_wb_o = 1'b1;
		case (CS)
			2'd0: begin
				data_req_o = data_req_ex_i;
				if (data_req_ex_i) begin
					lsu_ready_ex_o = 1'b0;
					if (data_gnt_i) begin
						lsu_ready_ex_o = 1'b1;
						if (ex_valid_i)
							NS = 2'd1;
						else
							NS = 2'd2;
					end
				end
			end
			2'd1: begin
				lsu_ready_wb_o = 1'b0;
				if (data_rvalid_i) begin
					lsu_ready_wb_o = 1'b1;
					data_req_o = data_req_ex_i;
					if (data_req_ex_i) begin
						lsu_ready_ex_o = 1'b0;
						if (data_gnt_i) begin
							lsu_ready_ex_o = 1'b1;
							if (ex_valid_i)
								NS = 2'd1;
							else
								NS = 2'd2;
						end
						else
							NS = 2'd0;
					end
					else if (data_rvalid_i)
						NS = 2'd0;
				end
			end
			2'd2: begin
				data_req_o = 1'b0;
				if (data_rvalid_i) begin
					if (ex_valid_i)
						NS = 2'd0;
					else
						NS = 2'd3;
				end
				else if (ex_valid_i)
					NS = 2'd1;
			end
			2'd3:
				if (ex_valid_i)
					NS = 2'd0;
			default: NS = 2'd0;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		data_misaligned_o = 1'b0;
		if ((data_req_ex_i == 1'b1) && (data_misaligned_ex_i == 1'b0))
			case (data_type_ex_i)
				2'b00:
					if (data_addr_int[1:0] != 2'b00)
						data_misaligned_o = 1'b1;
				2'b01:
					if (data_addr_int[1:0] == 2'b11)
						data_misaligned_o = 1'b1;
			endcase
	end
	assign data_addr_int = (addr_useincr_ex_i ? operand_a_ex_i + operand_b_ex_i : operand_a_ex_i);
	assign busy_o = (((CS == 2'd1) || (CS == 2'd2)) || (CS == 2'd3)) || (data_req_o == 1'b1);
	initial _sv2v_0 = 0;
endmodule
module riscv_mult (
	clk,
	rst_n,
	enable_i,
	operator_i,
	short_subword_i,
	short_signed_i,
	op_a_i,
	op_b_i,
	op_c_i,
	imm_i,
	dot_signed_i,
	dot_op_a_i,
	dot_op_b_i,
	dot_op_c_i,
	result_o,
	multicycle_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	parameter SHARED_DSP_MULT = 1;
	input wire clk;
	input wire rst_n;
	input wire enable_i;
	input wire [2:0] operator_i;
	input wire short_subword_i;
	input wire [1:0] short_signed_i;
	input wire [31:0] op_a_i;
	input wire [31:0] op_b_i;
	input wire [31:0] op_c_i;
	input wire [4:0] imm_i;
	input wire [1:0] dot_signed_i;
	input wire [31:0] dot_op_a_i;
	input wire [31:0] dot_op_b_i;
	input wire [31:0] dot_op_c_i;
	output reg [31:0] result_o;
	output reg multicycle_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [16:0] short_op_a;
	wire [16:0] short_op_b;
	wire [32:0] short_op_c;
	wire [33:0] short_mul;
	wire [33:0] short_mac;
	wire [31:0] short_round;
	wire [31:0] short_round_tmp;
	wire [33:0] short_result;
	wire short_mac_msb1;
	wire short_mac_msb0;
	wire [4:0] short_imm;
	wire [1:0] short_subword;
	wire [1:0] short_signed;
	wire short_shift_arith;
	reg [4:0] mulh_imm;
	reg [1:0] mulh_subword;
	reg [1:0] mulh_signed;
	reg mulh_shift_arith;
	reg mulh_carry_q;
	reg mulh_active;
	reg mulh_save;
	reg mulh_clearcarry;
	reg mulh_ready;
	reg [2:0] mulh_CS;
	reg [2:0] mulh_NS;
	assign short_round_tmp = 32'h00000001 << imm_i;
	localparam riscv_defines_MUL_IR = 3'b011;
	assign short_round = (operator_i == riscv_defines_MUL_IR ? {1'b0, short_round_tmp[31:1]} : {32 {1'sb0}});
	assign short_op_a[15:0] = (short_subword[0] ? op_a_i[31:16] : op_a_i[15:0]);
	assign short_op_b[15:0] = (short_subword[1] ? op_b_i[31:16] : op_b_i[15:0]);
	assign short_op_a[16] = short_signed[0] & short_op_a[15];
	assign short_op_b[16] = short_signed[1] & short_op_b[15];
	assign short_op_c = (mulh_active ? $signed({mulh_carry_q, op_c_i}) : $signed(op_c_i));
	assign short_mul = $signed(short_op_a) * $signed(short_op_b);
	assign short_mac = ($signed(short_op_c) + $signed(short_mul)) + $signed(short_round);
	assign short_result = $signed({short_shift_arith & short_mac_msb1, short_shift_arith & short_mac_msb0, short_mac[31:0]}) >>> short_imm;
	assign short_imm = (mulh_active ? mulh_imm : imm_i);
	assign short_subword = (mulh_active ? mulh_subword : {2 {short_subword_i}});
	assign short_signed = (mulh_active ? mulh_signed : short_signed_i);
	assign short_shift_arith = (mulh_active ? mulh_shift_arith : short_signed_i[0]);
	assign short_mac_msb1 = (mulh_active ? short_mac[33] : short_mac[31]);
	assign short_mac_msb0 = (mulh_active ? short_mac[32] : short_mac[31]);
	localparam riscv_defines_MUL_H = 3'b110;
	always @(*) begin
		if (_sv2v_0)
			;
		mulh_NS = mulh_CS;
		mulh_imm = 5'd0;
		mulh_subword = 2'b00;
		mulh_signed = 2'b00;
		mulh_shift_arith = 1'b0;
		mulh_ready = 1'b0;
		mulh_active = 1'b1;
		mulh_save = 1'b0;
		mulh_clearcarry = 1'b0;
		multicycle_o = 1'b0;
		case (mulh_CS)
			3'd0: begin
				mulh_active = 1'b0;
				mulh_ready = 1'b1;
				mulh_save = 1'b0;
				if ((operator_i == riscv_defines_MUL_H) && enable_i) begin
					mulh_ready = 1'b0;
					mulh_NS = 3'd1;
				end
			end
			3'd1: begin
				multicycle_o = 1'b1;
				mulh_imm = 5'd16;
				mulh_active = 1'b1;
				mulh_save = 1'b0;
				mulh_NS = 3'd2;
			end
			3'd2: begin
				multicycle_o = 1'b1;
				mulh_signed = {short_signed_i[1], 1'b0};
				mulh_subword = 2'b10;
				mulh_save = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd3;
			end
			3'd3: begin
				multicycle_o = 1'b1;
				mulh_signed = {1'b0, short_signed_i[0]};
				mulh_subword = 2'b01;
				mulh_imm = 5'd16;
				mulh_save = 1'b1;
				mulh_clearcarry = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd4;
			end
			3'd4: begin
				mulh_signed = short_signed_i;
				mulh_subword = 2'b11;
				mulh_ready = 1'b1;
				if (ex_ready_i)
					mulh_NS = 3'd0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			mulh_CS <= 3'd0;
			mulh_carry_q <= 1'b0;
		end
		else begin
			mulh_CS <= mulh_NS;
			if (mulh_save)
				mulh_carry_q <= ~mulh_clearcarry & short_mac[32];
			else if (ex_ready_i)
				mulh_carry_q <= 1'b0;
		end
	wire [31:0] int_op_a_msu;
	wire [31:0] int_op_b_msu;
	wire [31:0] int_result;
	wire int_is_msu;
	localparam riscv_defines_MUL_MSU32 = 3'b001;
	assign int_is_msu = operator_i == riscv_defines_MUL_MSU32;
	assign int_op_a_msu = op_a_i ^ {32 {int_is_msu}};
	assign int_op_b_msu = op_b_i & {32 {int_is_msu}};
	assign int_result = ($signed(op_c_i) + $signed(int_op_b_msu)) + ($signed(int_op_a_msu) * $signed(op_b_i));
	wire [31:0] dot_char_result;
	wire [31:0] dot_short_result;
	generate
		if (SHARED_DSP_MULT == 0) begin : genblk1
			wire [35:0] dot_char_op_a;
			wire [35:0] dot_char_op_b;
			wire [71:0] dot_char_mul;
			wire [33:0] dot_short_op_a;
			wire [33:0] dot_short_op_b;
			wire [67:0] dot_short_mul;
			assign dot_char_op_a[0+:9] = {dot_signed_i[1] & dot_op_a_i[7], dot_op_a_i[7:0]};
			assign dot_char_op_a[9+:9] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:8]};
			assign dot_char_op_a[18+:9] = {dot_signed_i[1] & dot_op_a_i[23], dot_op_a_i[23:16]};
			assign dot_char_op_a[27+:9] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:24]};
			assign dot_char_op_b[0+:9] = {dot_signed_i[0] & dot_op_b_i[7], dot_op_b_i[7:0]};
			assign dot_char_op_b[9+:9] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:8]};
			assign dot_char_op_b[18+:9] = {dot_signed_i[0] & dot_op_b_i[23], dot_op_b_i[23:16]};
			assign dot_char_op_b[27+:9] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:24]};
			assign dot_char_mul[0+:18] = $signed(dot_char_op_a[0+:9]) * $signed(dot_char_op_b[0+:9]);
			assign dot_char_mul[18+:18] = $signed(dot_char_op_a[9+:9]) * $signed(dot_char_op_b[9+:9]);
			assign dot_char_mul[36+:18] = $signed(dot_char_op_a[18+:9]) * $signed(dot_char_op_b[18+:9]);
			assign dot_char_mul[54+:18] = $signed(dot_char_op_a[27+:9]) * $signed(dot_char_op_b[27+:9]);
			assign dot_char_result = ((($signed(dot_char_mul[0+:18]) + $signed(dot_char_mul[18+:18])) + $signed(dot_char_mul[36+:18])) + $signed(dot_char_mul[54+:18])) + $signed(dot_op_c_i);
			assign dot_short_op_a[0+:17] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:0]};
			assign dot_short_op_a[17+:17] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:16]};
			assign dot_short_op_b[0+:17] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:0]};
			assign dot_short_op_b[17+:17] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]};
			assign dot_short_mul[0+:34] = $signed(dot_short_op_a[0+:17]) * $signed(dot_short_op_b[0+:17]);
			assign dot_short_mul[34+:34] = $signed(dot_short_op_a[17+:17]) * $signed(dot_short_op_b[17+:17]);
			assign dot_short_result = ($signed(dot_short_mul[31-:32]) + $signed(dot_short_mul[65-:32])) + $signed(dot_op_c_i);
		end
		else begin : genblk1
			assign dot_char_result = 1'sb0;
			assign dot_short_result = 1'sb0;
		end
	endgenerate
	localparam riscv_defines_MUL_DOT16 = 3'b101;
	localparam riscv_defines_MUL_DOT8 = 3'b100;
	localparam riscv_defines_MUL_I = 3'b010;
	localparam riscv_defines_MUL_MAC32 = 3'b000;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		(* full_case, parallel_case *)
		case (operator_i)
			riscv_defines_MUL_MAC32, riscv_defines_MUL_MSU32: result_o = int_result[31:0];
			riscv_defines_MUL_I, riscv_defines_MUL_IR, riscv_defines_MUL_H: result_o = short_result[31:0];
			riscv_defines_MUL_DOT8: result_o = dot_char_result[31:0];
			riscv_defines_MUL_DOT16: result_o = dot_short_result[31:0];
			default:
				;
		endcase
	end
	assign ready_o = mulh_ready;
	initial _sv2v_0 = 0;
endmodule
module riscv_prefetch_L0_buffer (
	clk,
	rst_n,
	req_i,
	branch_i,
	addr_i,
	hwloop_i,
	hwloop_target_i,
	ready_i,
	valid_o,
	rdata_o,
	addr_o,
	is_hwlp_o,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	busy_o
);
	reg _sv2v_0;
	parameter RDATA_IN_WIDTH = 128;
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] addr_i;
	input wire hwloop_i;
	input wire [31:0] hwloop_target_i;
	input wire ready_i;
	output wire valid_o;
	output wire [31:0] rdata_o;
	output wire [31:0] addr_o;
	output wire is_hwlp_o;
	output wire instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] instr_rdata_i;
	output wire busy_o;
	wire busy_L0;
	reg [3:0] CS;
	reg [3:0] NS;
	reg do_fetch;
	reg do_hwlp;
	reg do_hwlp_int;
	reg use_last;
	reg save_rdata_last;
	reg use_hwlp;
	reg save_rdata_hwlp;
	reg valid;
	wire hwlp_is_crossword;
	wire is_crossword;
	wire next_is_crossword;
	wire next_valid;
	wire next_upper_compressed;
	wire fetch_possible;
	wire upper_is_compressed;
	reg [31:0] addr_q;
	reg [31:0] addr_n;
	reg [31:0] addr_int;
	wire [31:0] addr_aligned_next;
	wire [31:0] addr_real_next;
	reg is_hwlp_q;
	reg is_hwlp_n;
	reg [31:0] rdata_last_q;
	wire valid_L0;
	wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] rdata_L0;
	wire [31:0] addr_L0;
	wire fetch_valid;
	wire fetch_gnt;
	wire [31:0] rdata;
	reg [31:0] rdata_unaligned;
	wire aligned_is_compressed;
	wire unaligned_is_compressed;
	wire hwlp_aligned_is_compressed;
	wire hwlp_unaligned_is_compressed;
	riscv_L0_buffer #(.RDATA_IN_WIDTH(RDATA_IN_WIDTH)) L0_buffer_i(
		.clk(clk),
		.rst_n(rst_n),
		.prefetch_i(do_fetch),
		.prefetch_addr_i(addr_real_next),
		.branch_i(branch_i),
		.branch_addr_i(addr_i),
		.hwlp_i(do_hwlp | do_hwlp_int),
		.hwlp_addr_i(hwloop_target_i),
		.fetch_gnt_o(fetch_gnt),
		.fetch_valid_o(fetch_valid),
		.valid_o(valid_L0),
		.rdata_o(rdata_L0),
		.addr_o(addr_L0),
		.instr_req_o(instr_req_o),
		.instr_addr_o(instr_addr_o),
		.instr_gnt_i(instr_gnt_i),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_rdata_i(instr_rdata_i),
		.busy_o(busy_L0)
	);
	assign rdata = (use_last || use_hwlp ? rdata_last_q : rdata_L0[addr_o[3:2] * 32+:32]);
	wire [16:1] sv2v_tmp_CDF49;
	assign sv2v_tmp_CDF49 = rdata[31:16];
	always @(*) rdata_unaligned[15:0] = sv2v_tmp_CDF49;
	always @(*) begin
		if (_sv2v_0)
			;
		case (addr_o[3:2])
			2'b00: rdata_unaligned[31:16] = rdata_L0[47-:16];
			2'b01: rdata_unaligned[31:16] = rdata_L0[79-:16];
			2'b10: rdata_unaligned[31:16] = rdata_L0[111-:16];
			2'b11: rdata_unaligned[31:16] = rdata_L0[15-:16];
		endcase
	end
	assign unaligned_is_compressed = rdata[17:16] != 2'b11;
	assign aligned_is_compressed = rdata[1:0] != 2'b11;
	assign upper_is_compressed = rdata_L0[113-:2] != 2'b11;
	assign is_crossword = (addr_o[3:1] == 3'b111) && ~upper_is_compressed;
	assign next_is_crossword = (((addr_o[3:1] == 3'b110) && aligned_is_compressed) && ~upper_is_compressed) || (((addr_o[3:1] == 3'b101) && ~unaligned_is_compressed) && ~upper_is_compressed);
	assign next_upper_compressed = (((addr_o[3:1] == 3'b110) && aligned_is_compressed) && upper_is_compressed) || (((addr_o[3:1] == 3'b101) && ~unaligned_is_compressed) && upper_is_compressed);
	assign next_valid = (((addr_o[3:2] != 2'b11) || next_upper_compressed) && ~next_is_crossword) && valid;
	assign fetch_possible = addr_o[3:2] == 2'b11;
	assign addr_aligned_next = {addr_o[31:2], 2'b00} + 32'h00000004;
	assign addr_real_next = (next_is_crossword ? {addr_o[31:4], 4'b0000} + 32'h00000016 : {addr_o[31:2], 2'b00} + 32'h00000004);
	assign hwlp_unaligned_is_compressed = rdata_L0[81-:2] != 2'b11;
	assign hwlp_aligned_is_compressed = rdata_L0[97-:2] != 2'b11;
	assign hwlp_is_crossword = (hwloop_target_i[3:1] == 3'b111) && ~upper_is_compressed;
	always @(*) begin
		if (_sv2v_0)
			;
		addr_int = addr_o;
		if (ready_i) begin
			if (addr_o[1]) begin
				if (unaligned_is_compressed)
					addr_int = {addr_aligned_next[31:2], 2'b00};
				else
					addr_int = {addr_aligned_next[31:2], 2'b10};
			end
			else if (aligned_is_compressed)
				addr_int = {addr_o[31:2], 2'b10};
			else
				addr_int = {addr_aligned_next[31:2], 2'b00};
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		do_fetch = 1'b0;
		do_hwlp = 1'b0;
		do_hwlp_int = 1'b0;
		use_last = 1'b0;
		use_hwlp = 1'b0;
		save_rdata_last = 1'b0;
		save_rdata_hwlp = 1'b0;
		valid = 1'b0;
		addr_n = addr_int;
		is_hwlp_n = is_hwlp_q;
		if (ready_i)
			is_hwlp_n = 1'b0;
		case (CS)
			4'd0:
				;
			4'd1: begin
				valid = 1'b0;
				do_fetch = fetch_possible;
				if (fetch_valid && ~is_crossword)
					valid = 1'b1;
				if (ready_i) begin
					if (hwloop_i) begin
						addr_n = addr_o;
						NS = 4'd2;
					end
					else if (next_valid) begin
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd12;
						end
						else
							NS = 4'd10;
					end
					else if (next_is_crossword) begin
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd9;
						end
						else
							NS = 4'd8;
					end
					else if (fetch_gnt)
						NS = 4'd7;
					else
						NS = 4'd6;
				end
				else if (fetch_valid) begin
					if (is_crossword) begin
						save_rdata_last = 1'b1;
						if (fetch_gnt)
							NS = 4'd9;
						else
							NS = 4'd8;
					end
					else if (fetch_gnt) begin
						save_rdata_last = 1'b1;
						NS = 4'd12;
					end
					else
						NS = 4'd10;
				end
			end
			4'd6: begin
				do_fetch = 1'b1;
				if (fetch_gnt)
					NS = 4'd7;
			end
			4'd7: begin
				valid = fetch_valid;
				do_hwlp = hwloop_i;
				if (fetch_valid)
					NS = 4'd10;
			end
			4'd8: begin
				do_fetch = 1'b1;
				if (fetch_gnt) begin
					save_rdata_last = 1'b1;
					NS = 4'd9;
				end
			end
			4'd9: begin
				valid = fetch_valid;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (fetch_valid) begin
					if (ready_i)
						NS = 4'd10;
					else
						NS = 4'd11;
				end
			end
			4'd10: begin
				valid = 1'b1;
				do_fetch = fetch_possible;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (next_is_crossword) begin
						do_fetch = 1'b1;
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd9;
						end
						else
							NS = 4'd8;
					end
					else if (~next_valid) begin
						if (fetch_gnt)
							NS = 4'd7;
						else
							NS = 4'd6;
					end
					else if (fetch_gnt) begin
						if (next_upper_compressed) begin
							save_rdata_last = 1'b1;
							NS = 4'd12;
						end
					end
				end
				else if (fetch_gnt) begin
					save_rdata_last = 1'b1;
					NS = 4'd12;
				end
			end
			4'd11: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i)
					NS = 4'd10;
			end
			4'd12: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (fetch_valid) begin
						if (next_is_crossword)
							NS = 4'd11;
						else if (next_upper_compressed)
							NS = 4'd13;
						else
							NS = 4'd10;
					end
					else if (next_is_crossword)
						NS = 4'd9;
					else if (next_upper_compressed)
						NS = 4'd12;
					else
						NS = 4'd7;
				end
				else if (fetch_valid)
					NS = 4'd13;
			end
			4'd13: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (next_is_crossword)
						NS = 4'd11;
					else if (next_upper_compressed)
						NS = 4'd13;
					else
						NS = 4'd10;
				end
			end
			4'd2: begin
				do_hwlp_int = 1'b1;
				if (fetch_gnt) begin
					is_hwlp_n = 1'b1;
					addr_n = hwloop_target_i;
					NS = 4'd1;
				end
			end
			4'd3: begin
				valid = 1'b1;
				use_hwlp = 1'b1;
				if (ready_i) begin
					addr_n = hwloop_target_i;
					if (fetch_valid) begin
						is_hwlp_n = 1'b1;
						if (hwlp_is_crossword)
							NS = 4'd8;
						else
							NS = 4'd10;
					end
					else
						NS = 4'd4;
				end
				else if (fetch_valid)
					NS = 4'd5;
			end
			4'd4: begin
				use_hwlp = 1'b1;
				if (fetch_valid) begin
					is_hwlp_n = 1'b1;
					if ((addr_L0[3:1] == 3'b111) && ~upper_is_compressed)
						NS = 4'd8;
					else
						NS = 4'd10;
				end
			end
			4'd5: begin
				valid = 1'b1;
				use_hwlp = 1'b1;
				if (ready_i) begin
					is_hwlp_n = 1'b1;
					addr_n = hwloop_target_i;
					if (hwlp_is_crossword)
						NS = 4'd8;
					else
						NS = 4'd10;
				end
			end
		endcase
		if (branch_i) begin
			is_hwlp_n = 1'b0;
			addr_n = addr_i;
			NS = 4'd1;
		end
		else if (hwloop_i) begin
			if (do_hwlp) begin
				if (ready_i) begin
					if (fetch_gnt) begin
						is_hwlp_n = 1'b1;
						addr_n = hwloop_target_i;
						NS = 4'd1;
					end
					else begin
						addr_n = addr_o;
						NS = 4'd2;
					end
				end
				else if (fetch_gnt) begin
					save_rdata_hwlp = 1'b1;
					NS = 4'd3;
				end
			end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			addr_q <= 1'sb0;
			is_hwlp_q <= 1'b0;
			CS <= 4'd0;
			rdata_last_q <= 1'sb0;
		end
		else begin
			addr_q <= addr_n;
			is_hwlp_q <= is_hwlp_n;
			CS <= NS;
			if (save_rdata_hwlp)
				rdata_last_q <= rdata_o;
			else if (save_rdata_last) begin
				if (ready_i)
					rdata_last_q <= rdata_L0[96+:32];
				else
					rdata_last_q <= rdata;
			end
		end
	assign rdata_o = (~addr_o[1] || use_hwlp ? rdata : rdata_unaligned);
	assign valid_o = valid & ~branch_i;
	assign addr_o = addr_q;
	assign is_hwlp_o = is_hwlp_q & ~branch_i;
	assign busy_o = busy_L0;
	initial _sv2v_0 = 0;
endmodule
module riscv_prefetch_buffer (
	clk,
	rst_n,
	req_i,
	branch_i,
	addr_i,
	hwloop_i,
	hwloop_target_i,
	hwlp_branch_o,
	ready_i,
	valid_o,
	rdata_o,
	addr_o,
	is_hwlp_o,
	instr_req_o,
	instr_gnt_i,
	instr_addr_o,
	instr_rdata_i,
	instr_rvalid_i,
	busy_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] addr_i;
	input wire hwloop_i;
	input wire [31:0] hwloop_target_i;
	output wire hwlp_branch_o;
	input wire ready_i;
	output wire valid_o;
	output wire [31:0] rdata_o;
	output wire [31:0] addr_o;
	output wire is_hwlp_o;
	output reg instr_req_o;
	input wire instr_gnt_i;
	output reg [31:0] instr_addr_o;
	input wire [31:0] instr_rdata_i;
	input wire instr_rvalid_i;
	output wire busy_o;
	reg [1:0] CS;
	reg [1:0] NS;
	reg [2:0] hwlp_CS;
	reg [2:0] hwlp_NS;
	reg [31:0] instr_addr_q;
	wire [31:0] fetch_addr;
	reg fetch_is_hwlp;
	reg addr_valid;
	reg fifo_valid;
	wire fifo_ready;
	reg fifo_clear;
	reg fifo_hwlp;
	wire valid_stored;
	reg hwlp_masked;
	reg hwlp_branch;
	reg hwloop_speculative;
	wire unaligned_is_compressed;
	assign busy_o = (CS != 2'd0) || instr_req_o;
	riscv_fetch_fifo fifo_i(
		.clk(clk),
		.rst_n(rst_n),
		.clear_i(fifo_clear),
		.in_addr_i(instr_addr_q),
		.in_rdata_i(instr_rdata_i),
		.in_valid_i(fifo_valid),
		.in_ready_o(fifo_ready),
		.in_replace2_i(fifo_hwlp),
		.in_is_hwlp_i(fifo_hwlp),
		.out_valid_o(valid_o),
		.out_ready_i(ready_i),
		.out_rdata_o(rdata_o),
		.out_addr_o(addr_o),
		.unaligned_is_compressed_o(unaligned_is_compressed),
		.out_valid_stored_o(valid_stored),
		.out_is_hwlp_o(is_hwlp_o)
	);
	assign fetch_addr = {instr_addr_q[31:2], 2'b00} + 32'd4;
	assign hwlp_branch_o = hwlp_branch;
	always @(*) begin
		if (_sv2v_0)
			;
		hwlp_NS = hwlp_CS;
		fifo_hwlp = 1'b0;
		fifo_clear = 1'b0;
		hwlp_branch = 1'b0;
		hwloop_speculative = 1'b0;
		(* full_case, parallel_case *)
		case (hwlp_CS)
			3'd0:
				if (hwloop_i) begin
					hwlp_masked = ~instr_addr_q[1];
					if ((valid_o & unaligned_is_compressed) & instr_addr_q[1]) begin
						hwlp_NS = 3'd4;
						hwloop_speculative = 1'b1;
					end
					else if (fetch_is_hwlp)
						hwlp_NS = 3'd2;
					else
						hwlp_NS = 3'd1;
					if (ready_i)
						fifo_clear = 1'b1;
				end
				else
					hwlp_masked = 1'b0;
			3'd4: begin
				hwlp_branch = 1'b1;
				hwlp_NS = 3'd2;
				fifo_clear = 1'b1;
			end
			3'd1: begin
				hwlp_masked = 1'b1;
				if (fetch_is_hwlp)
					hwlp_NS = 3'd2;
				if (ready_i)
					fifo_clear = 1'b1;
			end
			3'd2: begin
				hwlp_masked = 1'b0;
				fifo_hwlp = 1'b1;
				if (instr_rvalid_i & (CS != 2'd3)) begin
					if (valid_o & is_hwlp_o)
						hwlp_NS = 3'd0;
					else
						hwlp_NS = 3'd3;
				end
				else if (ready_i)
					fifo_clear = 1'b1;
			end
			3'd3: begin
				hwlp_masked = 1'b0;
				if (valid_o & is_hwlp_o)
					hwlp_NS = 3'd0;
			end
			default: begin
				hwlp_masked = 1'b0;
				hwlp_NS = 3'd0;
			end
		endcase
		if (branch_i) begin
			hwlp_NS = 3'd0;
			fifo_clear = 1'b1;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		instr_req_o = 1'b0;
		instr_addr_o = fetch_addr;
		fifo_valid = 1'b0;
		addr_valid = 1'b0;
		fetch_is_hwlp = 1'b0;
		NS = CS;
		(* full_case, parallel_case *)
		case (CS)
			2'd0: begin
				instr_addr_o = fetch_addr;
				instr_req_o = 1'b0;
				if (branch_i | hwlp_branch)
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
				else if (hwlp_masked & valid_stored)
					instr_addr_o = hwloop_target_i;
				if (req_i & (((fifo_ready | branch_i) | hwlp_branch) | (hwlp_masked & valid_stored))) begin
					instr_req_o = 1'b1;
					addr_valid = 1'b1;
					if (hwlp_masked & valid_stored)
						fetch_is_hwlp = 1'b1;
					if (instr_gnt_i)
						NS = 2'd2;
					else
						NS = 2'd1;
				end
			end
			2'd1: begin
				instr_addr_o = instr_addr_q;
				instr_req_o = 1'b1;
				if (branch_i | hwlp_branch) begin
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
					addr_valid = 1'b1;
				end
				else if (hwlp_masked & valid_stored) begin
					instr_addr_o = hwloop_target_i;
					addr_valid = 1'b1;
					fetch_is_hwlp = 1'b1;
				end
				if (instr_gnt_i)
					NS = 2'd2;
				else
					NS = 2'd1;
			end
			2'd2: begin
				instr_addr_o = fetch_addr;
				if (branch_i | hwlp_branch)
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
				else if (hwlp_masked)
					instr_addr_o = hwloop_target_i;
				if (req_i & (((fifo_ready | branch_i) | hwlp_branch) | hwlp_masked)) begin
					if (instr_rvalid_i) begin
						instr_req_o = 1'b1;
						fifo_valid = 1'b1;
						addr_valid = 1'b1;
						if (hwlp_masked)
							fetch_is_hwlp = 1'b1;
						if (instr_gnt_i)
							NS = 2'd2;
						else
							NS = 2'd1;
					end
					else if (branch_i | hwlp_branch) begin
						addr_valid = 1'b1;
						NS = 2'd3;
					end
					else if (hwlp_masked & valid_o) begin
						addr_valid = 1'b1;
						fetch_is_hwlp = 1'b1;
						NS = 2'd3;
					end
				end
				else if (instr_rvalid_i) begin
					fifo_valid = 1'b1;
					NS = 2'd0;
				end
			end
			2'd3: begin
				instr_addr_o = instr_addr_q;
				if (branch_i | hwlp_branch) begin
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
					addr_valid = 1'b1;
				end
				if (instr_rvalid_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 2'd2;
					else
						NS = 2'd1;
				end
			end
			default: begin
				NS = 2'd0;
				instr_req_o = 1'b0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			CS <= 2'd0;
			hwlp_CS <= 3'd0;
			instr_addr_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			hwlp_CS <= hwlp_NS;
			if (addr_valid)
				instr_addr_q <= (hwloop_speculative & ~branch_i ? hwloop_target_i : instr_addr_o);
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_register_file (
	clk,
	rst_n,
	test_en_i,
	fregfile_disable_i,
	raddr_a_i,
	rdata_a_o,
	raddr_b_i,
	rdata_b_o,
	raddr_c_i,
	rdata_c_o,
	waddr_a_i,
	wdata_a_i,
	we_a_i,
	waddr_b_i,
	wdata_b_i,
	we_b_i
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 5;
	parameter DATA_WIDTH = 32;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire fregfile_disable_i;
	input wire [ADDR_WIDTH - 1:0] raddr_a_i;
	output wire [DATA_WIDTH - 1:0] rdata_a_o;
	input wire [ADDR_WIDTH - 1:0] raddr_b_i;
	output wire [DATA_WIDTH - 1:0] rdata_b_o;
	input wire [ADDR_WIDTH - 1:0] raddr_c_i;
	output wire [DATA_WIDTH - 1:0] rdata_c_o;
	input wire [ADDR_WIDTH - 1:0] waddr_a_i;
	input wire [DATA_WIDTH - 1:0] wdata_a_i;
	input wire we_a_i;
	input wire [ADDR_WIDTH - 1:0] waddr_b_i;
	input wire [DATA_WIDTH - 1:0] wdata_b_i;
	input wire we_b_i;
	localparam NUM_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_FP_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_TOT_WORDS = (FPU ? NUM_WORDS + NUM_FP_WORDS : NUM_WORDS);
	reg [DATA_WIDTH - 1:0] mem [0:NUM_WORDS - 1];
	reg [NUM_TOT_WORDS - 1:1] waddr_onehot_a;
	reg [NUM_TOT_WORDS - 1:1] waddr_onehot_b;
	reg [NUM_TOT_WORDS - 1:1] waddr_onehot_b_q;
	wire [NUM_TOT_WORDS - 1:1] mem_clocks;
	reg [DATA_WIDTH - 1:0] wdata_a_q;
	reg [DATA_WIDTH - 1:0] wdata_b_q;
	wire [ADDR_WIDTH - 1:0] waddr_a;
	wire [ADDR_WIDTH - 1:0] waddr_b;
	wire clk_int;
	reg [DATA_WIDTH - 1:0] mem_fp [0:NUM_FP_WORDS - 1];
	reg [31:0] i;
	reg [31:0] j;
	reg [31:0] k;
	reg [31:0] l;
	genvar _gv_x_2;
	genvar _gv_y_2;
	wire fregfile_ena;
	assign fregfile_ena = (FPU ? ~fregfile_disable_i : 1'b1);
	generate
		if (FPU == 1) begin : genblk1
			assign rdata_a_o = (fregfile_ena & raddr_a_i[5] ? mem_fp[raddr_a_i[4:0]] : mem[raddr_a_i[4:0]]);
			assign rdata_b_o = (fregfile_ena & raddr_b_i[5] ? mem_fp[raddr_b_i[4:0]] : mem[raddr_b_i[4:0]]);
			assign rdata_c_o = (fregfile_ena & raddr_c_i[5] ? mem_fp[raddr_c_i[4:0]] : mem[raddr_c_i[4:0]]);
		end
		else begin : genblk1
			assign rdata_a_o = mem[raddr_a_i[4:0]];
			assign rdata_b_o = mem[raddr_b_i[4:0]];
			assign rdata_c_o = mem[raddr_c_i[4:0]];
		end
	endgenerate
	cluster_clock_gating CG_WE_GLOBAL(
		.clk_i(clk),
		.en_i(we_a_i | we_b_i),
		.test_en_i(test_en_i),
		.clk_o(clk_int)
	);
	always @(posedge clk_int or negedge rst_n) begin : sample_waddr
		if (~rst_n) begin
			wdata_a_q <= 1'sb0;
			wdata_b_q <= 1'sb0;
			waddr_onehot_b_q <= 1'sb0;
		end
		else begin
			if (we_a_i)
				wdata_a_q <= wdata_a_i;
			if (we_b_i)
				wdata_b_q <= wdata_b_i;
			waddr_onehot_b_q <= waddr_onehot_b;
		end
	end
	assign waddr_a = {fregfile_ena & waddr_a_i[5], waddr_a_i[4:0]};
	assign waddr_b = {fregfile_ena & waddr_b_i[5], waddr_b_i[4:0]};
	always @(*) begin : p_WADa
		if (_sv2v_0)
			;
		for (i = 1; i < NUM_TOT_WORDS; i = i + 1)
			begin : p_WordItera
				if ((we_a_i == 1'b1) && (waddr_a == i))
					waddr_onehot_a[i] = 1'b1;
				else
					waddr_onehot_a[i] = 1'b0;
			end
	end
	always @(*) begin : p_WADb
		if (_sv2v_0)
			;
		for (j = 1; j < NUM_TOT_WORDS; j = j + 1)
			begin : p_WordIterb
				if ((we_b_i == 1'b1) && (waddr_b == j))
					waddr_onehot_b[j] = 1'b1;
				else
					waddr_onehot_b[j] = 1'b0;
			end
	end
	generate
		for (_gv_x_2 = 1; _gv_x_2 < NUM_TOT_WORDS; _gv_x_2 = _gv_x_2 + 1) begin : CG_CELL_WORD_ITER
			localparam x = _gv_x_2;
			cluster_clock_gating CG_Inst(
				.clk_i(clk_int),
				.en_i(waddr_onehot_a[x] | waddr_onehot_b[x]),
				.test_en_i(test_en_i),
				.clk_o(mem_clocks[x])
			);
		end
	endgenerate
	always @(*) begin : latch_wdata
		if (_sv2v_0)
			;
		mem[0] = 1'sb0;
		for (k = 1; k < NUM_WORDS; k = k + 1)
			begin : w_WordIter
				if (mem_clocks[k] == 1'b1)
					mem[k] = (waddr_onehot_b_q[k] ? wdata_b_q : wdata_a_q);
			end
	end
	generate
		if (FPU == 1) begin : genblk3
			always @(*) begin : latch_wdata_fp
				if (_sv2v_0)
					;
				if (FPU == 1)
					for (l = 0; l < NUM_FP_WORDS; l = l + 1)
						begin : w_WordIter
							if (mem_clocks[l + NUM_WORDS] == 1'b1)
								mem_fp[l] = (waddr_onehot_b_q[l + NUM_WORDS] ? wdata_b_q : wdata_a_q);
						end
			end
		end
	endgenerate
	initial _sv2v_0 = 0;
endmodule
module cluster_clock_gating (
	clk_i,
	en_i,
	test_en_i,
	clk_o
);
	reg _sv2v_0;
	input wire clk_i;
	input wire en_i;
	input wire test_en_i;
	output wire clk_o;
	reg clk_en;
	always @(*) begin
		if (_sv2v_0)
			;
		if (clk_i == 1'b0)
			clk_en <= en_i | test_en_i;
	end
	assign clk_o = clk_i & clk_en;
	initial _sv2v_0 = 0;
endmodule
module cluster_clock_inverter (
	clk_i,
	clk_o
);
	input wire clk_i;
	output wire clk_o;
	assign clk_o = ~clk_i;
endmodule
module cluster_clock_mux2 (
	clk0_i,
	clk1_i,
	clk_sel_i,
	clk_o
);
	reg _sv2v_0;
	input wire clk0_i;
	input wire clk1_i;
	input wire clk_sel_i;
	output reg clk_o;
	always @(*) begin
		if (_sv2v_0)
			;
		if (clk_sel_i == 1'b0)
			clk_o = clk0_i;
		else
			clk_o = clk1_i;
	end
	initial _sv2v_0 = 0;
endmodule
module pulp_clock_gating (
	clk_i,
	en_i,
	test_en_i,
	clk_o
);
	reg _sv2v_0;
	input wire clk_i;
	input wire en_i;
	input wire test_en_i;
	output wire clk_o;
	reg clk_en;
	always @(*) begin
		if (_sv2v_0)
			;
		if (clk_i == 1'b0)
			clk_en <= en_i | test_en_i;
	end
	assign clk_o = clk_i & clk_en;
	initial _sv2v_0 = 0;
endmodule
module jtag_tap_top (
	tck_i,
	trst_ni,
	tms_i,
	td_i,
	td_o,
	soc_tck_o,
	soc_trstn_o,
	soc_tms_o,
	soc_tdi_o,
	soc_tdo_i,
	test_clk_i,
	test_rstn_i,
	soc_jtag_reg_i,
	soc_jtag_reg_o,
	sel_fll_clk_o,
	jtag_shift_dr_o,
	jtag_update_dr_o,
	jtag_capture_dr_o,
	axireg_sel_o,
	dbg_axi_scan_in_o,
	dbg_axi_scan_out_i
);
	input wire tck_i;
	input wire trst_ni;
	input wire tms_i;
	input wire td_i;
	output wire td_o;
	output wire soc_tck_o;
	output wire soc_trstn_o;
	output wire soc_tms_o;
	output wire soc_tdi_o;
	input wire soc_tdo_i;
	input wire test_clk_i;
	input wire test_rstn_i;
	input wire [7:0] soc_jtag_reg_i;
	output wire [7:0] soc_jtag_reg_o;
	output wire sel_fll_clk_o;
	output wire jtag_shift_dr_o;
	output wire jtag_update_dr_o;
	output wire jtag_capture_dr_o;
	output wire axireg_sel_o;
	output wire dbg_axi_scan_in_o;
	input wire dbg_axi_scan_out_i;
	wire s_scan_i;
	wire [8:0] s_confreg;
	wire confscan;
	wire confreg_sel;
	wire td_o_int;
	reg [7:0] r_soc_reg0;
	reg [7:0] r_soc_reg1;
	wire [7:0] s_soc_jtag_reg_sync;
	assign soc_trstn_o = trst_ni;
	assign soc_tms_o = tms_i;
	assign soc_tdi_o = td_o_int;
	assign soc_tck_o = tck_i;
	assign td_o = soc_tdo_i;
	tap_top tap_top_i(
		.tms_i(tms_i),
		.tck_i(tck_i),
		.rst_ni(trst_ni),
		.td_i(td_i),
		.td_o(td_o_int),
		.shift_dr_o(jtag_shift_dr_o),
		.update_dr_o(jtag_update_dr_o),
		.capture_dr_o(jtag_capture_dr_o),
		.axireg_sel_o(axireg_sel_o),
		.bbmuxreg_sel_o(),
		.clkgatereg_sel_o(),
		.confreg_sel_o(confreg_sel),
		.testmodereg_sel_o(),
		.bistreg_sel_o(),
		.scan_in_o(s_scan_i),
		.axireg_out_i(dbg_axi_scan_out_i),
		.bbmuxreg_out_i(1'b0),
		.clkgatereg_out_i(1'b0),
		.confreg_out_i(confscan),
		.testmodereg_out_i(1'b0),
		.bistreg_out_i(1'b0)
	);
	jtagreg #(
		.JTAGREGSIZE(9),
		.SYNC(0)
	) confreg(
		.clk_i(tck_i),
		.rst_ni(trst_ni),
		.enable_i(confreg_sel),
		.capture_dr_i(jtag_capture_dr_o),
		.shift_dr_i(jtag_shift_dr_o),
		.update_dr_i(jtag_update_dr_o),
		.jtagreg_in_i({1'b0, s_soc_jtag_reg_sync}),
		.mode_i(1'b1),
		.scan_in_i(s_scan_i),
		.jtagreg_out_o(s_confreg),
		.scan_out_o(confscan)
	);
	always @(posedge tck_i or negedge trst_ni)
		if (~trst_ni) begin
			r_soc_reg0 <= 0;
			r_soc_reg1 <= 0;
		end
		else begin
			r_soc_reg1 <= soc_jtag_reg_i;
			r_soc_reg0 <= r_soc_reg1;
		end
	assign s_soc_jtag_reg_sync = r_soc_reg0;
	assign dbg_axi_scan_in_o = s_scan_i;
	assign soc_jtag_reg_o = s_confreg[7:0];
	assign sel_fll_clk_o = s_confreg[8];
endmodule
module rtc_clock (
	clk_i,
	rstn_i,
	clock_update_i,
	clock_o,
	clock_i,
	init_sec_cnt_i,
	timer_update_i,
	timer_enable_i,
	timer_retrig_i,
	timer_target_i,
	timer_value_o,
	alarm_enable_i,
	alarm_update_i,
	alarm_clock_i,
	alarm_clock_o,
	event_o,
	update_day_o
);
	input wire clk_i;
	input wire rstn_i;
	input wire clock_update_i;
	output wire [21:0] clock_o;
	input wire [21:0] clock_i;
	input wire [9:0] init_sec_cnt_i;
	input wire timer_update_i;
	input wire timer_enable_i;
	input wire timer_retrig_i;
	input wire [16:0] timer_target_i;
	output wire [16:0] timer_value_o;
	input wire alarm_enable_i;
	input wire alarm_update_i;
	input wire [21:0] alarm_clock_i;
	output wire [21:0] alarm_clock_o;
	output wire event_o;
	output wire update_day_o;
	reg [7:0] r_seconds;
	reg [7:0] r_minutes;
	reg [6:0] r_hours;
	wire [7:0] s_seconds;
	wire [7:0] s_minutes;
	wire [6:0] s_hours;
	reg [7:0] r_alarm_seconds;
	reg [7:0] r_alarm_minutes;
	reg [6:0] r_alarm_hours;
	reg r_alarm_enable;
	wire [7:0] s_alarm_seconds;
	wire [7:0] s_alarm_minutes;
	wire [5:0] s_alarm_hours;
	reg [14:0] r_sec_counter;
	wire s_update_seconds;
	wire s_update_minutes;
	wire s_update_hours;
	wire s_alarm_match;
	reg r_alarm_match;
	wire s_alarm_event;
	wire s_timer_event;
	reg [16:0] r_timer;
	reg [16:0] r_timer_target;
	reg r_timer_en;
	reg r_timer_retrig;
	assign s_seconds = clock_i[7:0];
	assign s_minutes = clock_i[15:8];
	assign s_hours = clock_i[21:16];
	assign s_alarm_seconds = alarm_clock_i[7:0];
	assign s_alarm_minutes = alarm_clock_i[15:8];
	assign s_alarm_hours = alarm_clock_i[21:16];
	assign s_alarm_match = ((r_seconds == s_alarm_seconds) & (r_minutes == s_alarm_minutes)) & (r_hours == s_alarm_hours);
	assign s_alarm_event = (r_alarm_enable & s_alarm_match) & ~r_alarm_match;
	wire s_timer_match;
	assign s_timer_match = r_timer == r_timer_target;
	assign s_timer_event = r_timer_en & s_timer_match;
	assign s_update_seconds = r_sec_counter == 15'h7fff;
	assign s_update_minutes = s_update_seconds & (r_seconds == 8'h59);
	assign s_update_hours = s_update_minutes & (r_minutes == 8'h59);
	assign event_o = s_alarm_event | s_timer_event;
	assign update_day_o = s_update_hours & (r_hours == 6'h23);
	assign clock_o = {r_hours, r_minutes, r_seconds};
	assign alarm_clock_o = {r_alarm_hours, r_alarm_minutes, r_alarm_seconds};
	assign timer_value_o = r_timer;
	always @(posedge clk_i or negedge rstn_i)
		if (~rstn_i) begin
			r_alarm_seconds <= 'h0;
			r_alarm_minutes <= 'h0;
			r_alarm_hours <= 'h0;
			r_alarm_enable <= 'h1;
		end
		else if (alarm_update_i) begin
			r_alarm_enable <= alarm_enable_i;
			r_alarm_seconds <= s_alarm_seconds;
			r_alarm_minutes <= s_alarm_minutes;
			r_alarm_hours <= s_alarm_hours;
		end
		else if (s_alarm_event)
			r_alarm_enable <= 'h0;
	always @(posedge clk_i or negedge rstn_i)
		if (~rstn_i)
			r_alarm_match <= 'h0;
		else
			r_alarm_match <= s_alarm_match;
	always @(posedge clk_i or negedge rstn_i)
		if (~rstn_i) begin
			r_timer_en <= 'h0;
			r_timer_target <= 'h0;
			r_timer <= 'h0;
			r_timer_retrig <= 'h0;
		end
		else if (timer_update_i) begin
			r_timer_en <= timer_enable_i;
			r_timer_target <= timer_target_i;
			r_timer_retrig <= timer_retrig_i;
			r_timer <= 'h0;
		end
		else if (r_timer_en) begin
			if (s_timer_match) begin
				if (!r_timer_retrig)
					r_timer_en <= 0;
				r_timer <= 'h0;
			end
			else
				r_timer <= r_timer + 1;
		end
	always @(posedge clk_i or negedge rstn_i)
		if (~rstn_i)
			r_sec_counter <= 'h0;
		else if (clock_update_i)
			r_sec_counter <= {init_sec_cnt_i, 5'h00};
		else
			r_sec_counter <= r_sec_counter + 1;
	always @(posedge clk_i or negedge rstn_i)
		if (~rstn_i) begin
			r_seconds <= 0;
			r_minutes <= 0;
			r_hours <= 0;
		end
		else if (clock_update_i) begin
			r_seconds <= s_seconds;
			r_minutes <= s_minutes;
			r_hours <= s_hours;
		end
		else begin
			if (s_update_seconds) begin
				if (r_seconds[3:0] >= 4'h9)
					r_seconds[3:0] <= 4'h0;
				else
					r_seconds[3:0] <= r_seconds[3:0] + 4'h1;
				if (r_seconds >= 8'h59)
					r_seconds[7:4] <= 4'h0;
				else if (r_seconds[3:0] >= 4'h9)
					r_seconds[7:4] <= r_seconds[7:4] + 4'h1;
			end
			if (s_update_minutes) begin
				if (r_minutes[3:0] >= 4'h9)
					r_minutes[3:0] <= 4'h0;
				else
					r_minutes[3:0] <= r_minutes[3:0] + 4'h1;
				if (r_minutes >= 8'h59)
					r_minutes[7:4] <= 4'h0;
				else if (r_minutes[3:0] >= 4'h9)
					r_minutes[7:4] <= r_minutes[7:4] + 4'h1;
			end
			if (s_update_hours) begin
				if (r_hours >= 6'h23)
					r_hours <= 6'h00;
				else if (r_hours[3:0] >= 4'h9) begin
					r_hours[3:0] <= 4'h0;
					r_hours[5:4] <= r_hours[5:4] + 2'h1;
				end
				else
					r_hours[3:0] <= r_hours[3:0] + 4'h1;
			end
		end
endmodule
// ============================================================
// periph_bus_wrap_flat — SystemVerilog interfaces expanded to wires
// Auto-generated per Step 2 of flattening plan
// Replaces: periph_bus_wrap + apb_node_wrap (skipping interface layer)
// Directly instances: apb_node (which has plain wire ports)
// ============================================================

module periph_bus_wrap_flat (
    input  wire                         clk_i,
    input  wire                         rst_ni,

    input  wire [31:0]    apb_slave_paddr,
    input  wire [31:0]    apb_slave_pwdata,
    input  wire                         apb_slave_pwrite,
    input  wire                         apb_slave_psel,
    input  wire                         apb_slave_penable,
    output wire [31:0]    apb_slave_prdata,
    output wire                         apb_slave_pready,
    output wire                         apb_slave_pslverr,

    output wire [31:0]    fll_master_paddr,
    output wire [31:0]    fll_master_pwdata,
    output wire                         fll_master_pwrite,
    output wire                         fll_master_psel,
    output wire                         fll_master_penable,
    input  wire [31:0]    fll_master_prdata,
    input  wire                         fll_master_pready,
    input  wire                         fll_master_pslverr,

    output wire [31:0]    gpio_master_paddr,
    output wire [31:0]    gpio_master_pwdata,
    output wire                         gpio_master_pwrite,
    output wire                         gpio_master_psel,
    output wire                         gpio_master_penable,
    input  wire [31:0]    gpio_master_prdata,
    input  wire                         gpio_master_pready,
    input  wire                         gpio_master_pslverr,

    output wire [31:0]    udma_master_paddr,
    output wire [31:0]    udma_master_pwdata,
    output wire                         udma_master_pwrite,
    output wire                         udma_master_psel,
    output wire                         udma_master_penable,
    input  wire [31:0]    udma_master_prdata,
    input  wire                         udma_master_pready,
    input  wire                         udma_master_pslverr,

    output wire [31:0]    soc_ctrl_master_paddr,
    output wire [31:0]    soc_ctrl_master_pwdata,
    output wire                         soc_ctrl_master_pwrite,
    output wire                         soc_ctrl_master_psel,
    output wire                         soc_ctrl_master_penable,
    input  wire [31:0]    soc_ctrl_master_prdata,
    input  wire                         soc_ctrl_master_pready,
    input  wire                         soc_ctrl_master_pslverr,

    output wire [31:0]    adv_timer_master_paddr,
    output wire [31:0]    adv_timer_master_pwdata,
    output wire                         adv_timer_master_pwrite,
    output wire                         adv_timer_master_psel,
    output wire                         adv_timer_master_penable,
    input  wire [31:0]    adv_timer_master_prdata,
    input  wire                         adv_timer_master_pready,
    input  wire                         adv_timer_master_pslverr,

    output wire [31:0]    soc_evnt_gen_master_paddr,
    output wire [31:0]    soc_evnt_gen_master_pwdata,
    output wire                         soc_evnt_gen_master_pwrite,
    output wire                         soc_evnt_gen_master_psel,
    output wire                         soc_evnt_gen_master_penable,
    input  wire [31:0]    soc_evnt_gen_master_prdata,
    input  wire                         soc_evnt_gen_master_pready,
    input  wire                         soc_evnt_gen_master_pslverr,

    output wire [31:0]    eu_master_paddr,
    output wire [31:0]    eu_master_pwdata,
    output wire                         eu_master_pwrite,
    output wire                         eu_master_psel,
    output wire                         eu_master_penable,
    input  wire [31:0]    eu_master_prdata,
    input  wire                         eu_master_pready,
    input  wire                         eu_master_pslverr,

    output wire [31:0]    timer_master_paddr,
    output wire [31:0]    timer_master_pwdata,
    output wire                         timer_master_pwrite,
    output wire                         timer_master_psel,
    output wire                         timer_master_penable,
    input  wire [31:0]    timer_master_prdata,
    input  wire                         timer_master_pready,
    input  wire                         timer_master_pslverr,

    output wire [31:0]    hwpe_master_paddr,
    output wire [31:0]    hwpe_master_pwdata,
    output wire                         hwpe_master_pwrite,
    output wire                         hwpe_master_psel,
    output wire                         hwpe_master_penable,
    input  wire [31:0]    hwpe_master_prdata,
    input  wire                         hwpe_master_pready,
    input  wire                         hwpe_master_pslverr,

    output wire [31:0]    stdout_master_paddr,
    output wire [31:0]    stdout_master_pwdata,
    output wire                         stdout_master_pwrite,
    output wire                         stdout_master_psel,
    output wire                         stdout_master_penable,
    input  wire [31:0]    stdout_master_prdata,
    input  wire                         stdout_master_pready,
    input  wire                         stdout_master_pslverr,

    output wire [31:0]    mmap_debug_master_paddr,
    output wire [31:0]    mmap_debug_master_pwdata,
    output wire                         mmap_debug_master_pwrite,
    output wire                         mmap_debug_master_psel,
    output wire                         mmap_debug_master_penable,
    input  wire [31:0]    mmap_debug_master_prdata,
    input  wire                         mmap_debug_master_pready,
    input  wire                         mmap_debug_master_pslverr
);
    // Address range constants (from periph_bus_defines.sv)
    wire [10:0][31:0] start_addr_i;
    wire [10:0][31:0] end_addr_i;
    assign start_addr_i[0]  = 32'h1A100000;
    assign start_addr_i[1]  = 32'h1A101000;
    assign start_addr_i[2]  = 32'h1A102000;
    assign start_addr_i[3]  = 32'h1A104000;
    assign start_addr_i[4]  = 32'h1A105000;
    assign start_addr_i[5]  = 32'h1A106000;
    assign start_addr_i[6]  = 32'h1A109000;
    assign start_addr_i[7]  = 32'h1A10B000;
    assign start_addr_i[8]  = 32'h1A10C000;
    assign start_addr_i[9]  = 32'h1A10F000;
    assign start_addr_i[10] = 32'h1A110000;
    assign end_addr_i[0]  = 32'h1A100FFF;
    assign end_addr_i[1]  = 32'h1A101FFF;
    assign end_addr_i[2]  = 32'h1A104FFF;
    assign end_addr_i[3]  = 32'h1A104FFF;
    assign end_addr_i[4]  = 32'h1A105FFF;
    assign end_addr_i[5]  = 32'h1A106FFF;
    assign end_addr_i[6]  = 32'h1A109FFF;
    assign end_addr_i[7]  = 32'h1A10BFFF;
    assign end_addr_i[8]  = 32'h1A10CFFF;
    assign end_addr_i[9]  = 32'h1A10FFFF;
    assign end_addr_i[10] = 32'h1A11FFFF;

    // Internal wires connecting to apb_node
    wire                         apb_node_penable_i;
    wire                         apb_node_pwrite_i;
    wire [31:0]                  apb_node_paddr_i;
    wire [31:0]                  apb_node_pwdata_i;
    wire [31:0]                  apb_node_prdata_o;
    wire                         apb_node_pready_o;
    wire                         apb_node_pslverr_o;

    wire [10:0]                  apb_node_penable_o;
    wire [10:0]                  apb_node_pwrite_o;
    wire [10:0][31:0]            apb_node_paddr_o;
    wire [10:0]                  apb_node_psel_o;
    wire [10:0][31:0]            apb_node_pwdata_o;
    wire [10:0][31:0]            apb_node_prdata_i;
    wire [10:0]                  apb_node_pready_i;
    wire [10:0]                  apb_node_pslverr_i;

    // Slave port: external apb_slave <-> apb_node
    assign apb_node_penable_i = apb_slave_penable;
    assign apb_node_pwrite_i  = apb_slave_pwrite;
    assign apb_node_paddr_i   = apb_slave_paddr;
    assign apb_node_pwdata_i  = apb_slave_pwdata;
    assign apb_slave_prdata   = apb_node_prdata_o;
    assign apb_slave_pready   = apb_node_pready_o;
    assign apb_slave_pslverr  = apb_node_pslverr_o;

    // Master index 0: fll
    assign fll_master_penable = apb_node_penable_o[0];
    assign fll_master_pwrite  = apb_node_pwrite_o[0];
    assign fll_master_paddr   = apb_node_paddr_o[0];
    assign fll_master_psel    = apb_node_psel_o[0];
    assign fll_master_pwdata  = apb_node_pwdata_o[0];
    assign apb_node_prdata_i[0]  = fll_master_prdata;
    assign apb_node_pready_i[0]  = fll_master_pready;
    assign apb_node_pslverr_i[0] = fll_master_pslverr;

    assign gpio_master_penable = apb_node_penable_o[1];
    assign gpio_master_pwrite  = apb_node_pwrite_o[1];
    assign gpio_master_paddr   = apb_node_paddr_o[1];
    assign gpio_master_psel    = apb_node_psel_o[1];
    assign gpio_master_pwdata  = apb_node_pwdata_o[1];
    assign apb_node_prdata_i[1]  = gpio_master_prdata;
    assign apb_node_pready_i[1]  = gpio_master_pready;
    assign apb_node_pslverr_i[1] = gpio_master_pslverr;

    assign udma_master_penable = apb_node_penable_o[2];
    assign udma_master_pwrite  = apb_node_pwrite_o[2];
    assign udma_master_paddr   = apb_node_paddr_o[2];
    assign udma_master_psel    = apb_node_psel_o[2];
    assign udma_master_pwdata  = apb_node_pwdata_o[2];
    assign apb_node_prdata_i[2]  = udma_master_prdata;
    assign apb_node_pready_i[2]  = udma_master_pready;
    assign apb_node_pslverr_i[2] = udma_master_pslverr;

    assign soc_ctrl_master_penable = apb_node_penable_o[3];
    assign soc_ctrl_master_pwrite  = apb_node_pwrite_o[3];
    assign soc_ctrl_master_paddr   = apb_node_paddr_o[3];
    assign soc_ctrl_master_psel    = apb_node_psel_o[3];
    assign soc_ctrl_master_pwdata  = apb_node_pwdata_o[3];
    assign apb_node_prdata_i[3]  = soc_ctrl_master_prdata;
    assign apb_node_pready_i[3]  = soc_ctrl_master_pready;
    assign apb_node_pslverr_i[3] = soc_ctrl_master_pslverr;

    assign adv_timer_master_penable = apb_node_penable_o[4];
    assign adv_timer_master_pwrite  = apb_node_pwrite_o[4];
    assign adv_timer_master_paddr   = apb_node_paddr_o[4];
    assign adv_timer_master_psel    = apb_node_psel_o[4];
    assign adv_timer_master_pwdata  = apb_node_pwdata_o[4];
    assign apb_node_prdata_i[4]  = adv_timer_master_prdata;
    assign apb_node_pready_i[4]  = adv_timer_master_pready;
    assign apb_node_pslverr_i[4] = adv_timer_master_pslverr;

    assign soc_evnt_gen_master_penable = apb_node_penable_o[5];
    assign soc_evnt_gen_master_pwrite  = apb_node_pwrite_o[5];
    assign soc_evnt_gen_master_paddr   = apb_node_paddr_o[5];
    assign soc_evnt_gen_master_psel    = apb_node_psel_o[5];
    assign soc_evnt_gen_master_pwdata  = apb_node_pwdata_o[5];
    assign apb_node_prdata_i[5]  = soc_evnt_gen_master_prdata;
    assign apb_node_pready_i[5]  = soc_evnt_gen_master_pready;
    assign apb_node_pslverr_i[5] = soc_evnt_gen_master_pslverr;

    assign eu_master_penable = apb_node_penable_o[6];
    assign eu_master_pwrite  = apb_node_pwrite_o[6];
    assign eu_master_paddr   = apb_node_paddr_o[6];
    assign eu_master_psel    = apb_node_psel_o[6];
    assign eu_master_pwdata  = apb_node_pwdata_o[6];
    assign apb_node_prdata_i[6]  = eu_master_prdata;
    assign apb_node_pready_i[6]  = eu_master_pready;
    assign apb_node_pslverr_i[6] = eu_master_pslverr;

    assign timer_master_penable = apb_node_penable_o[7];
    assign timer_master_pwrite  = apb_node_pwrite_o[7];
    assign timer_master_paddr   = apb_node_paddr_o[7];
    assign timer_master_psel    = apb_node_psel_o[7];
    assign timer_master_pwdata  = apb_node_pwdata_o[7];
    assign apb_node_prdata_i[7]  = timer_master_prdata;
    assign apb_node_pready_i[7]  = timer_master_pready;
    assign apb_node_pslverr_i[7] = timer_master_pslverr;

    assign hwpe_master_penable = apb_node_penable_o[8];
    assign hwpe_master_pwrite  = apb_node_pwrite_o[8];
    assign hwpe_master_paddr   = apb_node_paddr_o[8];
    assign hwpe_master_psel    = apb_node_psel_o[8];
    assign hwpe_master_pwdata  = apb_node_pwdata_o[8];
    assign apb_node_prdata_i[8]  = hwpe_master_prdata;
    assign apb_node_pready_i[8]  = hwpe_master_pready;
    assign apb_node_pslverr_i[8] = hwpe_master_pslverr;

    assign stdout_master_penable = apb_node_penable_o[9];
    assign stdout_master_pwrite  = apb_node_pwrite_o[9];
    assign stdout_master_paddr   = apb_node_paddr_o[9];
    assign stdout_master_psel    = apb_node_psel_o[9];
    assign stdout_master_pwdata  = apb_node_pwdata_o[9];
    assign apb_node_prdata_i[9]  = stdout_master_prdata;
    assign apb_node_pready_i[9]  = stdout_master_pready;
    assign apb_node_pslverr_i[9] = stdout_master_pslverr;

    assign mmap_debug_master_penable = apb_node_penable_o[10];
    assign mmap_debug_master_pwrite  = apb_node_pwrite_o[10];
    assign mmap_debug_master_paddr   = apb_node_paddr_o[10];
    assign mmap_debug_master_psel    = apb_node_psel_o[10];
    assign mmap_debug_master_pwdata  = apb_node_pwdata_o[10];
    assign apb_node_prdata_i[10]  = mmap_debug_master_prdata;
    assign apb_node_pready_i[10]  = mmap_debug_master_pready;
    assign apb_node_pslverr_i[10] = mmap_debug_master_pslverr;

    // Instance apb_node directly (no interface layer needed)
    apb_node #(
        .NB_MASTER     (11),
        .APB_DATA_WIDTH(32),
        .APB_ADDR_WIDTH(32)
    ) apb_node_i (
        .penable_i (apb_node_penable_i),
        .pwrite_i  (apb_node_pwrite_i),
        .paddr_i   (apb_node_paddr_i),
        .pwdata_i  (apb_node_pwdata_i),
        .prdata_o  (apb_node_prdata_o),
        .pready_o  (apb_node_pready_o),
        .pslverr_o (apb_node_pslverr_o),

        .penable_o (apb_node_penable_o),
        .pwrite_o  (apb_node_pwrite_o),
        .paddr_o   (apb_node_paddr_o),
        .psel_o    (apb_node_psel_o),
        .pwdata_o  (apb_node_pwdata_o),
        .prdata_i  (apb_node_prdata_i),
        .pready_i  (apb_node_pready_i),
        .pslverr_i (apb_node_pslverr_i),

        .START_ADDR_i(start_addr_i),
        .END_ADDR_i  (end_addr_i)
    );

endmodule
//Analyze the wrapper & all the rtl using the tcl file
//Elaborate only the wrapper
/*top modules:

riscv_core
apb_gpio
periph_bus_wrap
soc_interconnect
apb_node
axi_address_decoder_AR
adbg_tap_top
mux_func
jtag_tap_top

*/

// Auto-generated stub for riscv_tracer (always included, original is `ifndef VERILATOR)
module riscv_tracer
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        fetch_enable,
  input  logic [3:0]  core_id,
  input  logic [5:0]  cluster_id,
  input  logic [31:0] pc,
  input  logic [31:0] instr,
  input  logic        compressed,
  input  logic        id_valid,
  input  logic        is_decoding,
  input  logic        pipe_flush,
  input  logic        mret,
  input  logic        uret,
  input  logic        ecall,
  input  logic        ebreak,
  input  logic [31:0] rs1_value,
  input  logic [31:0] rs2_value,
  input  logic [31:0] rs3_value,
  input  logic [31:0] rs2_value_vec,
  input  logic        rd_is_fp,
  input  logic        rs1_is_fp,
  input  logic        rs2_is_fp,
  input  logic        rs3_is_fp,
  input  logic        ex_valid,
  input  logic [ 5:0] ex_reg_addr,
  input  logic        ex_reg_we,
  input  logic [31:0] ex_reg_wdata,
  input  logic        ex_data_req,
  input  logic        ex_data_gnt,
  input  logic        ex_data_we,
  input  logic [31:0] ex_data_addr,
  input  logic [31:0] ex_data_wdata,
  input  logic        wb_bypass,
  input  logic        wb_valid,
  input  logic [ 5:0] wb_reg_addr,
  input  logic        wb_reg_we,
  input  logic [31:0] wb_reg_wdata,
  input  logic [31:0] imm_u_type,
  input  logic [31:0] imm_uj_type,
  input  logic [31:0] imm_i_type,
  input  logic [11:0] imm_iz_type,
  input  logic [31:0] imm_z_type,
  input  logic [31:0] imm_s_type,
  input  logic [31:0] imm_sb_type,
  input  logic [31:0] imm_s2_type,
  input  logic [31:0] imm_s3_type,
  input  logic [31:0] imm_vs_type,
  input  logic [31:0] imm_vu_type,
  input  logic [31:0] imm_shuffle_type,
  input  logic [ 4:0] imm_clip_type
);
endmodule


module top_wrapper_flat #(
  parameter N_EXT_PERF_COUNTERS =  0,
  parameter INSTR_RDATA_WIDTH   = 32,
  parameter PULP_SECURE         =  0,
  parameter PULP_CLUSTER        =  1,
  parameter FPU                 =  0,
  parameter SHARED_FP           =  0,
  parameter SHARED_DSP_MULT     =  0,
  parameter SHARED_INT_DIV      =  0,
  parameter SHARED_FP_DIVSQRT   =  0,
  parameter WAPUTYPE            =  0,
  parameter APU_NARGS_CPU       =  3,
  parameter APU_WOP_CPU         =  6,
  parameter APU_NDSFLAGS_CPU    = 15,
  parameter APU_NUSFLAGS_CPU    =  5,

  //apb_gpio parameters
  //parameter APB_ADDR_WIDTH = 12,

  //periph_bus_wrap parameters
  //parameter APB_ADDR_WIDTH = 32,
  //parameter APB_DATA_WIDTH = 32,

  //soc_interconnect parameters
  parameter USE_AXI           = 1,
  //parameter ADDR_WIDTH        = 32,
  parameter N_HWPE_PORTS      = 4,
  parameter N_PRIMARY_32       = 5+N_HWPE_PORTS,
  parameter N_PRIMARY_AXI_64   = 1,
  parameter DATA_WIDTH        = 32,
  parameter BE_WIDTH          = DATA_WIDTH/8,
  parameter ID_WIDTH          = N_PRIMARY_32+N_PRIMARY_AXI_64*4,
  parameter AUX_WIDTH         = 8,
  parameter N_L2_BANKS        = 4,
  parameter N_L2_BANKS_PRI    = 2,
  parameter ADDR_L2_WIDTH     = 12,
  parameter ADDR_L2_PRI_WIDTH = 12,
  parameter ROM_ADDR_WIDTH    = 10,
  // AXI PARAMS
  // 32 bit axi Interface
  parameter AXI_32_ID_WIDTH   = 12,
  parameter AXI_32_USER_WIDTH = 6,
  // 64 bit axi Interface
  parameter AXI_ADDR_WIDTH    = 32,
  parameter AXI_DATA_WIDTH    = 64,
  parameter AXI_STRB_WIDTH    = 8,
  parameter AXI_USER_WIDTH    = 6,
  parameter AXI_ID_WIDTH      = 7,

  //apb_node parameters
  parameter NB_PRIMARY = 8,
  //parameter APB_DATA_WIDTH = 32,
  //parameter APB_ADDR_WIDTH = 32,
  
  //axi_address_decoder_AR parameters
  //parameter  ADDR_WIDTH     = 32,
  parameter  N_INIT_PORT    = 8,
  parameter  N_REGION       = 4

  //adbg_tap_top parameters
  //none

  //rtc_clock parameters
  //none
)

(
input  logic clk_top,
input  logic rstn_top,

//riscv_core signals-------------------------------
  // Clock and Reset
  //input  logic        clk_i,
  //input  logic        rst_ni,

  input  logic        clock_en_i,    // enable clock, otherwise it is gated
  input  logic        test_en_i,     // enable all clock gates for testing

  input  logic        fregfile_disable_i,  // disable the fp regfile, using int regfile instead

  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [31:0] boot_addr_i,
  input  logic [ 3:0] core_id_i,
  input  logic [ 5:0] cluster_id_i,

  // Instruction memory interface
  output logic                         instr_req_o,
  input  logic                         instr_gnt_i,
  input  logic                         instr_rvalid_i,
  output logic                  [31:0] instr_addr_o,
  input  logic [INSTR_RDATA_WIDTH-1:0] instr_rdata_i,

  // Data memory interface
  output logic        data_req_o,
  input  logic        data_gnt_i,
  input  logic        data_rvalid_i,
  output logic        data_we_o,
  output logic [3:0]  data_be_o,
  output logic [31:0] data_addr_o,
  output logic [31:0] data_wdata_o,
  input  logic [31:0] data_rdata_i,
  input  logic        data_err_i,

  // apu-interconnect
  // handshake signals
  output logic                       apu_primary_req_o,
  output logic                       apu_primary_ready_o,
  input logic                        apu_primary_gnt_i,
  // request channel
  output logic [APU_NARGS_CPU*32-1:0]  apu_primary_operands_o,
  output logic [APU_WOP_CPU-1:0]      apu_primary_op_o,
  output logic [WAPUTYPE-1:0]         apu_primary_type_o,
  output logic [APU_NDSFLAGS_CPU-1:0] apu_primary_flags_o,
  // response channel
  input logic                        apu_primary_valid_i,
  input logic [31:0]                 apu_primary_result_i,
  input logic [APU_NUSFLAGS_CPU-1:0] apu_primary_flags_i,

  // Interrupt inputs
  input  logic        irq_i,                 // level sensitive IR lines
  input  logic [4:0]  irq_id_i,
  output logic        irq_ack_o,
  output logic [4:0]  irq_id_o,
  input  logic        irq_sec_i,

  output logic        sec_lvl_o,

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_gnt_o,
  output logic        debug_rvalid_o,
  input  logic [14:0] debug_addr_i,
  input  logic        debug_we_i,
  input  logic [31:0] debug_wdata_i,
  output logic [31:0] debug_rdata_o,
  output logic        debug_halted_o,
  input  logic        debug_halt_i,
  input  logic        debug_resume_i,

  // CPU Control Signals
  input  logic        fetch_enable_i,
  output logic        core_busy_o,

  input  logic [N_EXT_PERF_COUNTERS-1:0] ext_perf_counters_i,

//apb_gpio signals---------------------------------
    //    input  logic                      HCLK,
    //    input  logic                      HRESETn,

    input  logic                      dft_cg_enable_i,

    input  logic [12-1:0] PADDR, //APB_ADDR_WIDTH=12
    input  logic               [31:0] PWDATA,
    input  logic                      PWRITE,
    input  logic                      PSEL,
    input  logic                      PENABLE,
    output logic               [31:0] PRDATA,
    output logic                      PREADY,
    output logic                      PSLVERR,

    input  logic               [31:0] gpio_in,
    output logic               [31:0] gpio_in_sync,
    output logic               [31:0] gpio_out,
    output logic               [31:0] gpio_dir,
    output logic          [31:0][5:0] gpio_padcfg,
    output logic                      interrupt,

//periph_bus_wrap signals----------------------
    //    input logic    clk_i,
    //    input logic    rst_ni,
    input  wire [31:0]   apb_subordinate_paddr,
    input  wire [31:0]   apb_subordinate_pwdata,
    input  wire                        apb_subordinate_pwrite,
    input  wire                        apb_subordinate_psel,
    input  wire                        apb_subordinate_penable,
    output wire [31:0]   apb_subordinate_prdata,
    output wire                        apb_subordinate_pready,
    output wire                        apb_subordinate_pslverr,
    output wire [31:0]   fll_primary_paddr,
    output wire [31:0]   fll_primary_pwdata,
    output wire                        fll_primary_pwrite,
    output wire                        fll_primary_psel,
    output wire                        fll_primary_penable,
    input  wire [31:0]   fll_primary_prdata,
    input  wire                        fll_primary_pready,
    input  wire                        fll_primary_pslverr,
    output wire [31:0]   gpio_primary_paddr,
    output wire [31:0]   gpio_primary_pwdata,
    output wire                        gpio_primary_pwrite,
    output wire                        gpio_primary_psel,
    output wire                        gpio_primary_penable,
    input  wire [31:0]   gpio_primary_prdata,
    input  wire                        gpio_primary_pready,
    input  wire                        gpio_primary_pslverr,
    output wire [31:0]   udma_primary_paddr,
    output wire [31:0]   udma_primary_pwdata,
    output wire                        udma_primary_pwrite,
    output wire                        udma_primary_psel,
    output wire                        udma_primary_penable,
    input  wire [31:0]   udma_primary_prdata,
    input  wire                        udma_primary_pready,
    input  wire                        udma_primary_pslverr,
    output wire [31:0]   soc_ctrl_primary_paddr,
    output wire [31:0]   soc_ctrl_primary_pwdata,
    output wire                        soc_ctrl_primary_pwrite,
    output wire                        soc_ctrl_primary_psel,
    output wire                        soc_ctrl_primary_penable,
    input  wire [31:0]   soc_ctrl_primary_prdata,
    input  wire                        soc_ctrl_primary_pready,
    input  wire                        soc_ctrl_primary_pslverr,
    output wire [31:0]   adv_timer_primary_paddr,
    output wire [31:0]   adv_timer_primary_pwdata,
    output wire                        adv_timer_primary_pwrite,
    output wire                        adv_timer_primary_psel,
    output wire                        adv_timer_primary_penable,
    input  wire [31:0]   adv_timer_primary_prdata,
    input  wire                        adv_timer_primary_pready,
    input  wire                        adv_timer_primary_pslverr,
    output wire [31:0]   soc_evnt_gen_primary_paddr,
    output wire [31:0]   soc_evnt_gen_primary_pwdata,
    output wire                        soc_evnt_gen_primary_pwrite,
    output wire                        soc_evnt_gen_primary_psel,
    output wire                        soc_evnt_gen_primary_penable,
    input  wire [31:0]   soc_evnt_gen_primary_prdata,
    input  wire                        soc_evnt_gen_primary_pready,
    input  wire                        soc_evnt_gen_primary_pslverr,
    output wire [31:0]   eu_primary_paddr,
    output wire [31:0]   eu_primary_pwdata,
    output wire                        eu_primary_pwrite,
    output wire                        eu_primary_psel,
    output wire                        eu_primary_penable,
    input  wire [31:0]   eu_primary_prdata,
    input  wire                        eu_primary_pready,
    input  wire                        eu_primary_pslverr,
    output wire [31:0]   mmap_debug_primary_paddr,
    output wire [31:0]   mmap_debug_primary_pwdata,
    output wire                        mmap_debug_primary_pwrite,
    output wire                        mmap_debug_primary_psel,
    output wire                        mmap_debug_primary_penable,
    input  wire [31:0]   mmap_debug_primary_prdata,
    input  wire                        mmap_debug_primary_pready,
    input  wire                        mmap_debug_primary_pslverr,
    output wire [31:0]   timer_primary_paddr,
    output wire [31:0]   timer_primary_pwdata,
    output wire                        timer_primary_pwrite,
    output wire                        timer_primary_psel,
    output wire                        timer_primary_penable,
    input  wire [31:0]   timer_primary_prdata,
    input  wire                        timer_primary_pready,
    input  wire                        timer_primary_pslverr,
    output wire [31:0]   hwpe_primary_paddr,
    output wire [31:0]   hwpe_primary_pwdata,
    output wire                        hwpe_primary_pwrite,
    output wire                        hwpe_primary_psel,
    output wire                        hwpe_primary_penable,
    input  wire [31:0]   hwpe_primary_prdata,
    input  wire                        hwpe_primary_pready,
    input  wire                        hwpe_primary_pslverr,
    output wire [31:0]   stdout_primary_paddr,
    output wire [31:0]   stdout_primary_pwdata,
    output wire                        stdout_primary_pwrite,
    output wire                        stdout_primary_psel,
    output wire                        stdout_primary_penable,
    input  wire [31:0]   stdout_primary_prdata,
    input  wire                        stdout_primary_pready,
    input  wire                        stdout_primary_pslverr,

//soc_interconnect signals-------------------------
    //input  logic                                                clk,
    //input  logic                                                rst_n,
    input  logic                                                test_en_i_socint,
    output logic [N_L2_BANKS-1:0]     [DATA_WIDTH-1:0]          L2_D_o,
    output logic [N_L2_BANKS-1:0]     [ADDR_L2_WIDTH-1:0]       L2_A_o,
    output logic [N_L2_BANKS-1:0]                               L2_CEN_o,
    output logic [N_L2_BANKS-1:0]                               L2_WEN_o,
    output logic [N_L2_BANKS-1:0]     [BE_WIDTH-1:0]            L2_BE_o,
    output logic [N_L2_BANKS-1:0]     [31:0]                    L2_ID_o,
    input  logic [N_L2_BANKS-1:0]     [DATA_WIDTH-1:0]          L2_Q_i,
    //RISC DATA PORT
    input  logic                                                FC_DATA_req_i,
    input  logic [32-1:0]                               FC_DATA_add_i, //ADDR_WIDTH=32  
    input  logic                                                FC_DATA_wen_i,
    input  logic [DATA_WIDTH-1:0]                               FC_DATA_wdata_i,
    input  logic [BE_WIDTH-1:0]                                 FC_DATA_be_i,
    input  logic [AUX_WIDTH-1:0]                                FC_DATA_aux_i,
    output logic                                                FC_DATA_gnt_o,
    output logic [AUX_WIDTH-1:0]                                FC_DATA_r_aux_o,
    output logic                                                FC_DATA_r_valid_o,
    output logic [DATA_WIDTH-1:0]                               FC_DATA_r_rdata_o,
    output logic                                                FC_DATA_r_opc_o,
    // RISC INSTR PORT
    input  logic                                                FC_INSTR_req_i,
    input  logic [32-1:0]                               FC_INSTR_add_i, //ADDR_WIDTH=32  
    input  logic                                                FC_INSTR_wen_i,
    input  logic [DATA_WIDTH-1:0]                               FC_INSTR_wdata_i,
    input  logic [BE_WIDTH-1:0]                                 FC_INSTR_be_i,
    input  logic [AUX_WIDTH-1:0]                                FC_INSTR_aux_i,
    output logic                                                FC_INSTR_gnt_o,
    output logic [AUX_WIDTH-1:0]                                FC_INSTR_r_aux_o,
    output logic                                                FC_INSTR_r_valid_o,
    output logic [DATA_WIDTH-1:0]                               FC_INSTR_r_rdata_o,
    output logic                                                FC_INSTR_r_opc_o,
    // UDMA TX
    input  logic                                                UDMA_TX_req_i,
    input  logic [32-1:0]                               UDMA_TX_add_i,  //ADDR_WIDTH=32  
    input  logic                                                UDMA_TX_wen_i,
    input  logic [DATA_WIDTH-1:0]                               UDMA_TX_wdata_i,
    input  logic [BE_WIDTH-1:0]                                 UDMA_TX_be_i,
    input  logic [AUX_WIDTH-1:0]                                UDMA_TX_aux_i,
    output logic                                                UDMA_TX_gnt_o,
    output logic [AUX_WIDTH-1:0]                                UDMA_TX_r_aux_o,
    output logic                                                UDMA_TX_r_valid_o,
    output logic [DATA_WIDTH-1:0]                               UDMA_TX_r_rdata_o,
    output logic                                                UDMA_TX_r_opc_o,
    // UDMA RX
    input  logic                                                UDMA_RX_req_i,
    input  logic [32-1:0]                               UDMA_RX_add_i,  //ADDR_WIDTH=32  
    input  logic                                                UDMA_RX_wen_i,
    input  logic [DATA_WIDTH-1:0]                               UDMA_RX_wdata_i,
    input  logic [BE_WIDTH-1:0]                                 UDMA_RX_be_i,
    input  logic [AUX_WIDTH-1:0]                                UDMA_RX_aux_i,
    output logic                                                UDMA_RX_gnt_o,
    output logic [AUX_WIDTH-1:0]                                UDMA_RX_r_aux_o,
    output logic                                                UDMA_RX_r_valid_o,
    output logic [DATA_WIDTH-1:0]                               UDMA_RX_r_rdata_o,
    output logic                                                UDMA_RX_r_opc_o,
    // DBG
    input  logic                                                DBG_RX_req_i,
    input  logic [32-1:0]                               DBG_RX_add_i,   //ADDR_WIDTH=32  
    input  logic                                                DBG_RX_wen_i,
    input  logic [DATA_WIDTH-1:0]                               DBG_RX_wdata_i,
    input  logic [BE_WIDTH-1:0]                                 DBG_RX_be_i,
    input  logic [AUX_WIDTH-1:0]                                DBG_RX_aux_i,
    output logic                                                DBG_RX_gnt_o,
    output logic [AUX_WIDTH-1:0]                                DBG_RX_r_aux_o,
    output logic                                                DBG_RX_r_valid_o,
    output logic [DATA_WIDTH-1:0]                               DBG_RX_r_rdata_o,
    output logic                                                DBG_RX_r_opc_o,
    // HWPE
    input  logic [N_HWPE_PORTS-1:0]                             HWPE_req_i,
    input  logic [N_HWPE_PORTS-1:0]   [32-1:0]          HWPE_add_i, //ADDR_WIDTH=32  
    input  logic [N_HWPE_PORTS-1:0]                             HWPE_wen_i,
    input  logic [N_HWPE_PORTS-1:0]   [DATA_WIDTH-1:0]          HWPE_wdata_i,
    input  logic [N_HWPE_PORTS-1:0]   [BE_WIDTH-1:0]            HWPE_be_i,
    input  logic [N_HWPE_PORTS-1:0]   [AUX_WIDTH-1:0]           HWPE_aux_i,
    output logic [N_HWPE_PORTS-1:0]                             HWPE_gnt_o,
    output logic [N_HWPE_PORTS-1:0]   [AUX_WIDTH-1:0]           HWPE_r_aux_o,
    output logic [N_HWPE_PORTS-1:0]                             HWPE_r_valid_o,
    output logic [N_HWPE_PORTS-1:0]   [DATA_WIDTH-1:0]          HWPE_r_rdata_o,
    output logic [N_HWPE_PORTS-1:0]                             HWPE_r_opc_o,
    // AXI INTERFACE (FROM CLUSTER)
    input  logic [AXI_ADDR_WIDTH-1:0]                           AXI_Subordinate_aw_addr_i,
    input  logic [2:0]                                          AXI_Subordinate_aw_prot_i,
    input  logic [3:0]                                          AXI_Subordinate_aw_region_i,
    input  logic [7:0]                                          AXI_Subordinate_aw_len_i,
    input  logic [2:0]                                          AXI_Subordinate_aw_size_i,
    input  logic [1:0]                                          AXI_Subordinate_aw_burst_i,
    input  logic                                                AXI_Subordinate_aw_lock_i,
    input  logic [3:0]                                          AXI_Subordinate_aw_cache_i,
    input  logic [3:0]                                          AXI_Subordinate_aw_qos_i,
    input  logic [AXI_ID_WIDTH-1:0]                             AXI_Subordinate_aw_id_i,
    input  logic [AXI_USER_WIDTH-1:0]                           AXI_Subordinate_aw_user_i,
    input  logic                                                AXI_Subordinate_aw_valid_i,
    output logic                                                AXI_Subordinate_aw_ready_o,
    // ADDRESS READ CHANNEL
    input  logic [AXI_ADDR_WIDTH-1:0]                           AXI_Subordinate_ar_addr_i,
    input  logic [2:0]                                          AXI_Subordinate_ar_prot_i,
    input  logic [3:0]                                          AXI_Subordinate_ar_region_i,
    input  logic [7:0]                                          AXI_Subordinate_ar_len_i,
    input  logic [2:0]                                          AXI_Subordinate_ar_size_i,
    input  logic [1:0]                                          AXI_Subordinate_ar_burst_i,
    input  logic                                                AXI_Subordinate_ar_lock_i,
    input  logic [3:0]                                          AXI_Subordinate_ar_cache_i,
    input  logic [3:0]                                          AXI_Subordinate_ar_qos_i,
    input  logic [AXI_ID_WIDTH-1:0]                             AXI_Subordinate_ar_id_i,
    input  logic [AXI_USER_WIDTH-1:0]                           AXI_Subordinate_ar_user_i,
    input  logic                                                AXI_Subordinate_ar_valid_i,
    output logic                                                AXI_Subordinate_ar_ready_o,
    // WRITE CHANNEL
    input  logic [AXI_USER_WIDTH-1:0]                           AXI_Subordinate_w_user_i,
    input  logic [AXI_DATA_WIDTH-1:0]                           AXI_Subordinate_w_data_i,
    input  logic [AXI_STRB_WIDTH-1:0]                           AXI_Subordinate_w_strb_i,
    input  logic                                                AXI_Subordinate_w_last_i,
    input  logic                                                AXI_Subordinate_w_valid_i,
    output logic                                                AXI_Subordinate_w_ready_o,
    // WRITE RESPONSE CHANNEL
    output logic [AXI_ID_WIDTH-1:0]                             AXI_Subordinate_b_id_o,
    output logic [1:0]                                          AXI_Subordinate_b_resp_o,
    output logic [AXI_USER_WIDTH-1:0]                           AXI_Subordinate_b_user_o,
    output logic                                                AXI_Subordinate_b_valid_o,
    input  logic                                                AXI_Subordinate_b_ready_i,
    // READ CHANNEL
    output logic [AXI_ID_WIDTH-1:0]                             AXI_Subordinate_r_id_o,
    output logic [AXI_USER_WIDTH-1:0]                           AXI_Subordinate_r_user_o,
    output logic [AXI_DATA_WIDTH-1:0]                           AXI_Subordinate_r_data_o,
    output logic [1:0]                                          AXI_Subordinate_r_resp_o,
    output logic                                                AXI_Subordinate_r_last_o,
    output logic                                                AXI_Subordinate_r_valid_o,
    input  logic                                                AXI_Subordinate_r_ready_i,
    // BRIDGES
    // CH_0 --> APB
    output logic [32-1:0]                               APB_PADDR_o,    //ADDR_WIDTH=32  
    output logic [DATA_WIDTH-1:0]                               APB_PWDATA_o,
    output logic                                                APB_PWRITE_o,
    output logic                                                APB_PSEL_o,
    output logic                                                APB_PENABLE_o,
    input  logic [DATA_WIDTH-1:0]                               APB_PRDATA_i,
    input  logic                                                APB_PREADY_i,
    input  logic                                                APB_PSLVERR_i,
    // CH_1 --> AXI
    // ---------------------------------------------------------
    // AXI TARG Port Declarations ------------------------------
    // ---------------------------------------------------------
    //AXI write address bus -------------- // USED// -----------
    output logic [AXI_32_ID_WIDTH-1:0]                          AXI_Primary_aw_id_o,
    output logic [32-1:0]                               AXI_Primary_aw_addr_o,   //ADDR_WIDTH=32  
    output logic [7:0]                                          AXI_Primary_aw_len_o,
    output logic [2:0]                                          AXI_Primary_aw_size_o,
    output logic [1:0]                                          AXI_Primary_aw_burst_o,
    output logic                                                AXI_Primary_aw_lock_o,
    output logic [3:0]                                          AXI_Primary_aw_cache_o,
    output logic [2:0]                                          AXI_Primary_aw_prot_o,
    output logic [3:0]                                          AXI_Primary_aw_region_o,
    output logic [AXI_32_USER_WIDTH-1:0]                        AXI_Primary_aw_user_o,
    output logic [3:0]                                          AXI_Primary_aw_qos_o,
    output logic                                                AXI_Primary_aw_valid_o,
    input  logic                                                AXI_Primary_aw_ready_i,
    // ---------------------------------------------------------
    //AXI write data bus -------------- // USED// --------------
    output logic [DATA_WIDTH-1:0]                               AXI_Primary_w_data_o,
    output logic [BE_WIDTH-1:0]                                 AXI_Primary_w_strb_o,
    output logic                                                AXI_Primary_w_last_o,
    output logic [AXI_32_USER_WIDTH-1:0]                        AXI_Primary_w_user_o,
    output logic                                                AXI_Primary_w_valid_o,
    input  logic                                                AXI_Primary_w_ready_i,
    // ---------------------------------------------------------
    //AXI write response bus -------------- // USED// ----------
    input  logic [AXI_32_ID_WIDTH-1:0]                          AXI_Primary_b_id_i,
    input  logic [1:0]                                          AXI_Primary_b_resp_i,
    input  logic                                                AXI_Primary_b_valid_i,
    input  logic [AXI_32_USER_WIDTH-1:0]                        AXI_Primary_b_user_i,
    output logic                                                AXI_Primary_b_ready_o,
    // ---------------------------------------------------------
    //AXI read address bus -------------------------------------
    output logic [AXI_32_ID_WIDTH-1:0]                          AXI_Primary_ar_id_o,
    output logic [32-1:0]                               AXI_Primary_ar_addr_o,   //ADDR_WIDTH=32  
    output logic [7:0]                                          AXI_Primary_ar_len_o,
    output logic [2:0]                                          AXI_Primary_ar_size_o,
    output logic [1:0]                                          AXI_Primary_ar_burst_o,
    output logic                                                AXI_Primary_ar_lock_o,
    output logic [3:0]                                          AXI_Primary_ar_cache_o,
    output logic [2:0]                                          AXI_Primary_ar_prot_o,
    output logic [3:0]                                          AXI_Primary_ar_region_o,
    output logic [AXI_32_USER_WIDTH-1:0]                        AXI_Primary_ar_user_o,
    output logic [3:0]                                          AXI_Primary_ar_qos_o,
    output logic                                                AXI_Primary_ar_valid_o,
    input  logic                                                AXI_Primary_ar_ready_i,
    // ---------------------------------------------------------
    //AXI read data bus ----------------------------------------
    input  logic [AXI_32_ID_WIDTH-1:0]                          AXI_Primary_r_id_i,
    input  logic [DATA_WIDTH-1:0]                               AXI_Primary_r_data_i,
    input  logic [1:0]                                          AXI_Primary_r_resp_i,
    input  logic                                                AXI_Primary_r_last_i,
    input  logic [AXI_32_USER_WIDTH-1:0]                        AXI_Primary_r_user_i,
    input  logic                                                AXI_Primary_r_valid_i,
    output logic                                                AXI_Primary_r_ready_o,
    // CH_2 --> ROM
    output logic                                                rom_csn_o,
    output logic [ROM_ADDR_WIDTH-1:0]                           rom_add_o,
    input  logic [DATA_WIDTH-1:0]                               rom_rdata_i,
    // CH_3, CH_4 Private Mem Banks (L2)
    output logic [N_L2_BANKS_PRI-1:0] [DATA_WIDTH-1:0]          L2_pri_D_o,
    output logic [N_L2_BANKS_PRI-1:0] [ADDR_L2_PRI_WIDTH-1:0]   L2_pri_A_o,
    output logic [N_L2_BANKS_PRI-1:0]                           L2_pri_CEN_o,
    output logic [N_L2_BANKS_PRI-1:0]                           L2_pri_WEN_o,
    output logic [N_L2_BANKS_PRI-1:0] [BE_WIDTH-1:0]            L2_pri_BE_o,
    input  logic [N_L2_BANKS_PRI-1:0] [DATA_WIDTH-1:0]          L2_pri_Q_i,

//apb_node-------------------------------------------
    // SUBORDINATE PORT
    input  logic                                     penable_i,
    input  logic                                     pwrite_i,
    input  logic [31:0]                              paddr_i,
    input  logic [31:0]                              pwdata_i,
    output logic [31:0]                              prdata_o,
    output logic                                     pready_o,
    output logic                                     pslverr_o,

    // PRIMARY PORTS
    output logic [NB_PRIMARY-1:0]                     penable_o,
    output logic [NB_PRIMARY-1:0]                     pwrite_o,
    output logic [NB_PRIMARY-1:0][31:0]               paddr_o,
    output logic [NB_PRIMARY-1:0]                     psel_o,
    output logic [NB_PRIMARY-1:0][31:0]               pwdata_o,
    input  logic [NB_PRIMARY-1:0][31:0]               prdata_i,
    input  logic [NB_PRIMARY-1:0]                     pready_i,
    input  logic [NB_PRIMARY-1:0]                     pslverr_i,

    // CONFIGURATION PORT
    input  logic [NB_PRIMARY-1:0][32-1:0] START_ADDR_i,
    input  logic [NB_PRIMARY-1:0][32-1:0] END_ADDR_i,

//axi_address_decoder_AR------------------------------
    //input  logic                                                        clk,
    //input  logic                                                        rst_n,

    input  logic                                                        arvalid_i,
    input  logic [32-1:0]                                       araddr_i, //ADDR_WIDTH=32
    output logic                                                        arready_o,

    output logic [N_INIT_PORT-1:0]                                      arvalid_o,
    input  logic [N_INIT_PORT-1:0]                                      arready_i,

    input  logic [N_REGION-1:0][N_INIT_PORT-1:0][32-1:0]        START_ADDR_i_ar, //ADDR_WIDTH=32
    input  logic [N_REGION-1:0][N_INIT_PORT-1:0][32-1:0]        END_ADDR_i_ar, //ADDR_WIDTH=32
    input  logic [N_REGION-1:0][N_INIT_PORT-1:0]                        enable_region_i,

    input  logic [N_INIT_PORT-1:0]                                      connectivity_map_i,

    output logic                                                        incr_req_o,
    input  logic                                                        full_counter_i,
    input  logic                                                        outstanding_trans_i,

    output logic                                                        error_req_o,
    input  logic                                                        error_gnt_i,
    output logic                                                        sample_ardata_info_o,

//adbg_tap_top----------------------------------------
    // JTAG pins
    input logic    tms_pad_i,      // JTAG test mode select pad
    input logic    tck_pad_i,      // JTAG test clock pad
    input logic    trstn_pad_i,     // JTAG test reset pad
    input logic    tdi_pad_i,      // JTAG test data input pad
    output logic   tdo_pad_o,      // JTAG test data output logic  pad
    output logic   tdo_padoe_o,    // output logic  enable for JTAG test data output logic  pad 

    input logic    test_mode_i,     // test mode input

    // TAP states
    output logic   test_logic_reset_o,
    output logic   run_test_idle_o,
    output logic   shift_dr_o,
    output logic   pause_dr_o,
    output logic   update_dr_o,
    output logic   capture_dr_o,

    // Select signals for boundary scan or mbist
    output logic   extest_select_o,
    output logic   sample_preload_select_o,
    output logic   mbist_select_o,
    output logic   debug_select_o,

    // TDO signal that is connected to TDI of sub-modules.
    output logic   tdi_o,

    // TDI signals from sub-modules
    input logic    debug_tdo_i,    // from debug module
    input logic    bs_chain_tdo_i, // from Boundary Scan Chain
    input logic    mbist_tdo_i,    // from Mbist Chain

//rtc_clock-------------------------------------------
    //input  logic        clk_i,
	//input  logic        rstn_i,

	input  logic        clock_update_i,
	output logic [21:0] clock_o,
	input  logic [21:0] clock_i,

	input  logic  [9:0] init_sec_cnt_i,

	input  logic        timer_update_i,
	input  logic        timer_enable_i,
	input  logic        timer_retrig_i,
	input  logic [16:0] timer_target_i,
	output logic [16:0] timer_value_o,

	input  logic        alarm_enable_i,
	input  logic        alarm_update_i,
	input  logic [21:0] alarm_clock_i,
	output logic [21:0] alarm_clock_o,

	output logic        event_o,

	output logic        update_day_o,

//mux_func--------------------------------------------
  // global signals
  input  logic [127:0] a,
  input  logic [127:0] b,
  output logic [127:0] c,
  input  logic [127:0] d,
  //input  logic clk,
  //input  logic rst

//jtag_tap_top--------------------------------------------
    input  logic              tck_i,
    input  logic              trst_ni,
    input  logic              tms_i,
    input  logic              td_i,
    output logic              td_o,

    output logic              soc_tck_o,
    output logic              soc_trstn_o,
    output logic              soc_tms_o,
    output logic              soc_tdi_o,
    input  logic              soc_tdo_i,

    input  logic              test_clk_i,
    input  logic              test_rstn_i,

    input  logic        [7:0] soc_jtag_reg_i,
    output logic        [7:0] soc_jtag_reg_o,
    output logic              sel_fll_clk_o,

    // tap
    output logic               jtag_shift_dr_o,
    output logic               jtag_update_dr_o,
    output logic               jtag_capture_dr_o,
    output logic               axireg_sel_o,

    output logic               dbg_axi_scan_in_o,
    input  logic               dbg_axi_scan_out_i
);





//riscv_core instantiation------------------------------
riscv_core #(
 .N_EXT_PERF_COUNTERS(0),
 .INSTR_RDATA_WIDTH(32),
 .PULP_SECURE(0),
 .PULP_CLUSTER(1),
 .FPU(0),
 .SHARED_FP(0),
 .SHARED_DSP_MULT(0),
 .SHARED_INT_DIV(0),
 .SHARED_FP_DIVSQRT(0),
 .WAPUTYPE(0),
 .APU_NARGS_CPU(3),
 .APU_WOP_CPU(6),
 .APU_NDSFLAGS_CPU(15),
 .APU_NUSFLAGS_CPU(5)
 ) riscv_core(

  .clk_i (clk_top),
  .rst_ni (rstn_top),

  .clock_en_i (clock_en_i),
  .test_en_i (test_en_i),     // enable all clock gates for testing

  .fregfile_disable_i (fregfile_disable_i),  // disable the fp regfile, using int regfile instead

  // Core ID, Cluster ID and boot address are considered more or less static
  .boot_addr_i (boot_addr_i),
  .core_id_i (core_id_i),
  .cluster_id_i (cluster_id_i),

  // Instruction memory interface
  .instr_req_o (instr_req_o),
  .instr_gnt_i (instr_gnt_i),
  .instr_rvalid_i (instr_rvalid_i),
  .instr_addr_o (instr_addr_o),
  .instr_rdata_i (instr_rdata_i),

  // Data memory interface
  .data_req_o (data_req_o),
  .data_gnt_i (data_gnt_i),
  .data_rvalid_i (data_rvalid_i),
  .data_we_o (data_we_o),
  .data_be_o (data_be_o),
  .data_addr_o (data_addr_o),
  .data_wdata_o (data_wdata_o),
  .data_rdata_i (data_rdata_i),
  .data_err_i (data_err_i),

  // apu-interconnect
  // handshake signals
  .apu_master_req_o (apu_primary_req_o),
  .apu_master_ready_o (apu_primary_ready_o),
  .apu_master_gnt_i (apu_primary_gnt_i),
  // request channel
  .apu_master_operands_o (apu_primary_operands_o),
  .apu_master_op_o (apu_primary_op_o),
  .apu_master_type_o (apu_primary_type_o),
  .apu_master_flags_o (apu_primary_flags_o),
  // response channel
  .apu_master_valid_i (apu_primary_valid_i),
  .apu_master_result_i (apu_primary_result_i),
  .apu_master_flags_i (apu_primary_flags_i),

  // Interrupt inputs
  .irq_i (irq_i),                 // level sensitive IR lines
  .irq_id_i (irq_id_i),
  .irq_ack_o (irq_ack_o),
  .irq_id_o (irq_id_o),
  .irq_sec_i (irq_sec_i),

  .sec_lvl_o (sec_lvl_o),

  // Debug Interface
  .debug_req_i (debug_req_i),
  .debug_gnt_o (debug_gnt_o),
  .debug_rvalid_o (debug_rvalid_o),
  .debug_addr_i (debug_addr_i),
  .debug_we_i (debug_we_i),
  .debug_wdata_i (debug_wdata_i),
  .debug_rdata_o (debug_rdata_o),
  .debug_halted_o (debug_halted_o),
  .debug_halt_i (debug_halt_i),
  .debug_resume_i (debug_resume_i),

  // CPU Control Signals
  .fetch_enable_i (fetch_enable_i),
  .core_busy_o (core_busy_o),

  .ext_perf_counters_i (ext_perf_counters_i)
);

//apb_gpio instantiation------------------------------
apb_gpio #(
  .APB_ADDR_WIDTH(12)
  ) apb_gpio(
  .HCLK (clk_top),
  .HRESETn (rstn_top),

  .dft_cg_enable_i (dft_cg_enable_i),
  .PADDR (PADDR),
  .PWDATA (PWDATA),
  .PWRITE (PWRITE),
  .PSEL (PSEL),
  .PENABLE (PENABLE),
  .PRDATA (PRDATA),
  .PREADY (PREADY),
  .PSLVERR (PSLVERR),

  .gpio_in (gpio_in),
  .gpio_in_sync (gpio_in_sync),
  .gpio_out (gpio_out),
  .gpio_dir (gpio_dir),
  .gpio_padcfg (gpio_padcfg),
  .interrupt (interrupt)
);

//periph_bus_wrap instantiation----------------------
periph_bus_wrap_flat periph_bus_wrap_flat (
  .clk_i(clk_top),
  .rst_ni(rstn_top),
  .apb_slave_paddr(apb_subordinate_paddr),
  .apb_slave_pwdata(apb_subordinate_pwdata),
  .apb_slave_pwrite(apb_subordinate_pwrite),
  .apb_slave_psel(apb_subordinate_psel),
  .apb_slave_penable(apb_subordinate_penable),
  .apb_slave_prdata(apb_subordinate_prdata),
  .apb_slave_pready(apb_subordinate_pready),
  .apb_slave_pslverr(apb_subordinate_pslverr),
  .fll_master_paddr(fll_primary_paddr),
  .fll_master_pwdata(fll_primary_pwdata),
  .fll_master_pwrite(fll_primary_pwrite),
  .fll_master_psel(fll_primary_psel),
  .fll_master_penable(fll_primary_penable),
  .fll_master_prdata(fll_primary_prdata),
  .fll_master_pready(fll_primary_pready),
  .fll_master_pslverr(fll_primary_pslverr),
  .gpio_master_paddr(gpio_primary_paddr),
  .gpio_master_pwdata(gpio_primary_pwdata),
  .gpio_master_pwrite(gpio_primary_pwrite),
  .gpio_master_psel(gpio_primary_psel),
  .gpio_master_penable(gpio_primary_penable),
  .gpio_master_prdata(gpio_primary_prdata),
  .gpio_master_pready(gpio_primary_pready),
  .gpio_master_pslverr(gpio_primary_pslverr),
  .udma_master_paddr(udma_primary_paddr),
  .udma_master_pwdata(udma_primary_pwdata),
  .udma_master_pwrite(udma_primary_pwrite),
  .udma_master_psel(udma_primary_psel),
  .udma_master_penable(udma_primary_penable),
  .udma_master_prdata(udma_primary_prdata),
  .udma_master_pready(udma_primary_pready),
  .udma_master_pslverr(udma_primary_pslverr),
  .soc_ctrl_master_paddr(soc_ctrl_primary_paddr),
  .soc_ctrl_master_pwdata(soc_ctrl_primary_pwdata),
  .soc_ctrl_master_pwrite(soc_ctrl_primary_pwrite),
  .soc_ctrl_master_psel(soc_ctrl_primary_psel),
  .soc_ctrl_master_penable(soc_ctrl_primary_penable),
  .soc_ctrl_master_prdata(soc_ctrl_primary_prdata),
  .soc_ctrl_master_pready(soc_ctrl_primary_pready),
  .soc_ctrl_master_pslverr(soc_ctrl_primary_pslverr),
  .adv_timer_master_paddr(adv_timer_primary_paddr),
  .adv_timer_master_pwdata(adv_timer_primary_pwdata),
  .adv_timer_master_pwrite(adv_timer_primary_pwrite),
  .adv_timer_master_psel(adv_timer_primary_psel),
  .adv_timer_master_penable(adv_timer_primary_penable),
  .adv_timer_master_prdata(adv_timer_primary_prdata),
  .adv_timer_master_pready(adv_timer_primary_pready),
  .adv_timer_master_pslverr(adv_timer_primary_pslverr),
  .soc_evnt_gen_master_paddr(soc_evnt_gen_primary_paddr),
  .soc_evnt_gen_master_pwdata(soc_evnt_gen_primary_pwdata),
  .soc_evnt_gen_master_pwrite(soc_evnt_gen_primary_pwrite),
  .soc_evnt_gen_master_psel(soc_evnt_gen_primary_psel),
  .soc_evnt_gen_master_penable(soc_evnt_gen_primary_penable),
  .soc_evnt_gen_master_prdata(soc_evnt_gen_primary_prdata),
  .soc_evnt_gen_master_pready(soc_evnt_gen_primary_pready),
  .soc_evnt_gen_master_pslverr(soc_evnt_gen_primary_pslverr),
  .eu_master_paddr(eu_primary_paddr),
  .eu_master_pwdata(eu_primary_pwdata),
  .eu_master_pwrite(eu_primary_pwrite),
  .eu_master_psel(eu_primary_psel),
  .eu_master_penable(eu_primary_penable),
  .eu_master_prdata(eu_primary_prdata),
  .eu_master_pready(eu_primary_pready),
  .eu_master_pslverr(eu_primary_pslverr),
  .timer_master_paddr(timer_primary_paddr),
  .timer_master_pwdata(timer_primary_pwdata),
  .timer_master_pwrite(timer_primary_pwrite),
  .timer_master_psel(timer_primary_psel),
  .timer_master_penable(timer_primary_penable),
  .timer_master_prdata(timer_primary_prdata),
  .timer_master_pready(timer_primary_pready),
  .timer_master_pslverr(timer_primary_pslverr),
  .hwpe_master_paddr(hwpe_primary_paddr),
  .hwpe_master_pwdata(hwpe_primary_pwdata),
  .hwpe_master_pwrite(hwpe_primary_pwrite),
  .hwpe_master_psel(hwpe_primary_psel),
  .hwpe_master_penable(hwpe_primary_penable),
  .hwpe_master_prdata(hwpe_primary_prdata),
  .hwpe_master_pready(hwpe_primary_pready),
  .hwpe_master_pslverr(hwpe_primary_pslverr),
  .stdout_master_paddr(stdout_primary_paddr),
  .stdout_master_pwdata(stdout_primary_pwdata),
  .stdout_master_pwrite(stdout_primary_pwrite),
  .stdout_master_psel(stdout_primary_psel),
  .stdout_master_penable(stdout_primary_penable),
  .stdout_master_prdata(stdout_primary_prdata),
  .stdout_master_pready(stdout_primary_pready),
  .stdout_master_pslverr(stdout_primary_pslverr),
  .mmap_debug_master_paddr(mmap_debug_primary_paddr),
  .mmap_debug_master_pwdata(mmap_debug_primary_pwdata),
  .mmap_debug_master_pwrite(mmap_debug_primary_pwrite),
  .mmap_debug_master_psel(mmap_debug_primary_psel),
  .mmap_debug_master_penable(mmap_debug_primary_penable),
  .mmap_debug_master_prdata(mmap_debug_primary_prdata),
  .mmap_debug_master_pready(mmap_debug_primary_pready),
  .mmap_debug_master_pslverr(mmap_debug_primary_pslverr)
);;

//soc_interconnect instantiation------------------------------
soc_interconnect #(
  .USE_AXI(USE_AXI),
  .ADDR_WIDTH(32), //ADDR_WIDTH=32  
  .N_HWPE_PORTS(N_HWPE_PORTS),
  .N_MASTER_32(N_PRIMARY_32),
  .N_MASTER_AXI_64(N_PRIMARY_AXI_64),
  .DATA_WIDTH(DATA_WIDTH),
  .BE_WIDTH(BE_WIDTH),
  .ID_WIDTH(ID_WIDTH),
  .AUX_WIDTH(AUX_WIDTH),
  .N_L2_BANKS(N_L2_BANKS),
  .N_L2_BANKS_PRI(N_L2_BANKS_PRI),
  .ADDR_L2_WIDTH(ADDR_L2_WIDTH),
  .ADDR_L2_PRI_WIDTH(ADDR_L2_PRI_WIDTH),
  .ROM_ADDR_WIDTH(ROM_ADDR_WIDTH),
  // AXI PARAMS
  // 32 bit axi Interface
  .AXI_32_ID_WIDTH(AXI_32_ID_WIDTH),
  .AXI_32_USER_WIDTH(AXI_32_USER_WIDTH),
  // 64 bit axi Interface
  .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
  .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
  .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
  .AXI_USER_WIDTH(AXI_USER_WIDTH),
  .AXI_ID_WIDTH(AXI_ID_WIDTH)
  ) soc_interconnect (
  .clk(clk_top),
  .rst_n(rstn_top),
  .test_en_i(test_en_i),
  .L2_D_o(L2_D_o),
  .L2_A_o(L2_A_o),
  .L2_CEN_o(L2_CEN_o),
  .L2_WEN_o(L2_WEN_o),
  .L2_BE_o(L2_BE_o),
  .L2_ID_o(L2_ID_o),
  .L2_Q_i(L2_Q_i),
      //RISC DATA PORT
  .FC_DATA_req_i(FC_DATA_req_i),
  .FC_DATA_add_i(FC_DATA_add_i),
  .FC_DATA_wen_i(FC_DATA_wen_i),
  .FC_DATA_wdata_i(FC_DATA_wdata_i),
  .FC_DATA_be_i(FC_DATA_be_i),
  .FC_DATA_aux_i(FC_DATA_aux_i),
  .FC_DATA_gnt_o(FC_DATA_gnt_o),
  .FC_DATA_r_aux_o(FC_DATA_r_aux_o),
  .FC_DATA_r_valid_o(FC_DATA_r_valid_o),
  .FC_DATA_r_rdata_o(FC_DATA_r_rdata_o),
  .FC_DATA_r_opc_o(FC_DATA_r_opc_o),
      // RISC INSTR PORT
  .FC_INSTR_req_i(FC_INSTR_req_i),
  .FC_INSTR_add_i(FC_INSTR_add_i),
  .FC_INSTR_wen_i(FC_INSTR_wen_i),
  .FC_INSTR_wdata_i(FC_INSTR_wdata_i),
  .FC_INSTR_be_i(FC_INSTR_be_i),
  .FC_INSTR_aux_i(FC_INSTR_aux_i),
  .FC_INSTR_gnt_o(FC_INSTR_gnt_o),
  .FC_INSTR_r_aux_o(FC_INSTR_r_aux_o),
  .FC_INSTR_r_valid_o(FC_INSTR_r_valid_o),
  .FC_INSTR_r_rdata_o(FC_INSTR_r_rdata_o),
  .FC_INSTR_r_opc_o(FC_INSTR_r_opc_o),
      // UDMA TX
  .UDMA_TX_req_i(UDMA_TX_req_i),
  .UDMA_TX_add_i(UDMA_TX_add_i),
  .UDMA_TX_wen_i(UDMA_TX_wen_i),
  .UDMA_TX_wdata_i(UDMA_TX_wdata_i),
  .UDMA_TX_be_i(UDMA_TX_be_i),
  .UDMA_TX_aux_i(UDMA_TX_aux_i),
  .UDMA_TX_gnt_o(UDMA_TX_gnt_o),
  .UDMA_TX_r_aux_o(UDMA_TX_r_aux_o),
  .UDMA_TX_r_valid_o(UDMA_TX_r_valid_o),
  .UDMA_TX_r_rdata_o(UDMA_TX_r_rdata_o),
  .UDMA_TX_r_opc_o(UDMA_TX_r_opc_o),
      // UDMA RX
  .UDMA_RX_req_i(UDMA_RX_req_i),
  .UDMA_RX_add_i(UDMA_RX_add_i),
  .UDMA_RX_wen_i(UDMA_RX_wen_i),
  .UDMA_RX_wdata_i(UDMA_RX_wdata_i),
  .UDMA_RX_be_i(UDMA_RX_be_i),
  .UDMA_RX_aux_i(UDMA_RX_aux_i),
  .UDMA_RX_gnt_o(UDMA_RX_gnt_o),
  .UDMA_RX_r_aux_o(UDMA_RX_r_aux_o),
  .UDMA_RX_r_valid_o(UDMA_RX_r_valid_o),
  .UDMA_RX_r_rdata_o(UDMA_RX_r_rdata_o),
  .UDMA_RX_r_opc_o(UDMA_RX_r_opc_o),
      // DBG
  .DBG_RX_req_i(DBG_RX_req_i),
  .DBG_RX_add_i(DBG_RX_add_i),
  .DBG_RX_wen_i(DBG_RX_wen_i),
  .DBG_RX_wdata_i(DBG_RX_wdata_i),
  .DBG_RX_be_i(DBG_RX_be_i),
  .DBG_RX_aux_i(DBG_RX_aux_i),
  .DBG_RX_gnt_o(DBG_RX_gnt_o),
  .DBG_RX_r_aux_o(DBG_RX_r_aux_o),
  .DBG_RX_r_valid_o(DBG_RX_r_valid_o),
  .DBG_RX_r_rdata_o(DBG_RX_r_rdata_o),
  .DBG_RX_r_opc_o(DBG_RX_r_opc_o),
      // HWPE
  .HWPE_req_i(HWPE_req_i),
  .HWPE_add_i(HWPE_add_i),
  .HWPE_wen_i(HWPE_wen_i),
  .HWPE_wdata_i(HWPE_wdata_i),
  .HWPE_be_i(HWPE_be_i),
  .HWPE_aux_i(HWPE_aux_i),
  .HWPE_gnt_o(HWPE_gnt_o),
  .HWPE_r_aux_o(HWPE_r_aux_o),
  .HWPE_r_valid_o(HWPE_r_valid_o),
  .HWPE_r_rdata_o(HWPE_r_rdata_o),
  .HWPE_r_opc_o(HWPE_r_opc_o),
      // AXI INTERFACE (FROM CLUSTER)
  .AXI_Slave_aw_addr_i(AXI_Subordinate_aw_addr_i),
  .AXI_Slave_aw_prot_i(AXI_Subordinate_aw_prot_i),
  .AXI_Slave_aw_region_i(AXI_Subordinate_aw_region_i),
  .AXI_Slave_aw_len_i(AXI_Subordinate_aw_len_i),
  .AXI_Slave_aw_size_i(AXI_Subordinate_aw_size_i),
  .AXI_Slave_aw_burst_i(AXI_Subordinate_aw_burst_i),
  .AXI_Slave_aw_lock_i(AXI_Subordinate_aw_lock_i),
  .AXI_Slave_aw_cache_i(AXI_Subordinate_aw_cache_i),
  .AXI_Slave_aw_qos_i(AXI_Subordinate_aw_qos_i),
  .AXI_Slave_aw_id_i(AXI_Subordinate_aw_id_i),
  .AXI_Slave_aw_user_i(AXI_Subordinate_aw_user_i),
  .AXI_Slave_aw_valid_i(AXI_Subordinate_aw_valid_i),
  .AXI_Slave_aw_ready_o(AXI_Subordinate_aw_ready_o),
      // ADDRESS READ CHANNEL
  .AXI_Slave_ar_addr_i(AXI_Subordinate_ar_addr_i),
  .AXI_Slave_ar_prot_i(AXI_Subordinate_ar_prot_i),
  .AXI_Slave_ar_region_i(AXI_Subordinate_ar_region_i),
  .AXI_Slave_ar_len_i(AXI_Subordinate_ar_len_i),
  .AXI_Slave_ar_size_i(AXI_Subordinate_ar_size_i),
  .AXI_Slave_ar_burst_i(AXI_Subordinate_ar_burst_i),
  .AXI_Slave_ar_lock_i(AXI_Subordinate_ar_lock_i),
  .AXI_Slave_ar_cache_i(AXI_Subordinate_ar_cache_i),
  .AXI_Slave_ar_qos_i(AXI_Subordinate_ar_qos_i),
  .AXI_Slave_ar_id_i(AXI_Subordinate_ar_id_i),
  .AXI_Slave_ar_user_i(AXI_Subordinate_ar_user_i),
  .AXI_Slave_ar_valid_i(AXI_Subordinate_ar_valid_i),
  .AXI_Slave_ar_ready_o(AXI_Subordinate_ar_ready_o),
      // WRITE CHANNEL
  .AXI_Slave_w_user_i(AXI_Subordinate_w_user_i),
  .AXI_Slave_w_data_i(AXI_Subordinate_w_data_i),
  .AXI_Slave_w_strb_i(AXI_Subordinate_w_strb_i),
  .AXI_Slave_w_last_i(AXI_Subordinate_w_last_i),
  .AXI_Slave_w_valid_i(AXI_Subordinate_w_valid_i),
  .AXI_Slave_w_ready_o(AXI_Subordinate_w_ready_o),
      // WRITE RESPONSE CHANNEL
  .AXI_Slave_b_id_o(AXI_Subordinate_b_id_o),
  .AXI_Slave_b_resp_o(AXI_Subordinate_b_resp_o),
  .AXI_Slave_b_user_o(AXI_Subordinate_b_user_o),
  .AXI_Slave_b_valid_o(AXI_Subordinate_b_valid_o),
  .AXI_Slave_b_ready_i(AXI_Subordinate_b_ready_i),
      // READ CHANNEL
  .AXI_Slave_r_id_o(AXI_Subordinate_r_id_o),
  .AXI_Slave_r_user_o(AXI_Subordinate_r_user_o),
  .AXI_Slave_r_data_o(AXI_Subordinate_r_data_o),
  .AXI_Slave_r_resp_o(AXI_Subordinate_r_resp_o),
  .AXI_Slave_r_last_o(AXI_Subordinate_r_last_o),
  .AXI_Slave_r_valid_o(AXI_Subordinate_r_valid_o),
  .AXI_Slave_r_ready_i(AXI_Subordinate_r_ready_i),
      // BRIDGES
      // CH_0 --> APB
  .APB_PADDR_o(APB_PADDR_o),
  .APB_PWDATA_o(APB_PWDATA_o),
  .APB_PWRITE_o(APB_PWRITE_o),
  .APB_PSEL_o(APB_PSEL_o),
  .APB_PENABLE_o(APB_PENABLE_o),
  .APB_PRDATA_i(APB_PRDATA_i),
  .APB_PREADY_i(APB_PREADY_i),
  .APB_PSLVERR_i(APB_PSLVERR_i),
      // CH_1 --> AXI
      // ---------------------------------------------------------
      // AXI TARG Port Declarations ------------------------------
      // ---------------------------------------------------------
      //AXI write address bus -------------- // USED// -----------
  .AXI_Master_aw_id_o(AXI_Primary_aw_id_o),
  .AXI_Master_aw_addr_o(AXI_Primary_aw_addr_o),
  .AXI_Master_aw_len_o(AXI_Primary_aw_len_o),
  .AXI_Master_aw_size_o(AXI_Primary_aw_size_o),
  .AXI_Master_aw_burst_o(AXI_Primary_aw_burst_o),
  .AXI_Master_aw_lock_o(AXI_Primary_aw_lock_o),
  .AXI_Master_aw_cache_o(AXI_Primary_aw_cache_o),
  .AXI_Master_aw_prot_o(AXI_Primary_aw_prot_o),
  .AXI_Master_aw_region_o(AXI_Primary_aw_region_o),
  .AXI_Master_aw_user_o(AXI_Primary_aw_user_o),
  .AXI_Master_aw_qos_o(AXI_Primary_aw_qos_o),
  .AXI_Master_aw_valid_o(AXI_Primary_aw_valid_o),
  .AXI_Master_aw_ready_i(AXI_Primary_aw_ready_i),
      // ---------------------------------------------------------
      //AXI write data bus -------------- // USED// --------------
  .AXI_Master_w_data_o(AXI_Primary_w_data_o),
  .AXI_Master_w_strb_o(AXI_Primary_w_strb_o),
  .AXI_Master_w_last_o(AXI_Primary_w_last_o),
  .AXI_Master_w_user_o(AXI_Primary_w_user_o),
  .AXI_Master_w_valid_o(AXI_Primary_w_valid_o),
  .AXI_Master_w_ready_i(AXI_Primary_w_ready_i),
      // ---------------------------------------------------------
      //AXI write response bus -------------- // USED// ----------
  .AXI_Master_b_id_i(AXI_Primary_b_id_i),
  .AXI_Master_b_resp_i(AXI_Primary_b_resp_i),
  .AXI_Master_b_valid_i(AXI_Primary_b_valid_i),
  .AXI_Master_b_user_i(AXI_Primary_b_user_i),
  .AXI_Master_b_ready_o(AXI_Primary_b_ready_o),
      // ---------------------------------------------------------
      //AXI read address bus -------------------------------------
  .AXI_Master_ar_id_o(AXI_Primary_ar_id_o),
  .AXI_Master_ar_addr_o(AXI_Primary_ar_addr_o),
  .AXI_Master_ar_len_o(AXI_Primary_ar_len_o),
  .AXI_Master_ar_size_o(AXI_Primary_ar_size_o),
  .AXI_Master_ar_burst_o(AXI_Primary_ar_burst_o),
  .AXI_Master_ar_lock_o(AXI_Primary_ar_lock_o),
  .AXI_Master_ar_cache_o(AXI_Primary_ar_cache_o),
  .AXI_Master_ar_prot_o(AXI_Primary_ar_prot_o),
  .AXI_Master_ar_region_o(AXI_Primary_ar_region_o),
      // ---------------------------------------------------------
      //AXI read data bus ----------------------------------------
  .AXI_Master_r_id_i(AXI_Primary_r_id_i),
  .AXI_Master_r_data_i(AXI_Primary_r_data_i),
  .AXI_Master_r_resp_i(AXI_Primary_r_resp_i),
  .AXI_Master_r_last_i(AXI_Primary_r_last_i),
  .AXI_Master_r_user_i(AXI_Primary_r_user_i),
  .AXI_Master_r_valid_i(AXI_Primary_r_valid_i),
  .AXI_Master_r_ready_o(AXI_Primary_r_ready_o),
      // CH_2 --> ROM
  .rom_csn_o(rom_csn_o),
  .rom_add_o(rom_add_o),
  .rom_rdata_i(rom_rdata_i),
      // CH_3, CH_4 Private Mem Banks (L2)
  .L2_pri_D_o(L2_pri_D_o),
  .L2_pri_A_o(L2_pri_A_o),
  .L2_pri_CEN_o(L2_pri_CEN_o),
  .L2_pri_WEN_o(L2_pri_WEN_o),
  .L2_pri_BE_o(L2_pri_BE_o),
  .L2_pri_Q_i(L2_pri_Q_i)
);

//apb_node instantiation------------------------------
apb_node #(
  .NB_MASTER(NB_PRIMARY),
  .APB_DATA_WIDTH(32),
  .APB_ADDR_WIDTH(32)
  ) abpnode(
   // SUBORDINATE PORT
  .penable_i(penable_i),
  .pwrite_i(pwrite_i),
  .paddr_i(paddr_i),
  .pwdata_i(pwdata_i),
  .prdata_o(prdata_o),
  .pready_o(pready_o),
  .pslverr_o(pslverr_o),

      // MASTER PORTS
  .penable_o(penable_o),
  .pwrite_o(pwrite_o),
  .paddr_o(paddr_o),
  .psel_o(psel_o),
  .pwdata_o(pwdata_o),
  .prdata_i(prdata_i),
  .pready_i(pready_i),
  .pslverr_i(pslverr_i),

      // CONFIGURATION PORT
  .START_ADDR_i(START_ADDR_i),
  .END_ADDR_i(END_ADDR_i)
);

//axi_address_decoder_AR instantiation------------------------------
axi_address_decoder_AR #(
    .ADDR_WIDTH(32),
    .N_INIT_PORT(N_INIT_PORT),
    .N_REGION(N_REGION)
    ) axi_address_decoder_AR (
    .clk(clk_top),
    .rst_n(rstn_top),
    .arvalid_i(arvalid_i),
    .araddr_i(araddr_i),
    .arready_o(arready_o),
    .arvalid_o(arvalid_o),
    .arready_i(arready_i),

    .START_ADDR_i(START_ADDR_i_ar),
    .END_ADDR_i(END_ADDR_i_ar),
    .enable_region_i(enable_region_i),
    .connectivity_map_i(connectivity_map_i),
    .incr_req_o(incr_req_o),
    .full_counter_i(full_counter_i),
    .outstanding_trans_i(outstanding_trans_i),
    .error_req_o(error_req_o),
    .error_gnt_i(error_gnt_i),
    .sample_ardata_info_o(sample_ardata_info_o)
);

//adbg_tap_top instantiation------------------------------
adbg_tap_top adbg_tap_top (
    .tms_pad_i(tms_pad_i),      // JTAG test mode select pad
    .tck_pad_i(tck_pad_i),      // JTAG test clock pad
    .trstn_pad_i(trstn_pad_i),     // JTAG test reset pad
    .tdi_pad_i(tdi_pad_i),      // JTAG test data input pad
    .tdo_pad_o(tdo_pad_o),      // JTAG test data output logic  pad
    .tdo_padoe_o(tdo_padoe_o),    // output logic  enable for JTAG test data output logic  pad 

    .test_mode_i(test_mode_i),     // test mode input

    // TAP states
    .test_logic_reset_o(test_logic_reset_o),
    .run_test_idle_o(run_test_idle_o),
    .shift_dr_o(shift_dr_o),
    .pause_dr_o(pause_dr_o),
    .update_dr_o(update_dr_o),
    .capture_dr_o(capture_dr_o),

    // Select signals for boundary scan or mbist
    .extest_select_o(extest_select_o),
    .sample_preload_select_o(sample_preload_select_o),
    .mbist_select_o(mbist_select_o),
    .debug_select_o(debug_select_o),

    // TDO signal that is connected to TDI of sub-modules.
    .tdi_o(tdi_o),

    // TDI signals from sub-modules
    .debug_tdo_i(debug_tdo_i),    // from debug module
    .bs_chain_tdo_i(bs_chain_tdo_i), // from Boundary Scan Chain
    .mbist_tdo_i(mbist_tdo_i)    // from Mbist Chain
);

//rtc_clock instantiation------------------------------
rtc_clock rtc_clock(
    .clk_i(clk_top),
	.rstn_i(rstn_top),

	.clock_update_i(clock_update_i),
	.clock_o(clock_o),
	.clock_i(clock_i),

	.init_sec_cnt_i(init_sec_cnt_i),

	.timer_update_i(timer_update_i),
	.timer_enable_i(timer_enable_i),
	.timer_retrig_i(timer_retrig_i),
	.timer_target_i(timer_target_i),
	.timer_value_o(timer_value_o),

	.alarm_enable_i(alarm_enable_i),
	.alarm_update_i(alarm_update_i),
	.alarm_clock_i(alarm_clock_i),
	.alarm_clock_o(alarm_clock_o),

	.event_o(event_o),

	.update_day_o(update_day_o)
);

//mux_func instantiation------------------------------
mux_func mux_func(
    .a(a),
    .b(b),
    .c(c),
    .d(d),
    .clk(clk_top),
    .rst(rstn_top)
);

//jtag_tap_top instantiation------------------------------
jtag_tap_top jtag_tap_top(
    .tck_i(tck_i),
    .trst_ni(trst_ni),
    .tms_i(tms_i),
    .td_i(td_i),
    .td_o(td_o),

    .soc_tck_o(soc_tck_o),
    .soc_trstn_o(soc_trstn_o),
    .soc_tms_o(soc_tms_o),
    .soc_tdi_o(soc_tdi_o),
    .soc_tdo_i(soc_tdo_i),

    .test_clk_i(test_clk_i),
    .test_rstn_i(test_rstn_i),

    .soc_jtag_reg_i(soc_jtag_reg_i),
    .soc_jtag_reg_o(soc_jtag_reg_o),
    .sel_fll_clk_o(sel_fll_clk_o),
    .jtag_shift_dr_o(jtag_shift_dr_o),
    .jtag_update_dr_o(jtag_update_dr_o),
    .jtag_capture_dr_o(jtag_capture_dr_o),
    .axireg_sel_o(axireg_sel_o),

    .dbg_axi_scan_in_o(dbg_axi_scan_in_o),
    .dbg_axi_scan_out_i(dbg_axi_scan_out_i)
);



// periph_bus_defines (inlined for verilator)
`define NB_MASTER  11
`define FLL_START_ADDR           32'h1A10_0000
`define FLL_END_ADDR             32'h1A10_0FFF
`define GPIO_START_ADDR          32'h1A10_1000
`define GPIO_END_ADDR            32'h1A10_AFFF
`define UDMA_START_ADDR          32'h1A10_2000
`define UDMA_END_ADDR            32'h1A10_4FFF
`define SOC_CTRL_START_ADDR      32'h1A10_4000
`define SOC_CTRL_END_ADDR        32'h1A10_4FFF
`define ADV_TIMER_START_ADDR     32'h1A10_5000
`define ADV_TIMER_END_ADDR       32'h1A10_5FFF
`define SOC_EVENT_GEN_START_ADDR 32'h1A10_6000
`define SOC_EVENT_GEN_END_ADDR   32'h1A10_6FFF
`define EU_START_ADDR            32'h1A10_9000
`define EU_END_ADDR              32'h1A10_AFFF
`define TIMER_START_ADDR         32'h1A10_B000
`define TIMER_END_ADDR           32'h1A10_BFFF
`define HWPE_START_ADDR          32'h1A10_C000
`define HWPE_END_ADDR            32'h1A10_CFFF
`define STDOUT_START_ADDR        32'h1A10_F000
`define STDOUT_END_ADDR          32'h1A10_FFFF
`define DEBUG_START_ADDR         32'h1A11_0000
`define DEBUG_END_ADDR           32'h1A11_FFFF

  HACKDAC_p1: assert property (@(posedge clk_top) ((`SOC_CTRL_END_ADDR <= `UDMA_START_ADDR) && (`SOC_CTRL_START_ADDR >= `UDMA_END_ADDR))) else $display("ASSERTION VIOLATION: HACKDAC_p1");
  HACKDAC_p2: assert property (@(posedge clk_top) (~((soc_interconnect.TCDM_data_gnt_DEM_TO_XBAR) >> 1) && ((soc_interconnect.TCDM_data_add_DEM_TO_XBAR >= 32'h1C000000) && (soc_interconnect.TCDM_data_add_DEM_TO_XBAR <= 32'h1C080000)))) else $display("ASSERTION VIOLATION: HACKDAC_p2");
  HACKDAC_p3: assert property (@(posedge clk_top) (~((riscv_core.cs_registers_i.priv_lvl_n == 2'b11) && riscv_core.cs_registers_i.mstatus_n[1:0] == 2'b00))) else $display("ASSERTION VIOLATION: HACKDAC_p3");
  HACKDAC_p4: assert property (@(posedge clk_top) ~((apb_gpio.PWDATA == 32'h12345678) && ((apb_gpio.s_apb_addr ==5'b10010) && (apb_gpio.r_gpio_lock==32'h12345678)))) else $display("ASSERTION VIOLATION: HACKDAC_p4");
  HACKDAC_p5: cover property (@(posedge clk_top) ~((apb_gpio.HRESETn) || (apb_gpio.r_gpio_lock ==0)));
  HACKDAC_p6: assert property (@(posedge clk_top) (`GPIO_START_ADDR == 32'h1A101000) && (`GPIO_END_ADDR == 32'h1A101FFF)) else $display("ASSERTION VIOLATION: HACKDAC_p6");
  HACKDAC_p7: assert property (@(posedge clk_top) ((axi_address_decoder_AR.outstanding_trans_i) && (axi_address_decoder_AR.CS == axi_address_decoder_AR.NS))) else $display("ASSERTION VIOLATION: HACKDAC_p7");
  HACKDAC_p8: assert property (@(posedge clk_top) (((`GPIO_END_ADDR <= `UDMA_START_ADDR) && (`GPIO_START_ADDR >= `UDMA_END_ADDR)) && ((`SOC_CTRL_END_ADDR <= `UDMA_START_ADDR) && (`SOC_CTRL_START_ADDR >= `UDMA_END_ADDR)) && ((`SOC_CTRL_END_ADDR <= `GPIO_START_ADDR) && (`SOC_CTRL_START_ADDR >= `GPIO_END_ADDR)))) else $display("ASSERTION VIOLATION: HACKDAC_p8");
  HACKDAC_p9: assert property (@(posedge clk_top) ~((adbg_tap_top.passchk==1) && ~(adbg_tap_top.bitindex==32))) else $display("ASSERTION VIOLATION: HACKDAC_p9");
  HACKDAC_p10: assert property (@(posedge clk_top) (adbg_tap_top.passchk == 1) |-> (adbg_tap_top .bitindex == 32)) else $display("ASSERTION VIOLATION: HACKDAC_p10");
  HACKDAC_p11: assert property (@(posedge clk_top) (~( (riscv_core.debug_unit_i.dbg_halt != 1) && (riscv_core.debug_unit_i.rdata_sel_n == 3'd4)))) else $display("ASSERTION VIOLATION: HACKDAC_p11");
  HACKDAC_p12: assert property (@(posedge clk_top) (~(adbg_tap_top.passchk ==1) && (adbg_tap_top.correct <= 31))) else $display("ASSERTION VIOLATION: HACKDAC_p12");
  HACKDAC_p13: assert property (@(posedge clk_top) (riscv_core.id_stage_i.controller_i.ctrl_fsm_ns == 3'd0) |=> (riscv_core.id_stage_i.controller_i.ctrl_fsm_ns != 3'd0)) else $display("ASSERTION VIOLATION: HACKDAC_p13");
  HACKDAC_p14: assert property (@(posedge clk_top) (((riscv_core.ex_stage_i.alu_i.vector_mode_i == 2'b10) || (riscv_core.ex_stage_i.alu_i.vector_mode_i == 2'b11)) |-> riscv_core.ex_stage_i.alu_i.adder_in_a[18] == 1'b0)) else $display("ASSERTION VIOLATION: HACKDAC_p14");
  HACKDAC_p15: assert property (@(posedge clk_top) (rtc_clock.r_seconds < 8'h59)) else $display("ASSERTION VIOLATION: HACKDAC_p15");
  HACKDAC_p16: assert property (@(posedge clk_top) (adbg_tap_top.trstn_pad_i) || (adbg_tap_top.correct==0)) else $display("ASSERTION VIOLATION: HACKDAC_p16");
  HACKDAC_p21: assert property (@(posedge clk_top) (~(mux_func.c == mux_func.temperature_out))) else $display("ASSERTION VIOLATION: HACKDAC_p21");
  HACKDAC_p27: assert property (@(posedge clk_top) riscv_core.cs_registers_i.csr_we_int |-> 1'b0) else $display("ASSERTION VIOLATION: HACKDAC_p27");
  HACKDAC_p28: assert property (@(posedge clk_top) ((jtag_tap_top.td_i == 1 || jtag_tap_top.td_i == 0))) else $display("ASSERTION VIOLATION: HACKDAC_p28");
  HACKDAC_p29: assert property (@(posedge clk_top) mux_func.rst |-> mux_func.aes_out == 0 && mux_func.c == 0) else $display("ASSERTION VIOLATION: HACKDAC_p29");

endmodule

//APB_ADDR_WIDTH repeat
//APB_DATA_WIDTH repeat
//ADDR_WIDTH repeat
