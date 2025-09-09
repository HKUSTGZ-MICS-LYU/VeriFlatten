module main (
	clk,
	input_stream,
	stream_end
);
	input wire clk;
	input wire input_stream;
	output wire stream_end;
	wire main___clk;
	wire main___input_stream;
	wire main___stream_end;
	reg main___read_enable;
	reg [7:0] main___buffer;
	reg [7:0] main___data_buffer;
	reg main___packHeadStart;
	reg main___packParseStart;
	reg main___sysHeadStart;
	reg main___sysHeadParseStart;
	reg main___packetStart;
	reg main___packHeadStop;
	reg main___packParseStop;
	reg main___sysHeadStop;
	reg main___sysHeadParseStop;
	reg main___packetStop;
	reg [2:0] main___main_state;
	reg [39:0] main___scr;
	reg [23:0] main___muxRate;
	reg [15:0] main___sysHeadLen;
	reg [23:0] main___rateBnd;
	reg [15:0] main___flagInfo;
	reg [7:0] main___resvdByte;
	wire main___mod1___clk;
	wire main___mod1___input_stream;
	wire [7:0] main___mod1___output_buffer;
	reg [6:0] main___mod1___temp;
	wire main___mod2___clk;
	reg main___mod2___count_over;
	reg [2:0] main___mod2___count_value;
	wire [7:0] main___mod3___one_byte;
	wire main___mod3___start;
	wire main___mod3___enable;
	wire main___mod3___stop;
	reg [2:0] main___mod3___state;
	wire [7:0] main___mod4___one_byte;
	wire main___mod4___start;
	wire main___mod4___enable;
	wire main___mod4___stop;
	wire [39:0] main___mod4___scr_reg;
	wire [23:0] main___mod4___mux_rate_reg;
	reg [2:0] main___mod4___count;
	wire [7:0] main___mod5___one_byte;
	wire main___mod5___start;
	wire main___mod5___enable;
	wire main___mod5___stop;
	reg [2:0] main___mod5___state;
	wire [7:0] main___mod6___one_byte;
	wire main___mod6___start;
	wire main___mod6___enable;
	wire main___mod6___stop;
	wire [15:0] main___mod6___syshead_len_reg;
	wire [23:0] main___mod6___rate_bnd_reg;
	wire [15:0] main___mod6___flag_reg;
	wire [7:0] main___mod6___resvd_byte_reg;
	reg [2:0] main___mod6___count;
	reg main___mod6___flag;
	reg [1:0] main___mod6___count1;
	reg [23:0] main___mod6___Mem [0:7];
	reg [7:0] main___mod6___stream_id_temp;
	reg [15:0] main___mod6___stream_temp;
	reg [23:0] main___mod6___stream_temp1;
	reg [31:0] main___mod6___i;
	wire main___mod7___start;
	wire main___mod7___done;
	wire [7:0] main___mod7___input_stream;
	wire main___mod7___read_signal;
	wire [7:0] main___mod7___buffer_out;
	wire main___mod7___stream_done;
	reg main___mod7___prefix_stop;
	reg [3:0] main___mod7___state;
	reg [7:0] main___mod7___stream_id;
	reg [15:0] main___mod7___packet_length;
	reg main___mod7___STD_buffer_scale;
	reg [12:0] main___mod7___STD_buffer_size;
	reg main___mod7___time_stamp_start;
	reg main___mod7___time_stamp_stop;
	reg [15:0] main___mod7___NumBytes;
	reg [3:0] main___mod7___timeStampBytes;
	reg main___mod7___timeStampFlag;
	reg main___mod7___packet_done;
	wire main___mod7___module1___start;
	wire main___mod7___module1___done;
	wire [7:0] main___mod7___module1___input_stream;
	wire main___mod7___module1___read_signal;
	reg [1:0] main___mod7___module1___state;
	wire main___mod7___module2___start;
	wire main___mod7___module2___done;
	wire [7:0] main___mod7___module2___input_stream;
	wire main___mod7___module2___read_signal;
	wire [3:0] main___mod7___module2___timeBytes;
	wire main___mod7___module2___flag;
	reg [3:0] main___mod7___module2___state;
	reg [32:0] main___mod7___module2___PTS;
	reg [32:0] main___mod7___module2___DTS;
	reg [3:0] main___mod7___module2___nextstate;
	assign main___clk = clk;
	assign main___input_stream = input_stream;
	assign main___stream_end = stream_end;
	initial begin
		main___packHeadStart = 1'h1;
		main___packParseStart = 1'h0;
		main___sysHeadStart = 1'h0;
		main___sysHeadParseStart = 1'h0;
		main___packetStart = 1'h0;
		main___main_state = 3'h0;
	end
	always @(posedge clk) begin
		if (main___packHeadStop) begin
			main___packParseStart = 1'h1;
			main___packHeadStart = 1'h0;
		end
		if (main___packParseStop) begin
			main___sysHeadStart = 1'h1;
			main___packParseStart = 1'h0;
		end
		if (main___sysHeadStop) begin
			main___sysHeadParseStart = 1'h1;
			main___sysHeadStart = 1'h0;
		end
		if (main___sysHeadParseStop) begin
			main___packetStart = 1'h1;
			main___sysHeadParseStart = 1'h0;
		end
		if (main___packetStop) begin
			main___packetStart = 1'h0;
			main___packHeadStart = 1'h1;
		end
	end
	assign main___mod1___clk = main___clk;
	assign main___mod1___input_stream = main___input_stream;
	assign main___mod1___output_buffer = main___buffer;
	initial begin
		main___buffer = 8'hff;
		main___mod1___temp = 7'h7f;
	end
	always @(posedge clk) begin
		main___mod1___temp = main___buffer[3'h1+:32'h00000007];
		main___buffer = {input_stream, main___mod1___temp};
	end
	assign main___mod2___clk = main___clk;
	wire [1:1] sv2v_tmp_7ADA0;
	assign sv2v_tmp_7ADA0 = main___read_enable;
	always @(*) main___mod2___count_over = sv2v_tmp_7ADA0;
	initial begin
		main___read_enable = 1'h0;
		main___mod2___count_value = 3'h0;
	end
	always @(posedge clk)
		if (3'h7 == main___mod2___count_value) begin
			main___read_enable = 1'h1;
			main___mod2___count_value = 3'h0;
		end
		else begin
			main___read_enable = 1'h0;
			main___mod2___count_value = 3'h1 + main___mod2___count_value;
		end
	assign main___mod3___one_byte = main___buffer;
	assign main___mod3___start = main___packHeadStart;
	assign main___mod3___enable = main___clk;
	assign main___mod3___stop = main___packHeadStop;
	initial begin
		main___packHeadStop = 1'h0;
		main___mod3___state = 3'h0;
	end
	always @(posedge clk)
		if (main___packHeadStart)
			case ({29'b00000000000000000000000000000, main___mod3___state})
				32'sh00000000:
					case (main___buffer)
						8'h00: main___mod3___state = 3'h1;
						default: main___mod3___state = 3'h0;
					endcase
				32'sh00000001:
					case (main___buffer)
						8'h00: main___mod3___state = 3'h2;
						default: main___mod3___state = 3'h0;
					endcase
				32'sh00000002:
					case (main___buffer)
						8'h01: main___mod3___state = 3'h3;
						8'h00: main___mod3___state = 3'h1;
						default: main___mod3___state = 3'h0;
					endcase
				32'sh00000003:
					case (main___buffer)
						8'h5d: begin
							main___packHeadStop = 1'h1;
							main___mod3___state = 3'h0;
						end
						8'h00: main___mod3___state = 3'h1;
						default: main___mod3___state = 3'h0;
					endcase
			endcase
		else
			main___packHeadStop = 1'h0;
	assign main___mod4___one_byte = main___buffer;
	assign main___mod4___start = main___packParseStart;
	assign main___mod4___enable = main___read_enable;
	assign main___mod4___stop = main___packParseStop;
	assign main___mod4___scr_reg = main___scr;
	assign main___mod4___mux_rate_reg = main___muxRate;
	initial begin
		main___packParseStop = 1'h0;
		main___scr = 40'h0000000000;
		main___muxRate = 24'h000000;
		main___mod4___count = 3'h0;
	end
	always @(posedge main___read_enable)
		if (main___packParseStart)
			case (main___mod4___count)
				3'h0: begin
					main___scr[6'h00+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h1: begin
					main___scr[6'h08+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h2: begin
					main___scr[6'h10+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h3: begin
					main___scr[6'h18+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h4: begin
					main___scr[6'h20+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h5: begin
					main___muxRate[5'h00+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h6: begin
					main___muxRate[5'h08+:32'h00000008] = main___buffer;
					main___mod4___count = 3'h1 + main___mod4___count;
				end
				3'h7: begin
					main___muxRate[5'h10+:32'h00000008] = main___buffer;
					main___packParseStop = 1'h1;
					main___mod4___count = 3'h0;
				end
			endcase
		else
			main___packParseStop = 1'h0;
	assign main___mod5___one_byte = main___buffer;
	assign main___mod5___start = main___sysHeadStart;
	assign main___mod5___enable = main___read_enable;
	assign main___mod5___stop = main___sysHeadStop;
	initial begin
		main___sysHeadStop = 1'h0;
		main___mod5___state = 3'h0;
	end
	always @(posedge main___read_enable)
		if (main___sysHeadStart)
			case ({29'b00000000000000000000000000000, main___mod5___state})
				32'sh00000000:
					case (main___buffer)
						8'h00: main___mod5___state = 3'h1;
						default: main___mod5___state = 3'h0;
					endcase
				32'sh00000001:
					case (main___buffer)
						8'h00: main___mod5___state = 3'h2;
						default: main___mod5___state = 3'h0;
					endcase
				32'sh00000002:
					case (main___buffer)
						8'h80: main___mod5___state = 3'h3;
						default: main___mod5___state = 3'h0;
					endcase
				32'sh00000003:
					case (main___buffer)
						8'hdd: begin
							main___mod5___state = 3'h7;
							main___sysHeadStop = 1'h1;
						end
						8'h00: main___mod5___state = 3'h1;
						default: main___mod5___state = 3'h0;
					endcase
				32'sh00000007: main___sysHeadStop = 1'h0;
			endcase
	assign main___mod6___one_byte = main___buffer;
	assign main___mod6___start = main___sysHeadParseStart;
	assign main___mod6___enable = main___read_enable;
	assign main___mod6___stop = main___sysHeadParseStop;
	assign main___mod6___syshead_len_reg = main___sysHeadLen;
	assign main___mod6___rate_bnd_reg = main___rateBnd;
	assign main___mod6___flag_reg = main___flagInfo;
	assign main___mod6___resvd_byte_reg = main___resvdByte;
	initial begin
		main___sysHeadParseStop = 1'h0;
		main___sysHeadLen = 16'h0000;
		main___rateBnd = 24'h000000;
		main___flagInfo = 16'h0000;
		main___resvdByte = 8'h00;
		main___mod6___count = 3'h0;
		main___mod6___count1 = 2'h0;
		main___mod6___flag = 1'h0;
		main___mod6___Mem[3'h0] = 24'h000000;
		main___mod6___stream_temp = 16'h0000;
		main___mod6___stream_temp1 = 24'h000000;
		main___mod6___stream_id_temp = 8'h00;
	end
	always @(posedge main___read_enable) begin
		if (main___sysHeadParseStart & ~main___mod6___flag)
			case (main___mod6___count)
				3'h0: begin
					main___sysHeadLen[4'h0+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h1: begin
					main___sysHeadLen[4'h8+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h2: begin
					main___rateBnd[5'h00+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h3: begin
					main___rateBnd[5'h08+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h4: begin
					main___rateBnd[5'h10+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h5: begin
					main___flagInfo[4'h0+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h6: begin
					main___flagInfo[4'h8+:32'h00000008] = main___buffer;
					main___mod6___count = 3'h1 + main___mod6___count;
				end
				3'h7: begin
					main___resvdByte = main___buffer;
					main___mod6___flag = 1'h1;
					main___mod6___count = 3'h0;
				end
			endcase
		if (main___sysHeadParseStart & main___mod6___flag) begin
			if (main___buffer[3'h0+:32'h00000001])
				case (main___mod6___count1)
					2'h0: begin
						main___mod6___stream_id_temp = main___buffer;
						main___mod6___count1 = 2'h1 + main___mod6___count1;
					end
					2'h1: begin
						main___mod6___stream_temp[4'h0+:32'h00000008] = main___buffer;
						main___mod6___count1 = 2'h1 + main___mod6___count1;
					end
					2'h2: begin
						main___mod6___stream_temp[4'h8+:32'h00000008] = main___buffer;
						main___mod6___count1 = 2'h0;
						main___mod6___stream_temp1[5'h00+:32'h00000008] = main___mod6___stream_temp[32'h00000000+:32'h00000008];
						case (main___mod6___stream_id_temp)
							8'h03: begin
								main___mod6___Mem[3'h0] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'h83: begin
								main___mod6___Mem[3'h1] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'h43: begin
								main___mod6___Mem[3'h2] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'hc3: begin
								main___mod6___Mem[3'h3] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'h07: begin
								main___mod6___Mem[3'h4] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'h83: begin
								main___mod6___Mem[3'h5] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'h43: begin
								main___mod6___Mem[3'h6] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
							8'hc3: begin
								main___mod6___Mem[3'h7] = main___mod6___stream_temp1;
								main___sysHeadParseStop = 1'h1;
							end
						endcase
					end
				endcase
		end
		if (~main___sysHeadParseStart)
			main___sysHeadParseStop = 1'h0;
	end
	assign main___mod7___start = main___packetStart;
	assign main___mod7___done = main___packetStop;
	assign main___mod7___input_stream = main___buffer;
	assign main___mod7___read_signal = main___read_enable;
	assign main___mod7___buffer_out = main___data_buffer;
	assign main___mod7___stream_done = main___stream_end;
	initial begin
		main___mod7___state = 4'hf;
		main___mod7___time_stamp_start = 1'h0;
		main___mod7___timeStampFlag = 1'h0;
		main___packetStop = 1'h0;
		main___mod7___STD_buffer_size = 13'h0000;
		main___mod7___NumBytes = 16'h0000;
		main___mod7___STD_buffer_scale = 1'h0;
		main___mod7___packet_length = 16'h0000;
		main___data_buffer = 8'h00;
		main___mod7___stream_id = 8'h00;
		stream_end = 1'h0;
		main___mod7___packet_done = 1'h1;
	end
	always @(posedge main___read_enable)
		case ({28'b0000000000000000000000000000, main___mod7___state})
			32'sh0000000f:
				if (main___packetStart)
					main___mod7___state = 4'h0;
			32'sh00000000:
				if (main___mod7___prefix_stop) begin
					if (8'hba == main___buffer) begin
						main___packetStop = 1'h1;
						main___mod7___state = 4'h8;
					end
					else if (8'hb9 == main___buffer) begin
						main___packetStop = 1'h1;
						stream_end = 1'h1;
						main___mod7___state = 4'h8;
					end
					else begin
						main___mod7___state = 4'h2;
						main___mod7___packet_done = 1'h0;
						main___mod7___stream_id = main___buffer;
					end
				end
			32'sh00000002: begin
				main___mod7___packet_length[4'h8+:32'h00000008] = main___buffer;
				main___mod7___state = 4'h3;
			end
			32'sh00000003: begin
				main___mod7___packet_length[4'h0+:32'h00000008] = main___buffer;
				main___mod7___NumBytes = main___mod7___packet_length;
				if (8'hbf == main___mod7___stream_id) begin
					main___mod7___time_stamp_start = 1'h0;
					main___mod7___state = 4'h7;
				end
				else
					main___mod7___state = 4'h4;
			end
			32'sh00000004:
				if (8'hff != main___buffer) begin
					if (2'h1 == main___buffer[3'h0+:32'h00000002]) begin
						main___mod7___state = 4'h5;
						main___mod7___STD_buffer_scale = main___buffer[3'h2+:32'h00000001];
						main___mod7___STD_buffer_size[4'h0+:32'h00000005] = main___buffer[3'h3+:32'h00000005];
					end
					else begin
						main___mod7___state = 4'h6;
						main___mod7___timeStampFlag = 1'h1;
						main___mod7___time_stamp_start = 1'h1;
					end
				end
				else
					main___mod7___NumBytes = main___mod7___NumBytes - 16'h0001;
			32'sh00000005: begin
				main___mod7___state = 4'h6;
				main___mod7___STD_buffer_size[4'h5+:32'h00000008] = main___buffer;
				main___mod7___NumBytes = main___mod7___NumBytes - 16'h0001;
				main___mod7___timeStampFlag = 1'h0;
				main___mod7___time_stamp_start = 1'h1;
			end
			32'sh00000006:
				if (main___mod7___time_stamp_stop) begin
					if (32'sh00000000 == ({16'b0000000000000000, main___mod7___NumBytes} - {28'b0000000000000000000000000000, main___mod7___timeStampBytes}))
						main___mod7___state = 4'h0;
					else begin
						main___mod7___timeStampFlag = 1'h0;
						main___data_buffer = main___buffer;
						main___mod7___NumBytes = main___mod7___NumBytes - {12'b000000000000, main___mod7___timeStampBytes};
						if (16'h0001 < main___mod7___NumBytes)
							main___mod7___state = 4'h7;
						if (16'h0001 == main___mod7___NumBytes) begin
							main___mod7___state = 4'h0;
							main___mod7___packet_done = 1'h1;
						end
						main___mod7___time_stamp_start = 1'h0;
					end
				end
			32'sh00000007:
				if (16'h0001 < main___mod7___NumBytes) begin
					main___data_buffer = main___buffer;
					main___mod7___NumBytes = main___mod7___NumBytes - 16'h0001;
				end
				else begin
					main___data_buffer = main___buffer;
					main___mod7___state = 4'h0;
					main___mod7___packet_done = 1'h1;
				end
			32'sh00000008:
				if (~main___packetStart) begin
					main___packetStop = 1'h0;
					main___mod7___packet_done = 1'h0;
					main___mod7___state = 4'hf;
				end
		endcase
	assign main___mod7___module1___start = main___mod7___start;
	assign main___mod7___module1___done = main___mod7___prefix_stop;
	assign main___mod7___module1___input_stream = main___mod7___input_stream;
	assign main___mod7___module1___read_signal = main___mod7___read_signal;
	initial begin
		main___mod7___module1___state = 2'h0;
		main___mod7___prefix_stop = 1'h0;
	end
	always @(posedge main___read_enable)
		case (main___mod7___module1___state)
			2'h0:
				if ((8'h00 == main___buffer) & main___packetStart)
					main___mod7___module1___state = 2'h1;
			2'h1: begin
				if (8'h00 == main___buffer)
					main___mod7___module1___state = 2'h2;
				if (8'h00 != main___buffer)
					main___mod7___module1___state = 2'h0;
			end
			2'h2: begin
				if (8'h01 == main___buffer) begin
					main___mod7___module1___state = 2'h3;
					main___mod7___prefix_stop = 1'h1;
				end
				if (8'h00 == main___buffer)
					main___mod7___module1___state = 2'h1;
				if (8'h01 < main___buffer)
					main___mod7___module1___state = 2'h1;
			end
			2'h3:
				if (~main___packetStart) begin
					main___mod7___module1___state = 2'h0;
					main___mod7___prefix_stop = 1'h0;
				end
		endcase
	assign main___mod7___module2___start = main___mod7___time_stamp_start;
	assign main___mod7___module2___done = main___mod7___time_stamp_stop;
	assign main___mod7___module2___input_stream = main___mod7___input_stream;
	assign main___mod7___module2___read_signal = main___mod7___read_signal;
	assign main___mod7___module2___timeBytes = main___mod7___timeStampBytes;
	assign main___mod7___module2___flag = main___mod7___timeStampFlag;
	initial begin
		main___mod7___time_stamp_stop = 1'h0;
		main___mod7___module2___state = 4'h0;
		main___mod7___module2___PTS = 33'h000000000;
		main___mod7___module2___DTS = 33'h000000000;
		main___mod7___timeStampBytes = 4'h0;
		main___mod7___module2___nextstate = 4'h0;
	end
	always @(posedge main___read_enable)
		case ({28'b0000000000000000000000000000, main___mod7___module2___state})
			32'sh00000000: begin
				if (main___mod7___time_stamp_start & ~main___mod7___timeStampFlag) begin
					if (4'h2 == main___buffer[3'h0+:32'h00000004]) begin
						main___mod7___module2___state = 4'h1;
						main___mod7___module2___PTS[6'h1e+:32'h00000003] = main___buffer[3'h4+:32'h00000003];
					end
					if (4'h3 == main___buffer[3'h0+:32'h00000004]) begin
						main___mod7___module2___state = 4'h2;
						main___mod7___module2___PTS[6'h1e+:32'h00000003] = main___buffer[3'h4+:32'h00000003];
					end
					if (8'h0f == main___buffer) begin
						main___mod7___timeStampBytes = 4'h1;
						main___mod7___module2___state = 4'hf;
						main___mod7___time_stamp_stop = 1'h1;
					end
				end
				if (main___mod7___time_stamp_start & ~main___mod7___timeStampFlag)
					main___mod7___module2___state = main___mod7___module2___nextstate;
			end
			32'sh00000001: begin
				main___mod7___module2___PTS[6'h16+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'h3;
			end
			32'sh00000002: begin
				main___mod7___module2___PTS[6'h16+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'h6;
			end
			32'sh0000000f:
				if (~main___mod7___time_stamp_start) begin
					main___mod7___time_stamp_stop = 1'h0;
					main___mod7___module2___state = 4'h0;
				end
			32'sh00000003: begin
				main___mod7___module2___PTS[6'h0f+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___module2___state = 4'h4;
			end
			32'sh00000004: begin
				main___mod7___module2___PTS[6'h07+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'h5;
			end
			32'sh00000005: begin
				main___mod7___module2___PTS[6'h00+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___timeStampBytes = 4'h5;
				main___mod7___module2___state = 4'hf;
				main___mod7___time_stamp_stop = 1'h1;
			end
			32'sh00000006: begin
				main___mod7___module2___PTS[6'h0f+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___module2___state = 4'h7;
			end
			32'sh00000007: begin
				main___mod7___module2___PTS[6'h07+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'h8;
			end
			32'sh00000008: begin
				main___mod7___module2___PTS[6'h00+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___module2___state = 4'h9;
				main___mod7___time_stamp_stop = 1'h1;
			end
			32'sh00000009: begin
				main___mod7___module2___state = 4'ha;
				main___mod7___module2___DTS[6'h1e+:32'h00000003] = main___buffer[3'h4+:32'h00000003];
			end
			32'sh0000000a: begin
				main___mod7___module2___DTS[6'h16+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'hb;
			end
			32'sh0000000b: begin
				main___mod7___module2___DTS[6'h0f+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___module2___state = 4'hc;
			end
			32'sh0000000c: begin
				main___mod7___module2___DTS[6'h07+:32'h00000008] = main___buffer;
				main___mod7___module2___state = 4'hd;
			end
			32'sh0000000d: begin
				main___mod7___module2___DTS[6'h00+:32'h00000007] = main___buffer[3'h0+:32'h00000007];
				main___mod7___timeStampBytes = 4'ha;
				main___mod7___module2___state = 4'hf;
				main___mod7___time_stamp_stop = 1'h1;
			end
		endcase
endmodule
