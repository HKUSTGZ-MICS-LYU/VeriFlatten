module example_module (
    input  wire [15:0] mem_rdata_latched,
    output wire [31:0] data_out    
);

    assign data_out[31:20] = $signed({mem_rdata_latched[12], mem_rdata_latched[6:2]});
    assign data_out[20:0] = 0;
endmodule