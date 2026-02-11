module for_test (
    input  wire [7:0] in_data,
    output reg  [7:0] out_data
);

    integer i;

    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            out_data[i] = in_data[7-i];
        end
    end

endmodule
