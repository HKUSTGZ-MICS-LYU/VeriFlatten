module    divider_man
    #(parameter N=5,
      parameter M=3,
      parameter N_ACT = M+N-1)
    (
      input                     clk,
      input                     rstn,

      input                     data_rdy ,  //数据使能
      input [N-1:0]             dividend,   //被除数
      input [M-1:0]             divisor,    //除数

      output                    res_rdy ,
      output [N_ACT-M:0]        merchant ,  //商位宽：N
      output [M-1:0]            remainder ); //最终余数

    wire [N_ACT-M-1:0]   dividend_t [N_ACT-M:0] ;
    wire [M-1:0]         divisor_t [N_ACT-M:0] ;
    wire [M-1:0]         remainder_t [N_ACT-M:0];
    wire [N_ACT-M:0]     rdy_t ;
    wire [N_ACT-M:0]     merchant_t [N_ACT-M:0] ;

    //初始化首个运算单元
    divider_cell      #(.N(N_ACT), .M(M))
       u_divider_step0
    ( .clk              (clk),
      .rstn             (rstn),
      .en               (data_rdy),
      //用被除数最高位 1bit 数据做第一次单步运算的被除数，高位补0
      .dividend         ({{(M){1'b0}}, dividend[N-1]}), 
      .divisor          (divisor),                  
      .merchant_ci      ({(N_ACT-M+1){1'b0}}),   //商初始为0
      .dividend_ci      (dividend[N_ACT-M-1:0]), //原始被除数
      //output
      .dividend_kp      (dividend_t[N_ACT-M]),   //原始被除数信息传递
      .divisor_kp       (divisor_t[N_ACT-M]),    //原始除数信息传递
      .rdy              (rdy_t[N_ACT-M]),
      .merchant         (merchant_t[N_ACT-M]),   //第一次商结果
      .remainder        (remainder_t[N_ACT-M])   //第一次余数
      );

    genvar               i ;
    generate
        for(i=1; i<=N_ACT-M; i=i+1) begin: sqrt_stepx
            divider_cell      #(.N(N_ACT), .M(M))
              u_divider_step
              (.clk              (clk),
               .rstn             (rstn),
               .en               (rdy_t[N_ACT-M-i+1]),
               .dividend         ({remainder_t[N_ACT-M-i+1], dividend_t[N_ACT-M-i+1][N_ACT-M-i]}),   //余数与原始被除数单bit数据拼接
               .divisor          (divisor_t[N_ACT-M-i+1]),
               .merchant_ci      (merchant_t[N_ACT-M-i+1]), 
               .dividend_ci      (dividend_t[N_ACT-M-i+1]), 
               //output
               .divisor_kp       (divisor_t[N_ACT-M-i]),
               .dividend_kp      (dividend_t[N_ACT-M-i]),
               .rdy              (rdy_t[N_ACT-M-i]),
               .merchant         (merchant_t[N_ACT-M-i]),
               .remainder        (remainder_t[N_ACT-M-i])
              );
        end // block: sqrt_stepx
    endgenerate


    reg                res_rdy_d1,  res_rdy_d2;
    reg [N_ACT-M:0]    merchant_d1, merchant_d2;
    reg [M-1:0]        remainder_d1, remainder_d2;

    always @(posedge clk or negedge rstn) begin
        res_rdy_d1    <= rdy_t[0];
        res_rdy_d2    <= res_rdy_d1;
        merchant_d1   <= merchant_t[0];
        merchant_d2   <= merchant_d1;
        remainder_d1  <= remainder_t[0];
        remainder_d2  <= remainder_d1;
    end

    assign res_rdy   = res_rdy_d2;
    assign merchant  = merchant_d2;
    assign remainder = remainder_d2;
    // assign res_rdy       = rdy_t[0];
    // assign merchant      = merchant_t[0];  //最后一次商结果作为最终的商
    // assign remainder     = remainder_t[0]; //最后一次余数作为最终的余数

endmodule

// parameter M means the actual width of divisor
module    divider_cell
    #(parameter N=5,
      parameter M=3)
    (
      input                     clk,
      input                     rstn,
      input                     en,

      input [M:0]               dividend,
      input [M-1:0]             divisor,
      input [N-M:0]             merchant_ci , //上一级输出的商
      input [N-M-1:0]           dividend_ci , //原始除数

      output reg [N-M-1:0]      dividend_kp,  //原始被除数信息
      output reg [M-1:0]        divisor_kp,   //原始除数信息
      output reg                rdy ,
      output reg [N-M:0]        merchant ,  //运算单元输出商
      output reg [M-1:0]        remainder   //运算单元输出余数
    );

    always @(*) begin
        if (en) begin
            rdy            <= 1'b1 ;
            divisor_kp     <= divisor ;  //原始除数保持不变
            dividend_kp    <= dividend_ci ;  //原始被除数传递
            if (dividend >= {1'b0, divisor}) begin
                merchant    <= (merchant_ci<<1) + 1'b1 ; //商为1
                remainder   <= dividend - {1'b0, divisor} ; //求余
            end
            else begin
                merchant    <= merchant_ci<<1 ;  //商为0
                remainder   <= dividend ;        //余数不变
            end
        end // if (en)
        else begin
            rdy            <= 'b0 ;
            merchant       <= 'b0 ;
            remainder      <= 'b0 ;
            divisor_kp     <= 'b0 ;
            dividend_kp    <= 'b0 ;
        end
    end 

endmodule