/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Primitives                                                 ////
////  FPU Primitives                                             ////
////                                                             ////
////  Author: Rudolf Usselmann                                   ////
////          rudi@asics.ws                                      ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (prod) 2000 Rudolf Usselmann                         ////
////                    rudi@asics.ws                            ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR opa PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////


`timescale 1ns / 100ps


////////////////////////////////////////////////////////////////////////
//
// Add/Sub
//

module add_sub27(add, opa, opb, sum, co);
input		add;
input	[26:0]	opa, opb;
output	[26:0]	sum;
output		co;



assign {co, sum} = add ? (opa + opb) : (opa - opb);

endmodule

////////////////////////////////////////////////////////////////////////
//
// Multiply
//

module mul_r2(clk, opa, opb, prod);
input		clk;
input	[23:0]	opa, opb;
output	[47:0]	prod;

reg	[47:0]	prod1, prod;

always @(posedge clk)
	prod1 <=  opa * opb;

always @(posedge clk)
	prod <=  prod1;

endmodule


////////////////////////////////////////////////////////////////////////
//
// Divide
//

// module div_r2(clk, opa, opb, quo, rem);
// input		clk;
// input	[49:0]	opa;
// input	[23:0]	opb;
// output	[49:0]	quo, rem;

// reg	[49:0]	quo, rem, quo1, remainder;

// always @(posedge clk)
// 	quo1 <=  opa / opb;

// always @(posedge clk)
// 	quo <=  quo1;

// always @(posedge clk)
// 	remainder <=  opa % opb;

// always @(posedge clk)
// 	rem <=  remainder;

// endmodule


//parameter N means the actual width of dividend
//using 29/5=5...4
module divider_man
#(
    parameter N = 5,
    parameter M = 3,
    parameter N_ACT = M + N - 1,
    parameter N_ACT_M = N_ACT - M,
    parameter PART1 = N_ACT_M / 3,
    parameter PART2 = (N_ACT_M - PART1) / 2,
    parameter PART3 = N_ACT_M - PART1 - PART2
)
(
    input clk,
    input [N-1:0] dividend,   // 被除数
    input [M-1:0] divisor,    // 除数

    output [N_ACT-M:0] merchant,  // 商位宽：N
    output [M-1:0]     remainder  // 最终余数
);

    // 第一段流水线信号
    wire [N_ACT_M-1:0] dividend_t1 [PART1:0];
    wire [M-1:0]       divisor_t1  [PART1:0];
    wire [N_ACT_M:0]   merchant_t1 [PART1:0];
    wire [M-1:0]       remainder_t1[PART1:0];

    // 第二段流水线信号
    wire [N_ACT_M-1:0] dividend_t2 [PART2:0];
    wire [M-1:0]       divisor_t2  [PART2:0];
    wire [N_ACT_M:0]   merchant_t2 [PART2:0];
    wire [M-1:0]       remainder_t2[PART2:0];

    // 第三段流水线信号
    wire [N_ACT_M-1:0] dividend_t3 [PART3:0];
    wire [M-1:0]       divisor_t3  [PART3:0];
    wire [N_ACT_M:0]   merchant_t3 [PART3:0];
    wire [M-1:0]       remainder_t3[PART3:0];

    // 第一段初始
    divider_cell #(.N(N_ACT), .M(M)) u_divider_step0 (
        .dividend         ({{(M){1'b0}}, dividend[N-1]}),
        .divisor          (divisor),
        .merchant_ci      ({(N_ACT_M+1){1'b0}}),
        .dividend_ci      (dividend[N_ACT_M-1:0]),
        .dividend_kp      (dividend_t1[PART1]),
        .divisor_kp       (divisor_t1[PART1]),
        .merchant         (merchant_t1[PART1]),
        .remainder        (remainder_t1[PART1])
    );

    // 第一段for循环
    genvar i;
    generate
        for(i = 1; i <= PART1; i = i + 1) begin: gen_part1
            divider_cell #(.N(N_ACT), .M(M)) u_divider_step1 (
                .dividend     ({remainder_t1[PART1-i+1], dividend_t1[PART1-i+1][N_ACT_M-i]}),
                .divisor      (divisor_t1[PART1-i+1]),
                .merchant_ci  (merchant_t1[PART1-i+1]),
                .dividend_ci  (dividend_t1[PART1-i+1]),
                .divisor_kp   (divisor_t1[PART1-i]),
                .dividend_kp  (dividend_t1[PART1-i]),
                .merchant     (merchant_t1[PART1-i]),
                .remainder    (remainder_t1[PART1-i])
            );
        end
    endgenerate

    // 第一段到第二段寄存
    reg [N_ACT_M-1:0] dividend_t2_r;
    reg [M-1:0]       divisor_t2_r;
    reg [N_ACT_M:0]   merchant_t2_r;
    reg [M-1:0]       remainder_t2_r;

    always @(posedge clk) begin
        dividend_t2_r   <= dividend_t1[0];
        divisor_t2_r    <= divisor_t1[0];
        merchant_t2_r   <= merchant_t1[0];
        remainder_t2_r  <= remainder_t1[0];
    end

    assign dividend_t2[PART2]   = dividend_t2_r;
    assign divisor_t2[PART2]    = divisor_t2_r;
    assign merchant_t2[PART2]   = merchant_t2_r;
    assign remainder_t2[PART2]  = remainder_t2_r;

    // 第二段for循环
    genvar j;
    generate
        for(j = 1; j <= PART2; j = j + 1) begin: gen_part2
            divider_cell #(.N(N_ACT), .M(M)) u_divider_step2 (
                .dividend     ({remainder_t2[PART2-j+1], dividend_t2[PART2-j+1][N_ACT_M-PART1-j]}),
                .divisor      (divisor_t2[PART2-j+1]),
                .merchant_ci  (merchant_t2[PART2-j+1]),
                .dividend_ci  (dividend_t2[PART2-j+1]),
                .divisor_kp   (divisor_t2[PART2-j]),
                .dividend_kp  (dividend_t2[PART2-j]),
                .merchant     (merchant_t2[PART2-j]),
                .remainder    (remainder_t2[PART2-j])
            );
        end
    endgenerate

    // 第二段到第三段寄存
    reg [N_ACT_M-1:0] dividend_t3_r;
    reg [M-1:0]       divisor_t3_r;
    reg [N_ACT_M:0]   merchant_t3_r;
    reg [M-1:0]       remainder_t3_r;

    always @(posedge clk) begin
        dividend_t3_r   <= dividend_t2[0];
        divisor_t3_r    <= divisor_t2[0];
        merchant_t3_r   <= merchant_t2[0];
        remainder_t3_r  <= remainder_t2[0];
    end

    assign dividend_t3[PART3]   = dividend_t3_r;
    assign divisor_t3[PART3]    = divisor_t3_r;
    assign merchant_t3[PART3]   = merchant_t3_r;
    assign remainder_t3[PART3]  = remainder_t3_r;

    // 第三段for循环
    genvar k;
    generate
        for(k = 1; k <= PART3; k = k + 1) begin: gen_part3
            divider_cell #(.N(N_ACT), .M(M)) u_divider_step3 (
                .dividend     ({remainder_t3[PART3-k+1], dividend_t3[PART3-k+1][N_ACT_M-PART1-PART2-k]}),
                .divisor      (divisor_t3[PART3-k+1]),
                .merchant_ci  (merchant_t3[PART3-k+1]),
                .dividend_ci  (dividend_t3[PART3-k+1]),
                .divisor_kp   (divisor_t3[PART3-k]),
                .dividend_kp  (dividend_t3[PART3-k]),
                .merchant     (merchant_t3[PART3-k]),
                .remainder    (remainder_t3[PART3-k])
            );
        end
    endgenerate

    // 直接 assign 输出
    assign merchant  = merchant_t3[0];
    assign remainder = remainder_t3[0];

endmodule

// parameter M means the actual width of divisor
module divider_cell
    #(parameter N=5,
      parameter M=3)
    (

      input [M:0]               dividend,
      input [M-1:0]             divisor,
      input [N-M:0]             merchant_ci, //上一级输出的商
      input [N-M-1:0]           dividend_ci, //原始被除数

      output wire [N-M-1:0]     dividend_kp, //原始被除数信息
      output wire [M-1:0]       divisor_kp,  //原始除数信息
      output wire [N-M:0]       merchant,    //运算单元输出商
      output wire [M-1:0]       remainder    //运算单元输出余数
    );

    // 原始信号直连
    assign divisor_kp  = divisor;
    assign dividend_kp = dividend_ci;

    // 判断条件
    wire geq = (dividend >= {1'b0, divisor});

    // 商、余数逻辑
    assign merchant  = geq ? ((merchant_ci << 1) | 1'b1) : (merchant_ci << 1);
    assign remainder = geq ? (dividend - {1'b0, divisor}) : dividend;

endmodule