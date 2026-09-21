`timescale 1ns/1ps

// stage1 -- column 1 (BF, BF)

module stage1 #(
    parameter WIDTH = 8
) (
    input  wire                     clk,
    input  wire                     rst_n,     // active-low synchronous reset
    input  wire                     in_valid,

    input  wire signed [WIDTH-1:0]  x_k,       // x(k)
    input  wire signed [WIDTH-1:0]  x_k_n4,    // x(k + N/4)
    input  wire signed [WIDTH-1:0]  x_k_n2,    // x(k + N/2)
    input  wire signed [WIDTH-1:0]  x_k_3n4,   // x(k + 3N/4)

    output reg                      out_valid,
    output reg  signed [WIDTH:0]    s1_top_sum,   // -> s1[k]
    output reg  signed [WIDTH:0]    s1_top_diff,  // -> s1[k+8]
    output reg  signed [WIDTH:0]    s1_bot_sum,   // -> s1[k+4]
    output reg  signed [WIDTH:0]    s1_bot_diff   // -> s1[k+12]
);

    wire signed [WIDTH:0] top_sum_c, top_diff_c;
    wire signed [WIDTH:0] bot_sum_c, bot_diff_c;

    // top butterfly
    real_bf #(.WIDTH(WIDTH)) bf_top (
        .pass_thru(1'b0),
        .in1(x_k), .in2(x_k_n2),
        .out_sum(top_sum_c), .out_diff(top_diff_c)
    );

    // bottom butterfly
    real_bf #(.WIDTH(WIDTH)) bf_bot (
        .pass_thru(1'b0),
        .in1(x_k_n4), .in2(x_k_3n4),
        .out_sum(bot_sum_c), .out_diff(bot_diff_c)
    );

    // output register
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid   <= 1'b0;
            s1_top_sum  <= 0;
            s1_top_diff <= 0;
            s1_bot_sum  <= 0;
            s1_bot_diff <= 0;
        end else begin
            out_valid   <= in_valid;
            s1_top_sum  <= top_sum_c;
            s1_top_diff <= top_diff_c;
            s1_bot_sum  <= bot_sum_c;
            s1_bot_diff <= bot_diff_c;
        end
    end

endmodule
