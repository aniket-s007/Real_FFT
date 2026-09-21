`timescale 1ns/1ps

// top_stage1_stage2 -- stage1 -> stage2

module top_stage1_stage2 #(
    parameter WIDTH         = 8,
    parameter IN_WIDTH      = WIDTH + 1,  // stage1 output width
    parameter TWIDDLE_WIDTH = WIDTH       // twiddle bit-width
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,

    input  wire signed [WIDTH-1:0]    x_k,       // x(k)
    input  wire signed [WIDTH-1:0]    x_k_n4,    // x(k + N/4)
    input  wire signed [WIDTH-1:0]    x_k_n2,    // x(k + N/2)
    input  wire signed [WIDTH-1:0]    x_k_3n4,   // x(k + 3N/4)

    output wire                       out_valid,
    output wire signed [IN_WIDTH:0]   s2_top_sum,   // -> s2[k]
    output wire signed [IN_WIDTH:0]   s2_top_diff,  // -> s2[k+4]
    output wire signed [IN_WIDTH:0]   s2_bot_re,    // -> s2[k+8]
    output wire signed [IN_WIDTH:0]   s2_bot_im     // -> s2[k+12]
);

    wire                       s1_valid;
    wire signed [IN_WIDTH-1:0] s1_top_sum, s1_top_diff, s1_bot_sum, s1_bot_diff;

    // stage 1
    stage1 #(.WIDTH(WIDTH)) u_stage1 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s1_valid),
        .s1_top_sum(s1_top_sum), .s1_top_diff(s1_top_diff),
        .s1_bot_sum(s1_bot_sum), .s1_bot_diff(s1_bot_diff)
    );

    // stage 2 (inputs are cross-connected, not straight-through)
    stage2 #(.WIDTH(WIDTH), .TWIDDLE_WIDTH(TWIDDLE_WIDTH)) u_stage2 (
        .clk(clk), .rst_n(rst_n), .in_valid(s1_valid),
        .s1_k(s1_top_sum),    .s1_k4(s1_bot_sum),
        .s1_k8(s1_top_diff),  .s1_k12(s1_bot_diff),
        .out_valid(out_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re),   .s2_bot_im(s2_bot_im)
    );

endmodule
