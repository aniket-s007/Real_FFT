`timescale 1ns/1ps

// top_stage1_stage2_stage3 -- stage1 -> stage2 -> stage3

module top_stage1_stage2_stage3 #(
    parameter WIDTH         = 8,
    parameter S2_WIDTH      = WIDTH + 2,   // stage2 output width
    parameter S3A_WIDTH     = S2_WIDTH + 1,
    parameter S3B_WIDTH     = S2_WIDTH + 2,
    parameter TWIDDLE_WIDTH = WIDTH        // twiddle bit-width
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_valid,

    input  wire signed [WIDTH-1:0]     x_k,       // x(k)
    input  wire signed [WIDTH-1:0]     x_k_n4,    // x(k + N/4)
    input  wire signed [WIDTH-1:0]     x_k_n2,    // x(k + N/2)
    input  wire signed [WIDTH-1:0]     x_k_3n4,   // x(k + 3N/4)

    output wire                        out_valid,
    output wire signed [S3A_WIDTH-1:0] s3_p0,
    output wire signed [S3A_WIDTH-1:0] s3_p1,
    output wire signed [S3B_WIDTH-1:0] s3_p2,
    output wire signed [S3B_WIDTH-1:0] s3_p3
);

    wire                          s2_valid;
    wire signed [S2_WIDTH-1:0]    s2_top_sum, s2_top_diff, s2_bot_re, s2_bot_im;

    // stage 1 + stage 2
    top_stage1_stage2 #(.WIDTH(WIDTH), .TWIDDLE_WIDTH(TWIDDLE_WIDTH)) u_top12 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im)
    );

    // stage 3
    stage3 #(.WIDTH(WIDTH), .TWIDDLE_WIDTH(TWIDDLE_WIDTH)) u_stage3 (
        .clk(clk), .rst_n(rst_n), .in_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im),
        .out_valid(out_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3)
    );

endmodule
