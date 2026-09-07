`timescale 1ns/1ps

// top_stage1_stage2_stage3.v -- structural link of Columns 1+2+3 of the
// 4-parallel 16-point RFFT architecture, Salehi/Amirfattahi/Parhi 2013.
//
// Reuses top_stage1_stage2.v (Columns 1+2, unchanged) and adds stage3.v
// (Column 3, "SW1+2D, BF, CSDM") straight after it -- no crossing needed
// between Column 2 and Column 3 beyond what stage3.v already does
// internally (its own switch1 instances), since stage2's 4 output ports
// feed stage3 by name, not by a fixed physical-lane swap. Per this
// project's settled convention (see PROJECT_LOG.md, "the settled
// convention is one stageN.v per paper column, with topN_M.v structural
// modules doing the cross-stage wiring"), stage3 is instantiated here,
// not folded inside stage2.v or top_stage1_stage2.v.
//
// No extra logic of its own -- purely wiring plus the two sub-modules --
// so pipeline latency here is top_stage1_stage2's own 2 cycles plus
// stage3's 1-cycle output register, with out_valid handshaking straight
// through.

module top_stage1_stage2_stage3 #(
    parameter WIDTH     = 8,
    parameter S2_WIDTH  = WIDTH + 2,   // stage2's output width, stage3's input width
    parameter S3A_WIDTH = S2_WIDTH + 1,
    parameter S3B_WIDTH = S2_WIDTH + 2
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

    top_stage1_stage2 #(.WIDTH(WIDTH)) u_top12 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im)
    );

    stage3 #(.WIDTH(WIDTH)) u_stage3 (
        .clk(clk), .rst_n(rst_n), .in_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im),
        .out_valid(out_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3)
    );

endmodule
