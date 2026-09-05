`timescale 1ns/1ps

// top_stage1_stage2.v -- structural link of Columns 1+2 ("BF,BF" -> "BF,W^k")
// of the 4-parallel 16-point RFFT architecture, Salehi/Amirfattahi/Parhi 2013.
//
// Fig. 7 draws a direct wire crossing between Column 1 and Column 2 (no
// switch/delay symbol there -- that only appears later, in front of
// Column 3), because Stage 2's top BF combines the two SUM outputs of
// Stage 1 (one from stage1's top lane, one from its bottom lane) and its
// bottom W^k box combines the two DIFF outputs of Stage 1. This module is
// exactly that crossing, made into a synthesizable structural instance
// instead of living only inline inside tb_stage2.v. Cross-mapping (see
// stage2.v's own header comment, and identical to how tb_stage2.v already
// wires it -- that testbench passes all 16 samples with this exact
// connection):
//     stage1.s1_top_sum  -> stage2.s1_k
//     stage1.s1_bot_sum  -> stage2.s1_k4
//     stage1.s1_top_diff -> stage2.s1_k8
//     stage1.s1_bot_diff -> stage2.s1_k12
//
// No extra logic of its own -- purely wiring plus the two stage instances
// -- so pipeline latency here is the sum of the two stages (2 cycles),
// via out_valid handshaking straight through from stage1 to stage2.

module top_stage1_stage2 #(
    parameter WIDTH    = 8,
    parameter IN_WIDTH = WIDTH + 1   // stage1's output width, stage2's input width
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

    stage1 #(.WIDTH(WIDTH)) u_stage1 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s1_valid),
        .s1_top_sum(s1_top_sum), .s1_top_diff(s1_top_diff),
        .s1_bot_sum(s1_bot_sum), .s1_bot_diff(s1_bot_diff)
    );

    // documented cross-mapping -- do not straight-through these, see header
    stage2 #(.WIDTH(WIDTH)) u_stage2 (
        .clk(clk), .rst_n(rst_n), .in_valid(s1_valid),
        .s1_k(s1_top_sum),    .s1_k4(s1_bot_sum),
        .s1_k8(s1_top_diff),  .s1_k12(s1_bot_diff),
        .out_valid(out_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re),   .s2_bot_im(s2_bot_im)
    );

endmodule
