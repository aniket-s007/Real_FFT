`timescale 1ns/1ps

// stage1.v -- Column 1 ("BF, BF") of the 4-parallel 16-point RFFT
// architecture, Salehi/Amirfattahi/Parhi 2013. Exact hardware twin of
// stage1() in python/2013architecture_N16_4parallel.py:
//
//   def stage1(x):
//       s1 = [0.0] * N
//       for k in range(4):
//           xk, xk_n2   = x[k],     x[k + 8]
//           xk_n4, xk_3n4 = x[k+4], x[k + 12]
//           s1[k],   s1[k+8]  = bf(xk, xk_n2)          # top BF box
//           s1[k+4], s1[k+12] = bf(xk_n4, xk_3n4)      # bottom BF box
//       return s1
//
// Four real samples x(k), x(k+N/4), x(k+N/2), x(k+3N/4) stream in per
// clock, k = 0..3, across the two physical lanes drawn in the block
// diagram. No twiddle box exists in Stage 1 (see the flow graph's own
// comment: "No twiddle box anywhere in Stage 1"), so this column is
// purely two BF boxes -- registered once at the output to form one real
// pipeline stage (1 cycle of latency), matching how the diagram's BF
// boxes are combinational logic separated by explicit delay elements.
//
// WIDTH is parametrized (input sample width, signed fixed-point, e.g.
// Q1.15 at WIDTH=16) so later stages can be swept for an SQNR study
// against the floating-point golden model.

module stage1 #(
    parameter WIDTH = 8
) (
    input  wire                     clk,
    input  wire                     rst_n,     // active-low synchronous reset
    input  wire                     in_valid,  // high when x_k/.. are a valid cycle-k sample set

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

    // top lane: BF(x(k), x(k+N/2))
    real_bf #(.WIDTH(WIDTH)) bf_top (
        .pass_thru(1'b0),                       // always arithmetic in this column
        .in1(x_k), .in2(x_k_n2),
        .out_sum(top_sum_c), .out_diff(top_diff_c)
    );

    // bottom lane: BF(x(k+N/4), x(k+3N/4))
    real_bf #(.WIDTH(WIDTH)) bf_bot (
        .pass_thru(1'b0),                       // always arithmetic in this column
        .in1(x_k_n4), .in2(x_k_3n4),
        .out_sum(bot_sum_c), .out_diff(bot_diff_c)
    );

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
