`timescale 1ns/1ps

// top_stage1_stage2_stage3_stage4.v -- structural link of all 4 columns
// of the 4-parallel 16-point RFFT architecture, Salehi/Amirfattahi/Parhi
// 2013: the full pipeline, x(k)/x(k+N/4)/x(k+N/2)/x(k+3N/4) in, s4[] out.
//
// Reuses top_stage1_stage2_stage3.v (Columns 1-3, unchanged) and adds
// stage4.v (Column 4, "D+SW1, BF") straight after it -- stage3's 4 output
// ports feed stage4 by name, no extra crossing needed here (matching the
// same "topN_M.v does wiring, stageN.v does the column's own internal
// crossings" convention top_stage1_stage2_stage3.v already established).
//
// No extra logic of its own -- purely wiring plus the two sub-modules --
// so pipeline latency here is top_stage1_stage2_stage3's own 3 cycles
// (2 from stages 1-2, 1 from stage3's output register) plus stage4's
// 1-cycle output register, with out_valid handshaking straight through.

module top_stage1_stage2_stage3_stage4 #(
    parameter WIDTH     = 8,
    parameter S2_WIDTH  = WIDTH + 2,   // stage2's output width, stage3's input width
    parameter S3A_WIDTH = S2_WIDTH + 1,
    parameter S3B_WIDTH = S2_WIDTH + 2,
    parameter S4_WIDTH  = S3B_WIDTH + 1
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_valid,

    input  wire signed [WIDTH-1:0]     x_k,       // x(k)
    input  wire signed [WIDTH-1:0]     x_k_n4,    // x(k + N/4)
    input  wire signed [WIDTH-1:0]     x_k_n2,    // x(k + N/2)
    input  wire signed [WIDTH-1:0]     x_k_3n4,   // x(k + 3N/4)

    output wire                        out_valid,
    output wire signed [S4_WIDTH-1:0]  s4_p0,
    output wire signed [S4_WIDTH-1:0]  s4_p1,
    output wire signed [S4_WIDTH-1:0]  s4_p2,
    output wire signed [S4_WIDTH-1:0]  s4_p3
);

    wire                          s3_valid;
    wire signed [S3A_WIDTH-1:0]   s3_p0, s3_p1;
    wire signed [S3B_WIDTH-1:0]   s3_p2, s3_p3;

    top_stage1_stage2_stage3 #(.WIDTH(WIDTH)) u_top123 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s3_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3)
    );

    stage4 #(.WIDTH(WIDTH)) u_stage4 (
        .clk(clk), .rst_n(rst_n), .in_valid(s3_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3),
        .out_valid(out_valid),
        .s4_p0(s4_p0), .s4_p1(s4_p1), .s4_p2(s4_p2), .s4_p3(s4_p3)
    );

endmodule
