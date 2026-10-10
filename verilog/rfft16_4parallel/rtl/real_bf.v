`timescale 1ns/1ps

// real_bf -- halving radix-2 real butterfly (BF box)

module real_bf #(
    parameter WIDTH = 16
) (
    input  wire                    pass_thru,   // 0 = add/sub, 1 = pass through
    input  wire signed [WIDTH-1:0] in1,
    input  wire signed [WIDTH-1:0] in2,
    output wire signed [WIDTH-1:0] out_sum,
    output wire signed [WIDTH-1:0] out_diff
);

    // add/sub or pass through
    wire signed [WIDTH:0] sum = pass_thru ? in1 : (in1 + in2);
    wire signed [WIDTH:0] diff = pass_thru ? in2 : (in1 - in2);

    assign out_sum = sum[WIDTH:1];
    assign out_diff = diff[WIDTH:1];

endmodule
