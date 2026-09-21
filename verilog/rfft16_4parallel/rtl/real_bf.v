`timescale 1ns/1ps

// real_bf -- radix-2 real butterfly (BF box)

module real_bf #(
    parameter WIDTH = 16
) (
    input  wire                    pass_thru,   // 0 = add/sub, 1 = pass through
    input  wire signed [WIDTH-1:0] in1,
    input  wire signed [WIDTH-1:0] in2,
    output wire signed [WIDTH:0]   out_sum,
    output wire signed [WIDTH:0]   out_diff
);

    assign out_sum  = pass_thru ? in1 : (in1 + in2);
    assign out_diff = pass_thru ? in2 : (in1 - in2);

endmodule
