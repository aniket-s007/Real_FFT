`timescale 1ns/1ps

// rotator -- twiddle multiplier (W^k box)

module rotator #(
    parameter IN_WIDTH   = 17,
    parameter COEF_WIDTH = 16
) (
    input  wire signed [IN_WIDTH-1:0]   re_in, im_in,
    input  wire signed [COEF_WIDTH-1:0] cos_coef, sin_coef,
    output wire signed [IN_WIDTH:0]     re_out, im_out
);

    localparam SHIFT = COEF_WIDTH - 1;

    // re = re*cos - im*sin, im = im*cos + re*sin
    wire signed [IN_WIDTH+COEF_WIDTH:0] raw_re = re_in * cos_coef - im_in * sin_coef;
    wire signed [IN_WIDTH+COEF_WIDTH:0] raw_im = im_in * cos_coef + re_in * sin_coef;

    // scale back
    assign re_out = raw_re >>> SHIFT;
    assign im_out = raw_im >>> SHIFT;

endmodule
