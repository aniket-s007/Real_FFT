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

    localparam SHIFT      = COEF_WIDTH - 1;
    localparam PROD_WIDTH = IN_WIDTH + COEF_WIDTH;
    localparam SUM_WIDTH  = PROD_WIDTH + 1;

    // products
    wire signed [PROD_WIDTH-1:0] re_cos, im_sin, im_cos, re_sin;
    assign re_cos = re_in * cos_coef;
    assign im_sin = im_in * sin_coef;
    assign im_cos = im_in * cos_coef;
    assign re_sin = re_in * sin_coef;

    // re = re*cos - im*sin, im = im*cos + re*sin
    wire signed [SUM_WIDTH-1:0] raw_re, raw_im;
    assign raw_re = re_cos - im_sin;
    assign raw_im = im_cos + re_sin;

    // round, shift down, saturate
    function signed [IN_WIDTH:0] round_shift_sat;
        input signed [SUM_WIDTH-1:0] raw;
        reg   signed [SUM_WIDTH-1:0]       rounded;
        reg   signed [SUM_WIDTH-SHIFT-1:0] shifted;
        reg   signed [IN_WIDTH:0]          max_out, min_out;
        begin
            rounded = raw + (1 <<< (SHIFT-1));
            shifted = rounded >>> SHIFT;
            max_out = {1'b0, {IN_WIDTH{1'b1}}};
            min_out = {1'b1, {IN_WIDTH{1'b0}}};
            if (shifted > max_out)
                round_shift_sat = max_out;
            else if (shifted < min_out)
                round_shift_sat = min_out;
            else
                round_shift_sat = shifted[IN_WIDTH:0];
        end
    endfunction

    assign re_out = round_shift_sat(raw_re);
    assign im_out = round_shift_sat(raw_im);

endmodule
