`timescale 1ns/1ps

// rotator.v -- the "W^k" box (general complex twiddle multiplier):
// (re_in + j*im_in) * (cos_coef + j*(-sin_coef))... i.e. multiply by a
// twiddle coefficient exp(-j*2*pi*phi/N) supplied as (cos_coef, sin_coef)
// in Q1.(COEF_WIDTH-1) fixed point, matching the python model's
// rotator(re_in, im_in, phi):
//     re_out = re_in*cos - im_in*sin
//     im_out = im_in*cos + re_in*sin
//
// Unlike real_bf's add/sub, a multiply changes the fixed-point SCALE:
// multiplying two Q1.x fractions doubles the fractional-bit count, so
// this box rounds back down to the input's own fractional scale
// (>>> (COEF_WIDTH-1)) after combining the two cross products. That
// round step is the first place in this pipeline with genuine
// quantization error -- Stage 1's adds are exact, this is not. That's
// intentional and exactly what the later SQNR sweep (varying WIDTH) is
// meant to characterize.
//
// Output is IN_WIDTH+1 bits: since cos^2+sin^2=1 (unit-magnitude
// coefficient), Cauchy-Schwarz bounds the combined magnitude growth to
// at most sqrt(2), which fits inside 1 extra bit (sqrt(2) < 2) -- the
// same "+1 bit per box" rule real_bf already uses. A saturating clamp is
// included as a defensive safety net; with a genuine unit-magnitude
// coefficient it should never actually trigger.

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

    wire signed [PROD_WIDTH-1:0] re_cos, im_sin, im_cos, re_sin;
    assign re_cos = re_in * cos_coef;
    assign im_sin = im_in * sin_coef;
    assign im_cos = im_in * cos_coef;
    assign re_sin = re_in * sin_coef;

    wire signed [SUM_WIDTH-1:0] raw_re, raw_im;
    assign raw_re = re_cos - im_sin;
    assign raw_im = im_cos + re_sin;

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
