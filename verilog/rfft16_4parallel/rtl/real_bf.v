`timescale 1ns/1ps

// real_bf.v -- the "BF" box (Fig. 4 of Salehi/Amirfattahi/Parhi 2013):
// a combinational radix-2 real butterfly. out_sum = in1+in2, out_diff =
// in1-in2. Output is WIDTH+1 bits so the add/sub is exact (no bit growth
// is truncated away) -- the only quantization error in this datapath is
// whatever quantized the inputs before they reached this box. WIDTH is a
// parameter purely so later stages can be swept for SQNR analysis.
//
// Fig. 4 also has a real/imaginary passthrough mode (control signal c)
// used from Stage 3 onward -- not needed here, so it's left out of this
// first, minimal version rather than built unused.

module real_bf #(
    parameter WIDTH = 16
) (
    input  wire signed [WIDTH-1:0] in1,
    input  wire signed [WIDTH-1:0] in2,
    output wire signed [WIDTH:0]   out_sum,
    output wire signed [WIDTH:0]   out_diff
);

    assign out_sum  = in1 + in2;
    assign out_diff = in1 - in2;

endmodule
