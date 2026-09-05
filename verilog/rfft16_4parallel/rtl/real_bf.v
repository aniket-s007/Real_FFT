`timescale 1ns/1ps

// real_bf.v -- the "BF" box (Fig. 4 of Salehi/Amirfattahi/Parhi 2013):
// a combinational radix-2 real butterfly with the figure's two modes.
//
//   pass_thru = 0  (arithmetic)  : out_sum = in1+in2, out_diff = in1-in2
//   pass_thru = 1  (passthrough) : out_sum = in1,     out_diff = in2
//
// pass_thru is Fig. 4's control signal `c` (the python model's bf(...,c=))
// -- renamed here because a lone `c` sitting next to `clk` on a waveform is
// hard to read. The paper describes mode 1 as "for real-imaginary inputs,
// input data are simply transferred to the output": the box holds its place
// in the pipeline on cycles where it isn't combining two operands. Stage 3
// is the first user -- one of its two BF instances is time-multiplexed
// between the two modes, the other is hardwired arithmetic.
//
// Output is WIDTH+1 bits so the add/sub is exact (no bit growth is
// truncated away) -- the only quantization error in this datapath is
// whatever quantized the inputs before they reached this box. In
// passthrough mode the operands are just sign-extended into that width:
// the binary point doesn't move, only the integer range grows, so both
// modes hand downstream logic the same fixed-point format. WIDTH is a
// parameter purely so later stages can be swept for SQNR analysis.

module real_bf #(
    parameter WIDTH = 16
) (
    input  wire                    pass_thru,   // Fig. 4's `c`: 0 = add/sub, 1 = transfer
    input  wire signed [WIDTH-1:0] in1,
    input  wire signed [WIDTH-1:0] in2,
    output wire signed [WIDTH:0]   out_sum,
    output wire signed [WIDTH:0]   out_diff
);

    assign out_sum  = pass_thru ? in1 : (in1 + in2);
    assign out_diff = pass_thru ? in2 : (in1 - in2);

endmodule
