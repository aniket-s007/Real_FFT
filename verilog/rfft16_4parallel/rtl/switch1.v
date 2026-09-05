`timescale 1ns/1ps

// switch1.v -- the "SW1" box (Fig. 5(a)): a 2:1 select on a (re, im) pair.
// Exact hardware twin of the python model's sw1(control1, local, upper):
//     return upper if control1 else local
//
// Only one of the two sources is ever live at a time in this pipeline's
// own schedule (see stage3.v, which drives `control1`): on the cycles
// where SW1 is "on", the LOCAL complex-BF pair hasn't been computed yet;
// on the cycles where it's "off", the UPPER (Eq.(7)) pair isn't relevant.
// So this models the net ROUTING EFFECT of a 2:1 select between two
// time-multiplexed sources -- not a simultaneous 2-lane swap of the kind
// Fig. 5(a)'s picture suggests, which would need both sources live at
// once and leave one output permanently unused here. Same caveat as the
// python model's own sw1() docstring.

module switch1 #(
    parameter WIDTH = 8
) (
    input  wire                    control1,
    input  wire signed [WIDTH-1:0] in_top, 
    input  wire signed [WIDTH-1:0] in_bottom,
    output wire signed [WIDTH-1:0] out_top,
    output wire signed [WIDTH-1:0] out_bottom
);

    assign  out_top = control1 ?  in_bottom : in_top;
    assign  out_bottom = control1 ?  in_top : in_bottom;

endmodule
