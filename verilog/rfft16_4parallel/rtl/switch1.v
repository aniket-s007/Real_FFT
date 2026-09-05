`timescale 1ns/1ps

// switch1.v -- the "SW1" box (Fig. 5(a)): a 2:1 select on a (re, im) pair.
// Exact hardware twin of the python model's sw1(activate, local, upper):
//     return upper if activate else local
//
// Only one of the two sources is ever live at a time in this pipeline's
// own schedule (see stage3.v, which drives `activate`): on the cycles
// where SW1 is "on", the LOCAL complex-BF pair hasn't been computed yet;
// on the cycles where it's "off", the UPPER (Eq.(7)) pair isn't relevant.
// So this models the net ROUTING EFFECT of a 2:1 select between two
// time-multiplexed sources -- not a simultaneous 2-lane swap of the kind
// Fig. 5(a)'s picture suggests, which would need both sources live at
// once and leave one output permanently unused here. Same caveat as the
// python model's own sw1() docstring.

// NOTE on WIDTH: here it means "bus width of ONE component of the pair,"
// not the sample width WIDTH denotes everywhere else in this project. SW1
// sits downstream of Stage 3's butterflies, so it carries BF outputs, not
// samples -- at the Stage-3 instantiation that's WIDTH+3 bits (samples are
// WIDTH, stage1 adds a bit, stage2 another, stage3's BF one more). Passing
// the sample width here would silently truncate three bits, sign included.

module switch1 #(
    parameter WIDTH = 8
) (
    input  wire                    activate,
    input  wire signed [WIDTH-1:0] local_re, local_im,
    input  wire signed [WIDTH-1:0] upper_re, upper_im,
    output wire signed [WIDTH-1:0] out_re,   out_im
);

    assign out_re = activate ? upper_re : local_re;
    assign out_im = activate ? upper_im : local_im;

endmodule
