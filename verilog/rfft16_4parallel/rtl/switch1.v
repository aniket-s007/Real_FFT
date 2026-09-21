`timescale 1ns/1ps

// switch1 -- SW1 box, 2x2 crossbar (control1 = 1 swaps top and bottom)

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
