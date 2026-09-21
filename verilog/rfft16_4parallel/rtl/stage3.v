`timescale 1ns/1ps

// stage3 -- column 3 (SW1 + 2D, BF, CSDM)

module stage3 #(
    parameter WIDTH         = 8,
    parameter IN_WIDTH      = WIDTH + 2,  // stage2 output width
    parameter TWIDDLE_WIDTH = WIDTH       // twiddle bit-width (2..39)
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,

    input  wire signed [IN_WIDTH-1:0]   s2_top_sum,
    input  wire signed [IN_WIDTH-1:0]   s2_top_diff,
    input  wire signed [IN_WIDTH-1:0]   s2_bot_re,
    input  wire signed [IN_WIDTH-1:0]   s2_bot_im,

    output reg                          out_valid,
    output reg  signed [IN_WIDTH:0]     s3_p0,    // BF_A sum
    output reg  signed [IN_WIDTH:0]     s3_p1,    // BF_A diff / BF_B sum
    output reg  signed [IN_WIDTH+1:0]   s3_p2,    // rotator re
    output reg  signed [IN_WIDTH+1:0]   s3_p3     // rotator im
);

    // twiddle ROM (40-bit master constants)
    localparam MASTER_WIDTH = 40;

    // twiddle 0
    localparam signed [MASTER_WIDTH-1:0] COS0_M = 40'sd549755813887, SIN0_M = 40'sd0;
    // twiddle 2
    localparam signed [MASTER_WIDTH-1:0] COS2_M = 40'sd388736063997, SIN2_M = -40'sd388736063997;
    // twiddle 4
    localparam signed [MASTER_WIDTH-1:0] COS4_M = 40'sd0,            SIN4_M = -40'sd549755813888;

    localparam SHIFT = MASTER_WIDTH - TWIDDLE_WIDTH;

    // keep the top TWIDDLE_WIDTH bits of a master constant
    function signed [TWIDDLE_WIDTH-1:0] derive_coef;
        input signed [MASTER_WIDTH-1:0] raw;
        begin
            derive_coef = raw[MASTER_WIDTH-1 -: TWIDDLE_WIDTH];
        end
    endfunction

    // twiddle 0
    localparam signed [TWIDDLE_WIDTH-1:0] COS0 = derive_coef(COS0_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN0 = derive_coef(SIN0_M);
    // twiddle 2
    localparam signed [TWIDDLE_WIDTH-1:0] COS2 = derive_coef(COS2_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN2 = derive_coef(SIN2_M);
    // twiddle 4
    localparam signed [TWIDDLE_WIDTH-1:0] COS4 = derive_coef(COS4_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN4 = derive_coef(SIN4_M);

    // 2-cycle delay on bot_re, bot_im
    reg signed [IN_WIDTH-1:0] bot_re_d1, bot_re_d2;
    reg signed [IN_WIDTH-1:0] bot_im_d1, bot_im_d2;

    always @(posedge clk) begin
        if (!rst_n) begin
            bot_re_d1 <= 0; bot_re_d2 <= 0;
            bot_im_d1 <= 0; bot_im_d2 <= 0;
        end else begin
            bot_re_d1 <= s2_bot_re; bot_re_d2 <= bot_re_d1;
            bot_im_d1 <= s2_bot_im; bot_im_d2 <= bot_im_d1;
        end
    end

    // valid delay
    reg in_valid_d1, in_valid_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            in_valid_d1 <= 1'b0;
            in_valid_d2 <= 1'b0;
        end else begin
            in_valid_d1 <= in_valid;
            in_valid_d2 <= in_valid_d1;
        end
    end

    wire active     = in_valid | in_valid_d1 | in_valid_d2;
    wire taps_ready = in_valid_d2;

    // stage-2 cycle counter
    reg [1:0] c2_pos;
    always @(posedge clk) begin
        if (!rst_n) c2_pos <= 2'd0;
        else if (active) c2_pos <= c2_pos + 2'd1;
    end

    // switch control
    wire control1 = c2_pos[1];

    // twiddle select
    reg signed [TWIDDLE_WIDTH-1:0] rot_cos, rot_sin;
    always @(*) begin
        case (c2_pos)
            2'd0:    begin rot_cos = COS0; rot_sin = SIN0; end   // twiddle 0
            2'd1:    begin rot_cos = COS4; rot_sin = SIN4; end   // twiddle 4
            2'd2:    begin rot_cos = COS0; rot_sin = SIN0; end   // twiddle 0
            default: begin rot_cos = COS2; rot_sin = SIN2; end   // twiddle 2
        endcase
    end

    // switch top
    wire signed [IN_WIDTH-1:0] switch_top_out_top,    switch_top_out_bottom;
    switch1 #(.WIDTH(IN_WIDTH)) switch_top (
        .control1(control1),
        .in_top(s2_top_sum), .in_bottom(bot_re_d2),
        .out_top(switch_top_out_top), .out_bottom(switch_top_out_bottom)
    );

    // switch bottom
    wire signed [IN_WIDTH-1:0] switch_bottom_out_top, switch_bottom_out_bottom;
    switch1 #(.WIDTH(IN_WIDTH)) switch_bottom (
        .control1(control1),
        .in_top(s2_top_diff), .in_bottom(bot_im_d2),
        .out_top(switch_bottom_out_top), .out_bottom(switch_bottom_out_bottom)
    );

    // 2-cycle delay after the switches
    reg signed [IN_WIDTH-1:0] switch_top_out_top_d1,    switch_top_out_top_d2;
    reg signed [IN_WIDTH-1:0] switch_bottom_out_top_d1, switch_bottom_out_top_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            switch_top_out_top_d1    <= 0; switch_top_out_top_d2    <= 0;
            switch_bottom_out_top_d1 <= 0; switch_bottom_out_top_d2 <= 0;
        end else begin
            switch_top_out_top_d1    <= switch_top_out_top;
            switch_top_out_top_d2    <= switch_top_out_top_d1;
            switch_bottom_out_top_d1 <= switch_bottom_out_top;
            switch_bottom_out_top_d2 <= switch_bottom_out_top_d1;
        end
    end

    // top butterfly inputs
    wire signed [IN_WIDTH-1:0] s3_top_bf_top_in = switch_top_out_top_d2;
    wire signed [IN_WIDTH-1:0] s3_top_bf_bottom_in = switch_top_out_bottom;

    // bottom butterfly inputs (in2 is negated when control1 = 1)
    wire signed [IN_WIDTH-1:0] s3_bottom_bf_top_in = switch_bottom_out_top_d2;
    wire signed [IN_WIDTH-1:0] s3_bottom_bf_bottom_in = control1 ? -switch_bottom_out_bottom : switch_bottom_out_bottom;

    // top butterfly
    wire signed [IN_WIDTH:0] bf_a_sum_c, top_bf_out_diff;
    real_bf #(.WIDTH(IN_WIDTH)) bf_a (
        .pass_thru(1'b0),
        .in1(s3_top_bf_top_in), .in2(s3_top_bf_bottom_in),
        .out_sum(bf_a_sum_c), .out_diff(top_bf_out_diff)
    );

    // bottom butterfly
    wire signed [IN_WIDTH:0] bottom_bf_out_sum, bf_b_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_b (
        .pass_thru(control1),           // pass through while control1 = 1
        .in1(s3_bottom_bf_top_in), .in2(s3_bottom_bf_bottom_in),
        .out_sum(bottom_bf_out_sum), .out_diff(bf_b_diff_c)
    );

    // switch: butterfly outputs to s3_p1 and the rotator
    wire signed [IN_WIDTH:0] rot_re_in, p1_c;
    switch1 #(.WIDTH(IN_WIDTH+1)) sw_rot_re_and_p1 (
        .control1(~control1),
        .in_top(top_bf_out_diff), .in_bottom(bottom_bf_out_sum),
        .out_top(p1_c),          // -> s3_p1
        .out_bottom(rot_re_in)   // -> rotator.re_in
    );

    // twiddle rotator
    wire signed [IN_WIDTH+1:0] rot_re_c, rot_im_c;
    rotator #(.IN_WIDTH(IN_WIDTH+1), .COEF_WIDTH(TWIDDLE_WIDTH)) wk (
        .re_in(rot_re_in), .im_in(bf_b_diff_c),
        .cos_coef(rot_cos), .sin_coef(rot_sin),
        .re_out(rot_re_c), .im_out(rot_im_c)
    );

    // output register
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            s3_p0 <= 0; s3_p1 <= 0; s3_p2 <= 0; s3_p3 <= 0;
        end else begin
            out_valid <= taps_ready;
            s3_p0 <= bf_a_sum_c;
            s3_p1 <= p1_c;
            s3_p2 <= rot_re_c;
            s3_p3 <= rot_im_c;
        end
    end

endmodule
