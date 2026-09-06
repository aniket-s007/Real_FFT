`timescale 1ns/1ps

// stage3.v -- Column 3 ("SW1+2D, BF, CSDM") of the 4-parallel 16-point
// RFFT architecture, Salehi/Amirfattahi/Parhi 2013, Fig. 7. Exact
// hardware twin of stage3() in python/2013architecture_N16_4parallel.py.
//
// Unlike Stages 1-2 (one operation per cycle, straight off the flow
// graph), Stage 3 combines a LIVE stage-2 sample with one from 2 or 4
// cycles earlier on the SAME port -- so it needs real delay-line
// registers and real 2:1 switches, driven by one shared control signal,
// matching the paper's own claim that a single (n-1)-bit counter drives
// every switch in the design (paper text, Sec. III: "a single (n-1)-bit
// counter can be used to produce the required control signals for all
// of the switches in different stages").
//
// DERIVATION METHOD: Fig. 7's own raster (the only copy available) is
// 684x208px -- nowhere near enough resolution to trust pixel-tracing
// which side of a wire crossing a delay box sits on. So this module's
// internal routing was derived the other way around: from tap
// arithmetic against stage3()'s row-by-row math, mechanically checked
// bit-exact by python/verify/gen_stage34_schedule.py (see its schedule
// table). Wire crossings are static routing -- zero cycles, zero
// arithmetic -- so which side of one a delay register sits on is not
// observable at any port this module (or its testbench) exposes. Given
// that, this file's SPECIFIC choice of which routes get a genuine
// switch1 instance is a reconstruction, not a pixel-traced copy -- but
// it uses the real primitives the paper names (SW1, 2D/4D delay chains,
// BF's passthrough mode, one shared W^k/CSDM), driven by one shared
// control signal, and its port-level behavior is checked bit-exact
// against the golden model regardless of exactly how a real chip layout
// would draw the crossings.
//
// -------------------------------------------------------------------
// THE SCHEDULE (derived and verified by gen_stage34_schedule.py):
//
//   c2_pos | c3 | live? | operation                          | phi
//   -------|----|-------|------------------------------------|-----
//     2    | 0  |  yes  | top BF(sum,live); Eq7+W^k -> p2,p3 |  0
//     3    | 1  |  yes  | top BF(sum,live); Eq7+W^k -> p2,p3 |  2 (CSDM)
//     0    | 2  |  no   | cplx BF(re); cplx BF(im) -> p2,p3  |  0
//     1    | 3  |  no   | cplx BF(re); cplx BF(im) -> p2,p3  |  4
//
// c2_pos is THIS module's own "which stage-2 cycle is this?" counter
// (built the same way stage2.v's own `k` is, just applied to THIS
// module's in_valid) -- not a hierarchical peek into stage2.v. c3=2,3
// read only delayed taps, no live input at all, which is what lets a
// single frame drain cleanly 2 cycles after in_valid drops.
//
// Two physical combine boxes, BF_A and BF_B (real_bf.v, Fig. 4's box,
// reused from Stages 1-2): BF_A is ALWAYS arithmetic mode -- only which
// PAIR of delay taps feeds it changes. BF_B is mode-switched: passthrough
// (Fig. 4's c=1) while control1=1, arithmetic while control1=0 -- and
// control1 is driven by the exact same c2_pos bit that decides which
// taps feed BF_A, so the two can't drift apart.
//
//   control1 = c2_pos[1]   -- 1 during c2_pos=2,3 (c3=0,1), 0 during c2_pos=0,1 (c3=2,3)
//
// BF_A's operand pair and BF_B's operand pair are each selected from
// {top_sum/top_diff taps} vs {bot_re/bot_im taps} by switch1 instances
// gated on control1 -- literally the SW1 box, used here for its general
// 2:1-select role rather than only the "swap two adjacent lanes" case.
// One further switch1 (feeding the rotator's re_in) has BOTH outputs
// used productively: its out_top is the rotator's re_in, and its
// out_bottom is simultaneously stage3's own "p1" output port -- the one
// place in this file where a single SW1 instance's two outputs both do
// real work, which is the closest this module gets to a literal
// crossbar swap in Fig. 5(a)'s original sense.
//
// The Eq.(7) sign flip is applied OUTSIDE real_bf, on the LIVE top_diff
// tap, before it reaches BF_B: real_bf's passthrough mode hands back
// (in1, in2) unchanged, it does not negate anything.
// -------------------------------------------------------------------

module stage3 #(
    parameter WIDTH    = 8,
    parameter IN_WIDTH = WIDTH + 2   // stage2's own output width (this module's input width)
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,

    input  wire signed [IN_WIDTH-1:0]   s2_top_sum,
    input  wire signed [IN_WIDTH-1:0]   s2_top_diff,
    input  wire signed [IN_WIDTH-1:0]   s2_bot_re,
    input  wire signed [IN_WIDTH-1:0]   s2_bot_im,

    output reg                          out_valid,
    output reg  signed [IN_WIDTH:0]     s3_p0,    // BF_A.sum, always
    output reg  signed [IN_WIDTH:0]     s3_p1,    // BF_A.diff (c3<2) or BF_B.sum (c3>=2)
    output reg  signed [IN_WIDTH+1:0]   s3_p2,    // rotator.re_out, always
    output reg  signed [IN_WIDTH+1:0]   s3_p3     // rotator.im_out, always
);

    // ---- twiddle ROM: phi = 0, 2, 4 only (Stage 3 never needs 1 or 3) ----
    // Same MASTER-precision + derive_coef() technique as stage2.v (COS0/SIN0
    // and COS2/SIN2 constants copied verbatim -- same angles, same N=16).
    // phi=4 is new here: cos(-2*pi*4/16)=0, sin(-2*pi*4/16)=-1, both exactly
    // representable at any WIDTH (0, and the most-negative Q1.(WIDTH-1)
    // value) -- no clamping needed, unlike phi=0's cos=+1.0 case.
    localparam MASTER_WIDTH = 40;
    localparam signed [MASTER_WIDTH-1:0] COS0_M = 40'sd549755813887, SIN0_M = 40'sd0;
    localparam signed [MASTER_WIDTH-1:0] COS2_M = 40'sd388736063997, SIN2_M = -40'sd388736063997;
    localparam signed [MASTER_WIDTH-1:0] COS4_M = 40'sd0,            SIN4_M = -40'sd549755813888;

    localparam SHIFT = MASTER_WIDTH - WIDTH;

    // Same truncating bit-slice as stage2.v's derive_coef() -- left as-is
    // project-wide (round+saturate version not adopted there; matching it
    // here keeps every stage's coefficient rounding behavior consistent).
    function signed [WIDTH-1:0] derive_coef;
        input signed [MASTER_WIDTH-1:0] raw;
        begin
            derive_coef = raw[MASTER_WIDTH-1 -: WIDTH];
        end
    endfunction

    localparam signed [WIDTH-1:0] COS0 = derive_coef(COS0_M);
    localparam signed [WIDTH-1:0] SIN0 = derive_coef(SIN0_M);
    localparam signed [WIDTH-1:0] COS2 = derive_coef(COS2_M);
    localparam signed [WIDTH-1:0] SIN2 = derive_coef(SIN2_M);
    localparam signed [WIDTH-1:0] COS4 = derive_coef(COS4_M);
    localparam signed [WIDTH-1:0] SIN4 = derive_coef(SIN4_M);

    // ---- delay lines: 2D on top_sum/top_diff, 4D on bot_re/bot_im ----
    // Registered UNCONDITIONALLY every cycle (same convention as
    // stage1.v/stage2.v) -- only the valid/counter logic below decides
    // which cycles' taps are ever actually used.
    reg signed [IN_WIDTH-1:0] top_sum_d1,  top_sum_d2;
    reg signed [IN_WIDTH-1:0] top_diff_d1, top_diff_d2;
    reg signed [IN_WIDTH-1:0] bot_re_d1,  bot_re_d2,  bot_re_d3,  bot_re_d4;
    reg signed [IN_WIDTH-1:0] bot_im_d1,  bot_im_d2,  bot_im_d3,  bot_im_d4;

    always @(posedge clk) begin
        if (!rst_n) begin
            top_sum_d1 <= 0;  top_sum_d2 <= 0;
            top_diff_d1 <= 0; top_diff_d2 <= 0;
            bot_re_d1 <= 0; bot_re_d2 <= 0; bot_re_d3 <= 0; bot_re_d4 <= 0;
            bot_im_d1 <= 0; bot_im_d2 <= 0; bot_im_d3 <= 0; bot_im_d4 <= 0;
        end else begin
            top_sum_d1  <= s2_top_sum;   top_sum_d2  <= top_sum_d1;
            top_diff_d1 <= s2_top_diff;  top_diff_d2 <= top_diff_d1;
            bot_re_d1 <= s2_bot_re; bot_re_d2 <= bot_re_d1; bot_re_d3 <= bot_re_d2; bot_re_d4 <= bot_re_d3;
            bot_im_d1 <= s2_bot_im; bot_im_d2 <= bot_im_d1; bot_im_d3 <= bot_im_d2; bot_im_d4 <= bot_im_d3;
        end
    end

    // ---- c2_pos: this module's own "which stage-2 cycle is live" counter ----
    // Same self-generated-from-valid-pulses idea as stage2.v's `k`, but
    // extended to keep counting 2 cycles PAST in_valid dropping (c3=2,3
    // need history only, no live input -- `active` covers that tail;
    // `taps_ready` is the narrower 4-cycle window where this cycle's
    // combinational result is actually meaningful).
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

    wire active     = in_valid | in_valid_d1 | in_valid_d2;  // 6-cycle window: keeps c2_pos correctly phased through the tail
    wire taps_ready = in_valid_d2;                            // 4-cycle window: this cycle's combinational result is real

    reg [1:0] c2_pos;
    always @(posedge clk) begin
        if (!rst_n) c2_pos <= 2'd0;
        else if (active) c2_pos <= c2_pos + 2'd1;   // wraps mod 4 on the 2-bit reg
    end

    wire control1 = c2_pos[1];   // 1 during c2_pos=2,3 (c3=0,1); 0 during c2_pos=0,1 (c3=2,3)

    reg signed [WIDTH-1:0] rot_cos, rot_sin;
    always @(*) begin
        case (c2_pos)
            2'd0:    begin rot_cos = COS0; rot_sin = SIN0; end   // c3=2, phi=0
            2'd1:    begin rot_cos = COS4; rot_sin = SIN4; end   // c3=3, phi=4
            2'd2:    begin rot_cos = COS0; rot_sin = SIN0; end   // c3=0, phi=0
            default: begin rot_cos = COS2; rot_sin = SIN2; end   // c3=1, phi=2 (CSDM slot)
        endcase
    end

    // ---- route BF_A's and BF_B's operand pairs, via switch1 (SW1) ----
    wire signed [IN_WIDTH-1:0] neg_top_diff_live = -s2_top_diff;   // Eq.(7), applied outside the box, on the LIVE tap

    wire signed [IN_WIDTH-1:0] bf_b_in1, bf_b_in2;
    switch1 #(.WIDTH(IN_WIDTH)) sw_bf_b_op1 (
        .control1(control1), .in_top(bot_im_d4), .in_bottom(top_diff_d2),
        .out_top(bf_b_in1), .out_bottom()   // unused: the complement pairing has no consumer
    );
    switch1 #(.WIDTH(IN_WIDTH)) sw_bf_b_op2 (
        .control1(control1), .in_top(bot_im_d2), .in_bottom(neg_top_diff_live),
        .out_top(bf_b_in2), .out_bottom()
    );

    wire signed [IN_WIDTH-1:0] bf_a_in1, bf_a_in2;
    switch1 #(.WIDTH(IN_WIDTH)) sw_bf_a_op1 (
        .control1(control1), .in_top(bot_re_d4), .in_bottom(top_sum_d2),
        .out_top(bf_a_in1), .out_bottom()
    );
    switch1 #(.WIDTH(IN_WIDTH)) sw_bf_a_op2 (
        .control1(control1), .in_top(bot_re_d2), .in_bottom(s2_top_sum),
        .out_top(bf_a_in2), .out_bottom()
    );

    // ---- BF_A: always arithmetic. BF_B: mode-switched by control1 ----
    wire signed [IN_WIDTH:0] bf_a_sum_c, bf_a_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_a (
        .pass_thru(1'b0),
        .in1(bf_a_in1), .in2(bf_a_in2),
        .out_sum(bf_a_sum_c), .out_diff(bf_a_diff_c)
    );

    wire signed [IN_WIDTH:0] bf_b_sum_c, bf_b_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_b (
        .pass_thru(control1),           // same net that picked its own operands above
        .in1(bf_b_in1), .in2(bf_b_in2),
        .out_sum(bf_b_sum_c), .out_diff(bf_b_diff_c)
    );

    // ---- the one place a single SW1's two outputs both do real work ----
    wire signed [IN_WIDTH:0] rot_re_in, p1_c;
    switch1 #(.WIDTH(IN_WIDTH+1)) sw_rot_re_and_p1 (
        .control1(control1),
        .in_top(bf_a_diff_c), .in_bottom(bf_b_sum_c),
        .out_top(rot_re_in),   // -> rotator.re_in
        .out_bottom(p1_c)      // -> s3_p1
    );

    wire signed [IN_WIDTH+1:0] rot_re_c, rot_im_c;
    rotator #(.IN_WIDTH(IN_WIDTH+1), .COEF_WIDTH(WIDTH)) wk (
        .re_in(rot_re_in), .im_in(bf_b_diff_c),   // im_in is BF_B.diff unconditionally, both phases
        .cos_coef(rot_cos), .sin_coef(rot_sin),
        .re_out(rot_re_c), .im_out(rot_im_c)
    );

    // ---- register outputs, 1 cycle latency (same convention as stage1/stage2) ----
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
