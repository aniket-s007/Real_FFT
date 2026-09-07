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
// EXACTLY 3 switch1 (SW1) instances and 4 delay elements (all 2-deep),
// matching Fig. 6's ("Shuffling structure of the pipelined RFFT")
// worked N=16 example and the paper's own resource count for this
// column -- not the 5-switch/asymmetric-2D-4D version an earlier
// session built. That version reached the same s3[] values with a
// wrong topology: it put BOTH of top_sum's and top_diff's own 2-deep
// delays BEFORE their switch (never needed) and stacked FOUR deep
// registers directly on bot_re/bot_im with no switch involved in
// reaching the far tap. Per Fig. 6, the switch sits BETWEEN two
// 2-deep delays, not downstream of a single deep one:
//
//   s2_top_sum  ----------------------------> switch_top.in_top (LIVE, no delay)
//   s2_bot_re -> [2D] -----------------------> switch_top.in_bottom
//   switch_top.out_top -> [2D] -> BF_A.in1 (the "older" operand)
//   switch_top.out_bottom ------------------->  BF_A.in2 (the "newer" operand, live)
//
// and symmetrically s2_top_diff/s2_bot_im -> switch_bottom -> BF_B.
// (The name "switch_top"/"switch_bottom" here is Fig. 6's own, from the
// row each occupies -- not related to a signal's "top_sum" name.)
//
// Why this reproduces stage3()'s s3[] bit-exactly (checked by hand,
// then by the testbench): trace switch_top with control1 = c2_pos[1]
// following the pattern 0,0,1,1 over 4 stage-2 cycles c2=0..3 (s2_top_sum
// carries s2[c2], s2_bot_re carries s2[8+c2]):
//
//   g | c2_pos | control1 | switch_top.out_top | switch_top.out_bottom
//   --|--------|----------|---------------------|------------------------
//   0 |   0    |    0     | s2_top_sum(0)=s2[0] | bot_re_d2(0) (pre-stream, unused)
//   1 |   1    |    0     | s2_top_sum(1)=s2[1] | bot_re_d2(1) (pre-stream, unused)
//   2 |   2    |    1     | bot_re_d2(2)=s2[8]  | s2_top_sum(2)=s2[2]
//   3 |   3    |    1     | bot_re_d2(3)=s2[9]  | s2_top_sum(3)=s2[3]
//   4 |  0(wrap)|   0     | bot_re_d2(4)=s2[10] | s2_top_sum(4) (post-stream, unused)
//   5 |  1     |    0     | bot_re_d2(5)=s2[11] | s2_top_sum(5) (post-stream, unused)
//
// Delaying out_top by 2 MORE cycles (the 4th delay element) and pairing
// it with out_bottom LIVE at each g gives BF_A's actual two operands:
//
//   g=2: BF_A(out_top_d2(2)=out_top(0)=s2[0], out_bottom(2)=s2[2]) = bf(s2[0],s2[2])   -- c3=0
//   g=3: BF_A(out_top(1)=s2[1],               out_bottom(3)=s2[3]) = bf(s2[1],s2[3])   -- c3=1
//   g=4: BF_A(out_top(2)=s2[8],                out_bottom(4)=s2[10])= bf(s2[8],s2[10]) -- c3=2
//   g=5: BF_A(out_top(3)=s2[9],                out_bottom(5)=s2[11])= bf(s2[9],s2[11]) -- c3=3
//
// -- exactly stage3()'s 4 required top/bot_re pairs, in the exact order
// c2_pos/control1 already expects (see the schedule table below). The
// symmetric trace for switch_bottom (top_diff/bot_im) lands on
// bf(s2[4],-s2[6]), bf(s2[5],-s2[7]) [BF_B passthrough, Eq.(7) pair] then
// bf(s2[12],s2[14]), bf(s2[13],s2[15]) [BF_B arithmetic, BI pair] at the
// same g=2,3,4,5. The Eq.(7) negate only belongs on switch_bottom's
// out_bottom, and only while it's carrying top_diff's live tap
// (control1=1, g=2,3) -- NOT while it's carrying bot_im's delayed tap
// (control1=0, g=4,5), so it's gated by control1 rather than applied to
// a fixed wire the way the old 5-switch version did.
//
// A note on why the previous session's asymmetric-depth argument doesn't
// apply here: that argument assumed all four stage-2 signals reach the
// BF inputs un-shuffled, so all four row-groups become ready at the same
// two cycles and a 2-box/4-group collision forces a 4-cycle hold on two
// of them. Fig. 6's shuffle (this file, now) reorders top_sum/bot_re and
// top_diff/bot_im onto each switch's two OUTPUTS before either BF sees
// them, which is exactly what removes that collision -- each BF box
// only ever sees ONE ready pair per cycle, no arbitration between 4
// candidates needed. That earlier argument was wrong, not just a
// different valid topology; see PROJECT_LOG.md for the corrected note.
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

    // ---- delay elements 1,2: 2D on bot_re/bot_im, BEFORE their switch ----
    // Registered UNCONDITIONALLY every cycle (same convention as
    // stage1.v/stage2.v) -- only the valid/counter logic below decides
    // which cycles' taps are ever actually used. top_sum/top_diff need
    // NO delay here -- they feed their switch LIVE (see header comment).
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

    // ---- switch_top / switch_bottom: Fig. 6's shuffle, BEFORE the BFs ----
    // Each switch's 2 inputs are (this lane's LIVE sample, the OTHER
    // lane's sample already delayed by 2) -- exactly the wiring the user
    // pointed out from Fig. 6: top_sum/top_diff reach their switch
    // directly, bot_re/bot_im reach theirs through delay elements 1,2.
    wire signed [IN_WIDTH-1:0] switch_top_out_top,    switch_top_out_bottom;
    switch1 #(.WIDTH(IN_WIDTH)) switch_top (
        .control1(control1),
        .in_top(s2_top_sum), .in_bottom(bot_re_d2),
        .out_top(switch_top_out_top), .out_bottom(switch_top_out_bottom)
    );

    wire signed [IN_WIDTH-1:0] switch_bottom_out_top, switch_bottom_out_bottom;
    switch1 #(.WIDTH(IN_WIDTH)) switch_bottom (
        .control1(control1),
        .in_top(s2_top_diff), .in_bottom(bot_im_d2),
        .out_top(switch_bottom_out_top), .out_bottom(switch_bottom_out_bottom)
    );

    // ---- delay elements 3,4: 2D on each switch's out_top, AFTER the
    // switch -- reassembles the "older" operand for BF_A/BF_B (see the
    // g=0..5 trace in the header comment: out_top(g-2) always pairs
    // correctly with out_bottom(g)). ----
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

    // BF_A's operands: older (delayed) + newer (live off the switch).
    wire signed [IN_WIDTH-1:0] s3_top_bf_top_in = switch_top_out_top_d2;
    wire signed [IN_WIDTH-1:0] s3_top_bf_bottom_in = switch_top_out_bottom;

    // BF_B's operands: same shape, but the Eq.(7) negate belongs only on
    // the NEWER tap, and only while that tap is top_diff's own live
    // sample (control1=1) -- not while it's bot_im's delayed sample
    // (control1=0). Gating on control1 (rather than negating a fixed
    // wire before the switch, as the previous version did) is what
    // keeps bot_im's own pair un-negated.
    wire signed [IN_WIDTH-1:0] s3_bottom_bf_top_in = switch_bottom_out_top_d2;
    wire signed [IN_WIDTH-1:0] s3_bottom_bf_bottom_in = control1 ? -switch_bottom_out_bottom : switch_bottom_out_bottom;

    // ---- BF_A: always arithmetic. BF_B: mode-switched by control1 ----
    wire signed [IN_WIDTH:0] bf_a_sum_c, top_bf_out_diff;
    real_bf #(.WIDTH(IN_WIDTH)) bf_a (
        .pass_thru(1'b0),                                               //Top Butterfly
        .in1(s3_top_bf_top_in), .in2(s3_top_bf_bottom_in),
        .out_sum(bf_a_sum_c), .out_diff(top_bf_out_diff)
    );

    wire signed [IN_WIDTH:0] top_bf_out_sum, bf_b_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_b (                                  //Bottom Butterfly 
        .pass_thru(control1),           // same net that picked its own operands above
        .in1(s3_bottom_bf_top_in), .in2(s3_bottom_bf_bottom_in),
        .out_sum(top_bf_out_sum), .out_diff(bf_b_diff_c)
    );

    // ---- the one place a single SW1's two outputs both do real work ----
    wire signed [IN_WIDTH:0] rot_re_in, p1_c;
    switch1 #(.WIDTH(IN_WIDTH+1)) sw_rot_re_and_p1 (
        .control1(control1),
        .in_top(top_bf_out_diff), .in_bottom(top_bf_out_sum),
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
