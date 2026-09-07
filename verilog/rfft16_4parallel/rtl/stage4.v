`timescale 1ns/1ps

// stage4.v -- Column 4 ("D+SW1, BF", final) of the 4-parallel 16-point
// RFFT architecture, Salehi/Amirfattahi/Parhi 2013, Fig. 7. Exact hardware
// twin of stage4() in python/2013architecture_N16_4parallel.py. No W^k/
// CSDM box in this column -- every remaining twiddle is phi=0 (identity),
// matching stage4()'s use of a plain sign flip instead of rotator() for
// the one non-BF case (X4).
//
// Stage 4 combines each of Stage 3's 4 output ports (p0,p1,p2,p3) with
// its OWN value from 1 cycle earlier -- 4 independent same-port pairings
// (call them L0..L3, one per port), all becoming "ready" at the SAME two
// moments every frame: right after stage3's c3=1 arrives ("fire A", pairs
// c3=0 with c3=1) and right after c3=3 arrives ("fire B", pairs c3=2 with
// c3=3). Per the user's resource budget -- 2 switches, 1-cycle ("1D")
// delay elements, matching Fig. 7's 2 physical "D->BF" lanes -- only 2 of
// those 4 pairings can be combined per cycle, so each fire event is split
// across 2 output cycles.
//
// DERIVATION METHOD: mechanically derived and verified bit-exact (both a
// single frame and 4 frames back-to-back with no gap) by
// python/verify/gen_stage4_schedule.py before this file was written --
// see that script's header for the full cycle-by-cycle table. Summary:
//
//   pos | this cycle's live ports  | BF_X <= ...           | BF_Y <= ...
//   ----|--------------------------|------------------------|------------------------
//    0  | c3=0 of a NEW frame      | (p2_d2,p2_d1)  [L2,fireB of PREV frame]
//       | (stored for pos=1/3)     | (p3_d2,p3_d1)  [L3,fireB], pass_thru=0
//       |                          | out: s4[10],s4[11],s4[14],s4[15] of the PREVIOUS frame
//    1  | c3=1                     | (p0_d1,live p0) [L0,fireA] | (p1_d1,live p1) [L1,fireA], pass_thru=1
//       |                          | out: s4[0],s4[1],s4[2],s4[3]
//    2  | c3=2 (stored for later)  | (p2_d2,p2_d1)   [L2,fireA] | (p3_d2,p3_d1)   [L3,fireA], pass_thru=0
//       |                          | out: s4[4],s4[5],s4[6],s4[7]
//    3  | c3=3                     | (p0_d1,live p0) [L0,fireB] | (p1_d1,live p1) [L1,fireB], pass_thru=0
//       |                          | out: s4[8],s4[9],s4[12],s4[13]
//
// `pos` is mod-4 (NOT mod-5) -- matching the rate stage3 itself streams
// at, so it stays synchronized across back-to-back frames with no idle
// gap, the same reason stage3.v's own `c2_pos` is mod-4 rather than
// spanning its whole active window as a wider count. pos=0 does double
// duty: store-only AND (every occurrence after the very first) the cycle
// the previous frame's deferred L2/L3-fire-B pair finally fires, since
// p2_d1/p2_d2 still hold exactly that pair when pos wraps back to 0.
//
// Registers: p0_d1..p3_d1 (plain 1-cycle delay of s3_p0..p3, latched
// every cycle in_valid is high) and p2_d2,p3_d2 (one MORE 1-cycle delay
// on top of p2_d1/p3_d1). p0_d1/p1_d1 need no second delay stage -- L0/L1
// are always consumed the SAME cycle their live operand arrives (pos=1
// and pos=3), never held. All six are 1-cycle ("1D") elements, per the
// user's spec -- no 2D/4D delay anywhere in this column, unlike Stage 3.
//
// The two switches: each is a 2:1 mux choosing which of TWO CANDIDATE
// OPERAND PAIRS feeds its BF box this cycle -- "p0/p1's live pair" vs
// "p2/p3's held pair" -- not a single-sample crossbar the way stage3's
// switches were. Concatenating each pair onto one wide switch1 bus is the
// paper-faithful primitive here (python's own sw1(activate, local, upper)
// already takes PAIRS, e.g. local=(diff_re, diff_im)) -- unlike stage3,
// where an earlier, now-abandoned attempt concatenated two operands onto
// one switch1 bus and that was wrong; here the topology itself calls for
// selecting a whole pair, so packing is the correct primitive, not a
// workaround. Both switches share ONE control bit (control1 = pos[0]),
// matching the paper's claim that a single counter drives every switch.
//
// BF_X is always arithmetic. BF_Y is mode-switched: passthrough only at
// pos==1 (Fig. 4's c=1) -- the one cycle handling L1's "fire A" pairing,
// which is the paper's own X4 passthrough case -- arithmetic everywhere
// else. python's rule there is vals=[a,-b] (sum passed through unchanged,
// diff negated): BF_Y's passthrough out_sum already equals `a` needing no
// adjustment, but its out_diff (`b`) must be negated before reaching
// s4_p3 -- gated on pos==1 specifically, not baked into a fixed wire, the
// same pattern stage3.v's Eq.(7) negate already uses.
//
// Widths: s3_p0/p1 arrive at S3A_WIDTH, s3_p2/p3 at the 1-bit-wider
// S3B_WIDTH (rotator's 1-bit growth in Stage 3). Sign-extending the
// narrower ports up to S3B_WIDTH before they reach BF_X/BF_Y is exact,
// not approximate: real_bf never moves the binary point (out_sum/out_diff
// are just 1 bit wider for headroom) and rotator always rounds back to
// its input's own fractional scale, so every port already shares the same
// Q-format -- only the integer headroom differs. Output ports are
// declared uniformly at S3B_WIDTH+1 for the same reason; the couple of
// unused headroom bits on the L0/L1 outputs cost nothing.

module stage4 #(
    parameter WIDTH     = 8,
    parameter S2_WIDTH  = WIDTH + 2,
    parameter S3A_WIDTH = S2_WIDTH + 1,   // stage3's s3_p0/p1 width
    parameter S3B_WIDTH = S2_WIDTH + 2    // stage3's s3_p2/p3 width
) (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          in_valid,

    input  wire signed [S3A_WIDTH-1:0]   s3_p0,
    input  wire signed [S3A_WIDTH-1:0]   s3_p1,
    input  wire signed [S3B_WIDTH-1:0]   s3_p2,
    input  wire signed [S3B_WIDTH-1:0]   s3_p3,

    output reg                           out_valid,
    output reg  signed [S3B_WIDTH:0]     s4_p0,   // BF_X.sum
    output reg  signed [S3B_WIDTH:0]     s4_p1,   // BF_X.diff
    output reg  signed [S3B_WIDTH:0]     s4_p2,   // BF_Y.sum
    output reg  signed [S3B_WIDTH:0]     s4_p3    // BF_Y.diff (negated at pos==1 only)
);

    // ---- sign-extend the narrower ports up to S3B_WIDTH (exact -- see header) ----
    wire signed [S3B_WIDTH-1:0] s3_p0_ext = s3_p0;
    wire signed [S3B_WIDTH-1:0] s3_p1_ext = s3_p1;

    // ---- delay elements: all 1-cycle, but gated differently on purpose ----
    // p2_d2/p3_d2 advance EVERY cycle unconditionally (same convention as
    // every other stage's delay chain) -- they must keep sliding through
    // the tail cycle (pos=0 wrapped, in_valid already low) to deliver
    // fire-B's deferred L2/L3 pair. p0_d1..p3_d1 are gated on `in_valid`
    // instead: at the tail cycle there is no new stage-3 sample to latch
    // (its own out_valid has dropped, so s3_p0..p3 just hold stale data),
    // and p0_d1/p1_d1 in particular must NOT be overwritten there --
    // they're read live combinationally at the very next cycle (a new
    // frame's pos=1), so freezing them is what keeps a same-frame value
    // from leaking across the frame boundary. Both halves are exercised
    // and checked by gen_stage4_schedule.py's HwStage4 (same asymmetric
    // gating, back-to-back frames included).
    reg signed [S3B_WIDTH-1:0] p0_d1, p1_d1, p2_d1, p3_d1;
    reg signed [S3B_WIDTH-1:0] p2_d2, p3_d2;

    always @(posedge clk) begin
        if (!rst_n) begin
            p0_d1 <= 0; p1_d1 <= 0; p2_d1 <= 0; p3_d1 <= 0;
            p2_d2 <= 0; p3_d2 <= 0;
        end else begin
            p2_d2 <= p2_d1;
            p3_d2 <= p3_d1;
            if (in_valid) begin
                p0_d1 <= s3_p0_ext; p1_d1 <= s3_p1_ext;
                p2_d1 <= s3_p2;     p3_d1 <= s3_p3;
            end
        end
    end

    // ---- pos: this module's own mod-4 "which stage-3 cycle is live" counter ----
    reg in_valid_d1;
    always @(posedge clk) begin
        if (!rst_n) in_valid_d1 <= 1'b0;
        else        in_valid_d1 <= in_valid;
    end

    wire active     = in_valid | in_valid_d1;
    wire taps_ready = active & in_valid_d1;

    reg [1:0] pos;
    always @(posedge clk) begin
        if (!rst_n) pos <= 2'd0;
        else if (active) pos <= pos + 2'd1;   // wraps mod 4 on the 2-bit reg
    end

    wire control1    = pos[0];      // 1 at pos=1,3 (p0/p1's live pair); 0 at pos=2,0 (p2/p3's held pair)
    wire pass_thru_y = (pos == 2'd1);

    // ---- switch_x / switch_y: each selects a whole OPERAND PAIR (see header) ----
    // switch1 is a plain crossbar: out_top = control1 ? in_bottom : in_top.
    // control1=1 must select the p0/p1 pair (pos=1,3), so that pair goes on
    // in_bottom; the p2/p3 pair goes on in_top for control1=0 (pos=2,0).
    // Each pair is packed {older, newer} so out_top/out_bottom unpack
    // straight into real_bf's in1(older)/in2(newer) -- matching the
    // required diff = older-newer order (see header / gen_stage4_schedule.py).
    wire signed [S3B_WIDTH-1:0] switch_x_out_top, switch_x_out_bottom;
    switch1 #(.WIDTH(2*S3B_WIDTH)) switch_x (
        .control1(control1),
        .in_top({p2_d2, p2_d1}), .in_bottom({p0_d1, s3_p0_ext}),
        .out_top({switch_x_out_top, switch_x_out_bottom}), .out_bottom()
    );

    wire signed [S3B_WIDTH-1:0] switch_y_out_top, switch_y_out_bottom;
    switch1 #(.WIDTH(2*S3B_WIDTH)) switch_y (
        .control1(control1),
        .in_top({p3_d2, p3_d1}), .in_bottom({p1_d1, s3_p1_ext}),
        .out_top({switch_y_out_top, switch_y_out_bottom}), .out_bottom()
    );

    // ---- BF_X: always arithmetic. BF_Y: passthrough only at pos==1 ----
    wire signed [S3B_WIDTH:0] bf_x_sum_c, bf_x_diff_c;
    real_bf #(.WIDTH(S3B_WIDTH)) bf_x (
        .pass_thru(1'b0),
        .in1(switch_x_out_top), .in2(switch_x_out_bottom),
        .out_sum(bf_x_sum_c), .out_diff(bf_x_diff_c)
    );

    wire signed [S3B_WIDTH:0] bf_y_sum_c, bf_y_diff_raw_c;
    real_bf #(.WIDTH(S3B_WIDTH)) bf_y (
        .pass_thru(pass_thru_y),
        .in1(switch_y_out_top), .in2(switch_y_out_bottom),
        .out_sum(bf_y_sum_c), .out_diff(bf_y_diff_raw_c)
    );

    // s4[3] = -b at pos==1 (python: vals=[a,-b]); every other cycle BF_Y is
    // already in arithmetic mode and out_diff needs no extra negate.
    wire signed [S3B_WIDTH:0] bf_y_diff_c = pass_thru_y ? -bf_y_diff_raw_c : bf_y_diff_raw_c;

    // ---- register outputs, 1 cycle latency (same convention as stages 1-3) ----
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            s4_p0 <= 0; s4_p1 <= 0; s4_p2 <= 0; s4_p3 <= 0;
        end else begin
            out_valid <= taps_ready;
            s4_p0 <= bf_x_sum_c;
            s4_p1 <= bf_x_diff_c;
            s4_p2 <= bf_y_sum_c;
            s4_p3 <= bf_y_diff_c;
        end
    end

endmodule
