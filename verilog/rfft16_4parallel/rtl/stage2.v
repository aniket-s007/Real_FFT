`timescale 1ns/1ps

// stage2.v -- Column 2 ("BF / W^k") of the 4-parallel 16-point RFFT
// architecture, Salehi/Amirfattahi/Parhi 2013. Exact hardware twin of
// stage2() in python/2013architecture_N16_4parallel.py:
//
//   def stage2(s1):
//       s2 = [0.0] * N
//       for k in range(4):
//           s2[k], s2[k+4] = bf(s1[k], s1[k+4])       # top BF box
//           A = s1[8+k]
//           B = -s1[12+k]                              # Eq.(7) sign flip
//           s2[8+k], s2[12+k] = rotator(A, B, k)       # W^k box, boxes 0 1 2 3
//       return s2
//
// IMPORTANT WIRING NOTE: this stage's top BF combines the two SUM
// outputs of Stage 1 (s1[k] and s1[k+4] -- one from stage1's top lane,
// one from its bottom lane), and its bottom W^k box combines the two
// DIFF outputs of Stage 1 (s1[k+8] and s1[k+12]). That means Stage 2
// does NOT simply chain each of Stage 1's two physical lanes straight
// through -- it crosses them. This module's input ports are therefore
// named by which s1[] value they must carry, not by "top"/"bottom", so
// the required cross-connection is unambiguous when wiring stage1 into
// stage2:
//     stage1.s1_top_sum  -> stage2.s1_k
//     stage1.s1_bot_sum  -> stage2.s1_k4
//     stage1.s1_top_diff -> stage2.s1_k8
//     stage1.s1_bot_diff -> stage2.s1_k12
//
// Top lane here: one more plain real BF (real_bf.v, same block as Stage 1).
// Bottom lane: Eq.(7)'s sign flip (a free wire negate) feeding the W^k
// rotator -- the one genuine multiplier column that must serve all four
// twiddles W^0..W^3 every 4-cycle frame (k=1,3 need real multiplication;
// k=0,2 are trivial/CSDM-able in principle, but this is a single shared
// physical unit reused every cycle, matching the block diagram's one
// "W^k" box on the bottom lane).
//
// WIDTH is the ORIGINAL top-level sample width (fixes the fractional
// scale used everywhere in this pipeline); IN_WIDTH is Stage 1's actual
// output width (WIDTH+1).
//
// Twiddle ROM: `real` and $cos/$sin are simulation-only (outside
// Vivado's synthesizable subset per UG901), so they can't appear in
// this file at all if it's ever meant to go through synth_design --
// unlike everywhere else in this pipeline, "just simulate it" isn't
// enough here. Instead the trig is done once, offline, in
// python/gen_twiddle_master_4parallel.py, at a fixed high precision
// (MASTER_WIDTH bits) and hardcoded below as plain signed integers
// (COS0_M..SIN3_M -- no `real` anywhere). Re-running the SQNR sweep at
// a different WIDTH still "just works" from a single parameter change:
// derive_coef() rescales those MASTER-precision constants down to the
// current WIDTH via an ordinary integer round + arithmetic-shift +
// saturate -- the same rescale technique rotator.v already uses on its
// own post-multiply result, just applied at elaboration time instead of
// at runtime. Verified bit-exact (0 LSB diff) against the old
// per-WIDTH $cos/$sin/$rtoi values for WIDTH=4..24 by the generator
// script above before these constants were trusted enough to hardcode.
// cos(phi=0) rounds to exactly +1.0, not representable in signed
// Q1.(WIDTH-1) (max value is 1 - 2^-(WIDTH-1)); derive_coef()'s
// saturate clamp handles that case the same way for every WIDTH instead
// of silently wrapping to -1.0 in two's complement (caught by testing
// rotator.v standalone before this file was written -- see that
// module's own comment).

module stage2 #(
    parameter WIDTH    = 8,
    parameter IN_WIDTH = WIDTH + 1
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,

    input  wire signed [IN_WIDTH-1:0] s1_k,     // s1[k]    (stage1.s1_top_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k4,    // s1[k+4]  (stage1.s1_bot_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k8,    // s1[k+8]  (stage1.s1_top_diff)
    input  wire signed [IN_WIDTH-1:0] s1_k12,   // s1[k+12] (stage1.s1_bot_diff)

    output reg                        out_valid,
    output reg  signed [IN_WIDTH:0]   s2_top_sum,   // -> s2[k]
    output reg  signed [IN_WIDTH:0]   s2_top_diff,  // -> s2[k+4]
    output reg  signed [IN_WIDTH:0]   s2_bot_re,    // -> s2[k+8]
    output reg  signed [IN_WIDTH:0]   s2_bot_im     // -> s2[k+12]
);

    // ---- twiddle ROM: 4 entries, phi = 0..3 ----
    // MASTER-precision (Q1.(MASTER_WIDTH-1)) constants, derived offline
    // by python/gen_twiddle_master_4parallel.py -- plain signed integers,
    // no `real`, no $cos/$sin. Valid for WIDTH in [2, MASTER_WIDTH-1]
    // (checked for WIDTH=4..36, comfortably covering the WIDTH=32
    // default above); bump MASTER_WIDTH (and re-run the generator) if a
    // sweep point ever needs WIDTH >= 39.
    localparam MASTER_WIDTH = 40;
    localparam signed [MASTER_WIDTH-1:0] COS0_M = 40'sd549755813887, SIN0_M = 40'sd0;
    localparam signed [MASTER_WIDTH-1:0] COS1_M = 40'sd507908144330, SIN1_M = -40'sd210382441821;
    localparam signed [MASTER_WIDTH-1:0] COS2_M = 40'sd388736063997, SIN2_M = -40'sd388736063997;
    localparam signed [MASTER_WIDTH-1:0] COS3_M = 40'sd210382441821, SIN3_M = -40'sd507908144330;

    localparam SHIFT = MASTER_WIDTH - WIDTH;

    // Rescale a MASTER_WIDTH-bit constant down to the current WIDTH:
    // round (add half an LSB) then arithmetic-shift, then saturate --
    // same technique as rotator.v's own round_shift_sat, just run here
    // on constants at elaboration time instead of on signals at runtime.
    // Only ever called below with constant (localparam) arguments, so
    // it fully resolves to plain integers before synthesis sees it.
    // function signed [WIDTH-1:0] derive_coef;
    //     input signed [MASTER_WIDTH-1:0] raw;
    //     reg signed [MASTER_WIDTH:0] rounded, shifted;
    //     reg signed [MASTER_WIDTH:0] max_out, min_out;
    //     begin
    //         rounded = raw + (1 <<< (SHIFT-1));
    //         shifted = rounded >>> SHIFT;
    //         max_out = (1 <<< (WIDTH-1)) - 1;
    //         min_out = -(1 <<< (WIDTH-1));
    //         if (shifted > max_out)
    //             derive_coef = max_out[WIDTH-1:0];
    //         else if (shifted < min_out)
    //             derive_coef = min_out[WIDTH-1:0];
    //         else
    //             derive_coef = shifted[WIDTH-1:0];
    //     end
    // endfunction

    function signed [WIDTH-1:0] derive_coef;
        input signed [MASTER_WIDTH-1:0] raw;
        begin
            derive_coef= raw[MASTER_WIDTH-1 -: WIDTH];
        end
    endfunction

    localparam signed [WIDTH-1:0] COS0 = derive_coef(COS0_M);
    localparam signed [WIDTH-1:0] SIN0 = derive_coef(SIN0_M);
    localparam signed [WIDTH-1:0] COS1 = derive_coef(COS1_M);
    localparam signed [WIDTH-1:0] SIN1 = derive_coef(SIN1_M);
    localparam signed [WIDTH-1:0] COS2 = derive_coef(COS2_M);
    localparam signed [WIDTH-1:0] SIN2 = derive_coef(SIN2_M);
    localparam signed [WIDTH-1:0] COS3 = derive_coef(COS3_M);
    localparam signed [WIDTH-1:0] SIN3 = derive_coef(SIN3_M);

    // ---- k = 0..3 cycle counter, self-generated from in_valid pulses ----
    reg [1:0] k;
    always @(posedge clk) begin
        if (!rst_n)
            k <= 2'd0;
        else if (in_valid)
            k <= (k == 2'd3) ? 2'd0 : k + 2'd1;
    end

    reg signed [WIDTH-1:0] cos_coef, sin_coef;
    always @(*) begin
        case (k)
            2'd0: begin cos_coef = COS0; sin_coef = SIN0; end
            2'd1: begin cos_coef = COS1; sin_coef = SIN1; end
            2'd2: begin cos_coef = COS2; sin_coef = SIN2; end
            default: begin cos_coef = COS3; sin_coef = SIN3; end
        endcase
    end

    // ---- top lane: plain real BF ----
    wire signed [IN_WIDTH:0] top_sum_c, top_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_top (
        .pass_thru(1'b0),                       // always arithmetic in this column
        .in1(s1_k), .in2(s1_k4),
        .out_sum(top_sum_c), .out_diff(top_diff_c)
    );

    // ---- bottom lane: Eq.(7) sign flip, then W^k rotator ----
    wire signed [IN_WIDTH-1:0] eq7_b = -s1_k12;
    wire signed [IN_WIDTH:0] bot_re_c, bot_im_c;
    rotator #(.IN_WIDTH(IN_WIDTH), .COEF_WIDTH(WIDTH)) wk (
        .re_in(s1_k8), .im_in(eq7_b),
        .cos_coef(cos_coef), .sin_coef(sin_coef),
        .re_out(bot_re_c), .im_out(bot_im_c)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid   <= 1'b0;
            s2_top_sum  <= 0;
            s2_top_diff <= 0;
            s2_bot_re   <= 0;
            s2_bot_im   <= 0;
        end else begin
            out_valid   <= in_valid;
            s2_top_sum  <= top_sum_c;
            s2_top_diff <= top_diff_c;
            s2_bot_re   <= bot_re_c;
            s2_bot_im   <= bot_im_c;
        end
    end

endmodule
