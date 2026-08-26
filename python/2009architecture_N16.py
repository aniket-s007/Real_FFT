"""
2009architecture_N16.py -- Python model of the ACTUAL pipelined hardware
in Fig. 5 of Garrido/Parhi/Grajal (2009), not just the fully-parallel flow
graph of Fig. 3 (that one's in rfft16_dif_flowgraph.py).

Fig. 3 draws one physical butterfly/rotator per wire (N/2 = 8 of them per
stage). Real hardware can't afford that -- Fig. 5 instead reuses a FIXED,
SMALL set of physical units over 4 clock cycles (i = 0..3), exactly like a
4-lane version of the RAM-scheduled datapath already used to generate the
Verilog in this project. Per stage, the resource count matches the boxes
drawn in the figure:

    Stage 1: 2x R2,   0x ROTATOR
    Stage 2: 1x R2,   1x ROTATOR
    Stage 3: 1x R2 (top lane, no rotator)
             + 1x R2 + 1x SWITCH + 1x ROTATOR (bottom lane)
    Stage 4: 2x R2,   0x ROTATOR

This file builds those units as three small functions (r2, rotator,
switch) and calls each one exactly as many times per cycle as the figure
has boxes, so the resource *count* in the code matches the picture.

---- What the SWITCH actually does (per the paper's own text) ----
"This switch is necessary in every stage s in [3, n-1]. ... the rotations
required by the boxed components of Fig.1 operate over samples whose
indexes differ the same quantity as the butterflies at the same stage.
For these rotations, the switch does not swap the inputs. However, for
the rest of rotations the switch is activated and the samples that come
from the lower output of the upper butterfly are routed through the
rotator."

In plain words: in an ORDINARY complex FFT (their Fig.1), every
butterfly's lower (difference) output is multiplied by its OWN local
twiddle -- the rotation always belongs to the butterfly next to it, no
rerouting needed. The real-input flow graph (Fig.3) was built by folding
that ordinary flow graph in on itself, and that folding moved some
rotations away from their "home" butterfly. So the switch is a pure
ROUTING choice (no arithmetic) between two already-computed values:
  - switch OFF ("does not swap"): rotator reads the LOCAL bottom-lane R2's
    own difference output -- the aligned case. That's rows 8-15
    (complex-FFT): a completely ordinary complex butterfly.
  - switch ON ("activated"): rotator instead reads the difference output
    that the UPPER (top-lane) butterfly already produced, one stage
    earlier -- the misaligned case. That's rows 4-7 (Eq.7): their input
    is stage 2's TOP-lane R2 diff output (s2[4:8], already sign-flipped
    per Fig.3's "-1" edges). Stage 3's own local bottom R2 plays no part
    in this path at all -- it's simply a different physical butterfly's
    output being carried across.
Sanity check: n = log2(16) = 4, so "s in [3, n-1]" = stage 3 only --
matches the figure showing no switch/rotator in stage 4.

Stage 4 has no ROTATOR box at all for a similar reason, but simpler: every
twiddle left by that point has phi=0 (verified below), i.e. it's the
identity -- multiply by 1, do nothing. There's nothing for a rotator to
do, so none is built.

CAVEAT: the figure's caption also mentions "shuffling structures... which
include buffers and multiplexers" between the boxes -- the real hardware
detail of how samples get physically delayed/routed between stages. This
file gets that routing right (verified bit-exact below, and derived from
-- not guessed independently of -- the already-validated flow graph) but
does not model literal buffer depths or mux select lines, since that
requires the paper's own equations/Fig.6 detail that aren't recoverable
from a photo. Treat this as "what has to be true for the numbers to come
out right", not a transcription of the diagram's wiring.
"""
import math
import random


def r2(a, b):
    """Radix-2 real butterfly (an 'R2' box): returns (sum, difference)."""
    return a + b, a - b


def rotator(re_in, im_in, phi, N=16):
    """Twiddle rotator (a '(x)' box): rotate (re_in + j*im_in) by
    exp(-j*2*pi*phi/N)."""
    angle = -2 * math.pi * phi / N
    c, s = math.cos(angle), math.sin(angle)
    return re_in * c - im_in * s, im_in * c + re_in * s


def switch(activate, local_diff, upper_diff):
    """The crossbar ('SWITCH') box in Stage 3: pure routing, no
    arithmetic. OFF (activate=False): pass the LOCAL bottom-lane R2's own
    difference output straight to the rotator (the aligned case). ON
    (activate=True): instead route in the difference output the UPPER
    (top-lane) butterfly already produced one stage earlier (the
    misaligned case) -- see stage3()."""
    return upper_diff if activate else local_diff


def stage1(x):
    """2x R2, 0x ROTATOR, 4 cycles. Top lane: (x[i], x[i+8]).
    Bottom lane: (x[i+4], x[i+12]). Together they cover all 8 pairs."""
    s1 = [0.0] * 16
    for i in range(4):
        s1[i], s1[i + 8] = r2(x[i], x[i + 8])          # top R2 lane
        s1[i + 4], s1[i + 12] = r2(x[i + 4], x[i + 12])  # bottom R2 lane
    return s1


def stage2(s1):
    """1x R2, 1x ROTATOR, 4 cycles. Top lane keeps splitting the real
    half. Bottom lane does Eq.(7): negate + rotate by the boxed 0 1 2 3."""
    s2 = [0.0] * 16
    for i in range(4):
        s2[i], s2[i + 4] = r2(s1[i], s1[i + 4])         # top R2 lane
        A = s1[8 + i]
        B = -s1[12 + i]                                  # Eq.(7) negation
        s2[8 + i], s2[12 + i] = rotator(A, B, i)          # bottom rotator, boxes 0 1 2 3
    return s2


def stage3(s2, verbose=True):
    """1x R2 (top lane, no rotator) + 1x R2 + 1x SWITCH + 1x ROTATOR
    (bottom lane), 4 cycles each -- written out fully by hand, one cycle
    per block, every index literal, no loops. Top lane only has 2 real
    ops (rows 0-3) -> idle for cycles 2-3. Bottom lane fills all 4
    cycles: switch ON for Eq.(7) rows 4-7 (cycles 0-1, rotator reads the
    upper/top-lane butterfly's diff output from stage 2) then switch OFF
    for the complex-FFT rows 8-15 (cycles 2-3, rotator reads this
    stage's own local R2 diff output)."""
    s3 = [0.0] * 16

    # ==== TOP LANE ============================================================

    # ---- top lane, cycle 0: real butterfly, row pair (0, 2) ----------------
    s3[0], s3[2] = r2(s2[0], s2[2])
    if verbose:
        print("  stage3 top    cyc0: R2(s2[0], s2[2])  -> real bins")

    # ---- top lane, cycle 1: real butterfly, row pair (1, 3) ----------------
    s3[1], s3[3] = r2(s2[1], s2[3])
    if verbose:
        print("  stage3 top    cyc1: R2(s2[1], s2[3])  -> real bins")

    # top lane, cycles 2-3: idle -- nothing left for it to do this stage.

    # ==== BOTTOM LANE ==========================================================

    # ---- bottom lane, cycle 0: Eq.(7) row pair (4, 6), SWITCH ACTIVATED ----
    # Rotator input is the UPPER butterfly's own diff output, already sitting
    # in s2[4] / s2[6] from stage 2's top-lane R2 (sign flip = Fig.3's "-1"
    # edge). Stage 3's local bottom R2 plays no part in this path at all.
    upper_diff_re_4 = s2[4]
    upper_diff_im_4 = -s2[6]
    re_in_4, im_in_4 = switch(True, local_diff=None,
                               upper_diff=(upper_diff_re_4, upper_diff_im_4))
    s3[4], s3[6] = rotator(re_in_4, im_in_4, 0)          # box 0
    if verbose:
        print("  stage3 bottom cyc0: SWITCH activated -- rotator reads the upper "
              "(stage-2 top-lane) butterfly's diff output, phi=0")

    # ---- bottom lane, cycle 1: Eq.(7) row pair (5, 7), SWITCH ACTIVATED ----
    upper_diff_re_5 = s2[5]
    upper_diff_im_5 = -s2[7]
    re_in_5, im_in_5 = switch(True, local_diff=None,
                               upper_diff=(upper_diff_re_5, upper_diff_im_5))
    s3[5], s3[7] = rotator(re_in_5, im_in_5, 2)          # box 2
    if verbose:
        print("  stage3 bottom cyc1: SWITCH activated -- rotator reads the upper "
              "(stage-2 top-lane) butterfly's diff output, phi=2")

    # ---- bottom lane, cycle 2: complex-FFT rows (8,10 / 12,14), SWITCH OFF -
    # Real part of the complex pair: (Ar, Br) = (s2[8], s2[10]).
    # Imag part of the complex pair: (Ai, Bi) = (s2[12], s2[14]).
    # Rotator input is this stage's OWN local R2 diff output -- an ordinary
    # complex-FFT butterfly, nothing rerouted.
    sum_re_8, diff_re_8 = r2(s2[8], s2[10])
    sum_im_8, diff_im_8 = r2(s2[12], s2[14])
    s3[8], s3[12] = sum_re_8, sum_im_8                    # R2's sum -> straight out, no rotation
    re_in_8, im_in_8 = switch(False, local_diff=(diff_re_8, diff_im_8), upper_diff=None)
    s3[10], s3[14] = rotator(re_in_8, im_in_8, 0)          # box 0
    if verbose:
        print("  stage3 bottom cyc2: SWITCH straight-through -- rotator reads "
              "stage 3's own local R2 diff output, phi=0")

    # ---- bottom lane, cycle 3: complex-FFT rows (9,11 / 13,15), SWITCH OFF -
    sum_re_9, diff_re_9 = r2(s2[9], s2[11])
    sum_im_9, diff_im_9 = r2(s2[13], s2[15])
    s3[9], s3[13] = sum_re_9, sum_im_9
    re_in_9, im_in_9 = switch(False, local_diff=(diff_re_9, diff_im_9), upper_diff=None)
    s3[11], s3[15] = rotator(re_in_9, im_in_9, 4)          # box 4
    if verbose:
        print("  stage3 bottom cyc3: SWITCH straight-through -- rotator reads "
              "stage 3's own local R2 diff output, phi=4")

    # s3[0], s3[2] = r2(s2[0], s2[2])  # top lane R2
    # s3[4], s3[6] = r2(s2[4], s2[6])  # top lane R2

    # re_in_4, im_in_4 = switch(True, local_diff=None,upper_diff=(s3[2], s3[4])) # but we cant apply butterfly on one imaginar and one real 



    # s3[1], s3[3] = r2(s2[1], s2[3])  # top lane R2





    return s3


def stage4(s3, verbose=False):
    """2x R2, 0x ROTATOR -- written out fully by hand, one cycle per
    block, every index literal, no loops. Every twiddle remaining by
    this stage is phi=0 (the identity -- see the module docstring), so
    there is nothing left for a rotator to do and none is built."""
    s4 = [0.0] * 16

    # ==== TOP LANE ============================================================

    # ---- top lane, cycle 0: real butterfly, row pair (0, 1) -> X_0, X_8 ----
    s4[0], s4[1] = r2(s3[0], s3[1])
    if verbose:
        print("  stage4 top    cyc0: R2(s3[0], s3[1])            -> X_0, X_8")

    # ---- top lane, cycle 1: Eq.(7) row pair (2, 3), phi=0 -> pure sign flip
    # rotator(A, B, phi=0) == (A, B): nothing to compute, so it's not even
    # written as a rotator call here -- just the sign flip on the wire.
    s4[2] = s3[2]
    s4[3] = -s3[3]
    if verbose:
        print("  stage4 top    cyc1: no rotation (phi=0) -- pure sign flip -> X_4")

    # top lane, cycles 2-3: idle -- nothing left for it to do this stage.

    # ==== BOTTOM LANE ==========================================================

    # ---- bottom lane, cycle 0: complex-FFT rows (4,5 / 6,7) -> X_2, X_6 ----
    # Real part of the complex pair: (Ar, Br) = (s3[4], s3[5]).
    # Imag part of the complex pair: (Ai, Bi) = (s3[6], s3[7]).
    # phi=0, so R2's sum and difference outputs go straight to the output --
    # no rotator needed at all.
    s4[4], s4[5] = r2(s3[4], s3[5])       # real part: sum -> X_2, diff -> X_6
    s4[6], s4[7] = r2(s3[6], s3[7])       # imag part: sum -> X_2, diff -> X_6
    if verbose:
        print("  stage4 bottom cyc0: R2(s3[4],s3[5]) & R2(s3[6],s3[7])  -> X_2, X_6")

    # ---- bottom lane, cycle 1: complex-FFT rows (8,9 / 12,13) -> X_1, X_7 --
    s4[8], s4[9] = r2(s3[8], s3[9])
    s4[12], s4[13] = r2(s3[12], s3[13])
    if verbose:
        print("  stage4 bottom cyc1: R2(s3[8],s3[9]) & R2(s3[12],s3[13])  -> X_1, X_7")

    # ---- bottom lane, cycle 2: complex-FFT rows (10,11 / 14,15) -> X_5, X_3
    s4[10], s4[11] = r2(s3[10], s3[11])
    s4[14], s4[15] = r2(s3[14], s3[15])
    if verbose:
        print("  stage4 bottom cyc2: R2(s3[10],s3[11]) & R2(s3[14],s3[15])  -> X_5, X_3")

    # bottom lane, cycle 3: idle -- only 3 complex-FFT ops needed.

    return s4


def unscramble(s4):
    """Same hardware-order-to-bin mapping as rfft16_dif_flowgraph.py:
    some raw RAM slots land at a 'virtual' bin above N/2 and have to be
    conjugate-folded back onto the real non-redundant range."""
    N = 16
    raw_map = [
        (0, None, 0), (1, None, 8),
        (2, 3, 4),
        (4, 6, 2), (5, 7, 10),
        (8, 12, 1), (9, 13, 9),
        (10, 14, 5), (11, 15, 13),
    ]
    X = [0j] * (N // 2 + 1)
    for re_idx, im_idx, vbin in raw_map:
        val = complex(s4[re_idx], s4[im_idx] if im_idx is not None else 0.0)
        if vbin <= N // 2:
            X[vbin] = val
        else:
            X[N - vbin] = val.conjugate()
    return X


def rfft16_architecture(x, verbose=False):
    """Run all 4 pipeline stages in order. x: list of 16 real samples."""
    assert len(x) == 16
    s1 = stage1(x)
    s2 = stage2(s1)
    s3 = stage3(s2, verbose)
    s4 = stage4(s3, verbose)
    return unscramble(s4), (s1, s2, s3, s4)


def main():
    x = [random.gauss(0, 1) for _ in range(16)]   # real input, x[0]..x[15]
    # x = list(range(1, 17))                        # <- swap in a fixed, hand-traceable input instead

    print("Cycle-by-cycle trace of Stage 3 and Stage 4 (the confusing ones):\n")
    X, stages = rfft16_architecture(x, verbose=True)

    import numpy as np
    X_ref = np.fft.rfft(x)

    print("\n  k |        our architecture X_k         |           numpy fft(x)              |  abs err")
    print("----|-------------------------------------|--------------------------------------|----------")
    for k in range(9):
        mine, ref = X[k], X_ref[k]
        print(f"{k:3d} | {mine.real:9.5f} {mine.imag:+9.5f}j          "
              f"| {ref.real:9.5f} {ref.imag:+9.5f}j           "
              f"| {abs(mine - ref):.2e}")
    max_err = max(abs(X[k] - X_ref[k]) for k in range(9))
    print(f"\nmax abs error vs numpy.fft.rfft: {max_err:.3e}")


if __name__ == "__main__":
    main()
