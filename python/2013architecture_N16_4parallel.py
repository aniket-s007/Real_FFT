"""
2013architecture_N16_4parallel.py -- 4-parallel pipelined RFFT hardware
model for Salehi, Amirfattahi & Parhi, "Pipelined Architectures for
Real-Valued FFT and Hermitian-Symmetric IFFT With Real Datapaths," IEEE
TCAS-II, 2013.

Per the user's instruction, this file follows ONLY their hand-annotated
"Fig. 3" flow graph and the 4-parallel block-diagram sketch (BF -> BF ->
W^k -> SW1+2D -> BF -> CSDM -> D+SW1 -> BF, four input lanes x(k),
x(k+N/4), x(k+N/2), x(k+3N/4)) -- NOT the paper's own printed derivation
path (its Fig. 2 "regularized" structure and Fig. 7).

What the hand-drawn Fig. 3 actually is: its output bin order -- X(0),
X(8), X(4), X(2), X(10), X(1), X(9), X(5), X(13) -- is an exact match for
the paper's own Fig. 1, and the paper's text says Fig. 1 is inherited
unchanged from reference [8] (Garrido/Parhi/Grajal 2009). That's the same
flow graph already coded and validated bit-exact against numpy.fft.rfft
in rfft16_dif_flowgraph.py. So rfft16_flowgraph() below reproduces that
same stage-by-stage dataflow (kept self-contained in this file rather
than imported, matching this project's existing 2009architecture_N16.py
pattern), and serves as the ground truth the 4-parallel hardware model is
checked against.

The 4-parallel architecture: with L=4 parallel real samples arriving each
cycle, N=16 takes 4 cycles (k=0..3) to load. Working out which flow-graph
operations land in which cycle on which of the diagram's two physical
lanes reproduces -- operation-for-operation, bit-exact -- the same
resource schedule already derived and validated in 2009architecture_N16.py
for this same flow graph. That's expected, not circular: this file just
relabels that schedule with the 2013 paper's own block names (BF, W^k,
SW1, CSDM, D/2D delay) and frames it explicitly as 4 streaming lanes
instead of "a small reused unit," to match the block diagram picture
instead of the abstract R2/rotator/switch description used there.

A nice cross-check that the mapping is sound: the paper's text singles
out W^2 (phi=2) as "the only Wk-block in the third stage that needs
actual multiplication," realized in hardware by a CSDM. Independently
re-deriving the schedule from the flow graph (not from the paper's own
claim) lands phi=2 in exactly that one slot -- Stage 3, cycle 1, on the
lane feeding the diagram's CSDM box.

CSDM: per the user's choice, modeled as exact rotation math (not the
literal two-additions-plus-CSD-constant decomposition the paper
describes for real hardware), so the comparison against numpy stays
bit-exact rather than picking up CSD quantization error.

CAVEAT (same one already on 2009architecture_N16.py): SW1's role here is
the net ROUTING effect Fig. 5(a) describes (a periodic switch that either
passes each lane straight through or swaps them), not a cycle-accurate
model of the diagram's literal 2D/D shift-register depths or mux control
lines -- that would need the paper's own equations, not just the picture.
"""
import math
import random

N = 16


# ---------------------------------------------------------------------------
# Building blocks, named to match the boxes in the block-diagram picture.
# ---------------------------------------------------------------------------

def bf(in1, in2, c=0):
    """The BF box (Fig. 4's circuit). c=0: arithmetic mode, returns
    (in1+in2, in1-in2). c=1: real/imaginary passthrough mode -- the
    paper's own description of this box ("for real-imaginary inputs,
    input data are simply transferred to the output") -- returns
    (in1, in2) unchanged. Used below wherever a BF box sits at a wire
    position that isn't combining two operands this cycle, just holding
    a place in the pipeline."""
    if c:
        return in1, in2
    return in1 + in2, in1 - in2


def rotator(re_in, im_in, phi, n=N):
    """The W^k box: multiply (re_in + j*im_in) by exp(-j*2*pi*phi/n)."""
    angle = -2 * math.pi * phi / n
    c, s = math.cos(angle), math.sin(angle)
    re_out = re_in * c - im_in * s
    im_out = im_in * c + re_in * s
    return re_out, im_out


def csdm(re_in, im_in):
    """The CSDM box: multiply by W^2 (rotate 45 deg, scale by 1/sqrt(2)).
    Real hardware realizes the 1/sqrt(2) scale with two additions and a
    canonical-signed-digit constant multiply; here it's just rotator(...,
    phi=2) under the diagram's own name, using exact math as agreed."""
    return rotator(re_in, im_in, 2)


def sw1(activate, local, upper):
    """The SW1 shuffle switch (Fig. 5(a)): pure routing, no arithmetic.
    activate=False: pass this lane's own LOCAL just-computed pair
    through unchanged. activate=True: instead route in the pair the
    UPPER lane already produced earlier in the pipeline (carried across
    via the diagram's 2D delay). See file docstring caveat -- this
    models the net routing effect, not literal buffer depths."""
    return upper if activate else local


# ---------------------------------------------------------------------------
# Ground truth: the hand-drawn Fig. 3 flow graph, fully parallel (N/2 = 8
# butterflies per stage, no hardware resource limit). Self-contained twin
# of rfft16_dif_flowgraph.py's rfft16_flowgraph(), kept in this file so it
# can be cross-checked against the 4-parallel model below without an
# import across a filename that starts with a digit.
# ---------------------------------------------------------------------------

def rfft16_flowgraph(x):
    """x: list of 16 real samples. Returns the raw (still hardware-order)
    contents of the RAM/wires after each of the 4 stages."""
    assert len(x) == N

    # STAGE 1: plain real butterfly, rows (i, i+8). No twiddle box.
    s1 = [0.0] * N
    for i in range(8):
        A, B = x[i], x[i + 8]
        s1[i] = A + B
        s1[i + 8] = A - B

    # STAGE 2
    s2 = [0.0] * N
    for i in range(4):                       # rows 0-7: real, no rotation
        A, B = s1[i], s1[i + 4]
        s2[i] = A + B
        s2[i + 4] = A - B
    for k in range(4):                       # rows 8-15: Eq.(7) + boxes 0 1 2 3
        A = s1[8 + k]
        B = -s1[12 + k]
        r, im = rotator(A, B, k)
        s2[8 + k] = r
        s2[12 + k] = im

    # STAGE 3
    s3 = [0.0] * N
    for i in range(2):                       # rows 0-3: real, no rotation
        A, B = s2[i], s2[i + 2]
        s3[i] = A + B
        s3[i + 2] = A - B
    for k in range(2):                       # rows 4-7: Eq.(7) + boxes 0 2
        A = s2[4 + k]
        B = -s2[6 + k]
        r, im = rotator(A, B, 2 * k)
        s3[4 + k] = r
        s3[6 + k] = im
    for k in range(2):                       # rows 8-15: CFFT butterfly + boxes 0 4
        Ar, Br = s2[8 + k], s2[10 + k]
        Ai, Bi = s2[12 + k], s2[14 + k]
        s3[8 + k] = Ar + Br
        s3[12 + k] = Ai + Bi
        r, im = rotator(Ar - Br, Ai - Bi, 4 * k)
        s3[10 + k] = r
        s3[14 + k] = im

    # STAGE 4 (final)
    s4 = [0.0] * N
    s4[0] = s3[0] + s3[1]                    # -> X0, X8
    s4[1] = s3[0] - s3[1]
    r, im = rotator(s3[2], -s3[3], 0)         # -> X4
    s4[2], s4[3] = r, im
    Ar, Br, Ai, Bi = s3[4], s3[5], s3[6], s3[7]      # -> X2, X6
    s4[4], s4[6] = Ar + Br, Ai + Bi
    r, im = rotator(Ar - Br, Ai - Bi, 0)
    s4[5], s4[7] = r, im
    Ar, Br, Ai, Bi = s3[8], s3[9], s3[12], s3[13]    # -> X1, X7
    s4[8], s4[12] = Ar + Br, Ai + Bi
    r, im = rotator(Ar - Br, Ai - Bi, 0)
    s4[9], s4[13] = r, im
    Ar, Br, Ai, Bi = s3[10], s3[11], s3[14], s3[15]  # -> X5, X3
    s4[10], s4[14] = Ar + Br, Ai + Bi
    r, im = rotator(Ar - Br, Ai - Bi, 0)
    s4[11], s4[15] = r, im

    return unscramble(s4)


# ---------------------------------------------------------------------------
# The 4-parallel pipelined architecture: BF -> BF/W^k -> SW1+2D,BF,CSDM ->
# D+SW1,BF. Four real samples x(k), x(k+N/4), x(k+N/2), x(k+3N/4) arrive
# per cycle, k = 0..3, across the two physical top/bottom lanes drawn in
# the block diagram.
# ---------------------------------------------------------------------------

def stage1(x):
    """Column 1 ("BF, BF"): top lane pairs (x(k), x(k+N/2)), bottom lane
    pairs (x(k+N/4), x(k+3N/4)) -- together, over 4 cycles, the same 8
    real butterflies as the flow graph's Stage 1."""
    s1 = [0.0] * N
    for k in range(4):
        xk, xk_n2 = x[k], x[k + 8]
        xk_n4, xk_3n4 = x[k + 4], x[k + 12]
        s1[k], s1[k + 8] = bf(xk, xk_n2)          # top BF box
        s1[k + 4], s1[k + 12] = bf(xk_n4, xk_3n4)  # bottom BF box
    return s1


def stage2(s1):
    """Column 2 ("BF / W^k"): top BF box keeps splitting the still-real
    top half; bottom W^k box does Eq.(7)'s negate-and-rotate, boxed
    twiddles 0 1 2 3."""
    s2 = [0.0] * N
    for k in range(4):
        s2[k], s2[k + 4] = bf(s1[k], s1[k + 4])     # top BF box
        A = s1[8 + k]
        B = -s1[12 + k]                              # Eq.(7) sign flip
        s2[8 + k], s2[12 + k] = rotator(A, B, k)      # W^k box, boxes 0 1 2 3
    return s2


def stage3(s2, verbose=False):
    """Column 3 ("SW1+2D, BF, CSDM"): top lane finishes rows 0-3 (2
    cycles, then idle). Bottom lane, cycles 0-1: SW1 ON, routes in the
    TOP lane's own Eq.(7) pair from Stage 2 through a passthrough BF
    (Fig.4 c=1) into the W^k/CSDM box -- boxed twiddles 0, 2, and phi=2
    is exactly the CSDM slot the paper calls out. Bottom lane, cycles
    2-3: SW1 OFF, a genuine local BF forms the complex-FFT pair, boxed
    twiddles 0, 4 (phi=4 is the trivial -j swap-and-negate)."""
    s3 = [0.0] * N

    # top lane: rows 0-3, plain real BF, 2 cycles then idle
    s3[0], s3[2] = bf(s2[0], s2[2])
    s3[1], s3[3] = bf(s2[1], s2[3])
    if verbose:
        print("  stage3 top    cyc0-1: BF(s2[0],s2[2]), BF(s2[1],s2[3])  -> real bins")

    # bottom lane, cycles 0-1: SW1 on, rows 4-7 (Eq.7 -> W^k/CSDM)
    for k in range(2):
        upper_re, upper_im = bf(s2[4 + k], -s2[6 + k], c=1)   # passthrough BF, Fig.4 c=1
        re_in, im_in = sw1(True, local=None, upper=(upper_re, upper_im))
        phi = 2 * k
        if phi == 2:
            s3[4 + k], s3[6 + k] = csdm(re_in, im_in)          # CSDM box, boxed "2"
        else:
            s3[4 + k], s3[6 + k] = rotator(re_in, im_in, phi)  # W^k box, boxed "0"
        if verbose:
            tag = " (CSDM slot)" if phi == 2 else ""
            print(f"  stage3 bottom cyc{k}: SW1 on, routes Stage-2 top lane's Eq(7) "
                  f"pair, phi={phi}{tag}")

    # bottom lane, cycles 2-3: SW1 off, rows 8-15 (CFFT butterfly)
    for k in range(2):
        sum_re, diff_re = bf(s2[8 + k], s2[10 + k])
        sum_im, diff_im = bf(s2[12 + k], s2[14 + k])
        s3[8 + k], s3[12 + k] = sum_re, sum_im
        re_in, im_in = sw1(False, local=(diff_re, diff_im), upper=None)
        s3[10 + k], s3[14 + k] = rotator(re_in, im_in, 4 * k)  # W^k box, boxes 0 4
        if verbose:
            print(f"  stage3 bottom cyc{k + 2}: SW1 off, local BF feeds W^k, phi={4 * k}")

    return s3


def stage4(s3, verbose=False):
    """Column 4 ("D+SW1, BF", final): every remaining twiddle is phi=0
    (identity), matching the diagram having no W^k/CSDM box in this last
    column -- just the D+SW1 realignment feeding a plain BF."""
    s4 = [0.0] * N

    s4[0], s4[1] = bf(s3[0], s3[1])              # -> X0, X8
    s4[2], s4[3] = s3[2], -s3[3]                  # phi=0: pure sign flip -> X4
    s4[4], s4[5] = bf(s3[4], s3[5])               # -> X2, X6
    s4[6], s4[7] = bf(s3[6], s3[7])
    s4[8], s4[9] = bf(s3[8], s3[9])               # -> X1, X7
    s4[12], s4[13] = bf(s3[12], s3[13])
    s4[10], s4[11] = bf(s3[10], s3[11])           # -> X5, X3
    s4[14], s4[15] = bf(s3[14], s3[15])

    if verbose:
        print("  stage4: BF(s3[0],s3[1]) -> X0,X8; sign-flip -> X4; "
              "3x BF -> X2/X6, X1/X7, X5/X3")

    return s4


def unscramble(s4):
    """Hardware writes bins out of order; some land at a 'virtual' bin
    above N/2 that gets conjugate-folded back onto the real, non-
    redundant range. Same mapping as rfft16_dif_flowgraph.py and
    2009architecture_N16.py."""
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


def rfft16_4parallel_architecture(x, verbose=False):
    """Run all 4 pipeline columns in order. x: list of 16 real samples."""
    assert len(x) == N
    s1 = stage1(x)
    s2 = stage2(s1)
    s3 = stage3(s2, verbose)
    s4 = stage4(s3, verbose)
    return unscramble(s4)


def main():
    x = [random.gauss(0, 1) for _ in range(N)]   # real input, x[0]..x[15]
    # x = list(range(1, N + 1))                    # <- swap in a fixed, hand-traceable input instead

    print("Cycle-by-cycle trace of Stage 3 and Stage 4:\n")
    X_arch = rfft16_4parallel_architecture(x, verbose=True)
    X_flow = rfft16_flowgraph(x)

    import numpy as np
    X_ref = np.fft.rfft(x)

    print("\n  k |   4-parallel architecture X_k        |          numpy fft(x)               |  abs err")
    print("----|---------------------------------------|--------------------------------------|----------")
    for k in range(N // 2 + 1):
        mine, ref = X_arch[k], X_ref[k]
        print(f"{k:3d} | {mine.real:9.5f} {mine.imag:+9.5f}j          "
              f"| {ref.real:9.5f} {ref.imag:+9.5f}j           "
              f"| {abs(mine - ref):.2e}")

    max_err_numpy = max(abs(X_arch[k] - X_ref[k]) for k in range(N // 2 + 1))
    max_err_flow = max(abs(X_arch[k] - X_flow[k]) for k in range(N // 2 + 1))
    print(f"\nmax abs error, architecture vs numpy.fft.rfft: {max_err_numpy:.3e}")
    print(f"max abs error, architecture vs flow graph:      {max_err_flow:.3e}  (should be exactly 0.0)")


if __name__ == "__main__":
    main()
