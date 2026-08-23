"""
RFFT16_DIF_FLOWGRAPH -- 16-point real-input FFT, coded to LOOK like the
flow graph (Fig. 3, Garrido/Parhi/Grajal, "A Pipelined FFT Architecture
for Real-Valued Signals," IEEE TCAS-I, 2009).

Python twin of matlab/rfft16_dif_flowgraph.m -- same stage layout, same
indices (0-based here vs MATLAB's 1-based), same comments, so the two
files can be read side by side. Plain Python (lists, loops, math) --
numpy only shows up once, to fetch the reference answer we check against.

Each STAGE below is one vertical slice of that picture: a "top" block of
rows that stays real, a "bottom" block that gets Eq.(7)'d into a complex
signal, and -- wherever the figure draws a boxed number -- a twiddle()
rotation by that box's phi. Note every array here holds only real floats,
even the "complex" rows (re/im live in separate slots) -- that's the
figure's own caption: "All edges are real."

Run this file directly to compare against numpy.fft.rfft.
"""
import math
import random

N = 16


def twiddle(re_in, im_in, phi):
    """The boxed rotator in the figure: multiply (re_in + j*im_in) by
    exp(-j*2*pi*phi/N)."""
    angle = -2 * math.pi * phi / N
    c, s = math.cos(angle), math.sin(angle)
    re_out = re_in * c - im_in * s
    im_out = im_in * c + re_in * s
    return re_out, im_out


def rfft16_flowgraph(x):
    """Hand-mapped Fig. 3 flow graph. x is a list of 16 real samples.
    Returns X = [X_0 .. X_8] (the non-redundant rfft bins) and
    (s1, s2, s3, s4) = the raw contents of the RAM after each stage
    (still in hardware/scrambled order, exactly as drawn on the wires
    in the figure)."""
    assert len(x) == N

    # ---------------- STAGE 1 : plain real butterfly, rows (i, i+8) ----
    # No twiddle box anywhere in Stage 1 of the figure -> no rotation here.
    s1 = [0.0] * N
    for i in range(8):
        A, B = x[i], x[i + 8]
        s1[i] = A + B         # rows 0-7  -> feeds another real-FFT half
        s1[i + 8] = A - B     # rows 8-15 -> feeds the Eq.(7) box next stage

    # ---------------- STAGE 2 -------------------------------------------
    s2 = [0.0] * N

    # rows 0-7 (still real): plain butterfly again, still no rotation box
    for i in range(4):
        A, B = s1[i], s1[i + 4]
        s2[i] = A + B
        s2[i + 4] = A - B

    # rows 8-15: Eq.(7) box -- negate the bottom half, then rotate by the
    # four boxed numbers "0 1 2 3" drawn in Fig.3 Stage 2
    for k in range(4):
        A = s1[8 + k]
        B = -s1[12 + k]                  # Eq.(7) negation (the "-1" edges)
        r, im = twiddle(A, B, k)         # boxes: 0 1 2 3
        s2[8 + k] = r
        s2[12 + k] = im

    # ---------------- STAGE 3 --------------------------------------------
    s3 = [0.0] * N

    # rows 0-3 (real): plain butterfly, no rotation
    for i in range(2):
        A, B = s2[i], s2[i + 2]
        s3[i] = A + B
        s3[i + 2] = A - B

    # rows 4-7 (real): Eq.(7) box, boxed numbers "0 2"
    for k in range(2):
        A = s2[4 + k]
        B = -s2[6 + k]
        r, im = twiddle(A, B, 2 * k)
        s3[4 + k] = r
        s3[6 + k] = im

    # rows 8-15 (already complex: re = s2[8:12], im = s2[12:16]): a radix-2
    # complex-FFT butterfly -- the circled "Real .. comes together" block in
    # the figure -- with boxed numbers "0 4" (phi=4 is a trivial j-rotation,
    # which is exactly why the figure notes it "same as real": no multiplier
    # needed, just a swap-and-negate)
    for k in range(2):
        Ar, Br = s2[8 + k], s2[10 + k]
        Ai, Bi = s2[12 + k], s2[14 + k]
        s3[8 + k] = Ar + Br               # top half, real part
        s3[12 + k] = Ai + Bi              # top half, imag part
        r, im = twiddle(Ar - Br, Ai - Bi, 4 * k)
        s3[10 + k] = r                    # bottom half, real part
        s3[14 + k] = im                   # bottom half, imag part

    # ---------------- STAGE 4 (final) ---------------------------------------
    s4 = [0.0] * N

    # row 0-1 (real): plain butterfly -> final bins X_0 (DC), X_8 (Nyquist)
    s4[0] = s3[0] + s3[1]
    s4[1] = s3[0] - s3[1]

    # row 2-3 (real): Eq.(7) box, phi = 0 -> feeds bin X_4
    r, im = twiddle(s3[2], -s3[3], 0)
    s4[2], s4[3] = r, im

    # rows 4-7 (complex): CFFT butterfly, phi = 0 -> bins X_2 (direct), X_6 (folded)
    Ar, Br, Ai, Bi = s3[4], s3[5], s3[6], s3[7]
    s4[4], s4[6] = Ar + Br, Ai + Bi
    r, im = twiddle(Ar - Br, Ai - Bi, 0)
    s4[5], s4[7] = r, im

    # rows 8-11 (complex): CFFT butterfly, phi = 0 -> bins X_1 (direct), X_7 (folded)
    Ar, Br, Ai, Bi = s3[8], s3[9], s3[12], s3[13]
    s4[8], s4[12] = Ar + Br, Ai + Bi
    r, im = twiddle(Ar - Br, Ai - Bi, 0)
    s4[9], s4[13] = r, im

    # rows 12-15 (complex): CFFT butterfly, phi = 0 -> bins X_5 (direct), X_3 (folded)
    Ar, Br, Ai, Bi = s3[10], s3[11], s3[14], s3[15]
    s4[10], s4[14] = Ar + Br, Ai + Bi
    r, im = twiddle(Ar - Br, Ai - Bi, 0)
    s4[11], s4[15] = r, im

    # ---------------- OUTPUT UNSCRAMBLE ---------------------------------------
    # The hardware writes bins out of order, and some land at a "virtual" bin
    # number above N/2 that must be conjugate-folded back onto the real
    # non-redundant range: X_(N-b) = conj(raw value written at virtual bin b).
    #
    # (real-part RAM index, imag-part RAM index or None if purely real, virtual bin)
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

    return X, (s1, s2, s3, s4)


def main():
    x = [random.gauss(0, 1) for _ in range(N)]   # real input, x[0]..x[15]
    # x = list(range(1, N + 1))                   # <- swap in a fixed, hand-traceable input instead

    X, stages = rfft16_flowgraph(x)

    import numpy as np
    X_ref = np.fft.rfft(x)   # the "actual" FFT we're checking against

    print("  k |        our flow graph X_k          |           numpy fft(x)              |  abs err")
    print("----|-------------------------------------|--------------------------------------|----------")
    for k in range(N // 2 + 1):
        mine, ref = X[k], X_ref[k]
        print(f"{k:3d} | {mine.real:9.5f} {mine.imag:+9.5f}j          "
              f"| {ref.real:9.5f} {ref.imag:+9.5f}j           "
              f"| {abs(mine - ref):.2e}")
    max_err = max(abs(X[k] - X_ref[k]) for k in range(N // 2 + 1))
    print(f"\nmax abs error vs numpy.fft.rfft: {max_err:.3e}")

    # Uncomment to see the raw, still-scrambled RAM contents after each stage
    # (useful for tracing a single wire through the picture):
    # for name, arr in zip(("s1", "s2", "s3", "s4"), stages):
    #     print(name, arr)


if __name__ == "__main__":
    main()
