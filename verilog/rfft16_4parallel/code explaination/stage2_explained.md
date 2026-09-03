# Stage 2 (`rtl/stage2.v`) — Line-by-Line Explanation

Stage 2 is Column 2 ("BF / W^k") of the 4-parallel 16-point RFFT
architecture. It is the exact hardware twin of `stage2()` in
`python/2013architecture_N16_4parallel.py`:

```python
def stage2(s1):
    s2 = [0.0] * N
    for k in range(4):
        s2[k], s2[k+4] = bf(s1[k], s1[k+4])       # top BF box
        A = s1[8+k]
        B = -s1[12+k]                              # Eq.(7) sign flip
        s2[8+k], s2[12+k] = rotator(A, B, k)       # W^k box, boxes 0 1 2 3
    return s2
```

This file walks through `stage2.v` piece by piece: what each line does,
then **why** it's written that way.

---

## 1. Module header and ports (lines 65–83)

```verilog
module stage2 #(
    parameter WIDTH    = 32,
    parameter IN_WIDTH = WIDTH + 1
) (
```
- `WIDTH` — the *original* top-level sample width. This is the number
  fixed everywhere for the fractional scale (Q1.(WIDTH-1)).
- `IN_WIDTH = WIDTH + 1` — Stage 1 grew every value by 1 bit (its BF
  boxes are exact adds/subs, so no bit is dropped). Stage 2's inputs
  arrive at this wider width, not the original `WIDTH`.

```verilog
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,

    input  wire signed [IN_WIDTH-1:0] s1_k,     // s1[k]    (stage1.s1_top_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k4,    // s1[k+4]  (stage1.s1_bot_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k8,    // s1[k+8]  (stage1.s1_top_diff)
    input  wire signed [IN_WIDTH-1:0] s1_k12,   // s1[k+12] (stage1.s1_bot_diff)
```
- Standard clock/reset/valid handshake, same pattern as Stage 1.
- The four data inputs are named after the **Python index** they carry
  (`s1_k` = `s1[k]`), not "top"/"bottom" — because, as the header
  comment in the file explains, Stage 2 crosses Stage 1's two physical
  lanes (see §5 below on why).

```verilog
    output reg                        out_valid,
    output reg  signed [IN_WIDTH:0]   s2_top_sum,   // -> s2[k]
    output reg  signed [IN_WIDTH:0]   s2_top_diff,  // -> s2[k+4]
    output reg  signed [IN_WIDTH:0]   s2_bot_re,    // -> s2[k+8]
    output reg  signed [IN_WIDTH:0]   s2_bot_im     // -> s2[k+12]
);
```
- Four outputs, each `IN_WIDTH+1` bits (one bit wider again — both the
  BF box and the rotator box grow their output by 1 bit).

---

## 2. Twiddle ROM constants (lines 85–96)

```verilog
localparam MASTER_WIDTH = 40;
localparam signed [MASTER_WIDTH-1:0] COS0_M = 40'sd549755813887, SIN0_M = 40'sd0;
localparam signed [MASTER_WIDTH-1:0] COS1_M = 40'sd507908144330, SIN1_M = -40'sd210382441821;
localparam signed [MASTER_WIDTH-1:0] COS2_M = 40'sd388736063997, SIN2_M = -40'sd388736063997;
localparam signed [MASTER_WIDTH-1:0] COS3_M = 40'sd210382441821, SIN3_M = -40'sd507908144330;
```
- `MASTER_WIDTH = 40` — a fixed, very high precision (much higher than
  any `WIDTH` we'll actually sweep).
- `COSk_M` / `SINk_M` — `cos(-2*pi*k/16)` and `sin(-2*pi*k/16)` for
  `k = 0..3`, pre-computed **offline** (in Python, by
  `python/gen_twiddle_master_4parallel.py`) at `MASTER_WIDTH`-bit
  fixed-point precision and hardcoded here as plain signed integer
  literals — no `real`, no `$cos`/`$sin` anywhere in this file.

---

## 3. Rescaling the ROM down to the current `WIDTH` (lines 98–131)

```verilog
localparam SHIFT = MASTER_WIDTH - WIDTH;
```
- How many bits to shave off to go from 40-bit master precision down
  to whatever `WIDTH` this build is using.

```verilog
function signed [WIDTH-1:0] derive_coef;
    input signed [MASTER_WIDTH-1:0] raw;
    reg signed [MASTER_WIDTH:0] rounded, shifted;
    reg signed [MASTER_WIDTH:0] max_out, min_out;
    begin
        rounded = raw + (1 <<< (SHIFT-1));   // add half an LSB (round-to-nearest)
        shifted = rounded >>> SHIFT;         // arithmetic right-shift = rescale
        max_out = (1 <<< (WIDTH-1)) - 1;     // largest representable Q1.(WIDTH-1) value
        min_out = -(1 <<< (WIDTH-1));        // smallest representable value
        if (shifted > max_out)
            derive_coef = max_out[WIDTH-1:0];   // clamp high
        else if (shifted < min_out)
            derive_coef = min_out[WIDTH-1:0];   // clamp low
        else
            derive_coef = shifted[WIDTH-1:0];   // normal case
    end
endfunction
```
- One line each: round, shift, compute the two saturation bounds, then
  pick clamp-high / clamp-low / pass-through. This is a *function*, so
  it's pure combinational logic evaluated at elaboration time (all its
  call sites below use only `localparam` — i.e. constant — arguments).

```verilog
localparam signed [WIDTH-1:0] COS0 = derive_coef(COS0_M);
localparam signed [WIDTH-1:0] SIN0 = derive_coef(SIN0_M);
localparam signed [WIDTH-1:0] COS1 = derive_coef(COS1_M);
localparam signed [WIDTH-1:0] SIN1 = derive_coef(SIN1_M);
localparam signed [WIDTH-1:0] COS2 = derive_coef(COS2_M);
localparam signed [WIDTH-1:0] SIN2 = derive_coef(SIN2_M);
localparam signed [WIDTH-1:0] COS3 = derive_coef(COS3_M);
localparam signed [WIDTH-1:0] SIN3 = derive_coef(SIN3_M);
```
- Eight lines, one per coefficient: rescale each of the 4 twiddle
  entries' cos and sin from 40-bit master precision down to the
  build's actual `WIDTH`. These are what the ROM actually hands out.

---

## 4. Per-cycle twiddle selection (lines 134–150)

```verilog
reg [1:0] k;
always @(posedge clk) begin
    if (!rst_n)
        k <= 2'd0;                              // reset: start at k=0
    else if (in_valid)
        k <= (k == 2'd3) ? 2'd0 : k + 2'd1;      // advance 0->1->2->3->0...
end
```
- A free-running 2-bit counter, `k = 0..3`, that increments **only**
  on cycles where `in_valid` is high, and wraps back to 0 after 3.

```verilog
reg signed [WIDTH-1:0] cos_coef, sin_coef;
always @(*) begin
    case (k)
        2'd0: begin cos_coef = COS0; sin_coef = SIN0; end
        2'd1: begin cos_coef = COS1; sin_coef = SIN1; end
        2'd2: begin cos_coef = COS2; sin_coef = SIN2; end
        default: begin cos_coef = COS3; sin_coef = SIN3; end
    endcase
end
```
- Combinational 4-way mux: whatever `k` is *this* cycle picks which of
  the 4 precomputed `(cos, sin)` pairs feeds the rotator this cycle.

---

## 5. The two datapath lanes (lines 152–166)

```verilog
wire signed [IN_WIDTH:0] top_sum_c, top_diff_c;
real_bf #(.WIDTH(IN_WIDTH)) bf_top (
    .in1(s1_k), .in2(s1_k4),
    .out_sum(top_sum_c), .out_diff(top_diff_c)
);
```
- Top lane: reuse the same `real_bf` block from Stage 1 (a plain
  add/sub butterfly, no multiply). Feeds it `s1[k]` and `s1[k+4]`.

```verilog
wire signed [IN_WIDTH-1:0] eq7_b = -s1_k12;
wire signed [IN_WIDTH:0] bot_re_c, bot_im_c;
rotator #(.IN_WIDTH(IN_WIDTH), .COEF_WIDTH(WIDTH)) wk (
    .re_in(s1_k8), .im_in(eq7_b),
    .cos_coef(cos_coef), .sin_coef(sin_coef),
    .re_out(bot_re_c), .im_out(bot_im_c)
);
```
- `eq7_b` — a free wire negate implementing the paper's Eq.(7) sign
  flip (`B = -s1[k+12]`); costs no logic, just inverted routing.
- Bottom lane: the shared `rotator` module (the one real multiplier in
  this stage), fed `s1[k+8]` as the real part and the negated
  `s1[k+12]` as the imaginary part, rotated by whichever `(cos_coef,
  sin_coef)` the `k` mux selected this cycle.

---

## 6. Registered outputs (lines 168–182)

```verilog
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
```
- On reset: everything (including `out_valid`) forced to 0/false.
- Every other cycle: the two lanes' combinational results
  (`top_sum_c`, `top_diff_c`, `bot_re_c`, `bot_im_c`) get latched into
  the output registers, and `out_valid` is simply `in_valid` delayed
  by one cycle — this is what makes Stage 2 a 1-cycle-latency
  pipeline stage, and what lets a downstream module (or the
  testbench) know exactly which cycle carries valid data without
  hardcoding any latency number.

---

## Companion module used here: `rotator.v` (quick reference)

Stage 2's bottom lane instantiates `rotator.v`, the general complex
twiddle multiplier:
```
re_out = re_in*cos - im_in*sin
im_out = im_in*cos + re_in*sin
```
It forms the 4 cross products, combines them, then rounds (adds half
an LSB), arithmetic-shifts right by `COEF_WIDTH-1` to bring the result
back to the input's fractional scale, and saturates to `IN_WIDTH+1`
bits. This is the **first point of real quantization error** in the
whole pipeline (Stage 1's adds are exact; a multiply is not) — see
`rotator.v`'s own header comment for the full derivation. It isn't
re-explained line-by-line here since the ask was specifically about
`stage2.v`; ask for a `rotator_explained.md` if you want the same
treatment for it.

---

# How this was built, and why (the logic behind the code)

**1. Why the inputs are named `s1_k`/`s1_k4`/`s1_k8`/`s1_k12` instead
of "top"/"bottom":**
Looking at the flow graph, Stage 2's top BF box combines the two
*sum* outputs from Stage 1 (one from Stage 1's top physical lane, one
from its bottom physical lane), while its bottom `W^k` box combines
the two *diff* outputs. So Stage 2 does **not** just chain each of
Stage 1's two lanes straight through — the wiring crosses between
lanes. Naming the ports by which `s1[]` value they must carry (rather
than by "top"/"bottom") makes this crossing explicit and impossible to
wire wrong by accident; the required connection is spelled out in the
module's own header comment:
```
stage1.s1_top_sum  -> stage2.s1_k
stage1.s1_bot_sum  -> stage2.s1_k4
stage1.s1_top_diff -> stage2.s1_k8
stage1.s1_bot_diff -> stage2.s1_k12
```

**2. Why the twiddle ROM is precomputed offline instead of using
`$cos`/`$sin` directly in this file (like an earlier version of this
file did):**
`real` variables and `$cos`/`$sin` are simulation-only constructs —
they are outside Vivado's synthesizable subset (per UG901). Every
other file in this pipeline can get away with "just simulate it," but
the ROM in this file specifically cannot, if the design is ever meant
to go through `synth_design`. So the trigonometry is done exactly
once, offline, in `python/gen_twiddle_master_4parallel.py`, at a fixed
high precision (`MASTER_WIDTH` = 40 bits), and the results are baked
in here as plain signed integer literals. Nothing in `stage2.v` itself
needs `real` math anymore.

**3. Why bother rescaling from a 40-bit "master" constant instead of
just hardcoding 4 pairs of `WIDTH`-bit constants directly:**
The whole point of parametrizing `WIDTH` everywhere in this project is
to later run an SQNR sweep — build the same design at many different
bit-widths and compare quantization error. If the twiddle constants
were hardcoded *per width*, sweeping `WIDTH` would mean manually
regenerating and pasting in new constants every time. Instead,
`derive_coef()` rescales the single set of 40-bit master constants
down to whatever `WIDTH` is currently set, using an ordinary
round + arithmetic-shift + saturate — the exact same rescale technique
`rotator.v` already applies to its own post-multiply result at
runtime, just run here on constants at *elaboration* time instead.
This was verified bit-exact (0 LSB difference) against the old
per-width `$cos`/`$sin`/`$rtoi` values for `WIDTH = 4..24` by the
generator script before being trusted enough to hardcode.

**4. Why `derive_coef()` needs a saturation clamp at all:**
`cos(0) = 1.0` exactly, but the maximum value representable in signed
Q1.(WIDTH-1) fixed point is `1 - 2^-(WIDTH-1)` — `+1.0` itself doesn't
fit. Naively rounding `1.0 * 2^(WIDTH-1)` gives exactly `2^(WIDTH-1)`,
which overflows a `WIDTH`-bit signed value and silently **wraps
around to the most-negative number** in two's complement (i.e. it
would flip the sign of that whole twiddle coefficient). This exact bug
was caught earlier by testing `rotator.v` standalone before it was
ever wired into this file — see that module's header comment. The
clamp in `derive_coef()` handles it the same way for every `WIDTH`
instead of leaving it to silently wrap.

**5. Why the `k` counter is self-generated inside this module instead
of being passed in from outside:**
Stage 2 needs to know, every cycle, which of the 4 twiddle factors
(`W^0..W^3`) applies to the sample currently passing through the
bottom lane. Rather than requiring some external controller to hand it
a `k` value, the module derives it itself purely from the `in_valid`
handshake pulses it already receives — it increments on every valid
input cycle and wraps mod 4. This keeps the module self-contained: it
works correctly whether it's driven by a full top-level controller
later or, as it is today, directly by a testbench feeding 4 valid
input groups.

**6. Why the top lane is a plain `real_bf` but the bottom lane needs a
whole `rotator`:**
This directly mirrors the flow graph: the top box at this column is a
plain real add/sub (no multiply needed, so the same cheap `real_bf`
block from Stage 1 is reused as-is). The bottom box is the one
genuine multiplier column in this stage — it must serve all four
twiddles `W^0..W^3` across a 4-cycle frame using a single shared
physical unit (matching the block diagram's one "W^k" box on the
bottom lane, not four separate multiplier instances).

**7. Why `eq7_b` is just a wire negate (`-s1_k12`), not a subtractor
block:**
The paper's Eq.(7) requires feeding the rotator `B = -s1[k+12]` as its
imaginary input. Negation of a two's-complement signed value is free
in hardware terms when done as a continuous assignment — synthesis
turns `-s1_k12` into essentially routing plus (at most) a small
add-one, not a separate arithmetic block — so it's expressed directly
as a `wire` assignment rather than instantiating any extra module.

**8. Why every output is registered (1-cycle latency) rather than
combinational passthrough:**
Every stage in this pipeline (Stage 1 included) registers its outputs
exactly once. Keeping a uniform "1 clock cycle per stage" latency
convention throughout the whole 4-stage architecture makes the overall
pipeline latency simple to reason about and keeps `out_valid` tracking
correct and composable — each stage just delays `in_valid` by exactly
its own latency, so chaining stages (as `tb_stage2.v` already does
with Stage 1 + Stage 2) never requires hand-counting cycles anywhere.

**9. Why `out_valid <= in_valid` instead of a hardcoded delay
counter:**
Using the handshake signal itself (rather than counting clock edges)
makes the valid tracking robust to however many pipeline stages are
chained in front of or behind this one. A testbench (or a future
top-level module chaining all 4 stages) can always just watch
`out_valid` to know exactly which cycle's output is meaningful,
regardless of the total cumulative latency — this was already the
established pattern from Stage 1, kept consistent here.
