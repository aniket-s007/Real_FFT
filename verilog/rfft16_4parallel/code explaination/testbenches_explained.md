# Testbenches (`tb_stage1.v`, `tb_stage2.v`) — How They Work, and the Redundant-Code Cleanup

This covers the two self-checking testbenches for the 4-parallel 16-point
RFFT architecture (Salehi/Amirfattahi/Parhi 2013): `tb/tb_stage1.v` (checks
`rtl/stage1.v` alone) and `tb/tb_stage2.v` (checks `rtl/stage1.v` chained
into `rtl/stage2.v`). Both follow the same golden-reference methodology; the
second half of this file documents what redundant code was removed from
each and why.

---

## 1. The shared methodology

Both testbenches do the same five things, just at a different pipeline
depth:

1. **Declare one fixed 16-sample real-valued test frame**, `x_real[0:15]`
   — the same 16 numbers in both files, so results are directly
   comparable between them and reproducible run-to-run (no randomness).
2. **Quantize** that frame to `WIDTH`-bit signed Q1.(WIDTH-1) fixed point
   via `to_fixed()`, and stream it into the DUT 4 samples/cycle over 4
   cycles — exactly how the real 4-parallel architecture expects inputs:
   `x(k), x(k+N/4), x(k+N/2), x(k+3N/4)` for `k = 0..3`.
3. **Capture** the DUT's output into a testbench memory, indexed in the
   same order as the Python model's list (`s1[]` or `s2[]`), driven by
   the DUT's own `out_valid`/`s2_valid` handshake rather than a
   hardcoded cycle-count — so capture timing survives if a stage's
   pipeline latency ever changes.
4. **Self-check** each captured (and dequantized, via `fixed_to_real()`)
   value against an *exact, unquantized* expected value, within a
   tolerance sized for that stage's own arithmetic error budget.
5. **Report** a full per-sample PASS/FAIL table plus a max-error summary.

The key design decision covered in this doc is **where step 4's "exact
expected value" comes from**. Both files now source it the same way:
hardcoded literal numbers taken verbatim from running the actual Python
functions in `python/verify/2013architecture_N16_4parallel.py` — not
recomputed independently in Verilog. See §4 for why that matters.

---

## 2. `tb_stage1.v` walkthrough

Checks `stage1.v` (Column 1, "BF, BF") on its own. `WIDTH = 16` here.

- **DUT wiring** (lines 31–48): one `stage1` instance, `dut`, fed
  `x_k`/`x_k_n4`/`x_k_n2`/`x_k_3n4` each cycle.
- **Test frame** (lines 53–60): `x_real[0..15]`.
- **Quantization helpers** (lines 67–84): `to_fixed()` (real → signed
  fixed-point, round-half-away-from-zero) and `fixed_to_real()` (signed
  fixed-point → real, for reporting/checking).
- **Golden reference, `s1_exp[]`** (lines 88–109): 16 hardcoded literals,
  Python's own `stage1(NOMINAL_X)` output, indexed exactly like the
  Python list (`s1_exp[k]`/`s1_exp[k+4]`/`s1_exp[k+8]`/`s1_exp[k+12]`
  match `s1[k]`/`s1[k+4]`/`s1[k+8]`/`s1[k+12]`).
- **Capture** (lines 111–125): `always @(posedge clk)` latches
  `s1_top_sum`/`s1_top_diff`/`s1_bot_sum`/`s1_bot_diff` into `s1_mem[]`
  whenever `out_valid` is high, at the right offsets (`cap_idx`,
  `cap_idx+4`, `cap_idx+8`, `cap_idx+12`).
- **Drive + check** (lines 127–180): reset, stream in 4 cycles of
  quantized samples, wait one more cycle for the last registered output
  to land, then loop over all 16 captured samples comparing
  `fixed_to_real(s1_mem[i])` against `s1_exp[i]` with
  `tol = 1.5/SCALE` (1.5 LSB — Stage 1 is exact add/sub, so the only
  error possible is up to 0.5 LSB from quantizing each of the two summed
  inputs, giving a true bound of 1.0 LSB; 1.5 leaves headroom).

## 3. `tb_stage2.v` walkthrough

Checks `stage1.v` **chained into** `stage2.v` (Columns 1+2 together —
Stage 2 can't be checked standalone since its inputs are Stage 1's
outputs). `WIDTH = 8` here (smaller than `tb_stage1.v`'s 16 — an existing
asymmetry between the two files, not something this cleanup touched).

- **DUT wiring** (lines 26–54): `u_stage1` feeds `u_stage2` using the
  documented cross-lane mapping (Stage 2's top BF box combines Stage 1's
  two *sum* outputs; its bottom `W^k` box combines the two *diff*
  outputs — the two physical lanes cross between stages, which is why
  the header comment spells out `s1_top_sum -> s1_k`, `s1_bot_sum ->
  s1_k4`, `s1_top_diff -> s1_k8`, `s1_bot_diff -> s1_k12`).
- **Test frame / quantization helpers**: same shape as `tb_stage1.v`
  (lines 60–89), just re-declared in this file (see §5 on this
  duplication).
- **Golden reference, `s2_exp[]`** (lines 93–115): 16 hardcoded literals,
  Python's own `stage2(stage1(NOMINAL_X))` output.
- **Capture** (lines 117–130): latches `s2_top_sum`/`s2_top_diff`/
  `s2_bot_re`/`s2_bot_im` into `s2_mem[]` on `s2_valid`.
- **Drive + check** (lines 132–179): same shape as `tb_stage1.v`, but
  waits 2 extra cycles before checking (1 cycle through `stage1`, 1
  through `stage2`), and uses `tol = 6/SCALE` (6 LSB — looser than
  Stage 1's 1.5, because the rotator's own multiply-and-round
  quantization error adds on top of the propagated input-quantization
  error from Stage 1).

---

## 4. What was removed, and why

### `tb_stage2.v`: dropped `$cos`/`$sin` re-derivation of the golden values

**Before**: `s1_exp[]` was computed inline from `x_real[]` via plain
add/sub, then `s2_exp[]` was computed inline from `s1_exp[]` using
`$cos`/`$sin` calls on `angle = -2*PI*k/16`.

**Problem**: this is a *second, independent* implementation of
`stage2()`'s math, written in Verilog instead of Python. `$cos`/`$sin`
are not guaranteed to be bit-identical to Python's `math.cos`/`math.sin`
(different libm implementations aren't required to round transcendental
functions identically) — so the "expected" value the DUT was being
checked against could silently drift from the actual Python golden model
by a sub-ULP amount, for no good reason. (In practice we also found,
while debugging a related display-precision question, that Vivado's
`real` decimal-literal parser itself isn't correctly-rounding for
17-significant-digit literals — see §4's second entry — which made
relying on *any* from-scratch floating-point computation inside this
testbench worth being suspicious of.)

**Fix**: `s2_exp[]` is now 16 hardcoded literals (lines 107–114), copied
verbatim from actually running `stage2(stage1(NOMINAL_X))` in
`python/verify/2013architecture_N16_4parallel.py` and printed with
Python's `repr()` (the shortest decimal string guaranteed to round-trip
to the exact same IEEE-754 double). The DUT is now compared directly
against Python's own numbers, with zero independent re-derivation.

**Removed as a consequence** (no longer needed once `s2_exp[]` stopped
being *derived from* `s1_exp[]`):
- `localparam real PI = 3.14159265358979323846;`
- `real angle, c, s, A, B;` (scratch variables used only by the old
  per-`k` `$cos`/`$sin` loop)
- The entire `real s1_exp [0:15];` array and its assignments. Once
  `s2_exp[]` was hardcoded directly, `s1_exp[]` had nothing left to
  compute — it was declared and populated but **never read again
  anywhere in the file**. This was the literal dead code removed in this
  pass: `tb_stage2.v` doesn't check Stage 1's intermediate output at all
  (only the final `s2_mem[]` vs `s2_exp[]` comparison exists), so keeping
  a whole 16-entry golden array around with no reader was pure waste.

### `tb_stage1.v`: replaced the inline formula with the same hardcoded pattern

**Before**: `s1_exp[]` was computed inline —
```verilog
s1_exp[k]    = x_real[k]   + x_real[k+8];
s1_exp[k+8]  = x_real[k]   - x_real[k+8];
s1_exp[k+4]  = x_real[k+4] + x_real[k+12];
s1_exp[k+12] = x_real[k+4] - x_real[k+12];
```

**Why this one's different from the `tb_stage2.v` case**: this is plain
IEEE-754 add/sub, which *is* deterministic across conformant
implementations — Verilog's `+`/`-` on `real` and Python's `+`/`-` on
`float` were already guaranteed to agree bit-for-bit here. There was no
correctness bug being fixed.

**Why it was changed anyway**: it's still two independent copies of the
same logic (`stage1()`'s math) living in two languages, which is exactly
the kind of duplication the `tb_stage2.v` cleanup was removing — and
leaving it inconsistent between the two files (one hardcoded from
Python, one recomputed in Verilog) would be a confusing asymmetry with
no upside. `s1_exp[]` is now 16 hardcoded literals (lines 101–108),
Python's own `stage1(NOMINAL_X)` output, matching `tb_stage2.v`'s
pattern exactly. Functionally a no-op (same numbers, same PASS/FAIL
result); purely a consistency/maintainability change.

---

## 5. What was *not* changed (known remaining duplication)

`x_real[0:15]`, `to_fixed()`, and `fixed_to_real()` are declared
separately in both files, essentially identically (module-scoped Verilog
functions can't be shared across files without introducing a
`` `include`` header and touching the `Makefile`'s source lists). This
cross-file duplication wasn't part of this cleanup — everything removed
above was either genuinely dead code or a redundant *recomputation* of
values Python already produces, not boilerplate shared between files. If
this duplication becomes annoying (e.g. once Stage 3/4 testbenches exist
too), factoring the shared frame + helper functions into a
`` `include``-d common file is the natural next step.

Re-verified after every change in this pass: both testbenches still
build clean (`make STAGE=1`, `make STAGE=2`) and report **ALL 16 SAMPLES
PASS**.
