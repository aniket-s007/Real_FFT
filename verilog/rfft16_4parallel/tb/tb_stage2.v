`timescale 1ns/1ps

// tb_stage2.v -- self-checking testbench for stage2.v, chained after
// stage1.v (Columns 1+2 of the 4-parallel 16-point RFFT architecture).
//
// Same fixed 16-sample real test frame as tb_stage1.v, so the two
// testbenches are directly comparable. Wires stage1 into stage2 using
// the documented cross-lane mapping (see stage2.v's header comment):
//     stage1.s1_top_sum  -> stage2.s1_k
//     stage1.s1_bot_sum  -> stage2.s1_k4
//     stage1.s1_top_diff -> stage2.s1_k8
//     stage1.s1_bot_diff -> stage2.s1_k12
//
// Captures all 16 raw s2[] outputs into a testbench memory (python s2[]
// order) via the out_valid handshake -- decoupled from the exact
// cumulative pipeline latency (2 cycles: 1 through stage1, 1 through
// stage2), and self-checks against s2_exp[], a hardcoded literal copy of
// python's own stage2(stage1(NOMINAL_X)) output (see
// python/verify/2013architecture_N16_4parallel.py) -- not recomputed
// here, so there's no risk of this testbench's own float math (e.g.
// $cos/$sin) drifting from the golden model by even a sub-ULP amount.
// The DUT itself still uses the quantized ROM built into stage2.v.

module tb_stage2;

    parameter WIDTH = 8;
    localparam real SCALE = (1 << (WIDTH - 1));
    localparam IN_WIDTH = WIDTH + 1;   // stage1 output width

    reg clk, rst_n, in_valid;
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;

    wire s1_valid;
    wire signed [IN_WIDTH-1:0] s1_top_sum, s1_top_diff, s1_bot_sum, s1_bot_diff;

    stage1 #(.WIDTH(WIDTH)) u_stage1 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s1_valid),
        .s1_top_sum(s1_top_sum), .s1_top_diff(s1_top_diff),
        .s1_bot_sum(s1_bot_sum), .s1_bot_diff(s1_bot_diff)
    );

    wire s2_valid;
    wire signed [IN_WIDTH:0] s2_top_sum, s2_top_diff, s2_bot_re, s2_bot_im;

    stage2 #(.WIDTH(WIDTH)) u_stage2 (
        .clk(clk), .rst_n(rst_n), .in_valid(s1_valid),
        .s1_k(s1_top_sum), .s1_k4(s1_bot_sum),
        .s1_k8(s1_top_diff), .s1_k12(s1_bot_diff),
        .out_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im)
    );

    // debug probe: stage2's internal twiddle-select counter (rtl/stage2.v's
    // `k`), pulled up to this testbench's own scope via a hierarchical
    // reference so it shows up directly under tb_stage2 in the xsim
    // waveform viewer instead of having to drill into u_stage2's scope.
    wire [1:0] dbg_s2_k = u_stage2.k;

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as tb_stage1.v ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0]   x_mem  [0:15];   // quantized DUT input memory
    reg  signed [IN_WIDTH:0]  s2_mem [0:15];   // captured stage-2 output, python s2[] order
    real                      s2_exp [0:15];   // exact expected s2[], unquantized

    function signed [WIDTH-1:0] to_fixed;
        input real val;
        real scaled;
        begin
            scaled = val * SCALE;
            if (val >= 0.0)
                to_fixed = $rtoi(scaled + 0.5);
            else
                to_fixed = $rtoi(scaled - 0.5);
        end
    endfunction

    function real fixed_to_real;
        input signed [IN_WIDTH:0] val;
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // exact expected s2[] -- hardcoded, not recomputed in Verilog. This is
    // python's own stage2(stage1(NOMINAL_X)) output taken verbatim (Python
    // repr() precision: the shortest decimal that round-trips to the exact
    // same IEEE-754 double) from python/verify/2013architecture_N16_4parallel.py,
    // so this testbench compares the DUT against python's own numbers
    // instead of an independently re-derived Verilog computation --
    // $cos/$sin here isn't guaranteed bit-identical to python's
    // math.cos/sin, so the old inline recomputation could diverge from
    // the golden model by a sub-ULP amount even before quantization error
    // is considered.
    initial begin
        for (i = 0; i < 16; i = i + 1)
            x_mem[i] = to_fixed(x_real[i]);

        s2_exp[0]  = 0.20000000000000007;   s2_exp[1]  = 0.35000000000000003;
        s2_exp[2]  = 0.25;                  s2_exp[3]  = -0.2500000000000001;
        s2_exp[4]  = 0.29999999999999993;   s2_exp[5]  = 0.04999999999999999;
        s2_exp[6]  = -0.14999999999999994;  s2_exp[7]  = 0.35;
        s2_exp[8]  = -0.04999999999999999;  s2_exp[9]  = -0.5510448146666282;
        s2_exp[10] = 0.24748737341529164;   s2_exp[11] = 0.7609574450448195;
        s2_exp[12] = -1.35;                 s2_exp[13] = 0.4988482857833845;
        s2_exp[14] = -0.8131727983645298;   s2_exp[15] = 1.5599499244625963;
    end

    // ---- capture DUT (stage2) output into s2_mem, driven by s2_valid ----
    integer cap_idx;
    initial cap_idx = 0;
    always @(posedge clk) begin
        if (!rst_n)
            cap_idx <= 0;
        else if (s2_valid) begin
            s2_mem[cap_idx]    <= s2_top_sum;
            s2_mem[cap_idx+4]  <= s2_top_diff;
            s2_mem[cap_idx+8]  <= s2_bot_re;
            s2_mem[cap_idx+12] <= s2_bot_im;
            cap_idx <= cap_idx + 1;
        end
    end

    // ---- drive DUT, then check + report ----
    real max_err, tol, err;
    integer pass_count;
    initial begin
        rst_n = 0; in_valid = 0;
        x_k = 0; x_k_n4 = 0; x_k_n2 = 0; x_k_3n4 = 0;
        @(negedge clk);
        rst_n = 1;

        for (k = 0; k < 4; k = k + 1) begin
            @(negedge clk);
            in_valid = 1;
            x_k     = x_mem[k];
            x_k_n4  = x_mem[k+4];
            x_k_n2  = x_mem[k+8];
            x_k_3n4 = x_mem[k+12];
        end
        @(negedge clk);
        in_valid = 0;
        @(negedge clk);   // stage1's last output lands
        @(negedge clk);   // stage2's last output lands and gets captured

        // ---- report: input sequence, stage-2 output (DUT), and the
        // expected stage-2 value (computed the same way python's
        // stage2()/bf()/rotator() does), all in one table ----
        $display("\n idx |   x[idx]             |  stage2 output (DUT) |  stage2 expected  |  abs err  | pass?");
        $display("-----|-----------------------|----------------------|--------------------|-----------|------");
        max_err = 0.0;
        tol = 6.0 / SCALE;   // rotator's own rounding + propagated input quantization
        pass_count = 0;
        for (i = 0; i < 16; i = i + 1) begin
            err = fixed_to_real(s2_mem[i]) - s2_exp[i];
            if (err < 0.0) err = -err;
            if (err > max_err) max_err = err;
            if (err <= tol) pass_count = pass_count + 1;
            $display(" %3d   | %9.15f   |            %9.15f   |         %9.15f   |  %8.15f   | %s",
                      i, x_real[i], fixed_to_real(s2_mem[i]), s2_exp[i], err,
                      (err <= tol) ? "PASS" : "FAIL");
        end

        $display("\nmax abs error: %0.6f  (tolerance %0.6f, WIDTH=%0d)", max_err, tol, WIDTH);
        if (pass_count == 16)
            $display("STAGE2: ALL 16 SAMPLES PASS");
        else
            $display("STAGE2: %0d/16 PASS -- CHECK ABOVE", pass_count);

        $finish;
    end

endmodule
