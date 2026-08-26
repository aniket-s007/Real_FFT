`timescale 1ns/1ps

// tb_stage1.v -- self-checking testbench for stage1.v (Column 1 of the
// 4-parallel 16-point RFFT architecture).
//
// What it does:
//   1) Declares one fixed 16-sample real-valued test frame (x_real) --
//      paste these 16 numbers into python's x list to cross-check against
//      2013architecture_N16_4parallel.stage1(x) / rfft16_4parallel_architecture(x).
//   2) Quantizes it to WIDTH-bit signed fixed-point (Q1.(WIDTH-1)) and
//      streams it into the DUT 4 samples/cycle over 4 cycles, exactly as
//      the architecture expects (x(k), x(k+N/4), x(k+N/2), x(k+3N/4)).
//   3) Captures all 16 raw stage-1 outputs into a testbench memory
//      (s1_mem), indexed in the SAME order as python's s1[] list, using
//      the DUT's out_valid handshake (so capture timing doesn't have to
//      hardcode the pipeline's 1-cycle latency).
//   4) Self-checks each captured value against an exact, unquantized
//      floating-point expected value computed the same way python's
//      bf()/stage1() does, with a tolerance of 1.5 LSB (Stage 1's add/sub
//      is exact arithmetic -- the only error possible here is up to 0.5
//      LSB from quantizing each of the two summed inputs, so true error
//      is bounded by 1.0 LSB; 1.5 leaves headroom).
//   5) $displays the input frame and a full s1[] comparison table so you
//      can also eyeball it against the python output directly.
//
// WIDTH is a parameter specifically so this can be re-run at different
// widths later for the SQNR sweep (mirrors python/bearing_fault_demo/SqnrSweep.py's
// purpose, applied to this RTL instead).

module tb_stage1;

    parameter WIDTH = 16;
    localparam real SCALE = (1 << (WIDTH - 1));  // Q1.(WIDTH-1) fixed-point scale

    reg clk, rst_n, in_valid;
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;

    wire out_valid;
    wire signed [WIDTH:0] s1_top_sum, s1_top_diff, s1_bot_sum, s1_bot_diff;

    stage1 #(.WIDTH(WIDTH)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(out_valid),
        .s1_top_sum(s1_top_sum), .s1_top_diff(s1_top_diff),
        .s1_bot_sum(s1_bot_sum), .s1_bot_diff(s1_bot_diff)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- fixed 16-sample real test frame; paste into python as x[0..15] ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0] x_mem  [0:15];   // quantized DUT input memory
    reg  signed [WIDTH:0]   s1_mem [0:15];   // captured DUT output memory, python s1[] order
    real                    s1_exp [0:15];   // exact expected s1[], unquantized
    real                    xq;

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
        input signed [WIDTH:0] val;
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // ---- quantize inputs + compute exact expected output, both up front ----
    initial begin
        for (i = 0; i < 16; i = i + 1)
            x_mem[i] = to_fixed(x_real[i]);
        for (k = 0; k < 4; k = k + 1) begin
            s1_exp[k]    = x_real[k]   + x_real[k+8];
            s1_exp[k+8]  = x_real[k]   - x_real[k+8];
            s1_exp[k+4]  = x_real[k+4] + x_real[k+12];
            s1_exp[k+12] = x_real[k+4] - x_real[k+12];
        end
    end

    // ---- capture DUT output into s1_mem, driven by out_valid, not by a
    //      hardcoded latency count ----
    integer cap_idx;
    initial cap_idx = 0;
    always @(posedge clk) begin
        if (!rst_n)
            cap_idx <= 0;
        else if (out_valid) begin
            s1_mem[cap_idx]    <= s1_top_sum;
            s1_mem[cap_idx+8]  <= s1_top_diff;
            s1_mem[cap_idx+4]  <= s1_bot_sum;
            s1_mem[cap_idx+12] <= s1_bot_diff;
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
        @(negedge clk);   // let the last registered output land and get captured

        // ---- report ----
        $display("WIDTH = %0d, scale = 2^%0d = %0f", WIDTH, WIDTH-1, SCALE);

        $display("\ninput frame x[0..15] (paste into python's x list):");
        for (i = 0; i < 16; i = i + 1) begin
            xq = fixed_to_real({x_mem[i][WIDTH-1], x_mem[i]});
            $display("  x[%0d] = %8.5f  ->  fixed 0x%h = %8.5f", i, x_real[i], x_mem[i], xq);
        end

        $display("\n idx |  s1 (fixed -> real)  |  s1 expected (float)  |  abs err  | pass?");
        $display("-----|-----------------------|------------------------|-----------|------");
        max_err = 0.0;
        tol = 1.5 / SCALE;
        pass_count = 0;
        for (i = 0; i < 16; i = i + 1) begin
            err = fixed_to_real(s1_mem[i]) - s1_exp[i];
            if (err < 0.0) err = -err;
            if (err > max_err) max_err = err;
            if (err <= tol) pass_count = pass_count + 1;
            $display(" %3d |  0x%h -> %8.5f | %8.5f               | %8.5f  | %s",
                      i, s1_mem[i], fixed_to_real(s1_mem[i]), s1_exp[i], err,
                      (err <= tol) ? "PASS" : "FAIL");
        end

        $display("\nmax abs error: %0.6f  (tolerance %0.6f = 1.5 LSB at WIDTH=%0d)",
                  max_err, tol, WIDTH);
        if (pass_count == 16)
            $display("STAGE1: ALL 16 SAMPLES PASS");
        else
            $display("STAGE1: %0d/16 PASS -- CHECK ABOVE", pass_count);

        $finish;
    end

endmodule
