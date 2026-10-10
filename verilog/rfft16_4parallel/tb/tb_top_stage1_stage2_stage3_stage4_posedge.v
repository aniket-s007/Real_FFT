`timescale 1ns/1ps

// tb_top_stage1_stage2_stage3_stage4_posedge.v -- full-pipeline testbench
// with the inputs driven on the POSITIVE clock edge.
//
// Same DUT, same 16-sample frame, same s4_exp[] golden values, same capture
// and same PASS/FAIL report as tb_top_stage1_stage2_stage3_stage4.v (which
// drives on the negedge). Only the stimulus differs: every input change is a
// nonblocking assignment made right after @(posedge clk), so the DUT always
// samples the value that was set on the previous edge.
//
// Because an input set on one posedge is first sampled on the NEXT posedge,
// the whole output comes out one clock later than in the negedge bench.

module tb_top_stage1_stage2_stage3_stage4_posedge;

    parameter WIDTH = 16;
    parameter TWIDDLE_WIDTH = 14;   // twiddle coefficient bit-width, independent of WIDTH
    localparam real SCALE = (1 << (WIDTH - 1));
    localparam real DEATTENUATE = 16.0;   // undoes the pipeline's deliberate 1/16 gain

    reg clk, rst_n, in_valid;
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;
    reg stim_done;   // set once the stimulus and its drain tail are finished
    initial stim_done = 0;

    wire s4_valid;
    wire signed [WIDTH-1:0] s4_p0, s4_p1, s4_p2, s4_p3;

    top_stage1_stage2_stage3_stage4 #(.WIDTH(WIDTH), .TWIDDLE_WIDTH(TWIDDLE_WIDTH)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s4_valid),
        .s4_p0(s4_p0), .s4_p1(s4_p1), .s4_p2(s4_p2), .s4_p3(s4_p3)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as every other testbench in this project ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0] x_mem  [0:15];
    reg  signed [WIDTH-1:0] s4_mem [0:15];
    real                    s4_exp [0:15];

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
        input signed [WIDTH-1:0] val;
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // exact expected s4[] -- python's own unquantized
    // stage4(stage3(stage2(stage1(NOMINAL_X)))), full double precision.
    initial begin
        for (i = 0; i < 16; i = i + 1)
            x_mem[i] = to_fixed(x_real[i]);

        s4_exp[0]  = 0.55;                    s4_exp[1]  = 0.35000000000000014;
        s4_exp[2]  = -0.04999999999999993;    s4_exp[3]  = -0.6000000000000001;
        s4_exp[4]  = 0.08786796564403565;     s4_exp[5]  = 0.5121320343559642;
        s4_exp[6]  = -0.13284271247461907;    s4_exp[7]  = 0.4328427124746189;
        s4_exp[8]  = 0.40740000379348296;     s4_exp[9]  = -0.012425256962899645;
        s4_exp[10] = -1.3585890120945034;     s4_exp[11] = 0.7636142652639202;
        s4_exp[12] = -0.10437458811854894;    s4_exp[13] = -4.221971008610511;
        s4_exp[14] = 0.7751750580759775;      s4_exp[15] = -1.848829461346918;
    end

    // ---- stimulus: every input changes on the POSITIVE edge (nonblocking) ----
    initial begin
        rst_n = 0; in_valid = 0;
        x_k = 0; x_k_n4 = 0; x_k_n2 = 0; x_k_3n4 = 0;
        @(posedge clk);
        @(posedge clk);
        rst_n <= 1;

        for (k = 0; k < 4; k = k + 1) begin
            @(posedge clk);
            in_valid <= 1;
            x_k     <= x_mem[k];
            x_k_n4  <= x_mem[k+4];
            x_k_n2  <= x_mem[k+8];
            x_k_3n4 <= x_mem[k+12];
        end
        @(posedge clk);
        in_valid <= 0;
        x_k      <= 0; x_k_n4 <= 0; x_k_n2 <= 0; x_k_3n4 <= 0;

        // same tail margin as the negedge bench: 14 clocks after in_valid drops
        repeat (14) @(posedge clk);
        stim_done = 1;
    end

    // ---- clock counter, input/output timing report ----
    // Runs on posedge and reads in_valid/s4_valid before the NBA updates of the
    // same edge, i.e. the values the DUT itself samples.
    integer cyc, first_in_cyc, first_out_cyc;
    initial begin
        cyc = 0; first_in_cyc = -1; first_out_cyc = -1;
    end
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (in_valid && first_in_cyc < 0)   first_in_cyc  = cyc;
        if (s4_valid && first_out_cyc < 0)  first_out_cyc = cyc;
    end

    // ---- capture DUT (top-level) output into s4_mem, same port->index
    // mapping as tb_stage4.v (arrival order pos=1,2,3,0) ----
    integer cap_idx;
    initial cap_idx = 0;
    always @(posedge clk) begin
        if (!rst_n)
            cap_idx <= 0;
        else if (s4_valid) begin
            case (cap_idx)
                0: begin s4_mem[0]  <= s4_p0; s4_mem[1]  <= s4_p1; s4_mem[2]  <= s4_p2; s4_mem[3]  <= s4_p3; end
                1: begin s4_mem[4]  <= s4_p0; s4_mem[5]  <= s4_p1; s4_mem[6]  <= s4_p2; s4_mem[7]  <= s4_p3; end
                2: begin s4_mem[8]  <= s4_p0; s4_mem[9]  <= s4_p1; s4_mem[12] <= s4_p2; s4_mem[13] <= s4_p3; end
                default: begin s4_mem[10] <= s4_p0; s4_mem[11] <= s4_p1; s4_mem[14] <= s4_p2; s4_mem[15] <= s4_p3; end
            endcase
            cap_idx <= cap_idx + 1;
        end
    end

    real max_err, tol, err;
    integer pass_count;
    initial begin
        // wait until the stimulus block has finished its drain tail
        wait (stim_done);

        $display("\n idx  |        x[idx]        | top(s1..s4)*32 output|   true FFT expected  |       abs err        | pass?");
        $display("------|----------------------|----------------------|----------------------|----------------------|------");
        max_err = 0.0;
        // x16 amplifies the pipeline's own quantization noise by the same
        // factor -- observed max ~0.0008 at WIDTH=16, TWIDDLE_WIDTH=14;
        // 0.0012 leaves a small margin, not a generous one.
        tol = 0.0012;
        pass_count = 0;
        for (i = 0; i < 16; i = i + 1) begin
            err = (fixed_to_real(s4_mem[i]) * DEATTENUATE) - s4_exp[i];
            if (err < 0.0) err = -err;
            if (err > max_err) max_err = err;
            if (err <= tol) pass_count = pass_count + 1;
            $display(" %4d | %20.15f | %20.15f | %20.15f | %20.15f | %s",
                      i, x_real[i], fixed_to_real(s4_mem[i]) * DEATTENUATE, s4_exp[i], err,
                      (err <= tol) ? "PASS" : "FAIL");
        end

        $display("\nmax abs error: %0.6f  (tolerance %0.6f, WIDTH=%0d, TWIDDLE_WIDTH=%0d)", max_err, tol, WIDTH, TWIDDLE_WIDTH);
        $display("out_valid pulses captured: %0d/4", cap_idx);
        $display("timing: first in_valid sampled at clock %0d, first out_valid at clock %0d (latency %0d clocks)",
                 first_in_cyc, first_out_cyc, first_out_cyc - first_in_cyc);
        if (pass_count == 16 && cap_idx == 4)
            $display("TOP_STAGE1_STAGE2_STAGE3_STAGE4: ALL 16 SAMPLES PASS");
        else
            $display("TOP_STAGE1_STAGE2_STAGE3_STAGE4: %0d/16 PASS, %0d/4 PULSES -- CHECK ABOVE", pass_count, cap_idx);

        $finish;
    end

endmodule
