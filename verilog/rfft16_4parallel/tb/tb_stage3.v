`timescale 1ns/1ps

// tb_stage3.v -- self-checking testbench for stage3.v, chained after
// stage1.v and stage2.v (Columns 1-3 of the 4-parallel 16-point RFFT
// architecture).
//
// Same fixed 16-sample real test frame as tb_stage1.v/tb_stage2.v, so
// all three testbenches are directly comparable. Wires stage1->stage2
// using the same documented cross-mapping tb_stage2.v uses, then
// stage2's 4 output ports straight into stage3 (no crossing there --
// stage3.v takes them by name).
//
// Captures all 16 raw s3[] outputs into a testbench memory (python s3[]
// order) via the out_valid handshake, using the same port->index mapping
// verified by python/verify/gen_stage34_schedule.py (S3_PORTS). Checks
// against s3_exp[], a hardcoded literal copy of that script's own
// printed golden values (python's stage3(stage2(stage1(NOMINAL_X))),
// re-run fresh right before writing this file -- not re-derived here).

module tb_stage3;

    parameter WIDTH = 8;
    localparam real SCALE      = (1 << (WIDTH - 1));
    localparam       IN_WIDTH  = WIDTH + 1;       // stage1 output width
    localparam       S2_WIDTH  = WIDTH + 2;       // stage2 output width (stage3's input width)
    localparam       S3A_WIDTH = S2_WIDTH + 1;    // stage3's p0/p1 width
    localparam       S3B_WIDTH = S2_WIDTH + 2;    // stage3's p2/p3 width (rotator output)

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
    wire signed [S2_WIDTH-1:0] s2_top_sum, s2_top_diff, s2_bot_re, s2_bot_im;

    stage2 #(.WIDTH(WIDTH)) u_stage2 (
        .clk(clk), .rst_n(rst_n), .in_valid(s1_valid),
        .s1_k(s1_top_sum), .s1_k4(s1_bot_sum),
        .s1_k8(s1_top_diff), .s1_k12(s1_bot_diff),
        .out_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im)
    );

    wire s3_valid;
    wire signed [S3A_WIDTH-1:0] s3_p0, s3_p1;
    wire signed [S3B_WIDTH-1:0] s3_p2, s3_p3;

    stage3 #(.WIDTH(WIDTH)) u_stage3 (
        .clk(clk), .rst_n(rst_n), .in_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im),
        .out_valid(s3_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as tb_stage1.v/tb_stage2.v ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0]    x_mem  [0:15];   // quantized DUT input memory
    reg  signed [S3B_WIDTH-1:0] s3_mem [0:15];  // captured stage-3 output, python s3[] order
    real                        s3_exp [0:15];  // exact expected s3[], unquantized

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
        input signed [S3B_WIDTH-1:0] val;   // wide enough for both s3_p0/p1 and s3_p2/p3
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // exact expected s3[] -- hardcoded from gen_stage34_schedule.py's own
    // printed golden literals for the "nominal" test vector, re-run fresh
    // right before this file was written (not transcribed from memory, not
    // recomputed here -- see that script for how these were derived and
    // independently checked bit-exact against python's stage3()).
    initial begin
        for (i = 0; i < 16; i = i + 1)
            x_mem[i] = to_fixed(x_real[i]);

        s3_exp[0]  = 0.45000000000000007;  s3_exp[2]  = -0.04999999999999993;
        s3_exp[4]  = 0.29999999999999993;  s3_exp[6]  = 0.14999999999999994;
        s3_exp[1]  = 0.09999999999999992;  s3_exp[3]  = 0.6000000000000001;
        s3_exp[5]  = -0.21213203435596428; s3_exp[7]  = -0.282842712474619;
        s3_exp[8]  = 0.19748737341529166;  s3_exp[12] = -2.16317279836453;
        s3_exp[10] = -0.29748737341529163; s3_exp[14] = -0.5368272016354703;
        s3_exp[9]  = 0.2099126303781913;   s3_exp[13] = 2.058798210245981;
        s3_exp[11] = -1.0611016386792118;  s3_exp[15] = 1.3120022597114478;
    end

    // ---- capture DUT (stage3) output into s3_mem, driven by s3_valid ----
    // Port -> s3[] index mapping matches gen_stage34_schedule.py's
    // S3_PORTS table exactly: stage3's own c3 phase advances 0,1,2,3 on
    // successive out_valid pulses, so a plain cap_idx counter (not a
    // hierarchical peek at stage3's internal c2_pos) is enough to know
    // which phase each pulse belongs to.
    integer cap_idx;
    initial cap_idx = 0;
    always @(posedge clk) begin
        if (!rst_n)
            cap_idx <= 0;
        else if (s3_valid) begin
            case (cap_idx)
                0: begin s3_mem[0] <= s3_p0; s3_mem[2]  <= s3_p1; s3_mem[4]  <= s3_p2; s3_mem[6]  <= s3_p3; end
                1: begin s3_mem[1] <= s3_p0; s3_mem[3]  <= s3_p1; s3_mem[5]  <= s3_p2; s3_mem[7]  <= s3_p3; end
                2: begin s3_mem[8] <= s3_p0; s3_mem[12] <= s3_p1; s3_mem[10] <= s3_p2; s3_mem[14] <= s3_p3; end
                default: begin s3_mem[9] <= s3_p0; s3_mem[13] <= s3_p1; s3_mem[11] <= s3_p2; s3_mem[15] <= s3_p3; end
            endcase
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

        // stage3's own c3=2,3 phases read only delayed taps (no live
        // input), so its last valid outputs land several cycles after
        // in_valid drops -- wait generously, then confirm all 4 pulses
        // (cap_idx==4) actually arrived as part of the pass/fail signal.
        repeat (10) @(negedge clk);

        $display("\n idx |   x[idx]             |  stage3 output (DUT) |  stage3 expected  |  abs err  | pass?");
        $display("-----|-----------------------|----------------------|--------------------|-----------|------");
        max_err = 0.0;
        tol = 8.0 / SCALE;    // stage3's own rotator rounding + propagated stage1/stage2 quantization
        pass_count = 0;
        for (i = 0; i < 16; i = i + 1) begin
            err = fixed_to_real(s3_mem[i]) - s3_exp[i];
            if (err < 0.0) err = -err;
            if (err > max_err) max_err = err;
            if (err <= tol) pass_count = pass_count + 1;
            $display(" %3d   | %9.15f   |            %9.15f   |         %9.15f   |  %8.15f   | %s",
                      i, x_real[i], fixed_to_real(s3_mem[i]), s3_exp[i], err,
                      (err <= tol) ? "PASS" : "FAIL");
        end

        $display("\nmax abs error: %0.6f  (tolerance %0.6f, WIDTH=%0d)", max_err, tol, WIDTH);
        $display("out_valid pulses captured: %0d/4", cap_idx);
        if (pass_count == 16 && cap_idx == 4)
            $display("STAGE3: ALL 16 SAMPLES PASS");
        else
            $display("STAGE3: %0d/16 PASS, %0d/4 PULSES -- CHECK ABOVE", pass_count, cap_idx);

        $finish;
    end

endmodule
