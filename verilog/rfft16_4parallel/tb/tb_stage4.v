`timescale 1ns/1ps

// tb_stage4.v -- self-checking testbench for stage4.v, chained after
// stage1.v, stage2.v and stage3.v (all 4 columns of the 4-parallel
// 16-point RFFT architecture) -- the final stage, so this is the first
// testbench in the project that checks against the FULL s4[] output.
//
// Same fixed 16-sample real test frame as tb_stage1/2/3.v. Captures all
// 16 raw s4[] outputs into a testbench memory via the out_valid handshake.
// stage4's 4 out_valid pulses arrive in the order pos=1,2,3,0 (see
// stage4.v's header) -- a plain arrival-order counter (cap_idx) maps
// straight onto that sequence, same idea as tb_stage3.v's cap_idx, just
// against a different mapping table (this one from
// python/verify/gen_stage4_schedule.py's S4_BY_POS, not S3_PORTS).
//
// s4_exp[] is a hardcoded literal copy of python's own
// stage4(stage3(stage2(stage1(NOMINAL_X)))), full double precision, taken
// directly from a fresh run right before this file was written -- not
// transcribed from memory, not recomputed here.

module tb_stage4;

    parameter WIDTH = 8;
    localparam real SCALE      = (1 << (WIDTH - 1));
    localparam       IN_WIDTH  = WIDTH + 1;       // stage1 output width
    localparam       S2_WIDTH  = WIDTH + 2;       // stage2 output width (stage3's input width)
    localparam       S3A_WIDTH = S2_WIDTH + 1;    // stage3's p0/p1 width (stage4's input width)
    localparam       S3B_WIDTH = S2_WIDTH + 2;    // stage3's p2/p3 width (stage4's input width)
    localparam       S4_WIDTH  = S3B_WIDTH + 1;   // stage4's own output width

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

    wire s4_valid;
    wire signed [S4_WIDTH-1:0] s4_p0, s4_p1, s4_p2, s4_p3;

    stage4 #(.WIDTH(WIDTH)) u_stage4 (
        .clk(clk), .rst_n(rst_n), .in_valid(s3_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3),
        .out_valid(s4_valid),
        .s4_p0(s4_p0), .s4_p1(s4_p1), .s4_p2(s4_p2), .s4_p3(s4_p3)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as tb_stage1/2/3.v ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0]   x_mem  [0:15];   // quantized DUT input memory
    reg  signed [S4_WIDTH-1:0] s4_mem [0:15];  // captured stage-4 output, python s4[] order
    real                       s4_exp [0:15];  // exact expected s4[], unquantized

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
        input signed [S4_WIDTH-1:0] val;
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // exact expected s4[] -- python's stage4(stage3(stage2(stage1(NOMINAL_X)))),
    // full double precision, from a fresh run right before this file was written.
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

    // ---- capture DUT (stage4) output into s4_mem, driven by s4_valid ----
    // Arrival order (cap_idx 0,1,2,3) is pos=1,2,3,0 -- see stage4.v's
    // header table and gen_stage4_schedule.py's S4_BY_POS.
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

        // Stage 4's last pulse (the wrapped pos=0) lands several cycles
        // after in_valid drops, on top of stage3's own drain tail -- wait
        // generously, then confirm all 4 pulses (cap_idx==4) arrived.
        repeat (14) @(negedge clk);

        $display("\n idx |   x[idx]             |  stage4 output (DUT) |  stage4 expected  |  abs err  | pass?");
        $display("-----|-----------------------|----------------------|--------------------|-----------|------");
        max_err = 0.0;
        // one more add/sub on already-quantized stage3 operands: worst case
        // is |err_a|+|err_b| ~= 2x stage3's own tolerance.
        tol = 16.0 / SCALE;
        pass_count = 0;
        for (i = 0; i < 16; i = i + 1) begin
            err = fixed_to_real(s4_mem[i]) - s4_exp[i];
            if (err < 0.0) err = -err;
            if (err > max_err) max_err = err;
            if (err <= tol) pass_count = pass_count + 1;
            $display(" %3d   | %9.15f   |            %9.15f   |         %9.15f   |  %8.15f   | %s",
                      i, x_real[i], fixed_to_real(s4_mem[i]), s4_exp[i], err,
                      (err <= tol) ? "PASS" : "FAIL");
        end

        $display("\nmax abs error: %0.6f  (tolerance %0.6f, WIDTH=%0d)", max_err, tol, WIDTH);
        $display("out_valid pulses captured: %0d/4", cap_idx);
        if (pass_count == 16 && cap_idx == 4)
            $display("STAGE4: ALL 16 SAMPLES PASS");
        else
            $display("STAGE4: %0d/16 PASS, %0d/4 PULSES -- CHECK ABOVE", pass_count, cap_idx);

        $finish;
    end

endmodule
