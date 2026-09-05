`timescale 1ns/1ps

// tb_stage1_stage2_timed.v -- simple, explicit-timeline testbench for
// stage1 chained into stage2.
//
// Inputs are applied from one flat `initial` block using `#10` delays, so
// every event's time is visible straight off the code (t=10, t=20, ...)
// and lines up with the xsim waveform -- no driving loop.
//
// Two checks, done live as each stage's handshake pulses:
//   1. stage2's own input pins (read straight off u_stage2's ports via a
//      hierarchical reference) against stage1's known-correct values.
//   2. stage2's final output against stage2's known-correct values.
// Both golden value tables are copied verbatim from tb_stage1.v /
// tb_stage2.v (which are themselves copied verbatim from the python
// model) -- nothing is recomputed here.

module tb_stage1_stage2_timed;

    parameter WIDTH = 8;
    localparam real SCALE     = (1 << (WIDTH - 1));
    localparam       IN_WIDTH = WIDTH + 1;

    reg clk, rst_n, x_valid;   // drives stage1 only -- NOT the same signal as
                                // stage2's in_valid port (that's s1_valid,
                                // below, registered one cycle later)
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;

    wire s1_valid;
    wire signed [IN_WIDTH-1:0] s1_top_sum, s1_top_diff, s1_bot_sum, s1_bot_diff;
    wire [1:0] dbg_s2_k = u_stage2.k;

    stage1 #(.WIDTH(WIDTH)) u_stage1 (
        .clk(clk), .rst_n(rst_n), .in_valid(x_valid),
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

    initial clk = 0;
    always #5 clk = ~clk;   // 10ns period -> negedges at t=10,20,30,...

    // ---- fixed 16-sample test frame (same as tb_stage1.v / tb_stage2.v) ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg signed [WIDTH-1:0] x_mem [0:15];

    function signed [WIDTH-1:0] to_fixed;
        input real val;
        real scaled;
        begin
            scaled = val * SCALE;
            to_fixed = (val >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function real fixed_to_real;
        input signed [IN_WIDTH:0] val;   // wide enough for stage1's (IN_WIDTH-1:0) and stage2's (IN_WIDTH:0) outputs
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    // ---- golden values, copied verbatim from tb_stage1.v / tb_stage2.v ----
    integer i;
    real s1_exp [0:15];
    real s2_exp [0:15];
    initial begin
        for (i = 0; i < 16; i = i + 1)
            x_mem[i] = to_fixed(x_real[i]);

        s1_exp[0]  = 0.25;                  s1_exp[1]  = 0.2;
        s1_exp[2]  = 0.050000000000000044;  s1_exp[3]  = 0.04999999999999993;
        s1_exp[4]  = -0.04999999999999993;  s1_exp[5]  = 0.15000000000000002;
        s1_exp[6]  = 0.19999999999999998;   s1_exp[7]  = -0.30000000000000004;
        s1_exp[8]  = -0.04999999999999999;  s1_exp[9]  = -0.7;
        s1_exp[10] = 0.75;                  s1_exp[11] = -1.15;
        s1_exp[12] = 1.35;                  s1_exp[13] = -0.25;
        s1_exp[14] = 0.4;                   s1_exp[15] = -1.3;

        s2_exp[0]  = 0.20000000000000007;   s2_exp[1]  = 0.35000000000000003;
        s2_exp[2]  = 0.25;                  s2_exp[3]  = -0.2500000000000001;
        s2_exp[4]  = 0.29999999999999993;   s2_exp[5]  = 0.04999999999999999;
        s2_exp[6]  = -0.14999999999999994;  s2_exp[7]  = 0.35;
        s2_exp[8]  = -0.04999999999999999;  s2_exp[9]  = -0.5510448146666282;
        s2_exp[10] = 0.24748737341529164;   s2_exp[11] = 0.7609574450448195;
        s2_exp[12] = -1.35;                 s2_exp[13] = 0.4988482857833845;
        s2_exp[14] = -0.8131727983645298;   s2_exp[15] = 1.5599499244625963;
    end

    // ---- one shared compare-and-report helper: prints only on FAIL ----
    function chk;
        input real got, exp_v, tol;
        input integer idx;
        begin
            if (((got - exp_v) <= tol) && ((exp_v - got) <= tol)) begin
                chk = 1'b1;
            end else begin
                $display("FAIL idx=%0d got=%0.5f exp=%0.5f", idx, got, exp_v);
                chk = 1'b0;
            end
        end
    endfunction

    // ---- check 1: stage2's own input pins vs stage1's golden values,
    //      live, every time s1_valid pulses ----
    real    s1_tol;
    integer s1_idx, s1_pass;
    initial begin s1_idx = 0; s1_pass = 0; s1_tol = 1.5 / SCALE; end

    always @(posedge clk) begin
        if (!rst_n) begin
            s1_idx = 0;
        end else if (s1_valid) begin
            if (chk(fixed_to_real(u_stage2.s1_k),   s1_exp[s1_idx],    s1_tol, s1_idx))    s1_pass = s1_pass + 1;
            if (chk(fixed_to_real(u_stage2.s1_k4),  s1_exp[s1_idx+4],  s1_tol, s1_idx+4))  s1_pass = s1_pass + 1;
            if (chk(fixed_to_real(u_stage2.s1_k8),  s1_exp[s1_idx+8],  s1_tol, s1_idx+8))  s1_pass = s1_pass + 1;
            if (chk(fixed_to_real(u_stage2.s1_k12), s1_exp[s1_idx+12], s1_tol, s1_idx+12)) s1_pass = s1_pass + 1;
            s1_idx = s1_idx + 1;
        end
    end

    // ---- check 2: stage2's final output vs stage2's golden values,
    //      live, every time s2_valid pulses ----
    real    s2_tol;
    integer s2_idx, s2_pass;
    initial begin s2_idx = 0; s2_pass = 0; s2_tol = 6.0 / SCALE; end

    always @(posedge clk) begin
        if (!rst_n) begin
            s2_idx = 0;
        end else if (s2_valid) begin
            if (chk(fixed_to_real(s2_top_sum),  s2_exp[s2_idx],    s2_tol, s2_idx))    s2_pass = s2_pass + 1;
            if (chk(fixed_to_real(s2_top_diff), s2_exp[s2_idx+4],  s2_tol, s2_idx+4))  s2_pass = s2_pass + 1;
            if (chk(fixed_to_real(s2_bot_re),   s2_exp[s2_idx+8],  s2_tol, s2_idx+8))  s2_pass = s2_pass + 1;
            if (chk(fixed_to_real(s2_bot_im),   s2_exp[s2_idx+12], s2_tol, s2_idx+12)) s2_pass = s2_pass + 1;
            s2_idx = s2_idx + 1;
        end
    end

    // ---- drive stage1's inputs on a plain timeline, then report ----
    initial begin
        rst_n = 0; x_valid = 0;
        x_k = 0; x_k_n4 = 0; x_k_n2 = 0; x_k_3n4 = 0;

        #10 rst_n = 1;                                                                             // t=10

        #10 x_valid = 1; x_k = x_mem[0]; x_k_n4 = x_mem[4];  x_k_n2 = x_mem[8];  x_k_3n4 = x_mem[12]; // t=20
        #10          x_k = x_mem[1]; x_k_n4 = x_mem[5];  x_k_n2 = x_mem[9];  x_k_3n4 = x_mem[13];      // t=30
        #10          x_k = x_mem[2]; x_k_n4 = x_mem[6];  x_k_n2 = x_mem[10]; x_k_3n4 = x_mem[14];      // t=40
        #10          x_k = x_mem[3]; x_k_n4 = x_mem[7];  x_k_n2 = x_mem[11]; x_k_3n4 = x_mem[15];      // t=50

        #10 x_valid = 0;                                                                          // t=60
        #20;                                                                                        // t=80: stage2's last output has landed & been checked

        $display("\nstage1->stage2 pin checks: %0d/16 pass   stage2 output checks: %0d/16 pass", s1_pass, s2_pass);
        if (s1_pass == 16 && s2_pass == 16)
            $display("STAGE1_STAGE2_TIMED: ALL CHECKS PASS");
        else
            $display("STAGE1_STAGE2_TIMED: CHECK FAILURES ABOVE");

        $finish;
    end

endmodule
