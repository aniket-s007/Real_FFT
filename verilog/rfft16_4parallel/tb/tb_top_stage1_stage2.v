`timescale 1ns/1ps

// tb_top_stage1_stage2.v -- proves top_stage1_stage2.v's stage1->stage2
// cross-wiring is correct by driving it with the exact same stimulus and
// checking against the exact same expected s2[] values as tb_stage2.v
// (which wires stage1/stage2 inline and already passes all 16 samples).
// A bit-exact match here confirms the new structural module reproduces
// that same, already-validated connection -- not a new/independent check
// of the math itself.

module tb_top_stage1_stage2;

    parameter WIDTH = 8;
    localparam real SCALE = (1 << (WIDTH - 1));
    localparam IN_WIDTH = WIDTH + 1;

    reg clk, rst_n, in_valid;
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;

    wire s2_valid;
    wire signed [IN_WIDTH:0] s2_top_sum, s2_top_diff, s2_bot_re, s2_bot_im;

    top_stage1_stage2 #(.WIDTH(WIDTH)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s2_valid),
        .s2_top_sum(s2_top_sum), .s2_top_diff(s2_top_diff),
        .s2_bot_re(s2_bot_re), .s2_bot_im(s2_bot_im)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as tb_stage1.v / tb_stage2.v ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0]   x_mem  [0:15];
    reg  signed [IN_WIDTH:0]  s2_mem [0:15];
    real                      s2_exp [0:15];

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

    // identical to tb_stage2.v's s2_exp[] -- python's own
    // stage2(stage1(NOMINAL_X)) output taken verbatim.
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
        @(negedge clk);
        @(negedge clk);

        $display("\n idx |   x[idx]             |  top(s1+s2) output   |  stage2 expected  |  abs err  | pass?");
        $display("-----|-----------------------|-----------------------|--------------------|-----------|------");
        max_err = 0.0;
        tol = 6.0 / SCALE;
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
            $display("TOP_STAGE1_STAGE2: ALL 16 SAMPLES PASS");
        else
            $display("TOP_STAGE1_STAGE2: %0d/16 PASS -- CHECK ABOVE", pass_count);

        $finish;
    end

endmodule
