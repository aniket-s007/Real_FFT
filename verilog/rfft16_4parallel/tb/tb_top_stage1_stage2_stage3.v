`timescale 1ns/1ps

// tb_top_stage1_stage2_stage3.v -- proves top_stage1_stage2_stage3.v's
// wiring is correct by driving it with the exact same stimulus and
// checking against the exact same expected s3[] values as tb_stage3.v
// (which wires stage1/stage2/stage3 inline and already passes all 16
// samples). A bit-exact match here confirms the new structural module
// reproduces that same, already-validated connection -- not a new/
// independent check of the math itself.

module tb_top_stage1_stage2_stage3;

    parameter WIDTH = 8;
    localparam real SCALE      = (1 << (WIDTH - 1));
    localparam       S2_WIDTH  = WIDTH + 2;
    localparam       S3A_WIDTH = S2_WIDTH + 1;
    localparam       S3B_WIDTH = S2_WIDTH + 2;

    reg clk, rst_n, in_valid;
    reg signed [WIDTH-1:0] x_k, x_k_n4, x_k_n2, x_k_3n4;

    wire s3_valid;
    wire signed [S3A_WIDTH-1:0] s3_p0, s3_p1;
    wire signed [S3B_WIDTH-1:0] s3_p2, s3_p3;

    top_stage1_stage2_stage3 #(.WIDTH(WIDTH)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .x_k(x_k), .x_k_n4(x_k_n4), .x_k_n2(x_k_n2), .x_k_3n4(x_k_3n4),
        .out_valid(s3_valid),
        .s3_p0(s3_p0), .s3_p1(s3_p1), .s3_p2(s3_p2), .s3_p3(s3_p3)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- same fixed 16-sample frame as tb_stage1.v / tb_stage2.v / tb_stage3.v ----
    real x_real [0:15];
    initial begin
        x_real[0]  =  0.10; x_real[1]  = -0.25; x_real[2]  =  0.40; x_real[3]  = -0.55;
        x_real[4]  =  0.65; x_real[5]  = -0.05; x_real[6]  =  0.30; x_real[7]  = -0.80;
        x_real[8]  =  0.15; x_real[9]  =  0.45; x_real[10] = -0.35; x_real[11] =  0.60;
        x_real[12] = -0.70; x_real[13] =  0.20; x_real[14] = -0.10; x_real[15] =  0.50;
    end

    reg  signed [WIDTH-1:0]     x_mem  [0:15];
    reg  signed [S3B_WIDTH-1:0] s3_mem [0:15];
    real                        s3_exp [0:15];

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
        input signed [S3B_WIDTH-1:0] val;
        begin
            fixed_to_real = val / SCALE;
        end
    endfunction

    integer i, k;

    // identical to tb_stage3.v's s3_exp[] -- python's own
    // stage3(stage2(stage1(NOMINAL_X))) output taken verbatim, via
    // gen_stage34_schedule.py's own printed golden literals.
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

    // ---- capture DUT output into s3_mem, same port->index mapping as tb_stage3.v ----
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
        repeat (10) @(negedge clk);

        $display("\n idx |   x[idx]             |  top(s1+s2+s3) output |  stage3 expected  |  abs err  | pass?");
        $display("-----|-----------------------|------------------------|--------------------|-----------|------");
        max_err = 0.0;
        tol = 8.0 / SCALE;
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
            $display("TOP_STAGE1_STAGE2_STAGE3: ALL 16 SAMPLES PASS");
        else
            $display("TOP_STAGE1_STAGE2_STAGE3: %0d/16 PASS, %0d/4 PULSES -- CHECK ABOVE", pass_count, cap_idx);

        $finish;
    end

endmodule
