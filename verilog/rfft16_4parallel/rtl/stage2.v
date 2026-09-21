`timescale 1ns/1ps

// stage2 -- column 2 (BF, W^k)

module stage2 #(
    parameter WIDTH         = 8,
    parameter IN_WIDTH      = WIDTH + 1,
    parameter TWIDDLE_WIDTH = WIDTH   // twiddle bit-width (2..39)
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,

    input  wire signed [IN_WIDTH-1:0] s1_k,     // s1[k]    (stage1.s1_top_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k4,    // s1[k+4]  (stage1.s1_bot_sum)
    input  wire signed [IN_WIDTH-1:0] s1_k8,    // s1[k+8]  (stage1.s1_top_diff)
    input  wire signed [IN_WIDTH-1:0] s1_k12,   // s1[k+12] (stage1.s1_bot_diff)

    output reg                        out_valid,
    output reg  signed [IN_WIDTH:0]   s2_top_sum,   // -> s2[k]
    output reg  signed [IN_WIDTH:0]   s2_top_diff,  // -> s2[k+4]
    output reg  signed [IN_WIDTH:0]   s2_bot_re,    // -> s2[k+8]
    output reg  signed [IN_WIDTH:0]   s2_bot_im     // -> s2[k+12]
);

    // twiddle ROM (40-bit master constants)
    localparam MASTER_WIDTH = 40;

    // twiddle 0
    localparam signed [MASTER_WIDTH-1:0] COS0_M = 40'sd549755813887, SIN0_M = 40'sd0;
    // twiddle 1
    localparam signed [MASTER_WIDTH-1:0] COS1_M = 40'sd507908144330, SIN1_M = -40'sd210382441821;
    // twiddle 2
    localparam signed [MASTER_WIDTH-1:0] COS2_M = 40'sd388736063997, SIN2_M = -40'sd388736063997;
    // twiddle 3
    localparam signed [MASTER_WIDTH-1:0] COS3_M = 40'sd210382441821, SIN3_M = -40'sd507908144330;

    // keep the top TWIDDLE_WIDTH bits of a master constant
    function signed [TWIDDLE_WIDTH-1:0] derive_coef;
        input signed [MASTER_WIDTH-1:0] raw;
        begin
            derive_coef= raw[MASTER_WIDTH-1 -: TWIDDLE_WIDTH];
        end
    endfunction

    // twiddle 0
    localparam signed [TWIDDLE_WIDTH-1:0] COS0 = derive_coef(COS0_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN0 = derive_coef(SIN0_M);
    // twiddle 1
    localparam signed [TWIDDLE_WIDTH-1:0] COS1 = derive_coef(COS1_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN1 = derive_coef(SIN1_M);
    // twiddle 2
    localparam signed [TWIDDLE_WIDTH-1:0] COS2 = derive_coef(COS2_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN2 = derive_coef(SIN2_M);
    // twiddle 3
    localparam signed [TWIDDLE_WIDTH-1:0] COS3 = derive_coef(COS3_M);
    localparam signed [TWIDDLE_WIDTH-1:0] SIN3 = derive_coef(SIN3_M);

    // counter k = 0..3
    reg [1:0] k;
    always @(posedge clk) begin
        if (!rst_n)
            k <= 2'd0;
        else if (in_valid)
            k <= (k == 2'd3) ? 2'd0 : k + 2'd1;
    end

    // twiddle select
    reg signed [TWIDDLE_WIDTH-1:0] cos_coef, sin_coef;
    always @(*) begin
        case (k)
            2'd0: begin cos_coef = COS0; sin_coef = SIN0; end   // twiddle 0
            2'd1: begin cos_coef = COS1; sin_coef = SIN1; end   // twiddle 1
            2'd2: begin cos_coef = COS2; sin_coef = SIN2; end   // twiddle 2
            default: begin cos_coef = COS3; sin_coef = SIN3; end   // twiddle 3
        endcase
    end

    // top butterfly
    wire signed [IN_WIDTH:0] top_sum_c, top_diff_c;
    real_bf #(.WIDTH(IN_WIDTH)) bf_top (
        .pass_thru(1'b0),
        .in1(s1_k), .in2(s1_k4),
        .out_sum(top_sum_c), .out_diff(top_diff_c)
    );

    // bottom lane: Eq.(7) negate, then twiddle rotator
    wire signed [IN_WIDTH-1:0] eq7_b = -s1_k12;
    wire signed [IN_WIDTH:0] bot_re_c, bot_im_c;
    rotator #(.IN_WIDTH(IN_WIDTH), .COEF_WIDTH(TWIDDLE_WIDTH)) wk (
        .re_in(s1_k8), .im_in(eq7_b),
        .cos_coef(cos_coef), .sin_coef(sin_coef),
        .re_out(bot_re_c), .im_out(bot_im_c)
    );

    // output register
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

endmodule
