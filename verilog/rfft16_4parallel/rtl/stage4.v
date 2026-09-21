`timescale 1ns/1ps

// stage4 -- column 4 (D + SW1, BF)

module stage4 #(
    parameter WIDTH     = 8,
    parameter S2_WIDTH  = WIDTH + 2,
    parameter S3A_WIDTH = S2_WIDTH + 1,   // stage3 s3_p0/p1 width
    parameter S3B_WIDTH = S2_WIDTH + 2    // stage3 s3_p2/p3 width
) (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          in_valid,

    input  wire signed [S3A_WIDTH-1:0]   s3_p0,
    input  wire signed [S3A_WIDTH-1:0]   s3_p1,
    input  wire signed [S3B_WIDTH-1:0]   s3_p2,
    input  wire signed [S3B_WIDTH-1:0]   s3_p3,

    output reg                           out_valid,
    output reg  signed [S3B_WIDTH:0]     s4_p0,   // BF_X sum
    output reg  signed [S3B_WIDTH:0]     s4_p1,   // BF_X diff
    output reg  signed [S3B_WIDTH:0]     s4_p2,   // BF_Y sum
    output reg  signed [S3B_WIDTH:0]     s4_p3    // BF_Y diff
);

    // sign-extend s3_p0, s3_p1
    wire signed [S3B_WIDTH-1:0] s3_p0_ext = s3_p0;
    wire signed [S3B_WIDTH-1:0] s3_p1_ext = s3_p1;

    // 1-cycle delay registers
    reg signed [S3B_WIDTH-1:0] p0_d1, p1_d1, p2_d1, p3_d1;
    reg signed [S3B_WIDTH-1:0] p2_d2, p3_d2;

    always @(posedge clk) begin
        if (!rst_n) begin
            p0_d1 <= 0; p1_d1 <= 0; p2_d1 <= 0; p3_d1 <= 0;
            p2_d2 <= 0; p3_d2 <= 0;
        end else begin
            p2_d2 <= p2_d1;
            p3_d2 <= p3_d1;
            if (in_valid) begin
                p0_d1 <= s3_p0_ext; p1_d1 <= s3_p1_ext;
                p2_d1 <= s3_p2;     p3_d1 <= s3_p3;
            end
        end
    end

    // valid delay
    reg in_valid_d1;
    always @(posedge clk) begin
        if (!rst_n) in_valid_d1 <= 1'b0;
        else        in_valid_d1 <= in_valid;
    end

    wire active     = in_valid | in_valid_d1;
    wire taps_ready = active & in_valid_d1;

    // stage-3 cycle counter
    reg [1:0] pos;
    always @(posedge clk) begin
        if (!rst_n) pos <= 2'd0;
        else if (active) pos <= pos + 2'd1;
    end

    // switch control
    wire control1    = pos[0];
    wire pass_thru_y = (pos == 2'd1);

    // switch x
    wire signed [S3B_WIDTH-1:0] switch_x_out_top, switch_x_out_bottom;
    switch1 #(.WIDTH(2*S3B_WIDTH)) switch_x (
        .control1(control1),
        .in_top({p2_d2, p2_d1}), .in_bottom({p0_d1, s3_p0_ext}),
        .out_top({switch_x_out_top, switch_x_out_bottom}), .out_bottom()
    );

    // switch y
    wire signed [S3B_WIDTH-1:0] switch_y_out_top, switch_y_out_bottom;
    switch1 #(.WIDTH(2*S3B_WIDTH)) switch_y (
        .control1(control1),
        .in_top({p3_d2, p3_d1}), .in_bottom({p1_d1, s3_p1_ext}),
        .out_top({switch_y_out_top, switch_y_out_bottom}), .out_bottom()
    );

    // butterfly X
    wire signed [S3B_WIDTH:0] bf_x_sum_c, bf_x_diff_c;
    real_bf #(.WIDTH(S3B_WIDTH)) bf_x (
        .pass_thru(1'b0),
        .in1(switch_x_out_top), .in2(switch_x_out_bottom),
        .out_sum(bf_x_sum_c), .out_diff(bf_x_diff_c)
    );

    // butterfly Y
    wire signed [S3B_WIDTH:0] bf_y_sum_c, bf_y_diff_raw_c;
    real_bf #(.WIDTH(S3B_WIDTH)) bf_y (
        .pass_thru(pass_thru_y),
        .in1(switch_y_out_top), .in2(switch_y_out_bottom),
        .out_sum(bf_y_sum_c), .out_diff(bf_y_diff_raw_c)
    );

    // negate the butterfly Y diff in pass-through mode
    wire signed [S3B_WIDTH:0] bf_y_diff_c = pass_thru_y ? -bf_y_diff_raw_c : bf_y_diff_raw_c;

    // output register
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            s4_p0 <= 0; s4_p1 <= 0; s4_p2 <= 0; s4_p3 <= 0;
        end else begin
            out_valid <= taps_ready;
            s4_p0 <= bf_x_sum_c;
            s4_p1 <= bf_x_diff_c;
            s4_p2 <= bf_y_sum_c;
            s4_p3 <= bf_y_diff_c;
        end
    end

endmodule
