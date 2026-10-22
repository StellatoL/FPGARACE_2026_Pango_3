`timescale 1ns/1ps

module tb_color_mask;

reg clk;
reg rst_n;

reg         per_frame_vsync;
reg         per_frame_href;
reg         per_frame_clken;
reg [23:0]  matrix_p11, matrix_p12, matrix_p13;
reg [23:0]  matrix_p21, matrix_p22, matrix_p23;
reg [23:0]  matrix_p31, matrix_p32, matrix_p33;

wire        color_mask_frame_vsync;
wire        color_mask_frame_href;
wire        color_mask_frame_clken;
wire [1:0]  color_mask_img_mask;

integer error_count;

// 7 级流水线对齐：输出对应 6 个时钟之前的输入
reg [6:0]  exp_vsync_pipe;
reg [6:0]  exp_href_pipe;
reg [6:0]  exp_clken_pipe;
reg [13:0] exp_mask_pipe;

color_mask u_dut (
    .clk                  (clk),
    .rst_n                (rst_n),
    .per_frame_vsync      (per_frame_vsync),
    .per_frame_href       (per_frame_href),
    .per_frame_clken      (per_frame_clken),
    .matrix_p11           (matrix_p11),
    .matrix_p12           (matrix_p12),
    .matrix_p13           (matrix_p13),
    .matrix_p21           (matrix_p21),
    .matrix_p22           (matrix_p22),
    .matrix_p23           (matrix_p23),
    .matrix_p31           (matrix_p31),
    .matrix_p32           (matrix_p32),
    .matrix_p33           (matrix_p33),
    .color_mask_frame_vsync(color_mask_frame_vsync),
    .color_mask_frame_href (color_mask_frame_href),
    .color_mask_frame_clken(color_mask_frame_clken),
    .color_mask_img_mask   (color_mask_img_mask)
);

always #5 clk = ~clk;

function [1:0] calc_expected_mask;
    input [7:0] r_in;
    input [7:0] g_in;
    input [7:0] b_in;

    reg [11:0] sum_r;
    reg [11:0] sum_g;
    reg [11:0] sum_b;
    reg [13:0] sum_all;

    reg [7:0] r_avg;
    reg [7:0] g_avg;
    reg [7:0] b_avg;
    reg [7:0] local_lum;
    reg [7:0] max_rgb;
    reg [7:0] min_rgb;
    reg [7:0] delta;
    reg [7:0] sat_min;
    reg [7:0] lum_min;
    reg [7:0] ratio_k;
    reg [7:0] max_rgb_d1;

    reg [15:0] b_avg_100;
    reg [15:0] g_avg_100;
    reg [15:0] r_avg_100;
    reg [15:0] r_avg_k;
    reg [15:0] g_avg_k;
    reg [15:0] b_avg_k;
    reg [15:0] diff_rg_100;
    reg [15:0] max_rgb_15;

    reg valid_color;
    reg mask_blue;
    reg mask_green;
    reg mask_yellow;
begin
    // 与 DUT 保持一致：9 个像素均匀时，窗口求和就是 9 倍像素值
    sum_r = r_in * 12'd9;
    sum_g = g_in * 12'd9;
    sum_b = b_in * 12'd9;
    sum_all = sum_r + sum_g + sum_b;

    r_avg = ((sum_r * 6'd57) + 18'd256) >> 9;
    g_avg = ((sum_g * 6'd57) + 18'd256) >> 9;
    b_avg = ((sum_b * 6'd57) + 18'd256) >> 9;

    local_lum = (sum_all * 5'd19) >> 9;

    if (r_avg >= g_avg && r_avg >= b_avg)       max_rgb = r_avg;
    else if (g_avg >= r_avg && g_avg >= b_avg)  max_rgb = g_avg;
    else                                        max_rgb = b_avg;

    if (r_avg <= g_avg && r_avg <= b_avg)       min_rgb = r_avg;
    else if (g_avg <= r_avg && g_avg <= b_avg)  min_rgb = g_avg;
    else                                        min_rgb = b_avg;

    delta = max_rgb - min_rgb;
    max_rgb_d1 = max_rgb;

    if (local_lum > 8'd150 && local_lum <= 8'd200) begin
        ratio_k = 8'd120;
        sat_min = 8'd25;
        lum_min = 8'd50;
    end else if (local_lum > 8'd70 && local_lum <= 8'd150) begin
        ratio_k = 8'd115;
        sat_min = 8'd15;
        lum_min = 8'd25;
    end else begin
        ratio_k = 8'd110;
        sat_min = 8'd10;
        lum_min = 8'd15;
    end

    valid_color = (delta > sat_min) && (max_rgb_d1 > lum_min);

    b_avg_100 = b_avg * 8'd100;
    g_avg_100 = g_avg * 8'd100;
    r_avg_100 = r_avg * 8'd100;

    r_avg_k = r_avg * ratio_k;
    g_avg_k = g_avg * ratio_k;
    b_avg_k = b_avg * ratio_k;

    if (r_avg > g_avg)
        diff_rg_100 = (r_avg - g_avg) * 8'd100;
    else
        diff_rg_100 = (g_avg - r_avg) * 8'd100;

    max_rgb_15 = max_rgb_d1 * 8'd15;

    mask_blue   = (b_avg_100 > r_avg_k) && (b_avg_100 > g_avg_k);
    mask_green  = (g_avg_100 > r_avg_k) && (g_avg_100 > b_avg_k);
    mask_yellow = (r_avg_100 > b_avg_k) && (g_avg_100 > b_avg_k) && (diff_rg_100 < max_rgb_15);

    if (valid_color) begin
        if (mask_blue)
            calc_expected_mask = 2'd1;
        else if (mask_green)
            calc_expected_mask = 2'd2;
        else if (mask_yellow)
            calc_expected_mask = 2'd3;
        else
            calc_expected_mask = 2'd0;
    end else begin
        calc_expected_mask = 2'd0;
    end
end
endfunction

task set_uniform_matrix;
    input [7:0] r_val;
    input [7:0] g_val;
    input [7:0] b_val;
begin
    matrix_p11 = {r_val, g_val, b_val};
    matrix_p12 = {r_val, g_val, b_val};
    matrix_p13 = {r_val, g_val, b_val};
    matrix_p21 = {r_val, g_val, b_val};
    matrix_p22 = {r_val, g_val, b_val};
    matrix_p23 = {r_val, g_val, b_val};
    matrix_p31 = {r_val, g_val, b_val};
    matrix_p32 = {r_val, g_val, b_val};
    matrix_p33 = {r_val, g_val, b_val};
end
endtask

task push_expected;
    input exp_vsync;
    input exp_href;
    input exp_clken;
    input [1:0] exp_mask;
begin
    exp_vsync_pipe = {exp_vsync_pipe[5:0], exp_vsync};
    exp_href_pipe  = {exp_href_pipe[5:0],  exp_href};
    exp_clken_pipe = {exp_clken_pipe[5:0], exp_clken};
    exp_mask_pipe  = {exp_mask_pipe[11:0], exp_mask};
end
endtask

task check_outputs;
    reg [1:0] expected_mask;
begin
    expected_mask = exp_mask_pipe[13:12];

    if (color_mask_frame_vsync !== exp_vsync_pipe[6]) begin
        $display("[%0t] VSYNC mismatch: exp=%b got=%b", $time, exp_vsync_pipe[6], color_mask_frame_vsync);
        error_count = error_count + 1;
    end

    if (color_mask_frame_href !== exp_href_pipe[6]) begin
        $display("[%0t] HREF mismatch: exp=%b got=%b", $time, exp_href_pipe[6], color_mask_frame_href);
        error_count = error_count + 1;
    end

    if (color_mask_frame_clken !== exp_clken_pipe[6]) begin
        $display("[%0t] CLKEN mismatch: exp=%b got=%b", $time, exp_clken_pipe[6], color_mask_frame_clken);
        error_count = error_count + 1;
    end

    if (color_mask_frame_clken) begin
        if (color_mask_img_mask !== expected_mask) begin
            $display("[%0t] MASK mismatch: exp=%0d got=%0d", $time, expected_mask, color_mask_img_mask);
            error_count = error_count + 1;
        end
    end
end
endtask

task drive_cycle;
    input        vsync_val;
    input        href_val;
    input        clken_val;
    input [7:0]  r_val;
    input [7:0]  g_val;
    input [7:0]  b_val;
    reg [1:0]    expected_mask;
begin
    @(negedge clk);
    per_frame_vsync = vsync_val;
    per_frame_href  = href_val;
    per_frame_clken = clken_val;
    set_uniform_matrix(r_val, g_val, b_val);

    expected_mask = calc_expected_mask(r_val, g_val, b_val);
    push_expected(vsync_val, href_val, clken_val, expected_mask);

    @(posedge clk);
    #1;
    check_outputs();
end
endtask

initial begin
    clk = 1'b0;
    rst_n = 1'b0;

    per_frame_vsync = 1'b0;
    per_frame_href  = 1'b0;
    per_frame_clken = 1'b0;

    matrix_p11 = 24'd0; matrix_p12 = 24'd0; matrix_p13 = 24'd0;
    matrix_p21 = 24'd0; matrix_p22 = 24'd0; matrix_p23 = 24'd0;
    matrix_p31 = 24'd0; matrix_p32 = 24'd0; matrix_p33 = 24'd0;

    exp_vsync_pipe = 7'd0;
    exp_href_pipe  = 7'd0;
    exp_clken_pipe = 7'd0;
    exp_mask_pipe  = 14'd0;

    error_count = 0;

    $display("TB start: color_mask");

    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // 预热流水线，避免刚释放复位时的瞬态影响
    repeat (8) begin
        drive_cycle(1'b0, 1'b0, 1'b0, 8'd0, 8'd0, 8'd0);
    end

    // 纯蓝窗口：应输出 mask=1
    repeat (6) begin
        drive_cycle(1'b0, 1'b1, 1'b1, 8'd20, 8'd20, 8'd230);
    end

    // 纯绿窗口：应输出 mask=2
    repeat (6) begin
        drive_cycle(1'b0, 1'b1, 1'b1, 8'd20, 8'd230, 8'd20);
    end

    // 黄色综合：应输出 mask=3
    repeat (6) begin
        drive_cycle(1'b0, 1'b1, 1'b1, 8'd230, 8'd220, 8'd20);
    end

    // 中性灰：应输出 mask=0
    repeat (6) begin
        drive_cycle(1'b0, 1'b1, 1'b1, 8'd120, 8'd120, 8'd120);
    end

    // 退出有效区，检查输出是否能回到 0
    repeat (8) begin
        drive_cycle(1'b0, 1'b0, 1'b0, 8'd0, 8'd0, 8'd0);
    end

    if (error_count == 0) begin
        $display("color_mask tb PASSED.");
    end else begin
        $display("color_mask tb FAILED with %0d errors.", error_count);
    end

    $finish;
end

endmodule