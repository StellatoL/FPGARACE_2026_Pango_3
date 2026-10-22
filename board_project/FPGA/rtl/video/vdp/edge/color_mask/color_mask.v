module color_mask(
    input           clk,
    input           rst_n,
    input           per_frame_vsync,
    input           per_frame_href,
    input           per_frame_clken,
    input [23:0] matrix_p11, matrix_p12, matrix_p13,
    input [23:0] matrix_p21, matrix_p22, matrix_p23,
    input [23:0] matrix_p31, matrix_p32, matrix_p33,

    output          color_mask_frame_vsync,
    output          color_mask_frame_href,
    output          color_mask_frame_clken,
    output  [ 1:0]  color_mask_img_mask
);

wire [7:0] img_r_p11, img_g_p11, img_b_p11;
wire [7:0] img_r_p12, img_g_p12, img_b_p12;
wire [7:0] img_r_p13, img_g_p13, img_b_p13;
wire [7:0] img_r_p21, img_g_p21, img_b_p21;
wire [7:0] img_r_p22, img_g_p22, img_b_p22;
wire [7:0] img_r_p23, img_g_p23, img_b_p23;
wire [7:0] img_r_p31, img_g_p31, img_b_p31;
wire [7:0] img_r_p32, img_g_p32, img_b_p32;
wire [7:0] img_r_p33, img_g_p33, img_b_p33;

assign {img_r_p11, img_g_p11, img_b_p11} = matrix_p11;
assign {img_r_p12, img_g_p12, img_b_p12} = matrix_p12;
assign {img_r_p13, img_g_p13, img_b_p13} = matrix_p13;
assign {img_r_p21, img_g_p21, img_b_p21} = matrix_p21;
assign {img_r_p22, img_g_p22, img_b_p22} = matrix_p22;
assign {img_r_p23, img_g_p23, img_b_p23} = matrix_p23;
assign {img_r_p31, img_g_p31, img_b_p31} = matrix_p31;
assign {img_r_p32, img_g_p32, img_b_p32} = matrix_p32;
assign {img_r_p33, img_g_p33, img_b_p33} = matrix_p33;

// =================================================================
// 图像同步信号流水线打拍 (7 级流水线延迟对齐)
// =================================================================
reg [6:0] vsync_d;
reg [6:0] href_d;
reg [6:0] clken_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_d <= 7'd0;
        href_d  <= 7'd0;
        clken_d <= 7'd0;
    end else begin
        vsync_d <= {vsync_d[5:0], per_frame_vsync};
        href_d  <= {href_d[5:0],  per_frame_href};
        clken_d <= {clken_d[5:0], per_frame_clken};
    end
end

// =================================================================
// Stage 1: 计算 3x3 窗口求和
// =================================================================
reg [11:0] sum_r, sum_g, sum_b;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum_r <= 12'd0; sum_g <= 12'd0; sum_b <= 12'd0;
    end else if (per_frame_clken) begin
        // 求当前窗口 9 个像素之和
        sum_r <= {3'd0, img_r_p11} + {3'd0, img_r_p12} + {3'd0, img_r_p13} + {3'd0, img_r_p21} + {3'd0, img_r_p22} + {3'd0, img_r_p23} + {3'd0, img_r_p31} + {3'd0, img_r_p32} + {3'd0, img_r_p33};
        sum_g <= {3'd0, img_g_p11} + {3'd0, img_g_p12} + {3'd0, img_g_p13} + {3'd0, img_g_p21} + {3'd0, img_g_p22} + {3'd0, img_g_p23} + {3'd0, img_g_p31} + {3'd0, img_g_p32} + {3'd0, img_g_p33};
        sum_b <= {3'd0, img_b_p11} + {3'd0, img_b_p12} + {3'd0, img_b_p13} + {3'd0, img_b_p21} + {3'd0, img_b_p22} + {3'd0, img_b_p23} + {3'd0, img_b_p31} + {3'd0, img_b_p32} + {3'd0, img_b_p33};
    end
end

// =================================================================
// Stage 2: 计算 R_avg, G_avg, B_avg 和 总和 avg_sum
// (利用 X * 57 >> 9 来近似 X / 9)
// =================================================================
reg [7:0]  r_avg, g_avg, b_avg;
reg [13:0] sum_all; // 三通道总和 (最多 27 * 255 = 6885)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_avg <= 8'd0; g_avg <= 8'd0; b_avg <= 8'd0;
        sum_all <= 14'd0;
    end else begin
        r_avg <= ((sum_r * 6'd57) + 18'd256) >> 9;
        g_avg <= ((sum_g * 6'd57) + 18'd256) >> 9;
        b_avg <= ((sum_b * 6'd57) + 18'd256) >> 9;
        sum_all <= sum_r + sum_g + sum_b;
    end
end

// =================================================================
// Stage 3: 计算 局部光照 local_lum, 最大值 Max_RGB, 最小值 Min_RGB
// =================================================================
reg [7:0] local_lum;
reg [7:0] max_rgb, min_rgb;
reg [7:0] r_avg_d1, g_avg_d1, b_avg_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        local_lum <= 8'd0; max_rgb <= 8'd0; min_rgb <= 8'd0;
        r_avg_d1 <= 8'd0; g_avg_d1 <= 8'd0; b_avg_d1 <= 8'd0;
    end else begin
        // sum_all 包含 27 个像素，除以 27 近似为 X * 19 >> 9
        local_lum <= (sum_all * 5'd19) >> 9; 
        
        // Max 计算
        if (r_avg >= g_avg && r_avg >= b_avg) max_rgb <= r_avg;
        else if (g_avg >= r_avg && g_avg >= b_avg) max_rgb <= g_avg;
        else max_rgb <= b_avg;
        
        // Min 计算
        if (r_avg <= g_avg && r_avg <= b_avg) min_rgb <= r_avg;
        else if (g_avg <= r_avg && g_avg <= b_avg) min_rgb <= g_avg;
        else min_rgb <= b_avg;
        
        r_avg_d1 <= r_avg; g_avg_d1 <= g_avg; b_avg_d1 <= b_avg;
    end
end

// =================================================================
// Stage 4: 计算 Delta 及 动态门控参数生成
// =================================================================
reg [7:0] delta;
reg [7:0] sat_min, lum_min;
reg [7:0] ratio_k; // 将 1.10, 1.15, 1.20 放大 100 倍 -> 110, 115, 120
reg [7:0] r_avg_d2, g_avg_d2, b_avg_d2;
reg [7:0] max_rgb_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        delta <= 8'd0; sat_min <= 8'd10; lum_min <= 8'd15; ratio_k <= 8'd110;
        r_avg_d2 <= 8'd0; g_avg_d2 <= 8'd0; b_avg_d2 <= 8'd0; max_rgb_d1 <= 8'd0;
    end else begin
        delta <= max_rgb - min_rgb;
        max_rgb_d1 <= max_rgb;
        
        if (local_lum > 150 && local_lum <= 200) begin
            ratio_k <= 8'd120; sat_min <= 8'd25; lum_min <= 8'd50;
        end else if (local_lum > 70 && local_lum <= 150) begin
            ratio_k <= 8'd115; sat_min <= 8'd15; lum_min <= 8'd25;
        end else begin
            ratio_k <= 8'd110; sat_min <= 8'd10; lum_min <= 8'd15;
        end
        
        r_avg_d2 <= r_avg_d1; g_avg_d2 <= g_avg_d1; b_avg_d2 <= b_avg_d1;
    end
end

// =================================================================
// Stage 5: 严格门控计算与基础乘法
// =================================================================
reg valid_color;
reg [15:0] b_avg_100, g_avg_100, r_avg_100;
reg [15:0] r_avg_k, g_avg_k, b_avg_k;
reg [15:0] diff_rg_100, max_rgb_15;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_color <= 1'b0;
        b_avg_100 <= 16'd0; g_avg_100 <= 16'd0; r_avg_100 <= 16'd0;
        r_avg_k <= 16'd0;   g_avg_k <= 16'd0;   b_avg_k <= 16'd0;
        diff_rg_100 <= 16'd0; max_rgb_15 <= 16'd0;
    end else if (clken_d[3]) begin
        valid_color <= (delta > sat_min) && (max_rgb_d1 > lum_min);
        
        // 两边都等效放大 100 倍，避免浮点数
        b_avg_100 <= b_avg_d2 * 8'd100;
        g_avg_100 <= g_avg_d2 * 8'd100;
        r_avg_100 <= r_avg_d2 * 8'd100;
        
        r_avg_k <= r_avg_d2 * ratio_k;
        g_avg_k <= g_avg_d2 * ratio_k;
        b_avg_k <= b_avg_d2 * ratio_k;
        
        // abs(R - G) * 100
        if (r_avg_d2 > g_avg_d2) diff_rg_100 <= (r_avg_d2 - g_avg_d2) * 8'd100;
        else                     diff_rg_100 <= (g_avg_d2 - r_avg_d2) * 8'd100;
        
        // Max_RGB * 0.15 等效为 Max_RGB * 15 (因为前面 abs 放大了 100 倍，所以这里是对应放大的比例)
        max_rgb_15 <= max_rgb_d1 * 8'd15;
    end
end

// =================================================================
// Stage 6: 评估各颜色 Mask 的逻辑条件
// =================================================================
reg mask_blue, mask_green, mask_yellow;
reg valid_color_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mask_blue <= 1'b0; mask_green <= 1'b0; mask_yellow <= 1'b0;
        valid_color_d1 <= 1'b0;
    end else if (clken_d[4]) begin
        mask_blue   <= (b_avg_100 > r_avg_k) && (b_avg_100 > g_avg_k)
            && (delta > 8'd40);
        mask_green  <= (g_avg_100 > r_avg_k) && (g_avg_100 > b_avg_k)
            && (delta > 8'd25);
        mask_yellow <= (r_avg_100 > b_avg_k) && (g_avg_100 > b_avg_k) 
            && (diff_rg_100 < max_rgb_15) &&  (delta > 8'd60);
        
        valid_color_d1 <= valid_color;
    end
end

// =================================================================
// Stage 7: 输出标签选择
// =================================================================
reg [1:0] color_mask_out;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        color_mask_out <= 2'd0;
    end else if (clken_d[5]) begin
        if (valid_color_d1) begin
            if (mask_blue)       color_mask_out <= 2'd1;
            else if (mask_green) color_mask_out <= 2'd2;
            else if (mask_yellow)color_mask_out <= 2'd3;
            else                 color_mask_out <= 2'd0;
        end else begin
            color_mask_out <= 2'd0;
        end
    end
end

// =================================================================
// 最终对齐输出绑定
// =================================================================
assign color_mask_frame_vsync = vsync_d[6];
assign color_mask_frame_href  = href_d[6];
assign color_mask_frame_clken = clken_d[6];
assign color_mask_img_mask    = color_mask_out;

endmodule