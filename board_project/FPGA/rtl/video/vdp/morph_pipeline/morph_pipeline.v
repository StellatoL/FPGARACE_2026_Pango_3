module morph_pipeline #(
    parameter DATA_WIDTH = 1,     // 二值化图像，数据位宽必须为 1
    parameter IMG_HDISP  = 1280,
    parameter IMG_VDISP  = 720,
    parameter DEPTH      = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input                   clk,
    input                   rst_n,

    input                   pre_frame_vsync,    // 输入的场同步信号
    input                   pre_frame_href,     // 输入的行同步信号
    input                   pre_frame_clken,    // 输入的数据使能信号
    input                   pre_img_bit,        // 输入的二值图像数据 (1-bit)

    output                  post_frame_vsync,   // 输出的场同步信号
    output                  post_frame_href,    // 输出的行同步信号
    output                  post_frame_clken,   // 输出的数据使能信号
    output                  post_img_bit        // 输出的处理后图像数据
);

// =====================================================================
// Step 1: 3x3 膨胀处理 (Dilation) -> 消除小的内部孔洞或连接断裂的字符
// =====================================================================
wire matrix1_vsync, matrix1_href, matrix1_clken;
wire m1_p11, m1_p12, m1_p13;
wire m1_p21, m1_p22, m1_p23;
wire m1_p31, m1_p32, m1_p33;

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH), 
    .DEPTH(DEPTH)
) u_matrix_3x3 (
    .clk                (clk), 
    .rst_n              (rst_n),
    .pre_frame_vsync    (pre_frame_vsync),
    .pre_frame_href     (pre_frame_href),
    .pre_frame_clken    (pre_frame_clken),
    .pre_img_y          (pre_img_bit),
    
    .matrix_frame_vsync (matrix1_vsync),
    .matrix_frame_href  (matrix1_href),
    .matrix_frame_clken (matrix1_clken),
    
    .matrix_p11(m1_p11), .matrix_p12(m1_p12), .matrix_p13(m1_p13),
    .matrix_p21(m1_p21), .matrix_p22(m1_p22), .matrix_p23(m1_p23),
    .matrix_p31(m1_p31), .matrix_p32(m1_p32), .matrix_p33(m1_p33)
);

reg dilate1_vsync, dilate1_href, dilate1_clken, dilate1_img_bit;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dilate1_vsync <= 0; dilate1_href <= 0; dilate1_clken <= 0; dilate1_img_bit <= 0;
    end else begin
        dilate1_vsync <= matrix1_vsync;
        dilate1_href  <= matrix1_href;
        dilate1_clken <= matrix1_clken;
        // 膨胀 = 窗口内逻辑或
        if (matrix1_clken)
            dilate1_img_bit <= |{m1_p11, m1_p12, m1_p13, 
                                 m1_p21, m1_p22, m1_p23, 
                                 m1_p31, m1_p32, m1_p33};
        else
            dilate1_img_bit <= 0;
    end
end


// =====================================================================
// Step 2: 4x4 腐蚀处理 (Erosion) -> 消除车牌外部的细小孤立噪点
// =====================================================================
wire matrix2_vsync, matrix2_href, matrix2_clken;
wire m2_p11, m2_p12, m2_p13, m2_p14;
wire m2_p21, m2_p22, m2_p23, m2_p24;
wire m2_p31, m2_p32, m2_p33, m2_p34;
wire m2_p41, m2_p42, m2_p43, m2_p44;

vip_matrix_generate_4x4_8bit #(
    .DATA_WIDTH(DATA_WIDTH), 
    .DEPTH(DEPTH)
) u_matrix_4x4 (
    .clk                (clk), 
    .rst_n              (rst_n),
    .pre_frame_vsync    (dilate1_vsync), // 接入上一级的信号
    .pre_frame_href     (dilate1_href),
    .pre_frame_clken    (dilate1_clken),
    .pre_img_y          (dilate1_img_bit),
    
    .matrix_frame_vsync (matrix2_vsync),
    .matrix_frame_href  (matrix2_href),
    .matrix_frame_clken (matrix2_clken),
    
    .matrix_p11(m2_p11), .matrix_p12(m2_p12), .matrix_p13(m2_p13), .matrix_p14(m2_p14),
    .matrix_p21(m2_p21), .matrix_p22(m2_p22), .matrix_p23(m2_p23), .matrix_p24(m2_p24),
    .matrix_p31(m2_p31), .matrix_p32(m2_p32), .matrix_p33(m2_p33), .matrix_p34(m2_p34),
    .matrix_p41(m2_p41), .matrix_p42(m2_p42), .matrix_p43(m2_p43), .matrix_p44(m2_p44)
);

reg erode1_vsync, erode1_href, erode1_clken, erode1_img_bit;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        erode1_vsync <= 0; erode1_href <= 0; erode1_clken <= 0; erode1_img_bit <= 0;
    end else begin
        erode1_vsync <= matrix2_vsync;
        erode1_href  <= matrix2_href;
        erode1_clken <= matrix2_clken;
        // 腐蚀 = 窗口内逻辑与
        if (matrix2_clken)
            erode1_img_bit <= &{m2_p11, m2_p12, m2_p13, m2_p14, 
                                m2_p21, m2_p22, m2_p23, m2_p24,
                                m2_p31, m2_p32, m2_p33, m2_p34,
                                m2_p41, m2_p42, m2_p43, m2_p44};
        else
            erode1_img_bit <= 0;
    end
end


// =====================================================================
// Step 3: 3x18 膨胀处理 (Horizontal Dilation) -> 将车牌字符水平连通成一个大色块
// =====================================================================
wire matrix3_vsync, matrix3_href, matrix3_clken;

// 使用位拼接数组接收 54 个像素点，避免代码过长
wire [17:0] m3_row1, m3_row2, m3_row3;

vip_matrix_generate_3x18_8bit #(
    .DATA_WIDTH(DATA_WIDTH), 
    .DEPTH(DEPTH)
) u_matrix_3x18 (
    .clk                (clk), 
    .rst_n              (rst_n),
    .pre_frame_vsync    (erode1_vsync), // 接入上一级的信号
    .pre_frame_href     (erode1_href),
    .pre_frame_clken    (erode1_clken),
    .pre_img_y          (erode1_img_bit),
    
    .matrix_frame_vsync (matrix3_vsync),
    .matrix_frame_href  (matrix3_href),
    .matrix_frame_clken (matrix3_clken),
    
    // 第 1 行
    .matrix_p1_01(m3_row1[17]), .matrix_p1_02(m3_row1[16]), .matrix_p1_03(m3_row1[15]), .matrix_p1_04(m3_row1[14]), .matrix_p1_05(m3_row1[13]), .matrix_p1_06(m3_row1[12]),
    .matrix_p1_07(m3_row1[11]), .matrix_p1_08(m3_row1[10]), .matrix_p1_09(m3_row1[9]),  .matrix_p1_10(m3_row1[8]),  .matrix_p1_11(m3_row1[7]),  .matrix_p1_12(m3_row1[6]),
    .matrix_p1_13(m3_row1[5]),  .matrix_p1_14(m3_row1[4]),  .matrix_p1_15(m3_row1[3]),  .matrix_p1_16(m3_row1[2]),  .matrix_p1_17(m3_row1[1]),  .matrix_p1_18(m3_row1[0]),
    
    // 第 2 行
    .matrix_p2_01(m3_row2[17]), .matrix_p2_02(m3_row2[16]), .matrix_p2_03(m3_row2[15]), .matrix_p2_04(m3_row2[14]), .matrix_p2_05(m3_row2[13]), .matrix_p2_06(m3_row2[12]),
    .matrix_p2_07(m3_row2[11]), .matrix_p2_08(m3_row2[10]), .matrix_p2_09(m3_row2[9]),  .matrix_p2_10(m3_row2[8]),  .matrix_p2_11(m3_row2[7]),  .matrix_p2_12(m3_row2[6]),
    .matrix_p2_13(m3_row2[5]),  .matrix_p2_14(m3_row2[4]),  .matrix_p2_15(m3_row2[3]),  .matrix_p2_16(m3_row2[2]),  .matrix_p2_17(m3_row2[1]),  .matrix_p2_18(m3_row2[0]),
    
    // 第 3 行
    .matrix_p3_01(m3_row3[17]), .matrix_p3_02(m3_row3[16]), .matrix_p3_03(m3_row3[15]), .matrix_p3_04(m3_row3[14]), .matrix_p3_05(m3_row3[13]), .matrix_p3_06(m3_row3[12]),
    .matrix_p3_07(m3_row3[11]), .matrix_p3_08(m3_row3[10]), .matrix_p3_09(m3_row3[9]),  .matrix_p3_10(m3_row3[8]),  .matrix_p3_11(m3_row3[7]),  .matrix_p3_12(m3_row3[6]),
    .matrix_p3_13(m3_row3[5]),  .matrix_p3_14(m3_row3[4]),  .matrix_p3_15(m3_row3[3]),  .matrix_p3_16(m3_row3[2]),  .matrix_p3_17(m3_row3[1]),  .matrix_p3_18(m3_row3[0])
);

// --- 第 1 拍：计算每一行的局部结果 ---
reg row1_dilate, row2_dilate, row3_dilate;
reg stage1_vsync, stage1_href, stage1_clken;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row1_dilate <= 0; row2_dilate <= 0; row3_dilate <= 0;
        stage1_vsync <= 0; stage1_href <= 0; stage1_clken <= 0;
    end else begin
        // 同步信号打拍跟随
        stage1_vsync <= matrix3_vsync;
        stage1_href  <= matrix3_href;
        stage1_clken <= matrix3_clken;
        
        // 18 个数相或，综合出来只需要 1 层 6-LUT 结构，速度极快
        if (matrix3_clken) begin
            row1_dilate <= |m3_row1;
            row2_dilate <= |m3_row2;
            row3_dilate <= |m3_row3;
        end else begin
            row1_dilate <= 0; row2_dilate <= 0; row3_dilate <= 0;
        end
    end
end

// --- 第 2 拍：计算全局结果 ---
reg final_vsync, final_href, final_clken, final_img_bit;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        final_vsync <= 0; final_href <= 0; final_clken <= 0; final_img_bit <= 0;
    end else begin
        final_vsync <= stage1_vsync;
        final_href  <= stage1_href;
        final_clken <= stage1_clken;
        
        // 3 个数相或，闭着眼睛跑 500MHz 都没问题
        if (stage1_clken)
            final_img_bit <= row1_dilate | row2_dilate | row3_dilate;
        else
            final_img_bit <= 0;
    end
end

// =====================================================================
// 最终对齐输出
// =====================================================================
assign post_frame_vsync = final_vsync;
assign post_frame_href  = final_href;
assign post_frame_clken = final_clken;
assign post_img_bit     = final_img_bit;

endmodule