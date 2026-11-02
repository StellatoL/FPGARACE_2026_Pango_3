// 顶层功能说明：
// 1) 输入 RGB888 视频流
// 2) 暗通道支路计算 dark_img；
// 3) 基于暗通道和对应像素估计全局大气光 A；
// 4) 结合 A 与暗通道求透射率 t；
// 5) 使用 t 和 A 进行场景辐射恢复。
//
// 时序说明：
// - 各子模块均采用流接口(vsync/href/clken + 像素)；
// - 不同支路流水线深度不同，因此在本模块中对原始 RGB 做 1~8 拍延迟，
//   以保证在“同一有效像素时刻”参与同一公式计算。
module vip_dark_channel_top #(
    parameter IMG_HDISP = 640,
    parameter IMG_VDISP = 640,
    parameter DEPTH     = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input            clk              ,
    input            rst_n            ,
    input            enable           ,

    //图像处理前的数据接口
    input            pre_frame_vsync  ,
    input            pre_frame_href   ,
    input            pre_frame_de     ,
    input     [23:0] pre_rgb          ,

    //图像处理后的数据接口
    output           post_frame_vsync ,   // 场同步信号
    output           post_frame_href  ,   // 行同步信号
    output           post_frame_de    ,   // 数据输入使能
    output    [23:0] post_rgb             // RGB888颜色数据
);

//--------------------------------------------------------------------------------------
// RGB/同步信号延迟链
// 作用：对其最小值滤波中的3x3ram进行同像素对齐，保证后续计算的正确性。
// 直接与最小值滤波模块的3x3生成模块一样生成，保证时序一致。
wire [23:0] mult_rgb;
wire [7:0] pre_img_red;
wire [7:0] pre_img_green;
wire [7:0] pre_img_blue;

assign pre_img_red   = mult_rgb[23:16];
assign pre_img_green = mult_rgb[15: 8];
assign pre_img_blue  = mult_rgb[ 7: 0];

wire matrix_frame_vsync;
wire matrix_frame_href;
wire matrix_frame_clken;

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(24),
    .DEPTH(DEPTH)
)matrix_generate_3x3_inst(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .pre_frame_vsync    (pre_frame_vsync),
    .pre_frame_href     (pre_frame_href),
    .pre_frame_clken    (pre_frame_de),
    .pre_img_y          (pre_rgb),
    
    //处理后的图像数据
    .matrix_frame_vsync (matrix_frame_vsync),
    .matrix_frame_href  (matrix_frame_href),
    .matrix_frame_clken (matrix_frame_clken),
    .matrix_p11         (),
    .matrix_p12         (),
    .matrix_p13         (),
    .matrix_p21         (),
    .matrix_p22         (mult_rgb),
    .matrix_p23         (),
    .matrix_p31         (),
    .matrix_p32         (),
    .matrix_p33         ()

);

//--------------------------------------------------------------------------------------
// RGB/同步信号延迟链（1~7 拍）
// 作用：
// - d3：用于给大气光估计模块提供与 dark_img 对齐的原始 RGB；
// - d7：用于给场景辐射恢复模块提供与 transmission 对齐的原始 RGB；
// - vsync/href/de 同步延迟用于调试观测和后续可能的对齐扩展。
reg [7:0] pre_img_red_d1;
reg [7:0] pre_img_red_d2;
reg [7:0] pre_img_red_d3;
reg [7:0] pre_img_red_d4;
reg [7:0] pre_img_red_d5;
reg [7:0] pre_img_red_d6;
reg [7:0] pre_img_red_d7;

reg [7:0] pre_img_green_d1;
reg [7:0] pre_img_green_d2;
reg [7:0] pre_img_green_d3;
reg [7:0] pre_img_green_d4;
reg [7:0] pre_img_green_d5;
reg [7:0] pre_img_green_d6;
reg [7:0] pre_img_green_d7;

reg [7:0] pre_img_blue_d1;
reg [7:0] pre_img_blue_d2;
reg [7:0] pre_img_blue_d3;
reg [7:0] pre_img_blue_d4;
reg [7:0] pre_img_blue_d5;
reg [7:0] pre_img_blue_d6;
reg [7:0] pre_img_blue_d7;

reg pre_frame_vsync_d1;
reg pre_frame_vsync_d2;
reg pre_frame_vsync_d3;
reg pre_frame_vsync_d4;
reg pre_frame_vsync_d5;
reg pre_frame_vsync_d6;
reg pre_frame_vsync_d7;

reg pre_frame_href_d1;
reg pre_frame_href_d2;
reg pre_frame_href_d3;
reg pre_frame_href_d4;
reg pre_frame_href_d5;
reg pre_frame_href_d6;
reg pre_frame_href_d7;

reg pre_frame_de_d1;
reg pre_frame_de_d2;
reg pre_frame_de_d3;
reg pre_frame_de_d4;
reg pre_frame_de_d5;
reg pre_frame_de_d6;
reg pre_frame_de_d7;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pre_img_red_d1   <= 8'd0;
        pre_img_red_d2   <= 8'd0;
        pre_img_red_d3   <= 8'd0;
        pre_img_red_d4   <= 8'd0;
        pre_img_red_d5   <= 8'd0;
        pre_img_red_d6   <= 8'd0;
        pre_img_red_d7   <= 8'd0;

        pre_img_green_d1 <= 8'd0;
        pre_img_green_d2 <= 8'd0;
        pre_img_green_d3 <= 8'd0;
        pre_img_green_d4 <= 8'd0;
        pre_img_green_d5 <= 8'd0;
        pre_img_green_d6 <= 8'd0;
        pre_img_green_d7 <= 8'd0;

        pre_img_blue_d1  <= 8'd0;
        pre_img_blue_d2  <= 8'd0;
        pre_img_blue_d3  <= 8'd0;
        pre_img_blue_d4  <= 8'd0;
        pre_img_blue_d5  <= 8'd0;
        pre_img_blue_d6  <= 8'd0;
        pre_img_blue_d7  <= 8'd0;

        pre_frame_vsync_d1 <= 1'b0;
        pre_frame_vsync_d2 <= 1'b0;
        pre_frame_vsync_d3 <= 1'b0;
        pre_frame_vsync_d4 <= 1'b0;
        pre_frame_vsync_d5 <= 1'b0;
        pre_frame_vsync_d6 <= 1'b0;
        pre_frame_vsync_d7 <= 1'b0;

        pre_frame_href_d1 <= 1'b0;
        pre_frame_href_d2 <= 1'b0;
        pre_frame_href_d3 <= 1'b0;
        pre_frame_href_d4 <= 1'b0;
        pre_frame_href_d5 <= 1'b0;
        pre_frame_href_d6 <= 1'b0;
        pre_frame_href_d7 <= 1'b0;

        pre_frame_de_d1 <= 1'b0;
        pre_frame_de_d2 <= 1'b0;
        pre_frame_de_d3 <= 1'b0;
        pre_frame_de_d4 <= 1'b0;
        pre_frame_de_d5 <= 1'b0;
        pre_frame_de_d6 <= 1'b0;
        pre_frame_de_d7 <= 1'b0;

    end
    else begin
        // 仅在有效像素周期移位，避免消隐区数据冲刷延迟链导致对齐漂移。
        if(!matrix_frame_href) begin
            pre_img_red_d1   <= 8'd0;
            pre_img_red_d2   <= 8'd0;
            pre_img_red_d3   <= 8'd0;
            pre_img_red_d4   <= 8'd0;
            pre_img_red_d5   <= 8'd0;
            pre_img_red_d6   <= 8'd0;
            pre_img_red_d7   <= 8'd0;

            pre_img_green_d1 <= 8'd0;
            pre_img_green_d2 <= 8'd0;
            pre_img_green_d3 <= 8'd0;
            pre_img_green_d4 <= 8'd0;
            pre_img_green_d5 <= 8'd0;
            pre_img_green_d6 <= 8'd0;
            pre_img_green_d7 <= 8'd0;

            pre_img_blue_d1  <= 8'd0;
            pre_img_blue_d2  <= 8'd0;
            pre_img_blue_d3  <= 8'd0;
            pre_img_blue_d4  <= 8'd0;
            pre_img_blue_d5  <= 8'd0;
            pre_img_blue_d6  <= 8'd0;
            pre_img_blue_d7  <= 8'd0;
        end
        else if(matrix_frame_clken) begin
            pre_img_red_d1   <= pre_img_red;
            pre_img_red_d2   <= pre_img_red_d1;
            pre_img_red_d3   <= pre_img_red_d2;
            pre_img_red_d4   <= pre_img_red_d3;
            pre_img_red_d5   <= pre_img_red_d4;
            pre_img_red_d6   <= pre_img_red_d5;
            pre_img_red_d7   <= pre_img_red_d6;

            pre_img_green_d1 <= pre_img_green;
            pre_img_green_d2 <= pre_img_green_d1;
            pre_img_green_d3 <= pre_img_green_d2;
            pre_img_green_d4 <= pre_img_green_d3;
            pre_img_green_d5 <= pre_img_green_d4;
            pre_img_green_d6 <= pre_img_green_d5;
            pre_img_green_d7 <= pre_img_green_d6;

            pre_img_blue_d1  <= pre_img_blue;
            pre_img_blue_d2  <= pre_img_blue_d1;
            pre_img_blue_d3  <= pre_img_blue_d2;
            pre_img_blue_d4  <= pre_img_blue_d3;
            pre_img_blue_d5  <= pre_img_blue_d4;
            pre_img_blue_d6  <= pre_img_blue_d5;
            pre_img_blue_d7  <= pre_img_blue_d6;

            pre_frame_vsync_d1 <= matrix_frame_vsync;
            pre_frame_vsync_d2 <= pre_frame_vsync_d1;
            pre_frame_vsync_d3 <= pre_frame_vsync_d2;
            pre_frame_vsync_d4 <= pre_frame_vsync_d3;
            pre_frame_vsync_d5 <= pre_frame_vsync_d4;
            pre_frame_vsync_d6 <= pre_frame_vsync_d5;
            pre_frame_vsync_d7 <= pre_frame_vsync_d6;

            pre_frame_href_d1 <= matrix_frame_href;
            pre_frame_href_d2 <= pre_frame_href_d1;
            pre_frame_href_d3 <= pre_frame_href_d2;
            pre_frame_href_d4 <= pre_frame_href_d3;
            pre_frame_href_d5 <= pre_frame_href_d4;
            pre_frame_href_d6 <= pre_frame_href_d5;
            pre_frame_href_d7 <= pre_frame_href_d6;

            pre_frame_de_d1 <= matrix_frame_clken;
            pre_frame_de_d2 <= pre_frame_de_d1;
            pre_frame_de_d3 <= pre_frame_de_d2;
            pre_frame_de_d4 <= pre_frame_de_d3;
            pre_frame_de_d5 <= pre_frame_de_d4;
            pre_frame_de_d6 <= pre_frame_de_d5;
            pre_frame_de_d7 <= pre_frame_de_d6;
        end
    end
end

//--------------------------------------------------------------------------------------
// 暗通道计算
// 输出 dark_img 表示局部窗口内 RGB 三通道最小值特征，
// 后续用于估计大气光和透射率。
// 共消耗3拍：1拍计算 RGB 最小值，2拍流水线延迟输出。
// 实际输出与输入值有三行的时钟延迟，因为最小值滤波需要5x5窗口，且输出时同步信号也有相应延迟，直接在前面用一个5x5的矩阵生成模块来对齐。
wire        dark_frame_vsync;
wire        dark_frame_href;
wire        dark_frame_clken;
wire [7:0]  dark_img;

VIP_Dark_Channel #(
    .IMG_HDISP      (IMG_HDISP),
    .IMG_VDISP      (IMG_VDISP)
) u_VIP_Dark_Channel (
    .clk                (clk),
    .rst_n              (rst_n),

    .per_frame_vsync    (pre_frame_vsync),
    .per_frame_href     (pre_frame_href),
    .per_frame_clken    (pre_frame_de),
    .per_img_red        (pre_rgb[23:16]),
    .per_img_green      (pre_rgb[15:8]),
    .per_img_blue       (pre_rgb[7:0]),

    .post_frame_vsync   (dark_frame_vsync),
    .post_frame_href    (dark_frame_href),
    .post_frame_clken   (dark_frame_clken),
    .post_img_Dark      (dark_img)
);

//--------------------------------------------------------------------------------------
// 大气光估计
// 关键点：
// - 输出实际为上一帧的大气光，因为估计过程需要遍历整帧像素；
// - 输入 dark_img 来自暗通道支路；
// - RGB 选择 d3 拍延迟，与 dark_img 的像素时刻对齐；
// - atmospheric_pos_x/y 输出可用于调试观测“被选为大气光参考点”的位置。
wire [7:0] atmospheric_light_raw;
wire [11:0] atmospheric_pos_x;
wire [11:0] atmospheric_pos_y;
wire [7:0] atmospheric_light_safe;

// 防零保护：A=0 会导致后续公式出现除零风险，这里钳位到最小 1。
assign atmospheric_light_safe = (atmospheric_light_raw == 8'd0) ? 8'd1 : atmospheric_light_raw;

VIP_Atmospheric_Light #(
    .IMG_HDISP      (IMG_HDISP),
    .IMG_VDISP      (IMG_VDISP)
) u_VIP_Atmospheric_Light (
    .clk                (clk),
    .rst_n              (rst_n),

    .per_frame_vsync    (dark_frame_vsync),
    .per_frame_href     (dark_frame_href),
    .per_frame_clken    (dark_frame_clken),
    .per_img_Dark       (dark_img),

    .per_img_red        (pre_img_red_d3),
    .per_img_green      (pre_img_green_d3),
    .per_img_blue       (pre_img_blue_d3),

    .atmospheric_light  (atmospheric_light_raw),
    .atmospheric_pos_x  (atmospheric_pos_x),
    .atmospheric_pos_y  (atmospheric_pos_y)
);


//--------------------------------------------------------------------------------------
// 透射率计算
// 典型模型：t(x)=1-w*Dark(x)/A。该模块内部完成具体定点实现，
// 本顶层仅负责提供对齐后的 dark_img 与安全大气光 A。
// 共消耗4拍：1拍乘法，1拍查表，1拍右移四舍五入，1拍最终计算并限制最小值。
wire        transmission_frame_vsync;
wire        transmission_frame_href;
wire        transmission_frame_clken;
wire [7:0]  transmission;

VIP_Transmission_Map u_VIP_Transmission_Map (
    .clk                (clk),
    .rst_n              (rst_n),

    .per_frame_vsync    (dark_frame_vsync),
    .per_frame_href     (dark_frame_href),
    .per_frame_clken    (dark_frame_clken),
    .per_img_Dark       (dark_img),

    .atmospheric_light  (atmospheric_light_safe),

    .post_frame_vsync   (transmission_frame_vsync),
    .post_frame_href    (transmission_frame_href),
    .post_frame_clken   (transmission_frame_clken),
    .post_transmission  (transmission)
);

//------------------------------------------------------------------------------
// 均值滤波
// 作用：对透射率图进行均值滤波，减轻噪声影响，提升视觉效果。
wire mean_frame_vsync;
wire mean_frame_href;
wire mean_frame_clken;
wire [7:0] mean_transmission;
vip_mean_filter_3x3 #(
    .DATA_WIDTH(8),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_vip_mean_filter_3x3 (
    .clk                (clk),
    .rst_n              (rst_n),

    .pre_frame_vsync    (transmission_frame_vsync),
    .pre_frame_href     (transmission_frame_href),
    .pre_frame_clken    (transmission_frame_clken),
    .pre_frame_data     (transmission),

    .pos_frame_vsync   (mean_frame_vsync),
    .pos_frame_href    (mean_frame_href),
    .pos_frame_clken   (mean_frame_clken),
    .pos_frame_data    (mean_transmission),

    .median_data       () // 均值滤波不输出 median_data，留空即可
);

// 对rgb数据进行3x3窗口延迟，保证与均值滤波输出的透射率时序对齐
wire [23:0] mean_rgb_d0;
reg  [23:0] mean_rgb_d1;
reg  [23:0] mean_rgb_d2;
reg  [23:0] mean_rgb_d3;

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(24),
    .DEPTH(DEPTH)
)vip_matrix_generate_3x3_24bit_inst(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .pre_frame_vsync    (transmission_frame_vsync),
    .pre_frame_href     (transmission_frame_href), 
    .pre_frame_clken    (transmission_frame_clken),
    .pre_img_y          ({pre_img_red_d7, pre_img_green_d7, pre_img_blue_d7}), // 选择 d7 拍延迟，对齐时序
    
    //处理后的图像数据
    .matrix_frame_vsync (),
    .matrix_frame_href  (),
    .matrix_frame_clken (), 
    .matrix_p11         (),
    .matrix_p12         (),
    .matrix_p13         (),
    .matrix_p21         (),
    .matrix_p22         (mean_rgb_d0),
    .matrix_p23         (),
    .matrix_p31         (),
    .matrix_p32         (),
    .matrix_p33         ()
);

// 继续打3拍延迟，保证与均值滤波输出的透射率时序对齐
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        mean_rgb_d1 <= 24'd0;
        mean_rgb_d2 <= 24'd0;
        mean_rgb_d3 <= 24'd0;
    end
    else begin
        if(transmission_frame_clken) begin
            mean_rgb_d1 <= mean_rgb_d0;
            mean_rgb_d2 <= mean_rgb_d1;
            mean_rgb_d3 <= mean_rgb_d2;
        end
        else begin
            mean_rgb_d1 <= mean_rgb_d1;
            mean_rgb_d2 <= mean_rgb_d2;
            mean_rgb_d3 <= mean_rgb_d3;
        end
    end
end

//--------------------------------------------------------------------------------------
// 场景辐射恢复
// 恢复公式常见形式：J(x)=(I(x)-A)/max(t(x),t0)+A。
// 关键时序：
// - transmission 来自更深流水线，输入 RGB 需选 d7 才能时刻对齐；
// - t 做防零保护后再参与除法，避免数值异常。
wire [7:0] scene_red;
wire [7:0] scene_green;
wire [7:0] scene_blue;
wire [7:0] transmission_safe;

// 防零保护：t=0 时将其钳位为 1，保证硬件定点除法安全。
assign transmission_safe = (mean_transmission == 8'd0) ? 8'd1 : mean_transmission;

wire pos1_frame_vsync;
wire pos1_frame_href;
wire pos1_frame_clken;

VIP_Scene_radiance #(
    .IMG_HDISP      (IMG_HDISP),
    .IMG_VDISP      (IMG_VDISP)
) u_VIP_Scene_radiance (
    .clk                (clk),
    .rst_n              (rst_n),

    .per_frame_vsync    (mean_frame_vsync),
    .per_frame_href     (mean_frame_href),
    .per_frame_clken    (mean_frame_clken),

    .per_img_red        (mean_rgb_d3[23:16]),
    .per_img_green      (mean_rgb_d3[15:8]),
    .per_img_blue       (mean_rgb_d3[7:0]),

    .pre_transmission   (transmission_safe),
    .atmospheric_light  (atmospheric_light_safe),

    .post_frame_vsync   (pos1_frame_vsync),
    .post_frame_href    (pos1_frame_href),
    .post_frame_clken   (pos1_frame_clken),
    .post_img_red       (scene_red),
    .post_img_green     (scene_green),
    .post_img_blue      (scene_blue)
);

//--------------------------------------------------------------------------------------

// Reset is not a functional bypass control. Disabled outputs equal inputs.
assign post_rgb         = enable ? {scene_red, scene_green, scene_blue} : pre_rgb;
assign post_frame_vsync = enable ? pos1_frame_vsync : pre_frame_vsync;
assign post_frame_href  = enable ? pos1_frame_href  : pre_frame_href;
assign post_frame_de    = enable ? pos1_frame_clken : pre_frame_de;
endmodule
