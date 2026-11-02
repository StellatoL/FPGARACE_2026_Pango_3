//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           vip
// Created by:          正点原子
// Created date:        2023年9月12日17:52:55
// Version:             V1.0
// Descriptions:        图像处理
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module vip#(
    parameter   IMG_HDISP = 1024,
    parameter   IMG_VDISP = 600
)(
    //module clock
    input           clk               ,  // 时钟信号
    input           rst_n             ,  // 复位信号（低有效）
    input    [ 3:0] vip_crtl          ,  // [3]: global, [2]: light_enhance [1]: dehaze, [0]: white balance

    //图像处理前的数据接口
    input           pre_frame_vsync   ,
    input           pre_frame_href    ,
    input           pre_frame_de      ,
    input    [15:0] pre_rgb           ,

    //图像处理后的数据接口
    output          post_frame_vsync  ,   // 场同步信号
    output          post_frame_href  ,    // 行同步信号
    output          post_frame_de     ,   // 数据输入使能
    output   [23:0] post_rgb              // RGB565颜色数据

);

//--------------------------------------------------------------------------------------
// RGB565 -> RGB888
// 说明：5/6/5 位扩展到 8 位，低位使用高位复制法补齐，

wire [7:0] pre0_img_red;
wire [7:0] pre0_img_green;
wire [7:0] pre0_img_blue;
wire [23:0] pre0_rgb;
wire        vip_enable;
wire        dehaze_enable;
wire        wb_enable;

assign pre0_img_red   = {pre_rgb[15:11], pre_rgb[15:13]};
assign pre0_img_green = {pre_rgb[10:5 ], pre_rgb[10:9 ]};
assign pre0_img_blue  = {pre_rgb[ 4:0 ], pre_rgb[ 4:2 ]};

assign vip_enable    = vip_crtl[3];
assign light_enhance_enable = vip_enable & vip_crtl[2];
assign dehaze_enable = vip_enable & vip_crtl[1];
assign wb_enable     = vip_enable & vip_crtl[0];

//*****************************************************
//**                    main code
//*****************************************************
// 白平衡补偿
wire [23:0] white_balance_rgb;
wire white_balance_frame_vsync;
wire white_balance_frame_href;
wire white_balance_frame_clken;

wire [7:0] white_balance_r;
wire [7:0] white_balance_g;
wire [7:0] white_balance_b;
image_awb #(
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_image_awb (
    .i_clk          (clk    ),           // 时钟信号
    .i_rst_n        (rst_n  ),           // 复位信号（低有效）
    .i_enable       (wb_enable),         // 白平衡使能信号
    //图像处理前的数据接口
    .i_hs           (pre_frame_href),    // href信号
    .i_vs           (pre_frame_vsync ),  // vsync信号
    .i_de           (pre_frame_de),      // data enable信号
    .i_pic_valid    (pre_frame_de),      // 图像数据有效使能信号
    .i_r            (pre0_img_red),      // 输入图像数据R
    .i_g            (pre0_img_green),    // 输入图像数据G
    .i_b            (pre0_img_blue),     // 输入图像数据B
    //图像处理后的数据接口
    .o_hs           (white_balance_frame_href),  // href信号
    .o_vs           (white_balance_frame_vsync), // vsync信号
    .o_de           (white_balance_frame_clken), // data enable信号
    .o_pic_valid    (),                  // 图像数据有效使能信号
    .o_r            (white_balance_r),   // 输出图像数据R
    .o_g            (white_balance_g),   // 输出图像数据G
    .o_b            (white_balance_b)    // 输出图像数据B
);

assign white_balance_rgb = {white_balance_r, white_balance_g, white_balance_b};


// 暗通道去雾
wire pos_frame_vsync;
wire pos_frame_href;
wire pos_frame_clken;
wire [23:0] pos_rgb;

vip_dark_channel_top #(
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_vip_dark_channel_top(
    .clk             (clk    ),                 // 时钟信号
    .rst_n           (rst_n  ),
    .enable          (dehaze_enable),
    // .enable          (1'b0),
    //图像处理前的数据接口
    .pre_frame_vsync (white_balance_frame_vsync),   // vsync信号
    .pre_frame_href  (white_balance_frame_href ),   // href信号
    .pre_frame_de    (white_balance_frame_clken),   // data enable信号
    .pre_rgb         (white_balance_rgb),           // 输出图像数据RGB

    //图像处理后的数据接口
    .post_frame_vsync(pos_frame_vsync),   // vsync信号
    .post_frame_href (pos_frame_href ),   // href信号
    .post_frame_de   (pos_frame_clken),   // data enable信号
    .post_rgb        (pos_rgb)            // 输出图像数据RGB
);

//RGB转YCbCr模块
wire  [ 7:0]         img_y;
wire  [ 7:0]         img_cb;
wire  [ 7:0]         img_cr;
wire                 pe_frame_vsync;
wire                 pe_frame_href;
wire                 pe_frame_clken;
rgb2ycbcr u_rgb2ycbcr(
    //module clock
    .clk             (clk    ),            // 时钟信号
    .rst_n           (rst_n  ),            // 复位信号(低有效)
    //图像处理前的数据接口
    .pre_frame_vsync (pos_frame_vsync),    // vsync信号
    .pre_frame_href  (pos_frame_href ),    // href信号
    .pre_frame_de    (pos_frame_clken),    // data enable信号
    .img_red         (pos_rgb[16+:8] ),
    .img_green       (pos_rgb[ 8+:8] ),
    .img_blue        (pos_rgb[ 0+:8] ),
    //图像处理后的数据接口
    .post_frame_vsync(pe_frame_vsync),     // vsync信号
    .post_frame_href (pe_frame_href),      // href信号
    .post_frame_de   (pe_frame_clken),     // data enable信号
    .img_y           (img_y),              //灰度数据
    .img_cb          (img_cb),
    .img_cr          (img_cr)
);

wire  [ 7:0]         img_y_light;
wire  [ 7:0]         img_cb_light;
wire  [ 7:0]         img_cr_light;
wire                 light_frame_vsync;
wire                 light_frame_href;
wire                 light_frame_clken;

light_enhance #(
    .DATA_WIDTH(8),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_light_enhance(
    .clk             (clk    ),            // 时钟信号
    .rst_n           (rst_n & vip_enable),
    .enable          (light_enhance_enable),
    
    //预处理图像数据
    .per_frame_vsync (pe_frame_vsync),     // vsync信号
    .per_frame_href  (pe_frame_href),      // href信号
    .per_frame_clken (pe_frame_clken),     // data enable信号
    .img_y_in        (img_y),              //灰度数据
    .img_cb_in       (img_cb),
    .img_cr_in       (img_cr),
    //处理后的图像数据
    .post_frame_vsync(light_frame_vsync),   // vsync信号
    .post_frame_href (light_frame_href ),   // href信号
    .post_frame_clken(light_frame_clken),   // data enable信号
    .img_y_out       (img_y_light),         //增强后的灰度数据
    .img_cb_out      (img_cb_light),
    .img_cr_out      (img_cr_light)
);

//双边滤波
wire  [ 7:0]         pos1_img_y;
wire  [ 7:0]         pos1_img_cb;
wire  [ 7:0]         pos1_img_cr;
wire                 pos1_frame_vsync;
wire                 pos1_frame_href;
wire                 pos1_frame_clken;
vip_bilateral_filter #(
    .DATA_WIDTH(8),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_vip_bilateral_filter(
    .clk             (clk),   
    .rst_n           (rst_n & vip_enable),
    .enable          (vip_enable),
    
    //预处理图像数据
    .pre_frame_vsync (light_frame_vsync),     // vsync信号
    .pre_frame_href  (light_frame_href ),      // href信号
    .pre_frame_clken (light_frame_clken),     // data enable信号
    .pre_img_y       (img_y_light),
    .pre_img_cb      (img_cb_light),
    .pre_img_cr      (img_cr_light),
    //处理后的图像数据
    .pos_frame_vsync (pos1_frame_vsync),   // vsync信号
    .pos_frame_href  (pos1_frame_href ),   // href信号
    .pos_frame_clken (pos1_frame_clken),   // data enable信号
    .pos_img_y       (pos1_img_y),         //双边滤波后的灰度数据
    .pos_img_cb      (pos1_img_cb),
    .pos_img_cr      (pos1_img_cr)
);

//YCbCr转RGB模块
wire [7:0] pos2_img_red;
wire [7:0] pos2_img_green;
wire [7:0] pos2_img_blue;
wire pos2_frame_vsync;
wire pos2_frame_href;
wire pos2_frame_clken;
ycbcr2rgb u_ycbcr2rgb(
    //module clock
    .clk             (clk    ),            // 时钟信号
    .rst_n           (rst_n  ),            // 复位信号（低有效）
    //图像处理前的数据接口
    .pre_frame_vsync (pos1_frame_vsync),   // vsync信号
    .pre_frame_href  (pos1_frame_href ),   // href信号
    .pre_frame_de    (pos1_frame_clken),// data enable信号
    .img_y           (pos1_img_y),         //CLAHE处理后的灰度数据
    .img_cb          (pos1_img_cb),
    .img_cr          (pos1_img_cr),
    //图像处理后的数据接口
    .post_frame_vsync(pos2_frame_vsync),   // vsync信号
    .post_frame_href (pos2_frame_href ),   // href信号
    .post_frame_de   (pos2_frame_clken),   // data enable信号
    .img_red         (pos2_img_red),       // 输出图像数据R
    .img_green       (pos2_img_green),
    .img_blue        (pos2_img_blue)
);


// 输出图像数据
// vip_crtl[2] is the global VIP enable. When it is clear, the whole VIP is a
// same-cycle RGB565-to-RGB888 wire-through and timing is copied unchanged.
assign post_rgb         = vip_enable ? {pos2_img_red,pos2_img_green,pos2_img_blue} : pre0_rgb;
assign post_frame_vsync = vip_enable ? pos2_frame_vsync : pre_frame_vsync;
assign post_frame_href  = vip_enable ? pos2_frame_href  : pre_frame_href;
assign post_frame_de    = vip_enable ? pos2_frame_clken : pre_frame_de;

endmodule
