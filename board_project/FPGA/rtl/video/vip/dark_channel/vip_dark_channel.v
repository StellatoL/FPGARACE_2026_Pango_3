//计算彩色图像的暗通道，共消耗3个时钟周期
`timescale 1ns/1ns
module VIP_Dark_Channel #(
    parameter IMG_HDISP = 10'd640,
    parameter IMG_VDISP = 10'd640
)
(
    //global clock
    input               clk,
    input               rst_n,

    input               per_frame_vsync,
    input               per_frame_href,
    input               per_frame_clken,
    input       [7:0]   per_img_red,
    input       [7:0]   per_img_green,
    input       [7:0]   per_img_blue,

    output              post_frame_vsync,
    output              post_frame_href,
    output              post_frame_clken,
    output      [7:0]   post_img_Dark       //暗通道图像
);

//-----------------------------------------
//VIP 算法——计算RGB三个通道中的最小值，消耗一个时钟
wire            post0_frame_vsync;
wire            post0_frame_href;
wire            post0_frame_clken;
wire    [7:0]   post0_RGB_MIN;      //RGB三个通道中的最小值

VIP_RGB888_MIN u_VIP_RGB888_MIN
(
    //global clock
    .clk                (clk),
    .rst_n              (rst_n),

    //Image data prepred to be processd
    .per_frame_vsync    (per_frame_vsync),
    .per_frame_href     (per_frame_href),
    .per_frame_clken    (per_frame_clken),
    .per_img_red        (per_img_red),
    .per_img_green      (per_img_green),
    .per_img_blue       (per_img_blue),

    //Image data has been processd
    .post_frame_vsync   (post0_frame_vsync),
    .post_frame_href    (post0_frame_href),
    .post_frame_clken   (post0_frame_clken),
    .post_RGB_MIN       (post0_RGB_MIN)     //RGB三个通道中的最小值
);

//-----------------------------------------
wire            post1_frame_vsync;
wire            post1_frame_href;
wire            post1_frame_clken;
wire    [7:0]   post1_img_Y;

// 最小值滤波，共消耗2个时钟周期
VIP_Gray_Minimum_Filter #(
    .IMG_HDISP          (IMG_HDISP),
    .IMG_VDISP          (IMG_VDISP)
)
u_VIP_Gray_Minimum_Filter (
    .clk                (clk),
    .rst_n              (rst_n),

    //Image data prepred to be processd
    .pre_frame_vsync    (post0_frame_vsync),
    .pre_frame_href     (post0_frame_href),
    .pre_frame_clken    (post0_frame_clken),
    .pre_img_y          (post0_RGB_MIN),

    //Image data has been processd
    .pos_frame_vsync   (post1_frame_vsync),
    .pos_frame_href    (post1_frame_href),
    .pos_frame_clken   (post1_frame_clken),
    .pos_img_y         (post1_img_Y)
);

assign post_frame_vsync = post1_frame_vsync;
assign post_frame_href  = post1_frame_href;
assign post_frame_clken = post1_frame_clken;
assign post_img_Dark    = post1_img_Y;

endmodule