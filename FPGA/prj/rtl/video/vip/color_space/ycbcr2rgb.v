module ycbcr2rgb
(
    //module clock
    input               clk             ,   // 模块驱动时钟
    input               rst_n           ,   // 复位信号

    //图像处理前的数据接口
    input               pre_frame_vsync ,   // vsync信号
    input               pre_frame_href  ,   // hsync信号
    input               pre_frame_de    ,   // data enable信号
    input       [7:0]   img_y           ,   // 输入图像Y数据
    input       [7:0]   img_cb          ,   // 输入图像Cb数据
    input       [7:0]   img_cr          ,   // 输入图像Cr数据

    //图像处理后的数据接口
    output              post_frame_vsync,   // vsync信号
    output              post_frame_href ,   // hsync信号
    output              post_frame_de   ,   // data enable信号
    output      [7:0]   img_red         ,   // 输出图像数据R
    output      [7:0]   img_green       ,   // 输出图像数据G
    output      [7:0]   img_blue            // 输出图像数据B
);

//*****************************************************
//**                    main code
//*****************************************************

//--------------------------------------------
//YCbCr to RGB 888

/********************************************************
            YCbCr to RGB 888
R = Y + 1.402    * (Cr - 128)
G = Y - 0.344136 * (Cb - 128) - 0.714136 * (Cr - 128)
B = Y + 1.772    * (Cb - 128)

R = Y + ((359 * (Cr - 128)) >> 8)
G = Y - (( 88 * (Cb - 128) + 183 * (Cr - 128)) >> 8)
B = Y + ((454 * (Cb - 128)) >> 8)

*********************************************************/
reg  signed [8:0]  Cr_128,Cb_128;
reg         [7:0]  img_y_d1, img_y_d2;
reg                pre_frame_vsync_d1, pre_frame_vsync_d2, pre_frame_vsync_d3;
reg                pre_frame_href_d1 , pre_frame_href_d2 , pre_frame_href_d3 ;
reg                pre_frame_de_d1   , pre_frame_de_d2   , pre_frame_de_d3   ;

// step1
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        Cr_128 <= 9'sd0;
        Cb_128 <= 9'sd0;
    end
    else begin
        Cr_128 <= $signed({1'b0, img_cr}) - 9'sd128;
        Cb_128 <= $signed({1'b0, img_cb}) - 9'sd128;
    end
end

reg signed [18:0]  R_Cr, G_Cb, G_Cr, B_Cb;
wire signed [19:0] G_sum;
wire signed [18:0] Cr_128_ext;
wire signed [18:0] Cb_128_ext;

assign Cr_128_ext = {{10{Cr_128[8]}}, Cr_128};
assign Cb_128_ext = {{10{Cb_128[8]}}, Cb_128};

// step2
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        R_Cr <= 19'sd0;
        G_Cb <= 19'sd0;
        G_Cr <= 19'sd0;
        B_Cb <= 19'sd0;
    end
    else begin
        // 359 = 256 + 64 + 32 + 4 + 2 + 1
        R_Cr <= (Cr_128_ext <<< 8) + (Cr_128_ext <<< 6) + (Cr_128_ext <<< 5) + (Cr_128_ext <<< 2) + (Cr_128_ext <<< 1) + Cr_128_ext;
        // 88 = 64 + 16 + 8
        G_Cb <= (Cb_128_ext <<< 6) + (Cb_128_ext <<< 4) + (Cb_128_ext <<< 3);
        // 183 = 128 + 32 + 16 + 4 + 2 + 1
        G_Cr <= (Cr_128_ext <<< 7) + (Cr_128_ext <<< 5) + (Cr_128_ext <<< 4) + (Cr_128_ext <<< 2) + (Cr_128_ext <<< 1) + Cr_128_ext;
        // 454 = 256 + 128 + 64 + 4 + 2
        B_Cb <= (Cb_128_ext <<< 8) + (Cb_128_ext <<< 7) + (Cb_128_ext <<< 6) + (Cb_128_ext <<< 2) + (Cb_128_ext <<< 1);
    end
end

assign G_sum = $signed({G_Cb[18], G_Cb}) + $signed({G_Cr[18], G_Cr});

reg signed [10:0]  R, G, B;
// step3
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        R <= 11'sd0;
        G <= 11'sd0;
        B <= 11'sd0;
    end
    else begin
        R <= $signed({3'b000, img_y_d2}) + (R_Cr >>> 8);
        G <= $signed({3'b000, img_y_d2}) - (G_sum >>> 8);
        B <= $signed({3'b000, img_y_d2}) + (B_Cb >>> 8);
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        img_y_d1 <= 8'd0;
        img_y_d2 <= 8'd0;
        pre_frame_vsync_d1 <= 1'b0;
        pre_frame_vsync_d2 <= 1'b0;
        pre_frame_vsync_d3 <= 1'b0;
        pre_frame_href_d1  <= 1'b0;
        pre_frame_href_d2  <= 1'b0;
        pre_frame_href_d3  <= 1'b0;
        pre_frame_de_d1    <= 1'b0;
        pre_frame_de_d2    <= 1'b0;
        pre_frame_de_d3    <= 1'b0;
    end
    else begin
        img_y_d1 <= img_y;
        img_y_d2 <= img_y_d1;
        pre_frame_vsync_d1 <= pre_frame_vsync;
        pre_frame_vsync_d2 <= pre_frame_vsync_d1;
        pre_frame_vsync_d3 <= pre_frame_vsync_d2;
        pre_frame_href_d1  <= pre_frame_href;
        pre_frame_href_d2  <= pre_frame_href_d1;
        pre_frame_href_d3  <= pre_frame_href_d2;
        pre_frame_de_d1    <= pre_frame_de;
        pre_frame_de_d2    <= pre_frame_de_d1;
        pre_frame_de_d3    <= pre_frame_de_d2;
    end
end

wire [7:0] R_sat;
wire [7:0] G_sat;
wire [7:0] B_sat;

assign R_sat = (R < 11'sd0) ? 8'd0 : ((R > 11'sd255) ? 8'd255 : R[7:0]);
assign G_sat = (G < 11'sd0) ? 8'd0 : ((G > 11'sd255) ? 8'd255 : G[7:0]);
assign B_sat = (B < 11'sd0) ? 8'd0 : ((B > 11'sd255) ? 8'd255 : B[7:0]);

assign img_red   = R_sat;
assign img_green = G_sat;
assign img_blue  = B_sat;

assign post_frame_vsync = pre_frame_vsync_d3;
assign post_frame_href  = pre_frame_href_d3;
assign post_frame_de    = pre_frame_de_d3;

endmodule
