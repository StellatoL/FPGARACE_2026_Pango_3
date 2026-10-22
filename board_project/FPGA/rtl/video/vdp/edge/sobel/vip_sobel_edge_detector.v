module vip_sobel_edge_detector #(
    parameter DATA_WIDTH      = 8,   //输入图像数据位宽
    parameter IMG_HDISP       = 1024,//图像水平分辨率
    parameter IMG_VDISP       = 600  //图像垂直分辨率
)(
    input       clk,             //cmos 像素时钟
    input       rst_n,  
    //处理前数据
    input       pre_frame_vsync, 
    input       pre_frame_href,  
    input       pre_frame_clken, 
    input [7:0] matrix_p11, matrix_p12, matrix_p13,
    input [7:0] matrix_p21, matrix_p22, matrix_p23,
    input [7:0] matrix_p31, matrix_p32, matrix_p33,

    //处理后的数据
    output      post_frame_vsync, 
    output      post_frame_href,  
    output      post_frame_clken,
    output[7:0] post_img_y    
);

localparam LATENCY = 15; //总延迟周期数，包含矩阵生成和Sobel计算的所有步骤

//reg define 
reg [9:0]  gx_temp2; //第三列值
reg [9:0]  gx_temp1; //第一列值
reg [9:0]  gx_data;  //x方向的偏导数
reg [9:0]  gy_temp1; //第一行值
reg [9:0]  gy_temp2; //第三行值
reg [9:0]  gy_data;  //y方向的偏导数
reg [20:0] gxy_square;
reg [LATENCY - 1:0] pre_frame_vsync_r;
reg [LATENCY - 1:0] pre_frame_href_r;
reg [LATENCY - 1:0] pre_frame_clken_r;


//wire define 
wire        matrix_frame_vsync;
wire        matrix_frame_href;
wire        matrix_frame_clken;
wire [10:0] dim;

//*****************************************************
//**                    main code
//*****************************************************
assign post_frame_vsync = pre_frame_vsync_r[LATENCY - 1];
assign post_frame_href  = pre_frame_href_r[LATENCY - 1] ;
assign post_frame_clken = pre_frame_clken_r[LATENCY - 1];
assign post_img_y       = post_frame_href ? dim[7:0] : 8'd0;

//Sobel 算子
//         gx                  gy                  像素点
// [   -1  0   +1  ]   [   +1  +2   +1 ]     [   P11  P12   P13 ]
// [   -2  0   +2  ]   [   0   0    0  ]     [   P21  P22   P23 ]
// [   -1  0   +1  ]   [   -1  -2   -1 ]     [   P31  P32   P33 ]

//Step 1 计算x方向的偏导数
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        gy_temp1 <= 10'd0;
        gy_temp2 <= 10'd0;
        gy_data <=  10'd0;
    end
    else begin
        gy_temp1 <= matrix_p13 + (matrix_p23 << 1) + matrix_p33; 
        gy_temp2 <= matrix_p11 + (matrix_p21 << 1) + matrix_p31; 
        gy_data <= (gy_temp1 >= gy_temp2) ? gy_temp1 - gy_temp2 : 
                   (gy_temp2 - gy_temp1);
    end
end

//Step 2 计算y方向的偏导数
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        gx_temp1 <= 10'd0;
        gx_temp2 <= 10'd0;
        gx_data <=  10'd0;
    end
    else begin
        gx_temp1 <= matrix_p11 + (matrix_p12 << 1) + matrix_p13; 
        gx_temp2 <= matrix_p31 + (matrix_p32 << 1) + matrix_p33; 
        gx_data <= (gx_temp1 >= gx_temp2) ? gx_temp1 - gx_temp2 : 
                   (gx_temp2 - gx_temp1);
    end
end

//Step 3 计算平方和
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        gxy_square <= 21'd0;
    else
        gxy_square <= gx_data * gx_data + gy_data * gy_data;
end

//Step 4 开平方（梯度向量的大小） delay 12 cycle
sqrt 
   #(     
      .d_width (22),
      .q_width (11),
      .r_width (11) 
    )
   u_sqrt(
      .clk     (clk              ),
      .rst     (~rst_n           ),
      .i_vaild (1                ),
      .data_i  ({1'b0,gxy_square}), //输入
      
      .o_vaild (                 ),
      .data_o  (dim              ), //输出
      .data_r  (                 )  //余数
);  

//延迟15个周期同步
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        pre_frame_vsync_r <= 0;
        pre_frame_href_r  <= 0;
        pre_frame_clken_r <= 0;
    end
    else begin
        pre_frame_vsync_r  <=  {pre_frame_vsync_r[LATENCY - 2:0],pre_frame_vsync};
        pre_frame_href_r   <=  {pre_frame_href_r[LATENCY - 2:0] ,pre_frame_href };
        pre_frame_clken_r  <=  {pre_frame_clken_r[LATENCY - 2:0],pre_frame_clken};
    end
end

endmodule 