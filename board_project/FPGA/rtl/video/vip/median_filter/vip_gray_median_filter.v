module vip_gray_median_filter#(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP  = 1024,
    parameter IMG_VDISP  = 600
)(
    //时钟
    input       clk,  //50MHz
    input       rst_n,
    
    //处理前图像数据
    input       pre_frame_vsync,  //处理前图像数据场信号
    input       pre_frame_href,   //处理前图像数据行信号 
    input       pre_frame_clken,  //处理前图像数据输入使能效信号
    input [DATA_WIDTH-1:0] pre_img_y,        //灰度数据             
    
    //处理后的图像数据
    output       pos_frame_vsync, //处理后的图像数据场信号   
    output       pos_frame_href,  //处理后的图像数据行信号  
    output       pos_frame_clken, //处理后的图像数据输出使能效信号
    output [DATA_WIDTH-1:0] pos_img_y        //处理后的灰度数据           
);

//wire define
wire        matrix_frame_vsync;
wire        matrix_frame_href;
wire        matrix_frame_clken;
wire [DATA_WIDTH-1:0]  matrix_p11; //3X3 阵列输出
wire [DATA_WIDTH-1:0]  matrix_p12; 
wire [DATA_WIDTH-1:0]  matrix_p13;
wire [DATA_WIDTH-1:0]  matrix_p21; 
wire [DATA_WIDTH-1:0]  matrix_p22; 
wire [DATA_WIDTH-1:0]  matrix_p23;
wire [DATA_WIDTH-1:0]  matrix_p31; 
wire [DATA_WIDTH-1:0]  matrix_p32; 
wire [DATA_WIDTH-1:0]  matrix_p33;
wire [DATA_WIDTH-1:0]  mid_value;

//*****************************************************
//**                    main code
//*****************************************************
//在延迟后的行信号有效，将中值赋给灰度输出值
assign pos_img_y = pos_frame_href ? mid_value : {DATA_WIDTH{1'b0}};

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(IMG_HDISP)
)u_vip_matrix_generate_3x3_8bit(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .pre_frame_vsync    (pre_frame_vsync),
    .pre_frame_href     (pre_frame_href), 
    .pre_frame_clken    (pre_frame_clken),
    .pre_img_y          (pre_img_y),
    
    //处理后的图像数据
    .matrix_frame_vsync (matrix_frame_vsync),
    .matrix_frame_href  (matrix_frame_href),
    .matrix_frame_clken (matrix_frame_clken),
    .matrix_p11         (matrix_p11),    
    .matrix_p12         (matrix_p12),    
    .matrix_p13         (matrix_p13),
    .matrix_p21         (matrix_p21),    
    .matrix_p22         (matrix_p22),    
    .matrix_p23         (matrix_p23),
    .matrix_p31         (matrix_p31),    
    .matrix_p32         (matrix_p32),    
    .matrix_p33         (matrix_p33)
);

//3x3阵列的中值滤波，需要3个时钟
median_filter_3x3 #(
    .DATA_WIDTH(DATA_WIDTH)
) u_median_filter_3x3(
    .clk        (clk),
    .rst_n      (rst_n),
    
    .median_frame_vsync (matrix_frame_vsync),
    .median_frame_href  (matrix_frame_href),
    .median_frame_clken (matrix_frame_clken),
    
    //第一行
    .data11           (matrix_p11), 
    .data12           (matrix_p12), 
    .data13           (matrix_p13),
    //第二行              
    .data21           (matrix_p21), 
    .data22           (matrix_p22), 
    .data23           (matrix_p23),
    //第三行              
    .data31           (matrix_p31), 
    .data32           (matrix_p32), 
    .data33           (matrix_p33),
    
    .pos_median_vsync (pos_frame_vsync),
    .pos_median_href  (pos_frame_href),
    .pos_median_clken (pos_frame_clken),
    .target_data      (mid_value)
);

endmodule 