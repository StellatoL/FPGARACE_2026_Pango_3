module VIP_Gray_Minimum_Filter #(
    parameter   IMG_HDISP = 640,
    parameter   IMG_VDISP = 640,
    parameter   DEPTH     = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input               clk,
    input               rst_n,

    input               pre_frame_vsync,
    input               pre_frame_href,
    input               pre_frame_clken,
    input       [7:0]   pre_img_y,          //输入的灰度图像

    output              pos_frame_vsync,
    output              pos_frame_href,
    output              pos_frame_clken,
    output      [7:0]   pos_img_y          //输出的最小值滤波后的图像
);

//wire define
wire        matrix_frame_vsync;
wire        matrix_frame_href;
wire        matrix_frame_clken;
wire [7:0]  matrix_p11; //5x5 阵列输出
wire [7:0]  matrix_p12; 
wire [7:0]  matrix_p13;
wire [7:0]  matrix_p14;
wire [7:0]  matrix_p15;
wire [7:0]  matrix_p21; 
wire [7:0]  matrix_p22; 
wire [7:0]  matrix_p23;
wire [7:0]  matrix_p24;
wire [7:0]  matrix_p25;
wire [7:0]  matrix_p31; 
wire [7:0]  matrix_p32; 
wire [7:0]  matrix_p33;
wire [7:0]  matrix_p34;
wire [7:0]  matrix_p35;
wire [7:0]  matrix_p41;
wire [7:0]  matrix_p42;
wire [7:0]  matrix_p43;
wire [7:0]  matrix_p44;
wire [7:0]  matrix_p45;
wire [7:0]  matrix_p51;
wire [7:0]  matrix_p52;
wire [7:0]  matrix_p53;
wire [7:0]  matrix_p54;
wire [7:0]  matrix_p55;

wire [7:0]  min_value;

//*****************************************************
//**                    main code
//*****************************************************
//在延迟后的行信号有效，将最小值赋给灰度输出值
assign pos_img_y = (pos_frame_href && pos_frame_clken) ? min_value : 8'd0;


vip_matrix_generate_5x5_8bit#(
    .DATA_WIDTH(8),
    .DEPTH(DEPTH)
)u_vip_matrix_generate_5x5_8bit(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .per_frame_vsync    (pre_frame_vsync),
    .per_frame_href     (pre_frame_href), 
    .per_frame_clken    (pre_frame_clken),
    .per_img_y          (pre_img_y),
    
    //处理后的图像数据
    .matrix_frame_vsync (matrix_frame_vsync),
    .matrix_frame_href  (matrix_frame_href),
    .matrix_frame_clken (matrix_frame_clken),
    .matrix_p11         (matrix_p11),    
    .matrix_p12         (matrix_p12),    
    .matrix_p13         (matrix_p13),
    .matrix_p14         (matrix_p14),    
    .matrix_p15         (matrix_p15),
    .matrix_p21         (matrix_p21),    
    .matrix_p22         (matrix_p22),    
    .matrix_p23         (matrix_p23),
    .matrix_p24         (matrix_p24),
    .matrix_p25         (matrix_p25),
    .matrix_p31         (matrix_p31),    
    .matrix_p32         (matrix_p32),    
    .matrix_p33         (matrix_p33),
    .matrix_p34         (matrix_p34),
    .matrix_p35         (matrix_p35),
    .matrix_p41         (matrix_p41),
    .matrix_p42         (matrix_p42),
    .matrix_p43         (matrix_p43),
    .matrix_p44         (matrix_p44),
    .matrix_p45         (matrix_p45),
    .matrix_p51         (matrix_p51),
    .matrix_p52         (matrix_p52),
    .matrix_p53         (matrix_p53),
    .matrix_p54         (matrix_p54),
    .matrix_p55         (matrix_p55)
);

//5x5阵列的最小值滤波，需要2个时钟
minimum_filter_5x5 u_minimum_filter_5x5(
    .clk        (clk),
    .rst_n      (rst_n),

    .minimum_frame_vsync (matrix_frame_vsync),
    .minimum_frame_href  (matrix_frame_href),
    .minimum_frame_clken (matrix_frame_clken),

    //第一行
    .data11           (matrix_p11), 
    .data12           (matrix_p12), 
    .data13           (matrix_p13),
    .data14           (matrix_p14),
    .data15           (matrix_p15),
    //第二行
    .data21           (matrix_p21), 
    .data22           (matrix_p22), 
    .data23           (matrix_p23),
    .data24           (matrix_p24),
    .data25           (matrix_p25),
    //第三行
    .data31           (matrix_p31),
    .data32           (matrix_p32),
    .data33           (matrix_p33),
    .data34           (matrix_p34),
    .data35           (matrix_p35),
    //第四行
    .data41           (matrix_p41),
    .data42           (matrix_p42),
    .data43           (matrix_p43),
    .data44           (matrix_p44),
    .data45           (matrix_p45),
    //第五行
    .data51           (matrix_p51),
    .data52           (matrix_p52),
    .data53           (matrix_p53),
    .data54           (matrix_p54),
    .data55           (matrix_p55),
    
    .pos_minimum_vsync (pos_frame_vsync),
    .pos_minimum_href  (pos_frame_href),
    .pos_minimum_clken (pos_frame_clken),
    .target_data       (min_value)
);

endmodule