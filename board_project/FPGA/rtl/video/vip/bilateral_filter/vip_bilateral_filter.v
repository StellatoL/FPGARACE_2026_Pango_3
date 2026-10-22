module vip_bilateral_filter#(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP  = 1024,
    parameter IMG_VDISP  = 600,
    parameter DEPTH      = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    //时钟
    input       clk,  //50MHz
    input       rst_n,
    input       enable,
    
    //处理前图像数据
    input                    pre_frame_vsync,  //处理前图像数据场信号
    input                    pre_frame_href,   //处理前图像数据行信号 
    input                    pre_frame_clken,  //处理前图像数据输入使能效信号
    input [DATA_WIDTH - 1:0] pre_img_y,        //灰度数据
    input [DATA_WIDTH - 1:0] pre_img_cr,       //色度数据
    input [DATA_WIDTH - 1:0] pre_img_cb,       //色度数据
    
    //处理后的图像数据
    output                    pos_frame_vsync, //处理后图像数据场信号
    output                    pos_frame_href,  //处理后图像数据行信号
    output                    pos_frame_clken, //处理后图像数据输入使能信号
    output [DATA_WIDTH - 1:0] pos_img_y,       //处理后灰度数据
    output [DATA_WIDTH - 1:0] pos_img_cr,      //处理后色度数据
    output [DATA_WIDTH - 1:0] pos_img_cb       //处理后色度数据
);

//*****************************************************
//**                    main code
//*****************************************************
wire [DATA_WIDTH - 1:0] post_value_y; //双边滤波器输出的灰度值
wire [DATA_WIDTH - 1:0] post_value_cr; //双边滤波器输出的色度值
wire [DATA_WIDTH - 1:0] post_value_cb; //双边滤波器输出的色度值
wire                    post_frame_vsync; //双边滤波器输出的场信号
wire                    post_frame_href;  //双边滤波器输出的行信号
wire                    post_frame_clken; //双边滤波器输出的时钟使能信号

//在延迟后的行信号有效，将中间变量赋给灰度输出值
// assign pos_img_y  = pos_frame_href ? post_value_y  : {DATA_WIDTH{1'b0}};
// assign pos_img_cr = pos_frame_href ? post_value_cr : {DATA_WIDTH{1'b0}};
// assign pos_img_cb = pos_frame_href ? post_value_cb : {DATA_WIDTH{1'b0}};
assign pos_img_y  = enable ? (post_frame_href ? post_value_y  : pre_img_y)  : pre_img_y;
assign pos_img_cr = enable ? (post_frame_href ? post_value_cr : pre_img_cr) : pre_img_cr;
assign pos_img_cb = enable ? (post_frame_href ? post_value_cb : pre_img_cb) : pre_img_cb;

assign pos_frame_vsync = enable ? post_frame_vsync : pre_frame_vsync;
assign pos_frame_href  = enable ? post_frame_href  : pre_frame_href;
assign pos_frame_clken = enable ? post_frame_clken : pre_frame_clken;

//3x3像素阵列信号
wire        matrix_frame_vsync;
wire        matrix_frame_href;
wire        matrix_frame_clken;
wire [DATA_WIDTH - 1:0] matrix_p11_y; //3x3 阵列输出
wire [DATA_WIDTH - 1:0] matrix_p12_y;
wire [DATA_WIDTH - 1:0] matrix_p13_y;
wire [DATA_WIDTH - 1:0] matrix_p21_y;
wire [DATA_WIDTH - 1:0] matrix_p22_y;
wire [DATA_WIDTH - 1:0] matrix_p23_y;
wire [DATA_WIDTH - 1:0] matrix_p31_y;
wire [DATA_WIDTH - 1:0] matrix_p32_y;
wire [DATA_WIDTH - 1:0] matrix_p33_y;

wire [DATA_WIDTH - 1:0] matrix_p11_cr; //3x3 阵列输出
wire [DATA_WIDTH - 1:0] matrix_p12_cr;
wire [DATA_WIDTH - 1:0] matrix_p13_cr;
wire [DATA_WIDTH - 1:0] matrix_p21_cr;
wire [DATA_WIDTH - 1:0] matrix_p22_cr;
wire [DATA_WIDTH - 1:0] matrix_p23_cr;
wire [DATA_WIDTH - 1:0] matrix_p31_cr;
wire [DATA_WIDTH - 1:0] matrix_p32_cr;
wire [DATA_WIDTH - 1:0] matrix_p33_cr;

wire [DATA_WIDTH - 1:0] matrix_p11_cb; //3x3 阵列输出
wire [DATA_WIDTH - 1:0] matrix_p12_cb;
wire [DATA_WIDTH - 1:0] matrix_p13_cb;
wire [DATA_WIDTH - 1:0] matrix_p21_cb;
wire [DATA_WIDTH - 1:0] matrix_p22_cb;
wire [DATA_WIDTH - 1:0] matrix_p23_cb;
wire [DATA_WIDTH - 1:0] matrix_p31_cb;
wire [DATA_WIDTH - 1:0] matrix_p32_cb;
wire [DATA_WIDTH - 1:0] matrix_p33_cb;

// 生成3x3像素阵列
vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
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
    .matrix_p11         (matrix_p11_y),    
    .matrix_p12         (matrix_p12_y),    
    .matrix_p13         (matrix_p13_y),
    .matrix_p21         (matrix_p21_y),    
    .matrix_p22         (matrix_p22_y),    
    .matrix_p23         (matrix_p23_y),
    .matrix_p31         (matrix_p31_y),    
    .matrix_p32         (matrix_p32_y),
    .matrix_p33         (matrix_p33_y)
);

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
) u_vip_matrix_generate_3x3_8bit_cr(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .pre_frame_vsync    (pre_frame_vsync),
    .pre_frame_href     (pre_frame_href), 
    .pre_frame_clken    (pre_frame_clken),
    .pre_img_y          (pre_img_cr),
    
    //处理后的图像数据s
    .matrix_frame_vsync (), //不使用场同步信号
    .matrix_frame_href  (), //不使用行同步信号
    .matrix_frame_clken (), //不使用时钟使能信号
    .matrix_p11         (matrix_p11_cr),    
    .matrix_p12         (matrix_p12_cr),    
    .matrix_p13         (matrix_p13_cr),
    .matrix_p21         (matrix_p21_cr),    
    .matrix_p22         (matrix_p22_cr),    
    .matrix_p23         (matrix_p23_cr),
    .matrix_p31         (matrix_p31_cr),    
    .matrix_p32         (matrix_p32_cr),
    .matrix_p33         (matrix_p33_cr)
);
vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
) u_vip_matrix_generate_3x3_8bit_cb(
    .clk        (clk), 
    .rst_n      (rst_n),
    
    //处理前图像数据
    .pre_frame_vsync    (pre_frame_vsync),
    .pre_frame_href     (pre_frame_href), 
    .pre_frame_clken    (pre_frame_clken),
    .pre_img_y          (pre_img_cb),
    
    //处理后的图像数据
    .matrix_frame_vsync (), //不使用场同步信号
    .matrix_frame_href  (), //不使用行同步信号
    .matrix_frame_clken (), //不使用时钟使能信号
    .matrix_p11         (matrix_p11_cb),    
    .matrix_p12         (matrix_p12_cb),    
    .matrix_p13         (matrix_p13_cb),
    .matrix_p21         (matrix_p21_cb),    
    .matrix_p22         (matrix_p22_cb),    
    .matrix_p23         (matrix_p23_cb),
    .matrix_p31         (matrix_p31_cb),    
    .matrix_p32         (matrix_p32_cb),
    .matrix_p33         (matrix_p33_cb)
);

// 3x3阵列的双边滤波器，需要38个时钟
bilateral_filter #(
    .DATA_WIDTH(DATA_WIDTH)
)u_bilateral_filter(
    .clk        (clk),
    .rst_n      (rst_n),
    
    .bilateral_frame_vsync (matrix_frame_vsync),
    .bilateral_frame_href  (matrix_frame_href),
    .bilateral_frame_clken (matrix_frame_clken),

    .bilateral_p11         (matrix_p11_y),    
    .bilateral_p12         (matrix_p12_y),    
    .bilateral_p13         (matrix_p13_y),
    .bilateral_p21         (matrix_p21_y),    
    .bilateral_p22         (matrix_p22_y),    
    .bilateral_p23         (matrix_p23_y),
    .bilateral_p31         (matrix_p31_y),    
    .bilateral_p32         (matrix_p32_y),
    .bilateral_p33         (matrix_p33_y),

    .bilateral_p11_cr      (matrix_p11_cr),    
    .bilateral_p12_cr      (matrix_p12_cr),
    .bilateral_p13_cr      (matrix_p13_cr),
    .bilateral_p21_cr      (matrix_p21_cr),
    .bilateral_p22_cr      (matrix_p22_cr),
    .bilateral_p23_cr      (matrix_p23_cr),
    .bilateral_p31_cr      (matrix_p31_cr),
    .bilateral_p32_cr      (matrix_p32_cr),
    .bilateral_p33_cr      (matrix_p33_cr),

    .bilateral_p11_cb      (matrix_p11_cb),
    .bilateral_p12_cb      (matrix_p12_cb),
    .bilateral_p13_cb      (matrix_p13_cb),
    .bilateral_p21_cb      (matrix_p21_cb),
    .bilateral_p22_cb      (matrix_p22_cb),
    .bilateral_p23_cb      (matrix_p23_cb),
    .bilateral_p31_cb      (matrix_p31_cb),
    .bilateral_p32_cb      (matrix_p32_cb),
    .bilateral_p33_cb      (matrix_p33_cb),

    //处理后的图像数据
    .pos_frame_vsync       (post_frame_vsync),
    .pos_frame_href        (post_frame_href),
    .pos_frame_clken       (post_frame_clken),
    .pos_img_y             (post_value_y),
    .pos_img_cr            (post_value_cr),
    .pos_img_cb            (post_value_cb)

);

endmodule
