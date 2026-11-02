module  vip_matrix_generate_4x4_8bit #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 1024
)(
    input                     clk,  
    input                     rst_n,

    input                     pre_frame_vsync,
    input                     pre_frame_href,
    input                     pre_frame_clken,
    input      [DATA_WIDTH-1:0]  pre_img_y,
    
    output                    matrix_frame_vsync,
    output                    matrix_frame_href,
    output                    matrix_frame_clken,
    output reg [DATA_WIDTH-1:0]  matrix_p11, matrix_p12, matrix_p13, matrix_p14,
    output reg [DATA_WIDTH-1:0]  matrix_p21, matrix_p22, matrix_p23, matrix_p24,
    output reg [DATA_WIDTH-1:0]  matrix_p31, matrix_p32, matrix_p33, matrix_p34,
    output reg [DATA_WIDTH-1:0]  matrix_p41, matrix_p42, matrix_p43, matrix_p44
);

//wire define
wire [DATA_WIDTH-1:0] row1_data;  // 延迟 3 行的数据 (最顶行)
wire [DATA_WIDTH-1:0] row2_data;  // 延迟 2 行的数据
wire [DATA_WIDTH-1:0] row3_data;  // 延迟 1 行的数据
wire       read_frame_href;
wire       read_frame_clken;

//reg define
reg  [DATA_WIDTH-1:0] pre_img_y_d[2:0];
reg  [DATA_WIDTH-1:0] row4_data;  // 当前行数据 (最底行)
reg  [4:0] pre_frame_vsync_r;
reg  [4:0] pre_frame_href_r;
reg  [4:0] pre_frame_clken_r;

//*****************************************************
//**                    main code
//*****************************************************

assign read_frame_href    = pre_frame_href_r[3] ;
assign read_frame_clken   = pre_frame_clken_r[3];
assign matrix_frame_vsync = pre_frame_vsync_r[4];
assign matrix_frame_href  = pre_frame_href_r[4] ;
assign matrix_frame_clken = pre_frame_clken_r[4];

//当前数据延迟4拍后放在第4行 (最底行)
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        row4_data <= 0; 
        pre_img_y_d[0] <= 0;
        pre_img_y_d[1] <= 0;
        pre_img_y_d[2] <= 0;  
    end
    else begin
        pre_img_y_d[0] <= pre_img_y;
        pre_img_y_d[1] <= pre_img_y_d[0];
        pre_img_y_d[2] <= pre_img_y_d[1];
        row4_data <= pre_img_y_d[2];
    end
end

// 例化 3行 延迟缓存的 RAM 模块
line_shift_ram_3lines_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
     .DEPTH(DEPTH)
) u_line_shift_ram_3lines
(
    .clock          (clk),
    .rst_n          (rst_n),
    .clken          (pre_frame_clken),
    .pre_frame_href (pre_frame_href),
    
    .shiftin        (pre_img_y),   
    .taps0x         (row3_data),   // 延迟 1 行
    .taps1x         (row2_data),   // 延迟 2 行
    .taps2x         (row1_data)    // 延迟 3 行
);

//将同步信号延迟5拍，用于同步化处理
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pre_frame_vsync_r <= 0;
        pre_frame_href_r  <= 0;
        pre_frame_clken_r <= 0;
    end
    else begin
        pre_frame_vsync_r <= {pre_frame_vsync_r[3:0] , pre_frame_vsync};
        pre_frame_href_r  <= { pre_frame_href_r[3:0] , pre_frame_href };
        pre_frame_clken_r <= {pre_frame_clken_r[3:0] , pre_frame_clken};
    end
end

//在同步处理后的控制信号下，输出 4x4 图像矩阵
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        {matrix_p11, matrix_p12, matrix_p13, matrix_p14} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p21, matrix_p22, matrix_p23, matrix_p24} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p31, matrix_p32, matrix_p33, matrix_p34} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p41, matrix_p42, matrix_p43, matrix_p44} <= {(DATA_WIDTH*4){1'b0}};
    end
    else if(read_frame_href) begin
        if(read_frame_clken) begin
            // 数据移位：Pxx <= Px(x+1) , 新数据补充在最右侧 (P_x4)
            {matrix_p11, matrix_p12, matrix_p13, matrix_p14} <= {matrix_p12, matrix_p13, matrix_p14, row1_data};
            {matrix_p21, matrix_p22, matrix_p23, matrix_p24} <= {matrix_p22, matrix_p23, matrix_p24, row2_data};
            {matrix_p31, matrix_p32, matrix_p33, matrix_p34} <= {matrix_p32, matrix_p33, matrix_p34, row3_data};
            {matrix_p41, matrix_p42, matrix_p43, matrix_p44} <= {matrix_p42, matrix_p43, matrix_p44, row4_data};
        end
        else begin
            // 保持状态
            {matrix_p11, matrix_p12, matrix_p13, matrix_p14} <= {matrix_p11, matrix_p12, matrix_p13, matrix_p14};
            {matrix_p21, matrix_p22, matrix_p23, matrix_p24} <= {matrix_p21, matrix_p22, matrix_p23, matrix_p24};
            {matrix_p31, matrix_p32, matrix_p33, matrix_p34} <= {matrix_p31, matrix_p32, matrix_p33, matrix_p34};
            {matrix_p41, matrix_p42, matrix_p43, matrix_p44} <= {matrix_p41, matrix_p42, matrix_p43, matrix_p44};
        end
    end
    else begin
        {matrix_p11, matrix_p12, matrix_p13, matrix_p14} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p21, matrix_p22, matrix_p23, matrix_p24} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p31, matrix_p32, matrix_p33, matrix_p34} <= {(DATA_WIDTH*4){1'b0}};
        {matrix_p41, matrix_p42, matrix_p43, matrix_p44} <= {(DATA_WIDTH*4){1'b0}};
    end
end

endmodule