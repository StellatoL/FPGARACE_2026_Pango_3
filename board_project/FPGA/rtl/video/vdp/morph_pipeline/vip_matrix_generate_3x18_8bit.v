module vip_matrix_generate_3x18_8bit #(
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
    
    // 第 1 行 (18 个像素)
    output reg [DATA_WIDTH-1:0]  matrix_p1_01, matrix_p1_02, matrix_p1_03, matrix_p1_04, matrix_p1_05, matrix_p1_06,
    output reg [DATA_WIDTH-1:0]  matrix_p1_07, matrix_p1_08, matrix_p1_09, matrix_p1_10, matrix_p1_11, matrix_p1_12,
    output reg [DATA_WIDTH-1:0]  matrix_p1_13, matrix_p1_14, matrix_p1_15, matrix_p1_16, matrix_p1_17, matrix_p1_18,
    
    // 第 2 行 (18 个像素)
    output reg [DATA_WIDTH-1:0]  matrix_p2_01, matrix_p2_02, matrix_p2_03, matrix_p2_04, matrix_p2_05, matrix_p2_06, 
    output reg [DATA_WIDTH-1:0]  matrix_p2_07, matrix_p2_08, matrix_p2_09, matrix_p2_10, matrix_p2_11, matrix_p2_12, 
    output reg [DATA_WIDTH-1:0]  matrix_p2_13, matrix_p2_14, matrix_p2_15, matrix_p2_16, matrix_p2_17, matrix_p2_18,
    
    // 第 3 行 (18 个像素)
    output reg [DATA_WIDTH-1:0]  matrix_p3_01, matrix_p3_02, matrix_p3_03, matrix_p3_04, matrix_p3_05, matrix_p3_06, 
    output reg [DATA_WIDTH-1:0]  matrix_p3_07, matrix_p3_08, matrix_p3_09, matrix_p3_10, matrix_p3_11, matrix_p3_12, 
    output reg [DATA_WIDTH-1:0]  matrix_p3_13, matrix_p3_14, matrix_p3_15, matrix_p3_16, matrix_p3_17, matrix_p3_18
);

//wire define
wire [DATA_WIDTH-1:0] taps1x_raw; // RAM输出：延迟 2 行的原始数据 
wire [DATA_WIDTH-1:0] taps0x_raw; // RAM输出：延迟 1 行的原始数据 

wire [DATA_WIDTH-1:0] row1_data;  // 经过时序补偿后的第 1 行数据
wire [DATA_WIDTH-1:0] row2_data;  // 经过时序补偿后的第 2 行数据

wire       read_frame_href;
wire       read_frame_clken;

//reg define
reg  [DATA_WIDTH-1:0] pre_img_y_d[2:0];
reg  [DATA_WIDTH-1:0] row3_data;  // 当前行数据 (最底行)
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

// 当前数据延迟 4 拍后放在第 3 行 (最底行)
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        row3_data <= 0; 
        pre_img_y_d[0] <= 0;
        pre_img_y_d[1] <= 0;
        pre_img_y_d[2] <= 0;  
    end
    else begin
        pre_img_y_d[0] <= pre_img_y;
        pre_img_y_d[1] <= pre_img_y_d[0];
        pre_img_y_d[2] <= pre_img_y_d[1];
        row3_data <= pre_img_y_d[2];
    end
end

// 复用你原有的 2行 延迟缓存 RAM 模块 (原封不动)
line_shift_ram_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
     .DEPTH(DEPTH)
)u_line_shift_ram_8bit
(
    .clock          (clk),
    .rst_n          (rst_n),
    .clken          (pre_frame_clken),
    .pre_frame_href (pre_frame_href),
    
    .shiftin        (pre_img_y),   
    .taps0x         (taps0x_raw),   // 接入原始信号
    .taps1x         (taps1x_raw)    // 接入原始信号
);

// =========================================================
// 核心补偿逻辑：消化 RAM 带来的流水线错位 (Pipeline Skew)
// 为了和 row3_data 对齐，我们需要向 taps0x 补充 2 拍，向 taps1x 补充 3 拍
// =========================================================
reg [DATA_WIDTH-1:0] taps0x_d [1:0]; // 2拍移位寄存器
reg [DATA_WIDTH-1:0] taps1x_d [2:0]; // 3拍移位寄存器

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        taps0x_d[0] <= 0; taps0x_d[1] <= 0;
        taps1x_d[0] <= 0; taps1x_d[1] <= 0; taps1x_d[2] <= 0;
    end else begin
        // 不受 clken 限制，与 pre_img_y_d 保持相同的无条件移位策略
        taps0x_d[0] <= taps0x_raw;
        taps0x_d[1] <= taps0x_d[0];

        taps1x_d[0] <= taps1x_raw;
        taps1x_d[1] <= taps1x_d[0];
        taps1x_d[2] <= taps1x_d[1];
    end
end

// 将补偿后的数据接入矩阵逻辑
assign row2_data = taps0x_d[1];
assign row1_data = taps1x_d[2];

// =========================================================


// 将同步信号延迟 5 拍，用于同步化处理
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

// 在同步处理后的控制信号下，输出 3x18 图像矩阵
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        {matrix_p1_01, matrix_p1_02, matrix_p1_03, matrix_p1_04, matrix_p1_05, matrix_p1_06, matrix_p1_07, matrix_p1_08, matrix_p1_09, matrix_p1_10, matrix_p1_11, matrix_p1_12, matrix_p1_13, matrix_p1_14, matrix_p1_15, matrix_p1_16, matrix_p1_17, matrix_p1_18} <= {(DATA_WIDTH*18){1'b0}};
        {matrix_p2_01, matrix_p2_02, matrix_p2_03, matrix_p2_04, matrix_p2_05, matrix_p2_06, matrix_p2_07, matrix_p2_08, matrix_p2_09, matrix_p2_10, matrix_p2_11, matrix_p2_12, matrix_p2_13, matrix_p2_14, matrix_p2_15, matrix_p2_16, matrix_p2_17, matrix_p2_18} <= {(DATA_WIDTH*18){1'b0}};
        {matrix_p3_01, matrix_p3_02, matrix_p3_03, matrix_p3_04, matrix_p3_05, matrix_p3_06, matrix_p3_07, matrix_p3_08, matrix_p3_09, matrix_p3_10, matrix_p3_11, matrix_p3_12, matrix_p3_13, matrix_p3_14, matrix_p3_15, matrix_p3_16, matrix_p3_17, matrix_p3_18} <= {(DATA_WIDTH*18){1'b0}};
    end
    else if(read_frame_href) begin
        if(read_frame_clken) begin
            // 贪吃蛇移位逻辑：原有的向左平移一位，最右侧填入最新读出的行数据
            {matrix_p1_01, matrix_p1_02, matrix_p1_03, matrix_p1_04, matrix_p1_05, matrix_p1_06, matrix_p1_07, matrix_p1_08, matrix_p1_09, matrix_p1_10, matrix_p1_11, matrix_p1_12, matrix_p1_13, matrix_p1_14, matrix_p1_15, matrix_p1_16, matrix_p1_17, matrix_p1_18} <= 
            {matrix_p1_02, matrix_p1_03, matrix_p1_04, matrix_p1_05, matrix_p1_06, matrix_p1_07, matrix_p1_08, matrix_p1_09, matrix_p1_10, matrix_p1_11, matrix_p1_12, matrix_p1_13, matrix_p1_14, matrix_p1_15, matrix_p1_16, matrix_p1_17, matrix_p1_18, row1_data};
            
            {matrix_p2_01, matrix_p2_02, matrix_p2_03, matrix_p2_04, matrix_p2_05, matrix_p2_06, matrix_p2_07, matrix_p2_08, matrix_p2_09, matrix_p2_10, matrix_p2_11, matrix_p2_12, matrix_p2_13, matrix_p2_14, matrix_p2_15, matrix_p2_16, matrix_p2_17, matrix_p2_18} <= 
            {matrix_p2_02, matrix_p2_03, matrix_p2_04, matrix_p2_05, matrix_p2_06, matrix_p2_07, matrix_p2_08, matrix_p2_09, matrix_p2_10, matrix_p2_11, matrix_p2_12, matrix_p2_13, matrix_p2_14, matrix_p2_15, matrix_p2_16, matrix_p2_17, matrix_p2_18, row2_data};
            
            {matrix_p3_01, matrix_p3_02, matrix_p3_03, matrix_p3_04, matrix_p3_05, matrix_p3_06, matrix_p3_07, matrix_p3_08, matrix_p3_09, matrix_p3_10, matrix_p3_11, matrix_p3_12, matrix_p3_13, matrix_p3_14, matrix_p3_15, matrix_p3_16, matrix_p3_17, matrix_p3_18} <= 
            {matrix_p3_02, matrix_p3_03, matrix_p3_04, matrix_p3_05, matrix_p3_06, matrix_p3_07, matrix_p3_08, matrix_p3_09, matrix_p3_10, matrix_p3_11, matrix_p3_12, matrix_p3_13, matrix_p3_14, matrix_p3_15, matrix_p3_16, matrix_p3_17, matrix_p3_18, row3_data};
        end
    end
    else begin
        {matrix_p1_01, matrix_p1_02, matrix_p1_03, matrix_p1_04, matrix_p1_05, matrix_p1_06, matrix_p1_07, matrix_p1_08, matrix_p1_09, matrix_p1_10, matrix_p1_11, matrix_p1_12, matrix_p1_13, matrix_p1_14, matrix_p1_15, matrix_p1_16, matrix_p1_17, matrix_p1_18} <= {(DATA_WIDTH*18){1'b0}};
        {matrix_p2_01, matrix_p2_02, matrix_p2_03, matrix_p2_04, matrix_p2_05, matrix_p2_06, matrix_p2_07, matrix_p2_08, matrix_p2_09, matrix_p2_10, matrix_p2_11, matrix_p2_12, matrix_p2_13, matrix_p2_14, matrix_p2_15, matrix_p2_16, matrix_p2_17, matrix_p2_18} <= {(DATA_WIDTH*18){1'b0}};
        {matrix_p3_01, matrix_p3_02, matrix_p3_03, matrix_p3_04, matrix_p3_05, matrix_p3_06, matrix_p3_07, matrix_p3_08, matrix_p3_09, matrix_p3_10, matrix_p3_11, matrix_p3_12, matrix_p3_13, matrix_p3_14, matrix_p3_15, matrix_p3_16, matrix_p3_17, matrix_p3_18} <= {(DATA_WIDTH*18){1'b0}};
    end
end

endmodule