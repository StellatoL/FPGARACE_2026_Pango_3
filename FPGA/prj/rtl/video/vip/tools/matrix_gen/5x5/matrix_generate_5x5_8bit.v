module  vip_matrix_generate_5x5_8bit #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 1024
)(
    input             clk,  
    input             rst_n,

    input             per_frame_vsync,
    input             per_frame_href,
    input             per_frame_clken,
    input      [DATA_WIDTH-1:0]  per_img_y,
    
    output            matrix_frame_vsync,
    output            matrix_frame_href,
    output            matrix_frame_clken,
    
    // 扩展为 5x5 = 25 个像素输出
    output reg [DATA_WIDTH-1:0]  matrix_p11, output reg [DATA_WIDTH-1:0]  matrix_p12, output reg [DATA_WIDTH-1:0]  matrix_p13, output reg [DATA_WIDTH-1:0]  matrix_p14, output reg [DATA_WIDTH-1:0]  matrix_p15,
    output reg [DATA_WIDTH-1:0]  matrix_p21, output reg [DATA_WIDTH-1:0]  matrix_p22, output reg [DATA_WIDTH-1:0]  matrix_p23, output reg [DATA_WIDTH-1:0]  matrix_p24, output reg [DATA_WIDTH-1:0]  matrix_p25,
    output reg [DATA_WIDTH-1:0]  matrix_p31, output reg [DATA_WIDTH-1:0]  matrix_p32, output reg [DATA_WIDTH-1:0]  matrix_p33, output reg [DATA_WIDTH-1:0]  matrix_p34, output reg [DATA_WIDTH-1:0]  matrix_p35,
    output reg [DATA_WIDTH-1:0]  matrix_p41, output reg [DATA_WIDTH-1:0]  matrix_p42, output reg [DATA_WIDTH-1:0]  matrix_p43, output reg [DATA_WIDTH-1:0]  matrix_p44, output reg [DATA_WIDTH-1:0]  matrix_p45,
    output reg [DATA_WIDTH-1:0]  matrix_p51, output reg [DATA_WIDTH-1:0]  matrix_p52, output reg [DATA_WIDTH-1:0]  matrix_p53, output reg [DATA_WIDTH-1:0]  matrix_p54, output reg [DATA_WIDTH-1:0]  matrix_p55
);

//wire define
wire [DATA_WIDTH-1:0] row1_data;  // 最老的一行 (前 4 行)
wire [DATA_WIDTH-1:0] row2_data;  // 前 3 行
wire [DATA_WIDTH-1:0] row3_data;  // 前 2 行
wire [DATA_WIDTH-1:0] row4_data;  // 前 1 行
wire       read_frame_href;
wire       read_frame_clken;

//reg define
reg  [DATA_WIDTH-1:0] row5_data;  // 最新的一行 (当前输入行)
reg  [1:0] per_frame_vsync_r;
reg  [1:0] per_frame_href_r;
reg  [1:0] per_frame_clken_r;

//*****************************************************
//** main code
//*****************************************************

assign read_frame_href    = per_frame_href_r[0] ;
assign read_frame_clken   = per_frame_clken_r[0];
assign matrix_frame_vsync = per_frame_vsync_r[1];
assign matrix_frame_href  = per_frame_href_r[1] ;
assign matrix_frame_clken = per_frame_clken_r[1];

//当前数据放在第 5 行 (由于是 5x5，当前输入成了最底部的行)
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        row5_data <= 0;
    else begin
        if(per_frame_clken)
            row5_data <= per_img_y ;
        else
            row5_data <= row5_data ;
    end
end

//例化扩展后的列数据缓存 RAM
line_shift_ram_4lines_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_line_shift_ram_4lines
(
    .clock          (clk),
    .rst_n          (rst_n),
    .clken          (per_frame_clken),
    .pre_frame_href (per_frame_href),
    
    .shiftin        (per_img_y),   
    .taps0x         (row4_data),   
    .taps1x         (row3_data),
    .taps2x         (row2_data),   
    .taps3x         (row1_data)    
);

//将同步信号延迟两拍，用于同步化处理
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r <= 0;
        per_frame_href_r  <= 0;
        per_frame_clken_r <= 0;
    end
    else begin
        per_frame_vsync_r <= { per_frame_vsync_r[0], per_frame_vsync };
        per_frame_href_r  <= { per_frame_href_r[0],  per_frame_href  };
        per_frame_clken_r <= { per_frame_clken_r[0], per_frame_clken };
    end
end

//在同步处理后的控制信号下，输出 5x5 图像矩阵
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        // 复位时清零 5x8 = 40 bit
        {matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15} <= 40'h0;
        {matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25} <= 40'h0;
        {matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35} <= 40'h0;
        {matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45} <= 40'h0;
        {matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55} <= 40'h0;
    end
    else if(read_frame_href) begin
        if(read_frame_clken) begin
            // 数据移位：旧数据左移，新数据从右侧放入
            {matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15} <= {matrix_p12, matrix_p13, matrix_p14, matrix_p15, row1_data};
            {matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25} <= {matrix_p22, matrix_p23, matrix_p24, matrix_p25, row2_data};
            {matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35} <= {matrix_p32, matrix_p33, matrix_p34, matrix_p35, row3_data};
            {matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45} <= {matrix_p42, matrix_p43, matrix_p44, matrix_p45, row4_data};
            {matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55} <= {matrix_p52, matrix_p53, matrix_p54, matrix_p55, row5_data};
        end
        else begin
            // 保持不变
            {matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15} <= {matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15};
            {matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25} <= {matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25};
            {matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35} <= {matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35};
            {matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45} <= {matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45};
            {matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55} <= {matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55};
        end
    end
    else begin
        // 行场消隐期间清零
        {matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15} <= 40'h0;
        {matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25} <= 40'h0;
        {matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35} <= 40'h0;
        {matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45} <= 40'h0;
        {matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55} <= 40'h0;
    end
end

endmodule