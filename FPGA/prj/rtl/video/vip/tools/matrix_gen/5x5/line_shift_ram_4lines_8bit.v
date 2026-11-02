module line_shift_ram_4lines_8bit #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 2048
)(
    input          clock, 
    input          rst_n,  
    input          clken,
    input          pre_frame_href,
    
    input  [DATA_WIDTH-1:0]   shiftin,  
    output [DATA_WIDTH-1:0]   taps0x,   // 延迟 1 行
    output [DATA_WIDTH-1:0]   taps1x,   // 延迟 2 行
    output [DATA_WIDTH-1:0]   taps2x,   // 延迟 3 行
    output [DATA_WIDTH-1:0]   taps3x    // 延迟 4 行
);
localparam ADDR_WIDTH = $clog2(DEPTH);

//reg define
reg  [2:0]  clken_dly;
reg  [ADDR_WIDTH-1:0] ram_rd_addr;
reg  [ADDR_WIDTH-1:0] ram_rd_addr_d0;
reg  [ADDR_WIDTH-1:0] ram_rd_addr_d1;
reg  [DATA_WIDTH-1:0]  shiftin_d0;
reg  [DATA_WIDTH-1:0]  shiftin_d1;
reg  [DATA_WIDTH-1:0]  shiftin_d2;

//*****************************************************
//** main code
//*****************************************************

//在数据来到时，ram地址累加
always@(posedge clock or negedge rst_n)begin
    if(!rst_n)
        ram_rd_addr <= 0;
    else if(pre_frame_href)
        if(clken)
            ram_rd_addr <= ram_rd_addr + 1 ;
        else
            ram_rd_addr <= ram_rd_addr ;
    else
        ram_rd_addr <= 0 ;
end

//时钟使能信号延迟三拍
always@(posedge clock or negedge rst_n) begin
    if(!rst_n)
        clken_dly <= 0;
    else
        clken_dly <= { clken_dly[1:0] , clken };
end

//将ram地址延迟二拍
always@(posedge clock or negedge rst_n) begin
    if(!rst_n) begin
        ram_rd_addr_d0 <= 0;
        ram_rd_addr_d1 <= 0;
    end
    else begin
        ram_rd_addr_d0 <= ram_rd_addr;
        ram_rd_addr_d1 <= ram_rd_addr_d0;
    end
end

//输入数据延迟三拍
always@(posedge clock or negedge rst_n)begin
    if(!rst_n) begin
        shiftin_d0 <= 0;
        shiftin_d1 <= 0;
        shiftin_d2 <= 0;
    end
    else begin
        shiftin_d0 <= shiftin;
        shiftin_d1 <= shiftin_d0;
        shiftin_d2 <= shiftin_d1;
    end
end

// dualport_ram  blk_mem_gen_0

// RAM 0：存储当前行图像的RAM，输出前 1 行
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_2048x8_0 (
    .wr_data (shiftin_d2 ),   //input [DATA_WIDTH-1:0]ram写数据
    .wr_addr (ram_rd_addr_d1),//input [ADDR_WIDTH-1:0]ram写地址
    .wr_en   (clken_dly[2]),  //input在延迟的第三个时钟周期，当前行的数据写入RAM0
    .wr_clk  (clock ),        //input
    .wr_rst  (~rst_n ),       //input
    .rd_addr (ram_rd_addr ),  //input [ADDR_WIDTH-1:0]ram读地址
    .rd_data (taps0x ),       //output[DATA_WIDTH-1:0]ram读数据延迟一个时钟周期，输出RAM0中前一行图像的数据
    .rd_clk  (clock ),        //input
    .rd_rst  (~rst_n )        //input
);

// RAM 1：存储前 1 行图像的RAM，输出前 2 行
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_2048x8_1 (
    .wr_data (taps0x),        //input [DATA_WIDTH-1:0]ram写数据 (直接使用上一级的输出)
    .wr_addr (ram_rd_addr_d1),//input [ADDR_WIDTH-1:0]ram写地址
    .wr_en   (clken_dly[2] ), //input在延迟的第三个时钟周期，将前一行图像的数据写入RAM1
    .wr_clk  (clock ),        //input
    .wr_rst  (~rst_n ),       //input
    .rd_addr (ram_rd_addr ),  //input [ADDR_WIDTH-1:0]ram读地址
    .rd_data (taps1x ),       //output[DATA_WIDTH-1:0]ram读数据延迟一个时钟周期，输出RAM1中前前一行图像的数据
    .rd_clk  (clock ),        //input
    .rd_rst  (~rst_n )        //input
);

// RAM 2：存储前 2 行图像的RAM，输出前 3 行
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_2048x8_2 (
    .wr_data (taps1x),        //input [DATA_WIDTH-1:0]ram写数据
    .wr_addr (ram_rd_addr_d1),//input [ADDR_WIDTH-1:0]ram写地址
    .wr_en   (clken_dly[2] ), //input写使能时序保持一致
    .wr_clk  (clock ),        //input
    .wr_rst  (~rst_n ),       //input
    .rd_addr (ram_rd_addr ),  //input [ADDR_WIDTH-1:0]ram读地址
    .rd_data (taps2x ),       //output[DATA_WIDTH-1:0]ram读数据输出前 3 行图像的数据
    .rd_clk  (clock ),        //input
    .rd_rst  (~rst_n )        //input
);

// RAM 3：存储前 3 行图像的RAM，输出前 4 行
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_2048x8_3 (
    .wr_data (taps2x),        //input [DATA_WIDTH-1:0]ram写数据
    .wr_addr (ram_rd_addr_d1),//input [ADDR_WIDTH-1:0]ram写地址
    .wr_en   (clken_dly[2] ), //input写使能时序保持一致
    .wr_clk  (clock ),        //input
    .wr_rst  (~rst_n ),       //input
    .rd_addr (ram_rd_addr ),  //input [ADDR_WIDTH-1:0]ram读地址
    .rd_data (taps3x ),       //output[DATA_WIDTH-1:0]ram读数据输出前 4 行图像的数据
    .rd_clk  (clock ),        //input
    .rd_rst  (~rst_n )        //input
);

endmodule