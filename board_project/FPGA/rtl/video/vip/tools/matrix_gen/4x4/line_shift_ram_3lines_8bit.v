module line_shift_ram_3lines_8bit#(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 1024
)(
    input                  clock, 
    input                  rst_n,  
    input                  clken,
    input                  pre_frame_href,
    
    input   [DATA_WIDTH-1:0]  shiftin,  
    output  [DATA_WIDTH-1:0]  taps0x,   // 延迟 1 行
    output  [DATA_WIDTH-1:0]  taps1x,   // 延迟 2 行
    output  [DATA_WIDTH-1:0]  taps2x    // 延迟 3 行
);
localparam ADDR_WIDTH = $clog2(DEPTH);

//reg define
reg  [2:0]  clken_dly;
reg  [ADDR_WIDTH-1:0] ram_rd_addr;
reg  [ADDR_WIDTH-1:0] ram_rd_addr_d0;
reg  [ADDR_WIDTH-1:0] ram_rd_addr_d1;
reg  [DATA_WIDTH-1:0] shiftin_d0;
reg  [DATA_WIDTH-1:0] shiftin_d1;
reg  [DATA_WIDTH-1:0] shiftin_d2;

//*****************************************************
//**                    main code
//*****************************************************

//在数据来到时，ram地址累加
always@(posedge clock)begin
    if(pre_frame_href)
        if(clken)
            ram_rd_addr <= ram_rd_addr + 1 ;
        else
            ram_rd_addr <= ram_rd_addr ;
    else
        ram_rd_addr <= 0 ;
end

//时钟使能信号延迟三拍
always@(posedge clock) begin
    clken_dly <= { clken_dly[1:0] , clken };
end

//将ram地址延迟二拍
always@(posedge clock ) begin
    ram_rd_addr_d0 <= ram_rd_addr;
    ram_rd_addr_d1 <= ram_rd_addr_d0;
end

//输入数据延迟三拍
always@(posedge clock)begin
    shiftin_d0 <= shiftin;
    shiftin_d1 <= shiftin_d0;
    shiftin_d2 <= shiftin_d1;
end

// RAM 0: 存储当前行数据，输出前 1 行数据 (taps0x)
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_0 (
    .wr_data (shiftin_d2 ), 
    .wr_addr (ram_rd_addr_d1),
    .wr_en   (clken_dly[2]),  
    .wr_clk  (clock ),        
    .wr_rst  (~rst_n ),       
    .rd_addr (ram_rd_addr ),  
    .rd_data (taps0x ),       
    .rd_clk  (clock ),        
    .rd_rst  (~rst_n )        
);

// RAM 1: 存储前 1 行数据，输出前 2 行数据 (taps1x)
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_1 (
    .wr_data (taps0x),        
    .wr_addr (ram_rd_addr_d1),
    .wr_en   (clken_dly[2] ), 
    .wr_clk  (clock ),        
    .wr_rst  (~rst_n ),       
    .rd_addr (ram_rd_addr ),  
    .rd_data (taps1x ),       
    .rd_clk  (clock ),        
    .rd_rst  (~rst_n )        
);

// RAM 2: 存储前 2 行数据，输出前 3 行数据 (taps2x)
dualport_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_ram_2 (
    .wr_data (taps1x),        
    .wr_addr (ram_rd_addr_d1),
    .wr_en   (clken_dly[2] ), 
    .wr_clk  (clock ),        
    .wr_rst  (~rst_n ),       
    .rd_addr (ram_rd_addr ),  
    .rd_data (taps2x ),       
    .rd_clk  (clock ),        
    .rd_rst  (~rst_n )        
);

endmodule