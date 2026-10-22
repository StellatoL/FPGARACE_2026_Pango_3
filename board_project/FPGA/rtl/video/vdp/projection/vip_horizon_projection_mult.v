`timescale 1ns/1ns
module VIP_horizon_projection_mult #(
    parameter IMG_HDISP = 1600,
    parameter IMG_VDISP = 1100,
    parameter DATA_WIDTH = $clog2((IMG_HDISP > IMG_VDISP) ? IMG_HDISP : IMG_VDISP),
    // 投影抗噪阈值
    parameter MIN_HEIGHT = 20, // 车牌最小高度，低于此值视为噪点
    parameter PROJ_THRESHOLD = 45 
)(
    input                       clk,                
    input                       rst_n,              

    // Image data prepred to be processd
    input                       per_frame_vsync,    
    input                       per_frame_href,     
    input                       per_frame_clken,    
    input                       per_img_Bit,        
    
    // Image data has been processd
    output                      post_frame_vsync,   
    output                      post_frame_href,    
    output                      post_frame_clken,   
    output                      post_img_Bit,       
    
    input      [DATA_WIDTH-1:0] horizon_start,      
    input      [DATA_WIDTH-1:0] horizon_end,        

    // 下游模块读取 Y 轴有效条带的 FIFO 接口
    input                       fifo_rd_en,
    output [2*DATA_WIDTH-1:0]   fifo_rd_data, // 高位存 Y_start，低位存 Y_end
    output                      fifo_empty
);

// ------------------------------------------
// 1. 信号延迟与对齐
// ------------------------------------------
reg per_frame_vsync_r, per_frame_href_r, per_frame_clken_r, per_img_Bit_r;
reg per_frame_vsync_r2, per_frame_href_r2, per_frame_clken_r2, per_img_Bit_r2;

assign  post_frame_vsync   =   per_frame_vsync_r2;
assign  post_frame_href    =   per_frame_href_r2;  
assign  post_frame_clken   =   per_frame_clken_r2;
assign  post_img_Bit       =   per_img_Bit_r2;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r2 <= 0; per_frame_href_r2  <= 0; per_frame_clken_r2 <= 0; per_img_Bit_r2 <= 0;
        per_frame_vsync_r  <= 0; per_frame_href_r   <= 0; per_frame_clken_r  <= 0; per_img_Bit_r  <= 0;
    end else begin
        per_frame_vsync_r2 <= per_frame_vsync_r; per_frame_href_r2  <= per_frame_href_r; per_frame_clken_r2 <= per_frame_clken_r; per_img_Bit_r2 <= per_img_Bit_r;
        per_frame_vsync_r  <= per_frame_vsync;   per_frame_href_r   <= per_frame_href;   per_frame_clken_r  <= per_frame_clken;   per_img_Bit_r  <= per_img_Bit;
    end
end

wire vsync_pos_flag =   per_frame_vsync_r  & (~per_frame_vsync_r2);
wire vsync_neg_flag = (~per_frame_vsync_r) &   per_frame_vsync_r2;
wire hrefr_pos_flag =   per_frame_href_r   & (~per_frame_href_r2);
wire hrefr_neg_flag = (~per_frame_href_r)  & per_frame_href_r2;

// ------------------------------------------
// 2. 坐标计数
// ------------------------------------------
reg [DATA_WIDTH-1:0] x_cnt, y_cnt, x_cnt_r, y_cnt_r;
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        x_cnt <= {DATA_WIDTH{1'b0}}; y_cnt <= {DATA_WIDTH{1'b0}};
    end else if(vsync_neg_flag) begin
        x_cnt <= {DATA_WIDTH{1'b0}}; y_cnt <= {DATA_WIDTH{1'b0}};
    end else if(hrefr_neg_flag) begin
        x_cnt <= {DATA_WIDTH{1'b0}}; y_cnt <= y_cnt + 1'b1;
    end else if(per_frame_clken_r) begin
        x_cnt <= x_cnt + 1'b1;
    end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin x_cnt_r <= 0; y_cnt_r <= 0; end
    else begin       x_cnt_r <= x_cnt; y_cnt_r <= y_cnt; end
end

// ------------------------------------------
// 3. 【彻底重构】RAM 读写逻辑 (解决 BRAM 冲突漏洞)
// ------------------------------------------
reg                       ram_wr; 
reg  [DATA_WIDTH-1:0]     ram_wr_data;
wire [DATA_WIDTH-1:0]     ram_rd_data;
wire [DATA_WIDTH-1:0]     ram_wr_addr;
wire [DATA_WIDTH-1:0]     ram_rd_addr;

// 引入行累加寄存器，完全取代 RAM 内部自循环累加
reg [DATA_WIDTH-1:0] row_sum;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_sum <= 0;
    else if (hrefr_pos_flag)
        row_sum <= 0; // 每行行首清零
    else if (per_frame_clken_r && (x_cnt_r > horizon_start) && (x_cnt_r < horizon_end))
        row_sum <= row_sum + per_img_Bit_r; // 安全累加像素
end

always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        ram_wr      <= 1'b0;
        ram_wr_data <= 0;
    end
    else if(y_cnt == 0 && per_frame_clken) begin
        // 第一行：利用扫描时间快速清空整帧 RAM，保证无幽灵数据
        ram_wr      <= 1'b1;
        ram_wr_data <= 0;
    end
    else if(hrefr_neg_flag && y_cnt != IMG_VDISP - 1'b1) begin
        // 【核心修复】每行结束时，只执行1次写操作，将统计结果固化到 RAM 中
        ram_wr      <= 1'b1;
        ram_wr_data <= row_sum;
    end
    else begin
        ram_wr      <= 1'b0;
    end
end

assign ram_wr_addr = (y_cnt == 0 && per_frame_clken) ? x_cnt : y_cnt_r;
// 仅在最后一行（提取状态机启动时）才允许读取 RAM，其余时刻闲置降低功耗
assign ram_rd_addr = (y_cnt == IMG_VDISP - 1'b1) ? x_cnt : 0;

dualport_ram #(
    .DATA_WIDTH (DATA_WIDTH), .DEPTH (1 << DATA_WIDTH) 
) u_projection_ram (
    .wr_clk (clk), .wr_rst (~rst_n), .wr_addr (ram_wr_addr), .wr_data (ram_wr_data), .wr_en (ram_wr),
    .rd_clk (clk), .rd_rst (~rst_n), .rd_addr (ram_rd_addr), .rd_data (ram_rd_data)
);

reg [DATA_WIDTH-1:0] rd_data_d1, rd_data_d2;
always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin rd_data_d1 <= 0; rd_data_d2 <= 0; end
    else if(per_frame_clken) begin rd_data_d1 <= ram_rd_data; rd_data_d2 <= rd_data_d1; end
end

// ------------------------------------------
// 4. [极值提取状态机] - 包含容错防抖、断层修补与高度过滤
// ------------------------------------------
localparam S_IDLE  = 1'b0;
localparam S_TRACK = 1'b1;
localparam MAX_GAP = 10; 

reg state;
reg [DATA_WIDTH-1:0] temp_y_start;
reg [DATA_WIDTH-1:0] gap_cnt;       

reg                      fifo_wr_en;
reg [2*DATA_WIDTH-1:0]   fifo_wr_data;

wire [DATA_WIDTH-1:0] real_y = x_cnt_r - 2'd2; 

always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state        <= S_IDLE; temp_y_start <= 0; gap_cnt <= 0;
        fifo_wr_en   <= 1'b0;   fifo_wr_data <= 0;
    end
    else if(vsync_neg_flag) begin
        state        <= S_IDLE; temp_y_start <= 0; gap_cnt <= 0;
        fifo_wr_en   <= 1'b0;
    end
    else if(per_frame_clken && (y_cnt == IMG_VDISP - 1'b1)) begin
        fifo_wr_en <= 1'b0; 
        
        if (x_cnt_r >= 2 && x_cnt_r < IMG_VDISP + 2) begin
            case(state)
                S_IDLE: begin
                    gap_cnt <= 0; 
                    if(rd_data_d2 >= PROJ_THRESHOLD) begin
                        temp_y_start <= real_y; 
                        state        <= S_TRACK;
                    end
                end

                S_TRACK: begin
                    if(rd_data_d2 < PROJ_THRESHOLD) begin
                        if (gap_cnt >= MAX_GAP) begin
                            // 【核心修复】：计算条带高度，只有大于等于 MIN_HEIGHT 才允许写入 FIFO
                            if ( (real_y - gap_cnt) - temp_y_start >= MIN_HEIGHT ) begin
                                fifo_wr_en   <= 1'b1;
                                fifo_wr_data <= {temp_y_start, real_y - gap_cnt}; 
                            end
                            state        <= S_IDLE;
                        end else begin
                            gap_cnt <= gap_cnt + 1'b1;
                        end
                    end 
                    else begin
                        gap_cnt <= 0;
                    end
                    
                    if ( real_y == (IMG_VDISP - 1) ) begin
                        // 触底保护处同样增加高度过滤
                        if ( (real_y - gap_cnt) - temp_y_start >= MIN_HEIGHT ) begin
                            fifo_wr_en   <= 1'b1;
                            fifo_wr_data <= {temp_y_start, real_y - gap_cnt}; 
                        end
                        state        <= S_IDLE;
                    end
                end
            endcase
        end
    end
    else begin
        fifo_wr_en <= 1'b0;
    end
end
// ------------------------------------------
// 5. FIFO 实例化
// ------------------------------------------
wire fifo_full;

fifo_ram #(
    .DATA_WIDTH ( 2 * DATA_WIDTH ),
    .DATA_DEPTH ( 16 )              
) u_y_coord_fifo (
    .clk        ( clk          ),
    .wr_en      ( fifo_wr_en   ),
    .wr_data    ( fifo_wr_data ),
    .wr_full    ( fifo_full    ),
    .rd_en      ( fifo_rd_en   ),
    .rd_data    ( fifo_rd_data ),
    .rd_empty   ( fifo_empty   )
);

endmodule