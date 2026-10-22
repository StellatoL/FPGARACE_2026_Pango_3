`timescale 1ns/1ns

module VIP_vertical_projection_mult #(
    parameter IMG_HDISP = 640,  
    parameter IMG_VDISP = 480,  
    
    // 自动推导位宽
    parameter X_BITS = $clog2(IMG_HDISP),
    parameter Y_BITS = $clog2(IMG_VDISP),
    parameter CNT_BITS = $clog2(IMG_VDISP + 1),
    
    // 垂直投影抗噪阈值（建议比水平投影略低，因为列的白点通常不如行多）
    parameter PROJ_THRESHOLD = 15 
)(
    input               clk,
    input               rst_n,

    // 处理前图像数据
    input               per_frame_vsync,
    input               per_frame_href,
    input               per_frame_clken,
    input               per_img_Bit,
    
    // 处理后的图像数据 (同步输出)
    output              post_frame_vsync,
    output              post_frame_href,
    output              post_frame_clken,
    output              post_img_Bit,
    
    // 投影感兴趣区域 (ROI)，由水平投影/CPU给出
    input      [Y_BITS-1:0]    vertical_start,
    input      [Y_BITS-1:0]    vertical_end,

    // [新增] 下游模块读取 X 轴车牌左右边界的 FIFO 接口
    input                      fifo_rd_en,
    output [2*X_BITS-1:0]      fifo_rd_data, // 高位存 X_start，低位存 X_end
    output                     fifo_empty
);

//------------------------------------------
// 1. 信号打拍同步流水线
//------------------------------------------
reg         per_frame_vsync_r,  per_frame_vsync_r2;
reg         per_frame_href_r,   per_frame_href_r2;
reg         per_frame_clken_r,  per_frame_clken_r2;
reg         per_img_Bit_r,      per_img_Bit_r2;

assign  post_frame_vsync    =   per_frame_vsync_r2;
assign  post_frame_href     =   per_frame_href_r2;
assign  post_frame_clken    =   per_frame_clken_r2;
assign  post_img_Bit        =   per_img_Bit_r2;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r2 <= 0; per_frame_href_r2 <= 0; per_frame_clken_r2 <= 0; per_img_Bit_r2 <= 0;
        per_frame_vsync_r  <= 0; per_frame_href_r  <= 0; per_frame_clken_r  <= 0; per_img_Bit_r  <= 0;
    end else begin
        per_frame_vsync_r2 <= per_frame_vsync_r; per_frame_href_r2 <= per_frame_href_r; per_frame_clken_r2 <= per_frame_clken_r; per_img_Bit_r2    <= per_img_Bit_r;
        per_frame_vsync_r  <= per_frame_vsync;   per_frame_href_r  <= per_frame_href;   per_frame_clken_r  <= per_frame_clken;   per_img_Bit_r     <= per_img_Bit;
    end
end

wire vsync_neg_flag = (~per_frame_vsync_r) &   per_frame_vsync_r2;
wire hrefr_neg_flag = (~per_frame_href_r)  &   per_frame_href_r2;

//------------------------------------------
// 2. 坐标计数器
//------------------------------------------
reg [X_BITS-1:0]   x_cnt;
reg [Y_BITS-1:0]   y_cnt;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        x_cnt <= 'd0; y_cnt <= 'd0;
    end else if(vsync_neg_flag) begin
        x_cnt <= 'd0; y_cnt <= 'd0;
    end else if(hrefr_neg_flag) begin
        x_cnt <= 'd0; y_cnt <= y_cnt + 1'b1;
    end else if(per_frame_clken_r) begin
        x_cnt <= x_cnt + 1'b1;
    end
end

reg [X_BITS-1:0]   x_cnt_d1;
reg [X_BITS-1:0]   x_cnt_r;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        x_cnt_d1 <= 'd0; x_cnt_r <= 'd0;
    end else begin
        x_cnt_d1 <= x_cnt;
        x_cnt_r  <= x_cnt_d1;
    end
end

//------------------------------------------
// 3. 垂直投影 RAM 累加逻辑
//------------------------------------------
reg                 ram_wr;
wire [CNT_BITS-1:0] ram_wr_data;
wire [CNT_BITS-1:0] ram_rd_data;

always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) 
        ram_wr <= 1'b0;
    else 
        ram_wr <= per_frame_clken_r; 
end

// 巧妙的列累加：第一行强制清零，在 ROI 区间内累加像素
assign ram_wr_data = (y_cnt == 'd0) ? 'd0 : 
                     ((y_cnt > vertical_start) && (y_cnt < vertical_end)) ? (ram_rd_data + per_img_Bit_r) : 
                     ram_rd_data;

dualport_ram #(
    .DATA_WIDTH (CNT_BITS),
    .DEPTH      (1 << X_BITS) // 保持 2 的幂次方深度规范
) u_projection_ram (
    .wr_clk     (clk),
    .wr_rst     (~rst_n), 
    .wr_addr    (x_cnt_d1),
    .wr_data    (ram_wr_data),
    .wr_en      (ram_wr),
    
    .rd_clk     (clk),
    .rd_rst     (~rst_n),
    .rd_addr    (x_cnt),
    .rd_data    (ram_rd_data)
);

reg [CNT_BITS-1:0] rd_data_d1, rd_data_d2;
always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rd_data_d1 <= 'd0; rd_data_d2 <= 'd0;
    end else if(per_frame_clken_r) begin
        rd_data_d1 <= ram_rd_data;
        rd_data_d2 <= rd_data_d1;
    end
end

//------------------------------------------
// 4. 极值提取状态机 (横向扫描) - 包含宽度过滤
//------------------------------------------
localparam S_IDLE  = 1'b0;
localparam S_TRACK = 1'b1;

// 垂直方向字符缝隙大，必须给予更大的容忍度！
localparam MAX_GAP = 20; 

// 允许的最小车牌宽度，过滤掉细长的竖条噪点
// 车牌长宽比通常为 3:1 以上，可根据实际 MIN_HEIGHT 配合调整
localparam MIN_WIDTH = 40; 

reg state;
reg [X_BITS-1:0] temp_x_start;
reg [X_BITS-1:0] gap_cnt;       

reg                  fifo_wr_en;
reg [2*X_BITS-1:0]   fifo_wr_data;

// 解决位宽截断问题
wire [X_BITS-1:0] real_x = x_cnt_r - 2'd2; 
wire [Y_BITS-1:0] roi_height = (vertical_end > vertical_start) ? (vertical_end - vertical_start) : {Y_BITS{1'b0}};
wire [CNT_BITS-1:0] adaptive_threshold = (roi_height >> 2) < PROJ_THRESHOLD ? (roi_height >> 2) : PROJ_THRESHOLD;
wire [CNT_BITS-1:0] active_threshold = (adaptive_threshold < 5) ? 5 : adaptive_threshold;

always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state        <= S_IDLE;
        temp_x_start <= {X_BITS{1'b0}};
        gap_cnt      <= {X_BITS{1'b0}};
        fifo_wr_en   <= 1'b0;
        fifo_wr_data <= 0;
    end
    else if(vsync_neg_flag) begin
        state        <= S_IDLE;
        temp_x_start <= {X_BITS{1'b0}};
        gap_cnt      <= {X_BITS{1'b0}};
        fifo_wr_en   <= 1'b0;
    end
    // 在最后一行扫描时，由于每一列都在累加，最后的波形已经完全成型
    else if(per_frame_clken_r && (y_cnt == IMG_VDISP - 1'b1)) begin
        fifo_wr_en <= 1'b0; 
        
        if (x_cnt_r >= 2 && x_cnt_r < IMG_HDISP + 2) begin
            case(state)
                S_IDLE: begin
                    gap_cnt <= {X_BITS{1'b0}}; 
                    // 越过阈值，进入车牌左边界
                    if(rd_data_d2 >= active_threshold) begin
                        temp_x_start <= real_x; 
                        state        <= S_TRACK;
                    end
                end

                S_TRACK: begin
                    // 跌破阈值（可能是字符缝隙，也可能是真结束了）
                    if(rd_data_d2 < active_threshold) begin
                        if (gap_cnt >= MAX_GAP) begin
                            // 计算条带宽度，只有大于等于 MIN_WIDTH 才允许写入 FIFO！
                            if ( (real_x - gap_cnt) - temp_x_start >= MIN_WIDTH ) begin
                                fifo_wr_en   <= 1'b1;
                                fifo_wr_data <= {temp_x_start, real_x - gap_cnt}; 
                            end
                            state        <= S_IDLE;
                        end else begin
                            gap_cnt <= gap_cnt + 1'b1;
                        end
                    end 
                    else begin
                        // 遇到字符笔画，缝隙计数器清零，成功“缝合”断裂
                        gap_cnt <= {X_BITS{1'b0}};
                    end
                    
                    // 触底保护：如果车牌贴近画面最右侧
                    if ( real_x == (IMG_HDISP - 1) ) begin
                        // 触底保护处同样增加宽度过滤
                        if ( (real_x - gap_cnt) - temp_x_start >= MIN_WIDTH ) begin
                            fifo_wr_en   <= 1'b1;
                            fifo_wr_data <= {temp_x_start, real_x - gap_cnt}; 
                        end
                        state        <= S_IDLE;
                    end
                end
            endcase
        end
    end
    else if(hrefr_neg_flag && (y_cnt == IMG_VDISP - 1'b1) && (state == S_TRACK)) begin
        if ( ((IMG_HDISP - 1) - gap_cnt) - temp_x_start >= MIN_WIDTH ) begin
            fifo_wr_en   <= 1'b1;
            fifo_wr_data <= {temp_x_start, (IMG_HDISP - 1) - gap_cnt};
        end else begin
            fifo_wr_en <= 1'b0;
        end
        state   <= S_IDLE;
        gap_cnt <= {X_BITS{1'b0}};
    end
    else begin
        fifo_wr_en <= 1'b0;
    end
end

//------------------------------------------
// 5. 实例化 FIFO，输出多组左右坐标
//------------------------------------------
wire fifo_full;

fifo_ram #(
    .DATA_WIDTH ( 2 * X_BITS ), 
    .DATA_DEPTH ( 16 )              
) u_x_coord_fifo (
    .clk        ( clk          ),
    .wr_en      ( fifo_wr_en   ),
    .wr_data    ( fifo_wr_data ),
    .wr_full    ( fifo_full    ),
    .rd_en      ( fifo_rd_en   ),
    .rd_data    ( fifo_rd_data ),
    .rd_empty   ( fifo_empty   )
);

endmodule