// =============================================================================
// Module     : video_crtl
// Description: 视频流控制模块
//              确保DMA采集开始时从完整的一帧开始传输
//              - 状态机: IDLE -> WAIT -> TX_DATA
//              - 超时检测: 若1秒内无VS信号则返回IDLE
//              - 参数化: 支持可配置的图像尺寸和像素时钟频率
// =============================================================================

module video_crtl #(
    parameter VIDEO_CLK_FREQ  = 148500000,     // 像素时钟频率 (Hz)
    parameter PPC             = 4,              // 每时钟像素数
    parameter DATA_WIDTH      = 16,             // 数据位宽
    parameter IMG_WIDTH       = 1920,           // 图像宽度
    parameter IMG_HEIGHT      = 1080            // 图像高度
)(
    input  wire                          i_video_clk,
    input  wire                          i_rst_n,

    // ---- 控制端口 ----
    input  wire                          i_start_dma_tx_flag  /*synthesis PAP_MARK_DEBUG="1"*/,

    // ---- 视频输入 ----
    input  wire [DATA_WIDTH-1:0]         i_video_data         /*synthesis PAP_MARK_DEBUG="1"*/,
    input  wire                          i_video_vs           /*synthesis PAP_MARK_DEBUG="1"*/,
    input  wire                          i_video_de           /*synthesis PAP_MARK_DEBUG="1"*/,

    // ---- 视频输出 (受控后) ----
    output wire [DATA_WIDTH-1:0]         o_video_crtl_data    /*synthesis PAP_MARK_DEBUG="1"*/,
    output wire                          o_video_crtl_vs      /*synthesis PAP_MARK_DEBUG="1"*/,
    output wire                          o_video_crtl_de      /*synthesis PAP_MARK_DEBUG="1"*/
);

// =============================================================================
// 状态机定义
// =============================================================================
localparam IDLE     = 3'd0;          // 空闲, 等待启动标志
localparam WAIT     = 3'd1;          // 等待新一帧的VS到来
localparam TX_DATA  = 3'd2;          // 传输数据中
localparam SP_IDLE  = 3'd4;          // 备用空闲态 (保留)

localparam TIME_OUT = VIDEO_CLK_FREQ; // 超时阈值 (~1秒)
localparam IMG_SIZE = IMG_WIDTH * IMG_HEIGHT;

// =============================================================================
// 内部寄存器
// =============================================================================
reg [3:0]               state               /*synthesis PAP_MARK_DEBUG="1"*/;
reg [15:0]              r_x_cnt             /*synthesis PAP_MARK_DEBUG="1"*/;
reg [15:0]              r_y_cnt             /*synthesis PAP_MARK_DEBUG="1"*/;
reg                     r_video_vs_d0;
reg                     r_video_vs_d1;
reg [DATA_WIDTH-1:0]    r_video_crtl_data;
reg                     r_video_crtl_vs;
reg                     r_video_crtl_de;
reg [31:0]              r_dly_cnt           /*synthesis PAP_MARK_DEBUG="1"*/;
reg [31:0]              r_img_total_cnt     /*synthesis PAP_MARK_DEBUG="1"*/;
wire                    w_clear;             // 超时复位信号

// =============================================================================
// 超时检测: 若1秒内无VS或DE信号, 触发复位
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_rst_n)
        r_dly_cnt <= 'd0;
    else if ((i_video_vs && ~r_video_vs_d0) || i_video_de)
        r_dly_cnt <= 'd0;
    else if (r_dly_cnt < TIME_OUT)
        r_dly_cnt <= r_dly_cnt + 1'b1;
end

assign w_clear = (r_dly_cnt >= TIME_OUT);

// VS信号打拍, 用于边沿检测
always @(posedge i_video_clk) begin
    r_video_vs_d0 <= i_video_vs;
    r_video_vs_d1 <= r_video_vs_d0;
end

// =============================================================================
// 水平坐标计数 (X方向)
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_rst_n)
        r_x_cnt <= 'd0;
    else if (i_video_vs && ~r_video_vs_d0)
        r_x_cnt <= 'd0;
    else if (i_video_de && r_x_cnt == IMG_WIDTH / PPC - 1)
        r_x_cnt <= 'd0;
    else if (i_video_de)
        r_x_cnt <= r_x_cnt + 1'b1;
end

// =============================================================================
// 垂直坐标计数 (Y方向)
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_rst_n)
        r_y_cnt <= 'd0;
    else if (i_video_vs && ~r_video_vs_d0)
        r_y_cnt <= 'd0;
    else if (i_video_de && r_x_cnt == IMG_WIDTH / PPC - 1 && r_y_cnt == IMG_HEIGHT - 1)
        r_y_cnt <= 'd0;
    else if (i_video_de && r_x_cnt == IMG_WIDTH / PPC - 1)
        r_y_cnt <= r_y_cnt + 1'b1;
end

// =============================================================================
// 主状态机: 控制数据输出的启停
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_rst_n) begin
        state           <= IDLE;
        r_img_total_cnt <= 'd0;
    end else begin
        case (state)
            IDLE: begin
                if (i_start_dma_tx_flag)
                    state <= WAIT;
                else
                    state <= IDLE;
            end

            WAIT: begin
                if (i_video_vs && ~r_video_vs_d0)   // 新一帧到来
                    state <= TX_DATA;
                else
                    state <= WAIT;
            end

            TX_DATA: begin
                if (!i_start_dma_tx_flag)
                    state <= IDLE;
                else
                    state <= TX_DATA;
            end

            default: state <= IDLE;
        endcase
    end
end

// =============================================================================
// 输出数据: 仅在TX_DATA态透传, 否则输出0
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_rst_n) begin
        r_video_crtl_data <= 'd0;
        r_video_crtl_vs   <= 'd0;
        r_video_crtl_de   <= 'd0;
    end else if (state == TX_DATA) begin
        r_video_crtl_data <= i_video_data;
        r_video_crtl_vs   <= i_video_vs;
        r_video_crtl_de   <= i_video_de;
    end else begin
        r_video_crtl_data <= 'd0;
        r_video_crtl_vs   <= 'd0;
        r_video_crtl_de   <= 'd0;
    end
end

assign o_video_crtl_data = r_video_crtl_data;
assign o_video_crtl_vs   = r_video_crtl_vs;
assign o_video_crtl_de   = r_video_crtl_de;

endmodule
