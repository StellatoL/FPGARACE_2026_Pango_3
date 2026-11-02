// =============================================================================
// Module     : pcie_tx_fun
// Description: PCIe TX组包与DMA发送控制模块
//              - 视频数据通过异步FIFO跨域 (video_clk -> pcie_clk)
//              - 乒乓缓冲: 4帧地址空间, 自动切换
//              - 行级DMA请求: 每累积PACKET_LEN像素发起一次DMA
//              - 帧完成检测与索引管理
// =============================================================================
`default_nettype wire

module pcie_tx_fun #(
    parameter IMG_WIDTH          = 1280,
    parameter IMG_HEIGHT         = 720,
    parameter DMA_ADDR_WIDTH     = 64,         // DMA地址位宽
    parameter VIDEO_DATA_WIDTH   = 24,         // 视频数据位宽 (RGB888=24bit)
    parameter PCIE_DATA_WIDTH    = 256,        // PCIe数据位宽
    parameter PPC                = 4,          // 每时钟像素数
    parameter PIXCEL_BYTES       = 2,          // 每像素字节数
    parameter DMA_LEN            = 2560        // 单次DMA长度 (字节)
)(
    input  wire                              i_pcie_clk,
    input  wire                              i_pcie_rst_n,

    // ---- 配置接口 ----
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr2,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr3,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr4,
    input  wire                              i_dma_set_en,
    output wire [1:0]                        o_dma_wr_index     /* synthesis PAP_MARK_DEBUG="true" */,
    output wire                              o_dma_wr_done      /* synthesis PAP_MARK_DEBUG="true" */,
    input  wire                              i_start_tx_flag    /* synthesis PAP_MARK_DEBUG="true" */,

    // ---- DMA 端口 ----
    input  wire                              i_dma_cmd_rdy,
    output wire                              o_dma_req,
    output wire [DMA_ADDR_WIDTH-1:0]         o_dma_addr,
    input  wire                              i_dma_tx_done      /* synthesis PAP_MARK_DEBUG="true" */,

    // ---- 视频数据输入 (video_clk域) ----
    input  wire                              i_video_clk,
    input  wire                              i_video_rst_n,
    input  wire [VIDEO_DATA_WIDTH*PPC-1:0]   i_video_data,
    input  wire                              i_video_vs,
    input  wire                              i_video_de,

    // ---- 组包数据输出 (pcie_clk域) ----
    output wire [PCIE_DATA_WIDTH-1:0]        o_dma_rd_data,
    input  wire                              i_dma_rd_en
);

// =============================================================================
// 参数计算
// =============================================================================
localparam IMG_SIZE    = IMG_WIDTH * IMG_HEIGHT * PIXCEL_BYTES;
localparam SEND_TIMES  = IMG_SIZE / DMA_LEN;              // 一帧DMA次数
localparam PACKET_LEN  = DMA_LEN / PIXCEL_BYTES / PPC;    // 一次组包像素数
localparam RD_MAX_TIME = DMA_LEN / PCIE_DATA_WIDTH * 8;

// =============================================================================
// FIFO控制信号
// =============================================================================
wire        w_wr_fifo_rst;       // 写FIFO复位
wire        w_rd_fifo_rst;       // 读FIFO复位
wire        w_fifo_wr_full;
wire        w_fifo_rd_empty;
wire        w_fifo_wr_en;
wire        w_fifo_rd_en;
wire [PCIE_DATA_WIDTH-1:0] w_fifo_rd_data;
wire [9:0] fifo_read_words;
reg dma_inflight;

// =============================================================================
// Video时钟域寄存器
// =============================================================================
reg                             r_video_start_tx_flag_d0;
reg                             r_video_start_tx_flag_d1;
reg [15:0]                      r_wr_cnt;                // 写数据计数
reg                             r_line_req;              // 行请求标志
reg                             r_video_dma_req_d0;
reg                             r_video_dma_req_d1;
reg                             r_video_dma_req_d2;
reg                             r_video_vs;
reg                             r_video_vs_en;
reg [7:0]                       r_video_vs_ext_cnt;
reg                             r_video_vs_rst;
reg                             r_fifo_overflow /* synthesis PAP_MARK_DEBUG="true" */;

// =============================================================================
// PCIe时钟域寄存器
// =============================================================================
reg                             r_pci_line_req_d0;
reg                             r_pci_line_req_d1;
reg                             r_pci_line_req_d2;
reg                             r_dma_req;               // DMA请求
reg [15:0]                      r_rd_cnt;                // 读数据计数
reg [15:0]                      r_frame_cnt              /* synthesis PAP_MARK_DEBUG="true" */;
reg                             r_frame_done             /* synthesis PAP_MARK_DEBUG="true" */;
reg                             r_frame_done_d0;
reg                             r_frame_done_d1;
reg                             r_frame_done_d2          /* synthesis PAP_MARK_DEBUG="true" */;
reg [1:0]                       r_dma_index              /* synthesis PAP_MARK_DEBUG="true" */;
reg [1:0]                       r_dma_index_d0           /* synthesis PAP_MARK_DEBUG="true" */;
reg [DMA_ADDR_WIDTH-1:0]        r_dma_base_addr;
reg [DMA_ADDR_WIDTH-1:0]        r_dma_addr;
reg                             r_pcie_vs_rst_d0;
reg                             r_pcie_vs_rst_d1;
reg                             r_fifo_underflow /* synthesis PAP_MARK_DEBUG="true" */;

// =============================================================================
// FIFO复位控制
// =============================================================================
// Reset both sides at each accepted frame boundary.  This prevents a partial
// or aborted frame from shifting every subsequent RGB565 pixel in the FIFO.
reg video_ready_meta, video_ready_sync;
always @(posedge i_pcie_clk or negedge i_pcie_rst_n) begin
    if(!i_pcie_rst_n) begin video_ready_meta<=0;video_ready_sync<=0; end
    else begin video_ready_meta<=i_video_rst_n;video_ready_sync<=video_ready_meta; end
end
assign w_wr_fifo_rst = i_video_rst_n & r_video_start_tx_flag_d1 & ~r_video_vs_rst;
assign w_rd_fifo_rst = video_ready_sync & i_start_tx_flag & ~r_pcie_vs_rst_d1;

// Never write a full FIFO or read an empty FIFO.  An overflow invalidates the
// remainder of the current frame; the VS reset above starts the next frame at
// a clean 128-bit packing boundary.
assign w_fifo_wr_en = i_video_de & ~w_fifo_wr_full & ~r_fifo_overflow;
assign w_fifo_rd_en = i_dma_rd_en & ~w_fifo_rd_empty;
// Keep the FIFO output visible through the final successful read cycle.  The
// empty flag may assert at the same edge that produces the last valid word.
assign o_dma_rd_data = w_fifo_rd_data;

// =============================================================================
// Video时钟域: 跨域信号同步
// =============================================================================
always @(posedge i_video_clk) begin
    r_video_start_tx_flag_d0 <= i_start_tx_flag;
    r_video_start_tx_flag_d1 <= r_video_start_tx_flag_d0;
    r_video_dma_req_d0       <= r_dma_req;
    r_video_dma_req_d1       <= r_video_dma_req_d0;
    r_video_dma_req_d2       <= r_video_dma_req_d1;
    r_video_vs               <= i_video_vs;
end

always @(posedge i_video_clk) begin
    if (!i_video_rst_n || !r_video_start_tx_flag_d1 ||
        (i_video_vs && ~r_video_vs))
        r_fifo_overflow <= 1'b0;
    else if (i_video_de && w_fifo_wr_full)
        r_fifo_overflow <= 1'b1;
end

always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || !i_start_tx_flag || r_pcie_vs_rst_d1)
        r_fifo_underflow <= 1'b0;
    else if (i_dma_rd_en && w_fifo_rd_empty)
        r_fifo_underflow <= 1'b1;
end

// =============================================================================
// VS上升沿扩展 (用于复位控制)
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_video_rst_n)
        r_video_vs_en <= 1'd0;
    else if (r_video_vs_ext_cnt == 8'd15 && r_video_vs_en)
        r_video_vs_en <= 1'd0;
    else if (i_video_vs && ~r_video_vs)
        r_video_vs_en <= 1'd1;
end

always @(posedge i_video_clk) begin
    if (!i_video_rst_n) begin
        r_video_vs_rst     <= 1'd0;
        r_video_vs_ext_cnt <= 'd0;
    end else if (r_video_vs_ext_cnt == 8'd15 && r_video_vs_en) begin
        r_video_vs_rst     <= 1'd0;
        r_video_vs_ext_cnt <= 'd0;
    end else if (r_video_vs_en) begin
        r_video_vs_ext_cnt <= r_video_vs_ext_cnt + 1'b1;
        r_video_vs_rst     <= 1'd1;
    end
end

// =============================================================================
// PCIe时钟域: 跨域信号同步
// =============================================================================
always @(posedge i_pcie_clk) begin
    r_pci_line_req_d0 <= r_line_req;
    r_pci_line_req_d1 <= r_pci_line_req_d0;
    r_pci_line_req_d2 <= r_pci_line_req_d1;
    r_frame_done_d0   <= r_frame_done;
    r_frame_done_d1   <= r_frame_done_d0;
    r_frame_done_d2   <= r_frame_done_d1;
    r_pcie_vs_rst_d0  <= r_video_vs_rst;
    r_pcie_vs_rst_d1  <= r_pcie_vs_rst_d0;
end

// =============================================================================
// 写数据计数: 统计是否写满一包DMA数据
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_video_rst_n || r_video_vs_rst || r_video_start_tx_flag_d1 == 0)
        r_wr_cnt <= 'd0;
    else if (w_fifo_wr_en && r_wr_cnt == PACKET_LEN - 1)
        r_wr_cnt <= 'd0;
    else if (w_fifo_wr_en)
        r_wr_cnt <= r_wr_cnt + 1'b1;
end

// =============================================================================
// 行请求标志: 每写满一包数据产生一次行请求
// =============================================================================
always @(posedge i_video_clk) begin
    if (!i_video_rst_n || r_video_vs_rst || r_video_start_tx_flag_d1 == 0)
        r_line_req <= 1'd0;
    else if (r_video_dma_req_d2)
        r_line_req <= 1'd0;
    else if (r_fifo_overflow)
        r_line_req <= 1'd0;
    else if (w_fifo_wr_en && r_wr_cnt == PACKET_LEN - 1)
        r_line_req <= 1'd1;
end

// =============================================================================
// DMA请求: 检测到行请求且DMA就绪时发起
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || !video_ready_sync || !i_start_tx_flag || r_pcie_vs_rst_d1)
        r_dma_req <= 1'd0;
    else if (!r_dma_req && !dma_inflight && i_dma_cmd_rdy && fifo_read_words >= RD_MAX_TIME)
        r_dma_req <= 1'd1;
    else r_dma_req <= 1'd0;
end

// Track a submitted row until its DMA completion, including during a source
// change. A new source cannot issue a row while the old row is still in flight.
always @(posedge i_pcie_clk) begin
    if(!i_pcie_rst_n || !i_start_tx_flag) dma_inflight<=0;
    else if(r_dma_req) dma_inflight<=1;
    else if(i_dma_tx_done) dma_inflight<=0;
end

// =============================================================================
// 读数据计数: 统计读出一包DMA数据
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n)
        r_rd_cnt <= 'd0;
    else if (i_dma_rd_en && r_rd_cnt == RD_MAX_TIME - 1)
        r_rd_cnt <= 'd0;
    else if (i_dma_rd_en)
        r_rd_cnt <= r_rd_cnt + 1'b1;
end

// =============================================================================
// 帧发送次数计数
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || !video_ready_sync || r_pcie_vs_rst_d1 || i_start_tx_flag == 0)
        r_frame_cnt <= 'd0;
    else if (i_dma_tx_done && r_frame_cnt == SEND_TIMES - 1)
        r_frame_cnt <= 'd0;
    else if (i_dma_tx_done)
        r_frame_cnt <= r_frame_cnt + 1'b1;
end

// =============================================================================
// 帧完成标志
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || !video_ready_sync || r_pcie_vs_rst_d1 || i_start_tx_flag == 0)
        r_frame_done <= 1'd0;
    else if (i_dma_tx_done && r_frame_cnt == SEND_TIMES - 1)
        r_frame_done <= 1'd1;
    else
        r_frame_done <= 1'd0;
end

// =============================================================================
// 帧索引管理: 4帧乒乓, 每帧完成切换到下一地址空间
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || i_start_tx_flag == 0)
        r_dma_index <= 'd0;
    else if (r_frame_done && r_dma_index == 2'd3)
        r_dma_index <= 'd0;
    else if (r_frame_done)
        r_dma_index <= r_dma_index + 1'b1;
end

// 记录已写完的帧索引
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n || i_start_tx_flag == 0)
        r_dma_index_d0 <= 'd0;
    else if (r_frame_done)
        r_dma_index_d0 <= r_dma_index;
end

// =============================================================================
// DMA基地址切换 (4帧乒乓地址)
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n)
        r_dma_base_addr <= 'd0;
    else if (i_start_tx_flag == 0)
        r_dma_base_addr <= i_dma_base_addr;
    else if (r_frame_done_d0) begin
        case (r_dma_index)
            2'd0:       r_dma_base_addr <= i_dma_base_addr;
            2'd1:       r_dma_base_addr <= i_dma_base_addr2;
            2'd2:       r_dma_base_addr <= i_dma_base_addr3;
            2'd3:       r_dma_base_addr <= i_dma_base_addr4;
            default:    r_dma_base_addr <= i_dma_base_addr;
        endcase
    end
end

// =============================================================================
// DMA地址累加
// =============================================================================
always @(posedge i_pcie_clk) begin
    if (!i_pcie_rst_n)
        r_dma_addr <= 'd0;
    else if (i_start_tx_flag == 0)
        r_dma_addr <= i_dma_base_addr;
    else if (i_dma_set_en)
        r_dma_addr <= i_dma_base_addr;
    else if (r_frame_done_d1 || r_pcie_vs_rst_d1)
        r_dma_addr <= r_dma_base_addr;
    else if (i_dma_tx_done)
        r_dma_addr <= r_dma_addr + DMA_LEN;
end

// =============================================================================
// 端口输出
// =============================================================================
assign o_dma_wr_index = r_dma_index_d0[1:0];
assign o_dma_wr_done  = r_frame_done;
assign o_dma_req      = r_dma_req;
assign o_dma_addr     = r_dma_addr;

// =============================================================================
// 视频缓冲FIFO: video_clk -> pcie_clk 跨域
// =============================================================================
video_buffer video_buffer_inst (
    .wr_clk         (i_video_clk),
    .wr_rst         (~w_wr_fifo_rst),
    .wr_en          (w_fifo_wr_en),
    .wr_data        (i_video_data),
    .wr_full        (w_fifo_wr_full),
    .wr_water_level (),
    .almost_full    (),
    .rd_clk         (i_pcie_clk),
    .rd_rst         (~w_rd_fifo_rst),
    .rd_en          (w_fifo_rd_en),
    .rd_data        (w_fifo_rd_data),
    .rd_empty       (w_fifo_rd_empty),
    .rd_water_level (fifo_read_words),
    .almost_empty   ()
);

endmodule
