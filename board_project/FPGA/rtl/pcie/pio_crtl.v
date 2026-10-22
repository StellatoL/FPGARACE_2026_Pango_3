// =============================================================================
// Module     : pio_crtl
// Description: PCIe PIO (Programmed I/O) 控制模块
//              通过BAR空间寄存器实现上位机对视频采集的控制
//              - 启动/停止采集 (addr=0x000)
//              - 4帧乒乓DMA基地址配置 (addr=0x020~0x054)
//              - DMA配置使能 (addr=0x060)
//              - 帧完成状态读取 (addr=0x140)
// =============================================================================

module pio_crtl (
    input  wire         pcie_clk            /* synthesis PAP_MARK_DEBUG="true" */,
    input  wire         rst_n,

    // ---- 控制输出 ----
    output wire         start_flag          /* synthesis PAP_MARK_DEBUG="true" */,  // DMA传输启动标志
    output wire         set_dma_config_en   /* synthesis PAP_MARK_DEBUG="true" */,  // DMA配置使能
    output wire [63:0]  o_base_addr     /* synthesis PAP_MARK_DEBUG="true" */,  // 帧缓冲0基地址
    output wire [63:0]  o_base_addr2    /* synthesis PAP_MARK_DEBUG="true" */,  // 帧缓冲1基地址
    output wire [63:0]  o_base_addr3    /* synthesis PAP_MARK_DEBUG="true" */,  // 帧缓冲2基地址
    output wire [63:0]  o_base_addr4    /* synthesis PAP_MARK_DEBUG="true" */,  // 帧缓冲3基地址
    input  wire         i_wr_frame_done     /* synthesis PAP_MARK_DEBUG="true" */,  // 帧写入完成
    input  wire [1:0]   i_wr_index          /* synthesis PAP_MARK_DEBUG="true" */,  // 当前写帧索引

    // ---- PIO 写通道 ----
    input  wire         pio_wr_en           /* synthesis PAP_MARK_DEBUG="true" */,
    input  wire [9:0]   pio_wr_addr         /* synthesis PAP_MARK_DEBUG="true" */,
    input  wire [31:0]  pio_wr_data         /* synthesis PAP_MARK_DEBUG="true" */,

    // ---- PIO 读通道 ----
    input  wire         pio_rd_en           /* synthesis PAP_MARK_DEBUG="true" */,
    input  wire [9:0]   pio_rd_addr         /* synthesis PAP_MARK_DEBUG="true" */,
    output wire [31:0]  pio_rd_data         /* synthesis PAP_MARK_DEBUG="true" */
);

// =============================================================================
// 寄存器地址定义
// =============================================================================
localparam WR_FRAME_DONE   = 9'h140;    // 帧完成状态 (读)
localparam DMA_ADDR_L      = 9'h050;    // 帧缓冲0 低32位地址
localparam DMA_ADDR_H      = 9'h054;    // 帧缓冲0 高32位地址
localparam DMA_ADDR1_L     = 9'h020;    // 帧缓冲1 低32位地址
localparam DMA_ADDR1_H     = 9'h024;    // 帧缓冲1 高32位地址
localparam DMA_ADDR2_L     = 9'h040;    // 帧缓冲2 低32位地址
localparam DMA_ADDR2_H     = 9'h044;    // 帧缓冲2 高32位地址
localparam DMA_ADDR3_L     = 9'h030;    // 帧缓冲3 低32位地址
localparam DMA_ADDR3_H     = 9'h034;    // 帧缓冲3 高32位地址
localparam DMA_SET_EN      = 9'h060;    // DMA配置使能

// =============================================================================
// 内部寄存器
// =============================================================================
reg         r_start_flag;
reg         r_set_dma_config_en;
reg [31:0]  r_pio_rd_data;
reg         r_wr_frame_done;
reg [63:0]  r_base_addr      /* synthesis PAP_MARK_DEBUG="true" */;
reg [63:0]  r_base_addr2     /* synthesis PAP_MARK_DEBUG="true" */;
reg [63:0]  r_base_addr3     /* synthesis PAP_MARK_DEBUG="true" */;
reg [63:0]  r_base_addr4     /* synthesis PAP_MARK_DEBUG="true" */;

// =============================================================================
// 启动/停止控制 (addr=0x000)
//   0xffffffe5: 启动采集
//   0xffffff00: 停止采集
// =============================================================================
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_start_flag <= 1'd0;
    else if (pio_wr_en && pio_wr_addr == 10'd0 && pio_wr_data == 32'hffffffe5)
        r_start_flag <= 1'd1;
    else if (pio_wr_en && pio_wr_addr == 10'd0 && pio_wr_data == 32'hffffff00)
        r_start_flag <= 1'd0;
end

// =============================================================================
// DMA配置使能 (addr=0x060)
//   写入0x00000001: 锁存DMA长度和基地址
// =============================================================================
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_set_dma_config_en <= 1'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_SET_EN && pio_wr_data == 32'h00000001)
        r_set_dma_config_en <= 1'd1;
    else
        r_set_dma_config_en <= 1'd0;
end

// =============================================================================
// 帧缓冲基地址寄存器 (64位地址, 分高低32位写入)
// =============================================================================
// base_addr
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr[31:0] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR_L)
        r_base_addr[31:0] <= pio_wr_data;
end

always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr[63:32] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR_H)
        r_base_addr[63:32] <= pio_wr_data;
end

// base_addr2
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr2[31:0] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR1_L)
        r_base_addr2[31:0] <= pio_wr_data;
end

always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr2[63:32] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR1_H)
        r_base_addr2[63:32] <= pio_wr_data;
end

// base_addr3
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr3[31:0] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR2_L)
        r_base_addr3[31:0] <= pio_wr_data;
end

always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr3[63:32] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR2_H)
        r_base_addr3[63:32] <= pio_wr_data;
end

// base_addr4
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr4[31:0] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR3_L)
        r_base_addr4[31:0] <= pio_wr_data;
end

always @(posedge pcie_clk) begin
    if (!rst_n)
        r_base_addr4[63:32] <= 'd0;
    else if (pio_wr_en && pio_wr_addr == DMA_ADDR3_H)
        r_base_addr4[63:32] <= pio_wr_data;
end

// =============================================================================
// 帧完成状态 (高1bit=完成标志, 低2bit=帧索引)
//   读后自动清除完成标志
// =============================================================================
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_wr_frame_done <= 1'd0;
    else if (pio_rd_en && pio_rd_addr == WR_FRAME_DONE)
        r_wr_frame_done <= 1'd0;
    else if (i_wr_frame_done)
        r_wr_frame_done <= 1'd1;
end

// =============================================================================
// PIO读数据返回
// =============================================================================
always @(posedge pcie_clk) begin
    if (!rst_n)
        r_pio_rd_data <= 32'd0;
    else if (pio_rd_en) begin
        case (pio_rd_addr)
            WR_FRAME_DONE: r_pio_rd_data <= {29'd0, i_wr_index, r_wr_frame_done};
            default:       r_pio_rd_data <= 32'd0;
        endcase
    end
end

// =============================================================================
// 输出端口
// =============================================================================
assign start_flag       = r_start_flag;
assign set_dma_config_en = r_set_dma_config_en;
assign pio_rd_data      = r_pio_rd_data;
assign o_base_addr  = r_base_addr;
assign o_base_addr2 = r_base_addr2;
assign o_base_addr3 = r_base_addr3;
assign o_base_addr4 = r_base_addr4;

endmodule
