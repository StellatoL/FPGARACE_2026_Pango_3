// =============================================================================
// Module     : fram_buf
// Description: DDR3 帧缓存顶层模块
//              集成写缓冲(wr_buf)、读缓冲(rd_buf)和AXI读写控制器(wr_rd_ctrl_top)
//              - 写端口: 视频输入数据写入DDR3
//              - 读端口: 从DDR3读取数据用于显示
//              - AXI接口: 连接到DDR3控制器
// =============================================================================
`timescale 1ns / 1ps
`define UD #1

module fram_buf #(
    parameter MEM_ROW_WIDTH    = 15,
    parameter MEM_COLUMN_WIDTH = 10,
    parameter MEM_BANK_WIDTH   = 3,
    parameter CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter MEM_DQ_WIDTH     = 16,
    parameter H_NUM            = 12'd1280,       // 水平有效像素数
    parameter V_NUM            = 12'd720,        // 垂直有效行数
    parameter PIX_WIDTH        = 16,             // 像素数据位宽 (RGB565=16bit)
    parameter ADDR_OFFSET      = 0,              // frame base in DDR address units
    parameter LINE_ADDR_WIDTH = 22,
    parameter QUAD_MODE       = 0
)(
    // ---- 写端口 (视频数据输入) ----
    input  wire                          vin_clk,
    input  wire                          wr_fsync,
    input  wire                          wr_en,
    input  wire [PIX_WIDTH-1:0]          wr_data,
    output reg                           init_done = 0,

    // ---- 系统时钟与复位 ----
    input  wire                          ddr_clk,
    input  wire                          ddr_rstn,

    // ---- 读端口 (视频数据输出) ----
    input  wire                          vout_clk,
    input  wire                          rd_fsync,
    input  wire                          rd_en,
    output wire                          vout_de,
    output wire [PIX_WIDTH-1:0]          vout_data,

    // ---- AXI 写地址通道 ----
    output wire [CTRL_ADDR_WIDTH-1:0]     axi_awaddr,
    output wire [3:0]                     axi_awid,
    output wire [3:0]                     axi_awlen,
    output wire [2:0]                     axi_awsize,
    output wire [1:0]                     axi_awburst,
    input  wire                           axi_awready,
    output wire                           axi_awvalid,

    // ---- AXI 写数据通道 ----
    output wire [MEM_DQ_WIDTH*8-1:0]      axi_wdata,
    output wire [MEM_DQ_WIDTH-1:0]        axi_wstrb,
    input  wire                           axi_wlast,
    output wire                           axi_wvalid,
    input  wire                           axi_wready,
    input  wire [3:0]                     axi_bid,

    // ---- AXI 读地址通道 ----
    output wire [CTRL_ADDR_WIDTH-1:0]     axi_araddr,
    output wire [3:0]                     axi_arid,
    output wire [3:0]                     axi_arlen,
    output wire [2:0]                     axi_arsize,
    output wire [1:0]                     axi_arburst,
    output wire                           axi_arvalid,
    input  wire                           axi_arready,

    // ---- AXI 读数据通道 ----
    output wire                           axi_rready,
    input  wire [MEM_DQ_WIDTH*8-1:0]      axi_rdata,
    input  wire                           axi_rvalid,
    input  wire                           axi_rlast,
    input  wire [3:0]                     axi_rid
);

// =============================================================================
// 内部参数
// =============================================================================
localparam LEN_WIDTH       = 32;
localparam FRAME_CNT_WIDTH = CTRL_ADDR_WIDTH - LINE_ADDR_WIDTH;

// =============================================================================
// 写缓冲 <-> AXI控制器 互联信号
// =============================================================================
wire                            ddr_wreq;
wire [CTRL_ADDR_WIDTH-1:0]      ddr_waddr;
wire [LEN_WIDTH-1:0]            ddr_wr_len;
wire                            ddr_wrdy;
wire                            ddr_wdone;
wire [8*MEM_DQ_WIDTH-1:0]       ddr_wdata;
wire                            ddr_wdata_req;
wire                            ddr_wr_bac;
wire                            frame_wirq;        // 帧写中断 (首帧写入完成)
wire [FRAME_CNT_WIDTH-1:0] write_frame;
wire [1:0] pinned_frame;
reg [1:0] completed_frame;
reg [1:0] write_frame_d;
reg [11:0] completed_lines;

// =============================================================================
// 读缓冲 <-> AXI控制器 互联信号
// =============================================================================
wire                            rd_cmd_en;
wire [CTRL_ADDR_WIDTH-1:0]      rd_cmd_addr;
wire [LEN_WIDTH-1:0]            rd_cmd_len;
wire                            rd_cmd_ready;
wire                            rd_cmd_done;
wire                            read_ready = 1'b1;
wire [MEM_DQ_WIDTH*8-1:0]       read_rdata;
wire                            read_en;

// =============================================================================
// 写缓冲 (视频流 -> DDR3写命令+FIFO)
// =============================================================================
wr_buf #(
    .ADDR_WIDTH         (CTRL_ADDR_WIDTH),
    .ADDR_OFFSET        (ADDR_OFFSET),
    .H_NUM              (H_NUM),
    .V_NUM              (V_NUM),
    .DQ_WIDTH           (MEM_DQ_WIDTH),
    .LEN_WIDTH          (LEN_WIDTH),
    .PIX_WIDTH          (PIX_WIDTH),
    .LINE_ADDR_WIDTH    (LINE_ADDR_WIDTH),
    .FRAME_CNT_WIDTH    (FRAME_CNT_WIDTH),
    .QUAD_MODE          (QUAD_MODE)
) u_wr_buf (
    .ddr_clk            (ddr_clk),
    .ddr_rstn           (ddr_rstn),
    .wr_clk             (vin_clk),
    .wr_fsync           (wr_fsync),
    .wr_en              (wr_en),
    .wr_data            (wr_data),
    .rd_bac             (ddr_wr_bac),
    .ddr_wreq           (ddr_wreq),
    .ddr_waddr          (ddr_waddr),
    .ddr_wr_len         (ddr_wr_len),
    .ddr_wrdy           (ddr_wrdy),
    .ddr_wdone          (ddr_wdone),
    .ddr_wdata          (ddr_wdata),
    .ddr_wdata_req      (ddr_wdata_req),
    .frame_wcnt         (write_frame),
    .pinned_frame       (pinned_frame),
    .completed_frame    (completed_frame),
    .frame_wirq         (frame_wirq)
);

// init_done: 首帧写入完成后拉高, 指示可以开始读取
always @(posedge ddr_clk or negedge ddr_rstn) begin
    if (!ddr_rstn) begin
        init_done <= 0; completed_frame <= 0;
        write_frame_d <= 0; completed_lines <= 0;
    end else if (QUAD_MODE) begin
        write_frame_d <= write_frame[1:0];
        if (write_frame_d != write_frame[1:0]) completed_lines <= 0;
        else if (ddr_wdone) begin
            if (completed_lines == V_NUM-1) begin
                completed_frame <= write_frame[1:0];
                init_done <= 1;
                completed_lines <= 0;
            end else completed_lines <= completed_lines + 1'b1;
        end
    end else if (frame_wirq) init_done <= 1;
end

// =============================================================================
// 读缓冲 (DDR3读数据 -> 视频流输出)
// =============================================================================
rd_buf #(
    .ADDR_WIDTH         (CTRL_ADDR_WIDTH),
    .ADDR_OFFSET        (ADDR_OFFSET),
    .H_NUM              (H_NUM),
    .V_NUM              (V_NUM),
    .DQ_WIDTH           (MEM_DQ_WIDTH),
    .LEN_WIDTH          (LEN_WIDTH),
    .PIX_WIDTH          (PIX_WIDTH),
    .LINE_ADDR_WIDTH    (LINE_ADDR_WIDTH),
    .FRAME_CNT_WIDTH    (FRAME_CNT_WIDTH),
    .QUAD_MODE          (QUAD_MODE)
) u_rd_buf (
    .ddr_clk            (ddr_clk),
    .ddr_rstn           (ddr_rstn),
    .vout_clk           (vout_clk),
    .rd_fsync           (rd_fsync),
    .rd_en              (rd_en),
    .vout_de            (vout_de),
    .vout_data          (vout_data),
    .init_done          (init_done),
    .completed_frame    (completed_frame),
    .pinned_frame       (pinned_frame),
    .ddr_rreq           (rd_cmd_en),
    .ddr_raddr          (rd_cmd_addr),
    .ddr_rd_len         (rd_cmd_len),
    .ddr_rrdy           (rd_cmd_ready),
    .ddr_rdone          (rd_cmd_done),
    .ddr_rdata          (read_rdata),
    .ddr_rdata_en       (read_en)
);

// =============================================================================
// AXI 读写控制器
// =============================================================================
wr_rd_ctrl_top #(
    .CTRL_ADDR_WIDTH    (CTRL_ADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH)
) u_wr_rd_ctrl_top (
    .clk                (ddr_clk),
    .rstn               (ddr_rstn),
    // 写命令通道
    .wr_cmd_en          (ddr_wreq),
    .wr_cmd_addr        (ddr_waddr),
    .wr_cmd_len         (ddr_wr_len),
    .wr_cmd_ready       (ddr_wrdy),
    .wr_cmd_done        (ddr_wdone),
    .wr_bac             (ddr_wr_bac),
    .wr_ctrl_data       (ddr_wdata),
    .wr_data_re         (ddr_wdata_req),
    // 读命令通道
    .rd_cmd_en          (rd_cmd_en),
    .rd_cmd_addr        (rd_cmd_addr),
    .rd_cmd_len         (rd_cmd_len),
    .rd_cmd_ready       (rd_cmd_ready),
    .rd_cmd_done        (rd_cmd_done),
    // 读数据返回
    .read_ready         (read_ready),
    .read_rdata         (read_rdata),
    .read_en            (read_en),
    // AXI 写通道
    .axi_awaddr         (axi_awaddr),
    .axi_awid           (axi_awid),
    .axi_awlen          (axi_awlen),
    .axi_awsize         (axi_awsize),
    .axi_awburst        (axi_awburst),
    .axi_awready        (axi_awready),
    .axi_awvalid        (axi_awvalid),
    .axi_wdata          (axi_wdata),
    .axi_wstrb          (axi_wstrb),
    .axi_wlast          (axi_wlast),
    .axi_wvalid         (axi_wvalid),
    .axi_wready         (axi_wready),
    .axi_bid            (4'd0),
    .axi_bresp          (2'd0),
    .axi_bvalid         (1'b0),
    .axi_bready         (),
    // AXI 读通道
    .axi_araddr         (axi_araddr),
    .axi_arid           (axi_arid),
    .axi_arlen          (axi_arlen),
    .axi_arsize         (axi_arsize),
    .axi_arburst        (axi_arburst),
    .axi_arvalid        (axi_arvalid),
    .axi_arready        (axi_arready),
    .axi_rready         (axi_rready),
    .axi_rdata          (axi_rdata),
    .axi_rvalid         (axi_rvalid),
    .axi_rlast          (axi_rlast),
    .axi_rid            (axi_rid),
    .axi_rresp          (2'd0)
);

endmodule
