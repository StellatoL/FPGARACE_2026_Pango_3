// =============================================================================
// Module     : wr_rd_ctrl_top
// Description: DDR3 AXI 读写命令转换控制器
//              将内部读写命令协议转换为标准AXI4总线协议
//              - 集成 wr_cmd_trans: 写命令FIFO管理与转换
//              - 集成 wr_ctrl: AXI写通道控制
//              - 集成 rd_ctrl: AXI读通道控制
// =============================================================================
`timescale 1ns / 1ps
`define UD #1

module wr_rd_ctrl_top #(
    parameter CTRL_ADDR_WIDTH    = 28,
    parameter MEM_DQ_WIDTH       = 16
)(
    input  wire                          clk,
    input  wire                          rstn,

    // ---- 写命令通道 (来自wr_buf) ----
    input  wire                          wr_cmd_en,
    input  wire [CTRL_ADDR_WIDTH-1:0]    wr_cmd_addr,
    input  wire [31:0]                   wr_cmd_len,
    output wire                          wr_cmd_ready,
    output wire                          wr_cmd_done,
    output wire                          wr_bac,
    input  wire [MEM_DQ_WIDTH*8-1:0]     wr_ctrl_data,
    output wire                          wr_data_re,

    // ---- 读命令通道 (来自rd_buf) ----
    input  wire                          rd_cmd_en,
    input  wire [CTRL_ADDR_WIDTH-1:0]    rd_cmd_addr,
    input  wire [31:0]                   rd_cmd_len,
    output wire                          rd_cmd_ready,
    output wire                          rd_cmd_done,

    // ---- 读数据返回 (到rd_buf) ----
    input  wire                          read_ready  /* synthesis PAP_MARK_DEBUG="true" */,
    output wire [MEM_DQ_WIDTH*8-1:0]     read_rdata  /* synthesis PAP_MARK_DEBUG="true" */,
    output wire                          read_en     /* synthesis PAP_MARK_DEBUG="true" */,

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
    input  wire [1:0]                     axi_bresp,
    input  wire                           axi_bvalid,
    output wire                           axi_bready,

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
    input  wire [3:0]                     axi_rid,
    input  wire [1:0]                     axi_rresp
);

// =============================================================================
// wr_cmd_trans <-> wr_ctrl/rd_ctrl 互联信号
// =============================================================================
wire                            wr_en       /* synthesis PAP_MARK_DEBUG="true" */;
wire [CTRL_ADDR_WIDTH-1:0]      wr_addr     /* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0]                      wr_id       /* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0]                      wr_len      /* synthesis PAP_MARK_DEBUG="true" */;
wire                            wr_done     /* synthesis PAP_MARK_DEBUG="true" */;
wire                            wr_ready    /* synthesis PAP_MARK_DEBUG="true" */;
wire                            wr_data_en  /* synthesis PAP_MARK_DEBUG="true" */;
wire [MEM_DQ_WIDTH*8-1:0]       wr_data     /* synthesis PAP_MARK_DEBUG="true" */;

wire                            rd_en       /* synthesis PAP_MARK_DEBUG="true" */;
wire [CTRL_ADDR_WIDTH-1:0]      rd_addr     /* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0]                      rd_id       /* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0]                      rd_len      /* synthesis PAP_MARK_DEBUG="true" */;
wire                            rd_done_p   /* synthesis PAP_MARK_DEBUG="true" */;

// =============================================================================
// 写/读 命令转换模块
// =============================================================================
wr_cmd_trans #(
    .CTRL_ADDR_WIDTH    (CTRL_ADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH)
) u_wr_cmd_trans (
    .clk                (clk),
    .rstn               (rstn),
    // 写命令输入
    .wr_cmd_en          (wr_cmd_en),
    .wr_cmd_addr        (wr_cmd_addr),
    .wr_cmd_len         (wr_cmd_len),
    .wr_cmd_ready       (wr_cmd_ready),
    .wr_cmd_done        (wr_cmd_done),
    .wr_bac             (wr_bac),
    .wr_ctrl_data       (wr_ctrl_data),
    .wr_data_re         (wr_data_re),
    // 写命令输出
    .wr_en              (wr_en),
    .wr_addr            (wr_addr),
    .wr_id              (wr_id),
    .wr_len             (wr_len),
    .wr_data_en         (wr_data_en),
    .wr_data            (wr_data),
    .wr_ready           (wr_ready),
    .wr_done            (wr_done),
    // 读命令输入
    .rd_cmd_en          (rd_cmd_en),
    .rd_cmd_addr        (rd_cmd_addr),
    .rd_cmd_len         (rd_cmd_len),
    .rd_cmd_ready       (rd_cmd_ready),
    .rd_cmd_done        (rd_cmd_done),
    .read_en            (read_en),
    // 读命令输出
    .rd_en              (rd_en),
    .rd_addr            (rd_addr),
    .rd_id              (rd_id),
    .rd_len             (rd_len),
    .rd_done_p          (rd_done_p)
);

// =============================================================================
// AXI 写通道控制器
// =============================================================================
wr_ctrl #(
    .CTRL_ADDR_WIDTH    (CTRL_ADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH)
) u_wr_ctrl (
    .clk                (clk),
    .rst_n              (rstn),
    // 内部写接口
    .wr_en              (wr_en),
    .wr_addr            (wr_addr),
    .wr_id              (wr_id),
    .wr_len             (wr_len),
    .wr_cmd_done        (wr_done),
    .wr_ready           (wr_ready),
    .wr_data_en         (wr_data_en),
    .wr_data            (wr_data),
    .wr_bac             (wr_bac),
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
    .axi_bid            (axi_bid),
    .axi_bresp          (axi_bresp),
    .axi_bvalid         (axi_bvalid),
    .axi_bready         (axi_bready),
    .test_wr_state      ()
);

// =============================================================================
// AXI 读通道控制器
// =============================================================================
rd_ctrl #(
    .CTRL_ADDR_WIDTH    (CTRL_ADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH)
) u_rd_ctrl (
    .clk                (clk),
    .rst_n              (rstn),
    // 内部读接口
    .read_addr          (rd_addr),
    .read_id            (rd_id),
    .read_len           (rd_len),
    .read_en            (rd_en),
    .read_done_p        (rd_done_p),
    .read_ready         (read_ready),
    .read_rdata         (read_rdata),
    .read_rdata_en      (read_en),
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
    .axi_rresp          (axi_rresp)
);

endmodule
