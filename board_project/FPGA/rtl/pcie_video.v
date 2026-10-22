// =============================================================================
// Module     : pcie_video
// Description: PCIe视频采集与显示顶层模块
//              - 四路OV5640 -> 640x360 -> DDR3 -> 1280x720四宫格
//              - HDMI 1080p60 -> 720p30 -> PCIe，绕过预处理与DDR3
//              - PCIe Gen2 x2 DMA 传输视频数据到主机
//              - 摄像头拼接读出使用独立50MHz光栅时钟
// =============================================================================

module pcie_video #(
    parameter MEM_ROW_WIDTH       = 15,       // DDR3 行地址位宽
    parameter MEM_COLUMN_WIDTH    = 10,       // DDR3 列地址位宽
    parameter MEM_BANK_WIDTH      = 3,        // DDR3 Bank地址位宽
    parameter MEM_DQ_WIDTH        = 16,       // DDR3 数据位宽
    parameter MEM_DQS_WIDTH       = 2,        // DDR3 DQS位宽
    parameter [3:0] CAMERA_ENABLE = 4'b1111,  // enable all four independent camera paths
    parameter [3:0] CAMERA_TEST_PATTERN = 4'b1011 // test CAM1/CAM2/CAM4; CAM3 remains physical
)(
    // =========================================================================
    // Part 1: 系统时钟与复位
    // =========================================================================
    input  wire                    sys_clk,           // 系统时钟 27MHz
    input  wire                    button_rst_n,      // 按键复位, 低有效
    input  wire                    key1,              // 视频源切换按键

    // ARM/RK I2C control interface (FPGA slave address 7'h66)
    input  wire                    i2c_scl,
    inout  wire                    i2c_sda,

    // =========================================================================
    // Part 2: PCIe 接口
    // =========================================================================
    input  wire                    ref_clk_n,         // PCIe参考时钟差分对 100MHz (N)
    input  wire                    ref_clk_p,         // PCIe参考时钟差分对 100MHz (P)
    input  wire        [1:0]       rxn,               // PCIe接收差分对
    input  wire        [1:0]       rxp,               // PCIe接收差分对
    output wire        [1:0]       txn,               // PCIe发送差分对
    output wire        [1:0]       txp,               // PCIe发送差分对

    // =========================================================================
    // Part 3: PCIe 链路状态指示
    // =========================================================================
    output reg                     ref_led,           // PCIe参考时钟LED指示
    output reg                     pclk_led,          // PCIe用户时钟LED指示
    output wire                    smlh_link_up,      // PCIe Secondary Media Link Layer Up
    output wire                    rdlh_link_up,      // PCIe Receive Data Link Layer Up

    // =========================================================================
    // Part 4: DDR3 存储器接口
    // =========================================================================
    output wire                    mem_cs_n,          // 片选
    output wire                    mem_rst_n,         // 复位
    output wire                    mem_ck,            // 差分时钟正
    output wire                    mem_ck_n,          // 差分时钟负
    output wire                    mem_cke,           // 时钟使能
    output wire                    mem_ras_n,         // 行地址选通
    output wire                    mem_cas_n,         // 列地址选通
    output wire                    mem_we_n,          // 写使能
    output wire                    mem_odt,           // 片内终端电阻
    output wire [MEM_ROW_WIDTH-1:0]   mem_a,          // 地址总线
    output wire [MEM_BANK_WIDTH-1:0]  mem_ba,         // Bank地址
    inout  wire [MEM_DQS_WIDTH-1:0]   mem_dqs,        // 数据选通
    inout  wire [MEM_DQS_WIDTH-1:0]   mem_dqs_n,      // 数据选通(差分负)
    inout  wire [MEM_DQ_WIDTH-1:0]    mem_dq,         // 数据总线
    output wire [MEM_DQS_WIDTH-1:0]   mem_dm,         // 数据掩码
    output wire                    ddr_init_done,     // DDR3初始化完成标志
    output reg                     heart_beat_led,    // DDR3心跳指示灯

    // =========================================================================
    // Part 5: OV5640 摄像头接口
    // =========================================================================
    inout  wire                    cmos_scl,         // SCCB时钟
    inout  wire                    cmos_sda,         // SCCB数据
    input  wire                    cmos_vsync,       // 场同步 /* synthesis syn_keep = 1 */
    input  wire                    cmos_href,        // 行参考/数据有效 /* synthesis syn_keep = 1 */
    input  wire                    cmos_pclk,        // 像素时钟 /* synthesis syn_keep = 1 */
    input  wire        [7:0]       cmos_data,        // 像素数据 /* synthesis syn_keep = 1 */
    output wire                    cmos_reset,       // 摄像头复位
    // Additional independent OV5640 inputs. For the current single-camera
    // The existing camera connector is currently mapped to cmos3_* for testing;
    // bind cmos_*, cmos2_* and cmos4_* in the board constraints when the other
    // three camera connectors are available.
    inout wire cmos2_scl, inout wire cmos2_sda,
    input wire cmos2_vsync, input wire cmos2_href, input wire cmos2_pclk,
    input wire [7:0] cmos2_data, output wire cmos2_reset,
    inout wire cmos3_scl, inout wire cmos3_sda,
    input wire cmos3_vsync, input wire cmos3_href, input wire cmos3_pclk,
    input wire [7:0] cmos3_data, output wire cmos3_reset,
    inout wire cmos4_scl, inout wire cmos4_sda,
    input wire cmos4_vsync, input wire cmos4_href, input wire cmos4_pclk,
    input wire [7:0] cmos4_data, output wire cmos4_reset,

    // =========================================================================
    // Part 6: HDMI 输入接口
    // =========================================================================
    output wire                    hd_scl,            // HDMI I2C时钟 (MS72xx配置)
    inout  wire                    hd_sda,            // HDMI I2C数据 (MS72xx配置)
    input  wire                    pixclk_in,         // 像素时钟 (1080p ~148.5MHz) /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire                    vs_in,             // 场同步 /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire                    hs_in,             // 行同步 /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire                    de_in,             // 数据有效 /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire        [7:0]       r_in,              // 红色分量 /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire        [7:0]       g_in,              // 绿色分量 /*synthesis PAP_MARK_DEBUG="true"*/
    input  wire        [7:0]       b_in               // 蓝色分量 /*synthesis PAP_MARK_DEBUG="true"*/
);

// =============================================================================
// 内部参数定义
// =============================================================================
localparam CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH;
localparam TH_1S            = 27'd33_000_000;    // core_clk下1秒计数阈值

// =============================================================================
// 摄像头拼接输出参数: 1280x720, 50MHz/(1650*750)约40.4帧/秒
// =============================================================================
localparam V_TOTAL  = 12'd750;                    // 场扫描周期 (720+5+20+5)
localparam V_FP     = 12'd5;                      // 场显示前沿
localparam V_BP     = 12'd20;                     // 场显示后沿
localparam V_SYNC   = 12'd5;                      // 场同步脉冲
localparam V_ACT    = 12'd720;                    // 场有效数据
localparam H_TOTAL  = 12'd1650;                   // 行扫描周期 (1280+110+220+40)
localparam H_FP     = 12'd110;                    // 行显示前沿
localparam H_BP     = 12'd220;                    // 行显示后沿
localparam H_SYNC   = 12'd40;                     // 行同步脉冲
localparam H_ACT    = 12'd1280;                   // 行数据有效
localparam X_WIDTH  = 4'd12;                      // X坐标位宽
localparam Y_WIDTH  = 4'd12;                      // Y坐标位宽

// =============================================================================
// 视频DMA打包参数
// =============================================================================
localparam IMG_WIDTH     = 1280;                  // 图像宽度 (像素)
localparam IMG_HEIGHT    = 720;                   // 图像高度 (像素)
localparam PPC           = 1;                     // 每时钟像素数
localparam PIXCEL_BYTES  = 2;                     // 每像素字节数 (RGB565=2)
localparam DMA_LEN       = 2560;                  // 单次DMA传输字节数
localparam IMG_SIZE       = IMG_WIDTH * IMG_HEIGHT * PIXCEL_BYTES;
localparam SEND_TIMES     = IMG_SIZE / DMA_LEN;   // 一帧需要DMA传输的次数

// =============================================================================
// PCIe设备参数
// =============================================================================
localparam DEVICE_TYPE    = 3'b000;               // 0:EP, 4:RC
localparam AXIS_SLAVE_NUM = 3;                    // AXIS Slave通道数

// =============================================================================
// Part A: 时钟与复位信号
// =============================================================================
wire        cfg_clk;           // PLL输出的配置时钟
wire        clk_50m;           // PLL输出的50MHz时钟
wire        locked;            // PLL锁定标志
wire        core_clk;          // DDR3控制器输出的核心时钟
wire        core_rst_n;        // PCIe IP输出的核心复位 (低有效)
wire        pll_rst_n;         // PLL复位 (与core_rst_n同步)
wire        sync_button_rst_n; // 消抖后的按键复位
wire        sync_perst_n;      // 消抖后的PCIe复位
wire        ref_core_rst_n;    // ref_clk域的core复位同步信号
wire        s_pclk_rstn;       // pclk域的core复位同步信号
wire        rstn_out;          // 上电延迟后的系统复位输出
wire        perst_n;           // 按键复位别名

assign pll_rst_n = core_rst_n;
assign perst_n   = button_rst_n;

// =========================================================================
// Part B: PLL 时钟生成
// =========================================================================
PLL_50mHZ U_PLL (
    .clkout0    (cfg_clk),       // 配置时钟输出
    .clkout1    (clk_50m),       // 50MHz时钟输出
    .lock       (locked),        // 锁定指示
    .clkin1     (sys_clk)        // 27MHz输入
);

// =========================================================================
// Part B.1: ARM/RK I2C slave control
//   0x00: read/write loopback register (kept compatible with the manual)
//   0x01: video source command, bit[0]: 0=camera, 1=HDMI
//   0x02: read-only current video source, bit[0]: 0=camera, 1=HDMI
//   0x03: preprocessing master, bit[0]: 0=off, 1=on
//   0x04: plate box drawing master, bit[0]: 0=off, 1=on
//   0x05: white balance feature, bit[0]: 0=off, 1=on
//   0x06: dark-channel dehaze feature, bit[0]: 0=off, 1=on
// =========================================================================
wire [7:0] rk_i2c_reg0;
wire [7:0] rk_video_ctrl;
wire [7:0] rk_preprocess_ctrl;
wire [7:0] rk_box_draw_ctrl;
wire [7:0] rk_white_balance_ctrl;
wire [7:0] rk_dehaze_ctrl;
wire       rk_video_update_toggle;
wire       current_video_select;
reg        current_video_meta;
reg        current_video_i2c;

// Synchronize the readback status from sys_clk to the 50 MHz I2C domain.
always @(posedge clk_50m or negedge button_rst_n) begin
    if (!button_rst_n) begin
        current_video_meta <= 1'b0;
        current_video_i2c  <= 1'b0;
    end else begin
        current_video_meta <= current_video_select;
        current_video_i2c  <= current_video_meta;
    end
end

i2cSlave u_rk_i2c_slave (
    .clk               (clk_50m),
    .rst               (~button_rst_n | ~locked),
    .sda               (i2c_sda),
    .scl               (i2c_scl),
    .myReg0            (rk_i2c_reg0),
    .myReg1            (rk_video_ctrl),
    .myReg2            ({7'd0, current_video_i2c}),
    .myReg3            (rk_preprocess_ctrl),
    .myReg4            (rk_box_draw_ctrl),
    .myReg5            (rk_white_balance_ctrl),
    .myReg6            (rk_dehaze_ctrl),
    .videoUpdateToggle (rk_video_update_toggle)
);

// =========================================================================
// Part C: 上电复位延迟 (cfg_clk域, ~1ms)
// =========================================================================
reg [15:0] rstn_1ms;

always @(posedge cfg_clk) begin
    if (!locked)
        rstn_1ms <= 16'd0;
    else if (rstn_1ms != 16'h2710)
        rstn_1ms <= rstn_1ms + 1'b1;
end

assign rstn_out = (rstn_1ms == 16'h2710);

// =========================================================================
// Part D: OV5640 摄像头驱动
// =========================================================================
wire        cam_de;            // 摄像头数据有效
wire        cam_valid;         // 摄像头像素有效
wire        cam_vs;            // 摄像头场同步
wire [15:0] cam_data;          // 摄像头RGB565数据
wire        cam_clk;           // 摄像头像素时钟(2分频)
wire [3:0]  test_cam_vs, test_cam_de;
wire [63:0] test_cam_data;

// Test sources use the known-good camera-3 clock and frame timing.  This
// isolates DDR3 stitching from the electrical/configuration state of the
// other three camera connectors.
genvar test_source;
generate
  for (test_source=0; test_source<4; test_source=test_source+1) begin: TEST_SOURCE
    if (CAMERA_TEST_PATTERN[test_source]) begin: ENABLED
      camera_test_pattern u_pattern (
        .clk(camera_clk[2]), .rst_n(button_rst_n & locked), .in_vs(camera_vs[2]),
        .in_de(camera_de[2]),
        .out_vs(test_cam_vs[test_source]), .out_de(test_cam_de[test_source]),
        .out_data(test_cam_data[test_source*16 +: 16])
      );
    end else begin: DISABLED
      assign test_cam_vs[test_source]=1'b0;
      assign test_cam_de[test_source]=1'b0;
      assign test_cam_data[test_source*16 +: 16]=16'd0;
    end
  end
endgenerate

generate
if (CAMERA_TEST_PATTERN[0]) begin: CAM1_TEST
assign cmos_scl = 1'bz; assign cmos_sda = 1'bz; assign cmos_reset = 1'b0;
end else if (CAMERA_ENABLE[0]) begin: CAM1
ov5640 ov5640_inst (
    .video_clk      (),
    .clk_50m        (clk_50m),
    .rst_n          (button_rst_n & locked),
    .ov_rst_n       (),
    .cam_pclk       (cmos_pclk),
    .cam_vsync      (cmos_vsync),
    .cam_href       (cmos_href),
    .cam_data       (cmos_data),
    .cam_rst_n      (cmos_reset),
    .cam_pwdn       (),
    .cam_scl        (cmos_scl),
    .cam_sda        (cmos_sda),
    .clk_div2       (cam_clk),
    .video_vsync    (cam_vs),
    .video_hsync    (cam_de),
    .video_valid    (cam_valid),
    .video_data     (cam_data)
);
end else begin: CAM1_OFF
    assign cmos_scl = 1'bz;
    assign cmos_sda = 1'bz;
    assign cmos_reset = 1'b0;
end
endgenerate

// =========================================================================
// Part E: HDMI 输入处理 (MS72xx HDMI Receiver)
// =========================================================================
wire init_over;          // MS72xx发送端初始化完成
wire hdmi_rx_init_done;  // MS7200接收端初始化完成

ms72xx_ctl ms72xx_ctl_inst (
    .clk            (cfg_clk),
    .rst_n          (rstn_out),
    .init_over_rx   (hdmi_rx_init_done),
    .init_over      (init_over),
    .iic_scl        (hd_scl),
    .iic_sda        (hd_sda)
);

// HDMI 接收器与 FPGA 之间是源同步接口。先在 pixclk_in 域寄存一次，
// 避免 RGB/DE/HS/VS 经较长组合路径后才被采样，并保证控制与像素对齐。
reg         hdmi_vs_d1;
reg         hdmi_hs_d1;
reg         hdmi_de_d1;
reg  [23:0] hdmi_data_d1 /* synthesis PAP_MARK_DEBUG="true" */;

always @(posedge pixclk_in or negedge button_rst_n) begin
    if (!button_rst_n) begin
        hdmi_vs_d1   <= 1'b0;
        hdmi_hs_d1   <= 1'b0;
        hdmi_de_d1   <= 1'b0;
        hdmi_data_d1 <= 24'd0;
    end else begin
        hdmi_vs_d1   <= vs_in;
        hdmi_hs_d1   <= hs_in;
        hdmi_de_d1   <= de_in;
        // Keep the receiver's full RGB888 precision through the scaler.  The
        // stream is quantized to RGB565 only after anti-alias filtering.
        hdmi_data_d1 <= {r_in, g_in, b_in};
    end
end

// =========================================================================
// Part F: 视频源选择 (HDMI测试源 / 摄像头)
// =========================================================================
wire        mux_pclk        /* synthesis PAP_MARK_DEBUG="true" */;
wire        mux_vs          /* synthesis PAP_MARK_DEBUG="true" */;
wire        mux_href;
wire        mux_de          /* synthesis PAP_MARK_DEBUG="true" */;
wire [15:0] mux_rgb565      /* synthesis PAP_MARK_DEBUG="true" */;
wire        raw_mux_vs;
wire        raw_mux_href;
wire        raw_mux_de;
wire [15:0] raw_mux_rgb565;

// 下采样器输出信号 (1080p -> 720p)
wire        pclk_in_out;     // 下采样后的像素时钟 (=pixclk_in透传)
wire        hdmi_vs_in;      // 下采样后的场同步
wire        hdmi_href_in;    // compacted 1280-pixel line active
wire        hdmi_de_in;      // 下采样后的数据有效
wire [15:0] hdmi_rgb565;     // 下采样后的RGB565数据
wire        out_vs;
wire        out_href;
wire        out_de;
wire        out_clk;
wire [3:0] camera_clk, camera_vs, camera_de;
wire [63:0] camera_data;
wire [3:0] camera_small_vs, camera_small_de;
wire [63:0] camera_small_data;
wire [3:0] camera_store_vs, camera_store_de;
genvar camera;
generate
  if(CAMERA_TEST_PATTERN[0]) begin: CAM1_STREAM_TEST
    assign camera_clk[0]=camera_clk[2];
    assign camera_vs[0]=test_cam_vs[0];
    assign camera_de[0]=test_cam_de[0];
    assign camera_data[15:0]=test_cam_data[15:0];
  end else if(CAMERA_ENABLE[0]) begin: CAM1_STREAM
    assign camera_clk[0]=cam_clk;
    assign camera_vs[0]=cam_vs;
    assign camera_de[0]=cam_de;
    assign camera_data[15:0]=cam_data;
  end else begin: CAM1_STREAM_OFF
    assign camera_clk[0]=clk_50m;
    assign camera_vs[0]=1'b0;
    assign camera_de[0]=1'b0;
    assign camera_data[15:0]=16'd0;
  end
  if(CAMERA_TEST_PATTERN[1]) begin: CAM2_TEST
    assign cmos2_scl=1'bz; assign cmos2_sda=1'bz; assign cmos2_reset=1'b0;
    assign camera_clk[1]=camera_clk[2];
    assign camera_vs[1]=test_cam_vs[1];
    assign camera_de[1]=test_cam_de[1];
    assign camera_data[31:16]=test_cam_data[31:16];
  end else if(CAMERA_ENABLE[1]) begin: CAM2
    ov5640 u_cam(.video_clk(),.clk_50m(clk_50m),.rst_n(button_rst_n & locked),.ov_rst_n(),
      .cam_pclk(cmos2_pclk),.cam_vsync(cmos2_vsync),.cam_href(cmos2_href),.cam_data(cmos2_data),
      .cam_rst_n(cmos2_reset),.cam_pwdn(),.cam_scl(cmos2_scl),.cam_sda(cmos2_sda),
      .clk_div2(camera_clk[1]),.video_vsync(camera_vs[1]),.video_hsync(camera_de[1]),.video_valid(),.video_data(camera_data[31:16]));
  end else begin: CAM2_OFF
    assign cmos2_scl=1'bz;assign cmos2_sda=1'bz;assign cmos2_reset=0;
    assign camera_clk[1]=clk_50m;assign camera_vs[1]=0;assign camera_de[1]=0;assign camera_data[31:16]=0;
  end
  if(CAMERA_TEST_PATTERN[2]) begin: CAM3_TEST
    assign cmos3_scl=1'bz; assign cmos3_sda=1'bz; assign cmos3_reset=1'b0;
    assign camera_clk[2]=cam_clk;
    assign camera_vs[2]=test_cam_vs[2];
    assign camera_de[2]=test_cam_de[2];
    assign camera_data[47:32]=test_cam_data[47:32];
  end else if(CAMERA_ENABLE[2]) begin: CAM3
    ov5640 u_cam(.video_clk(),.clk_50m(clk_50m),.rst_n(button_rst_n & locked),.ov_rst_n(),
      .cam_pclk(cmos3_pclk),.cam_vsync(cmos3_vsync),.cam_href(cmos3_href),.cam_data(cmos3_data),
      .cam_rst_n(cmos3_reset),.cam_pwdn(),.cam_scl(cmos3_scl),.cam_sda(cmos3_sda),
      .clk_div2(camera_clk[2]),.video_vsync(camera_vs[2]),.video_hsync(camera_de[2]),.video_valid(),.video_data(camera_data[47:32]));
  end else begin: CAM3_OFF
    assign cmos3_scl=1'bz;assign cmos3_sda=1'bz;assign cmos3_reset=0;
    assign camera_clk[2]=clk_50m;assign camera_vs[2]=0;assign camera_de[2]=0;assign camera_data[47:32]=0;
  end
  if(CAMERA_TEST_PATTERN[3]) begin: CAM4_TEST
    assign cmos4_scl=1'bz; assign cmos4_sda=1'bz; assign cmos4_reset=1'b0;
    assign camera_clk[3]=camera_clk[2];
    assign camera_vs[3]=test_cam_vs[3];
    assign camera_de[3]=test_cam_de[3];
    assign camera_data[63:48]=test_cam_data[63:48];
  end else if(CAMERA_ENABLE[3]) begin: CAM4
    ov5640 u_cam(.video_clk(),.clk_50m(clk_50m),.rst_n(button_rst_n & locked),.ov_rst_n(),
      .cam_pclk(cmos4_pclk),.cam_vsync(cmos4_vsync),.cam_href(cmos4_href),.cam_data(cmos4_data),
      .cam_rst_n(cmos4_reset),.cam_pwdn(),.cam_scl(cmos4_scl),.cam_sda(cmos4_sda),
      .clk_div2(camera_clk[3]),.video_vsync(camera_vs[3]),.video_hsync(camera_de[3]),.video_valid(),.video_data(camera_data[63:48]));
  end else begin: CAM4_OFF
    assign cmos4_scl=1'bz;assign cmos4_sda=1'bz;assign cmos4_reset=0;
    assign camera_clk[3]=clk_50m;assign camera_vs[3]=0;assign camera_de[3]=0;assign camera_data[63:48]=0;
  end
  for(camera=0;camera<4;camera=camera+1) begin: RESHAPE
    reg hdmi_meta, hdmi_sync, small_vs_d, store_frame;
    always @(posedge camera_clk[camera] or negedge button_rst_n) begin
      if(!button_rst_n) begin hdmi_meta<=0;hdmi_sync<=0;small_vs_d<=0;store_frame<=0; end
      else begin
        hdmi_meta<=current_video_select;hdmi_sync<=hdmi_meta;
        small_vs_d<=camera_small_vs[camera];
        if(hdmi_sync) store_frame<=0;
        else if(camera_small_vs[camera] && !small_vs_d) store_frame<=CAMERA_ENABLE[camera];
      end
    end
    assign camera_store_vs[camera]=camera_small_vs[camera] && !hdmi_sync && CAMERA_ENABLE[camera];
    assign camera_store_de[camera]=camera_small_de[camera] && store_frame && !hdmi_sync;
    camera_quad_reshape u_reshape(.clk(camera_clk[camera]),.rst_n(button_rst_n & locked),
      .in_vs(camera_vs[camera]),.in_de(camera_de[camera]),.in_data(camera_data[camera*16 +:16]),
      .out_vs(camera_small_vs[camera]),.out_de(camera_small_de[camera]),.out_data(camera_small_data[camera*16 +:16]));
  end
endgenerate
wire mosaic_vs;
reg [2:0] mosaic_vs_delay;
always @(posedge clk_50m or negedge button_rst_n) begin
    if(!button_rst_n) mosaic_vs_delay<=0;
    else mosaic_vs_delay<={mosaic_vs_delay[1:0],vs_reg};
end
assign mosaic_vs=mosaic_vs_delay[2];
reg source_meta, source_sync, source_last;
reg [5:0] source_settle;
wire video_source_rst_n = pll_rst_n && button_rst_n &&
                         (current_video_select==source_sync) && source_settle==0;
always @(posedge mux_pclk or negedge button_rst_n) begin
  if(!button_rst_n) begin source_meta<=0;source_sync<=0;source_last<=0;source_settle<=32; end
  else begin
    source_meta<=current_video_select;source_sync<=source_meta;source_last<=source_sync;
    if(source_sync!=source_last) source_settle<=32;
    else if(source_settle!=0) source_settle<=source_settle-1'b1;
  end
end

// 1080p -> 720p 下采样
downscaler_1080p_to_720p_ddr u_downscaler (
    .rst_n          (button_rst_n),
    .pclk_in        (pixclk_in),
    .vs_in          (hdmi_vs_d1),
    .hs_in          (hdmi_hs_d1),
    .de_in          (hdmi_de_d1),
    .rgb_in         (hdmi_data_d1),
    .pclk_in_out    (pclk_in_out),
    .vs_in_out      (hdmi_vs_in),
    .href_in_out    (hdmi_href_in),
    .de_in_out      (hdmi_de_in),
    .rgb565_out     (hdmi_rgb565)
);

frame_rate_halver u_frame_rate_halver(
    .clk_sys    ()       ,     // 系统高速时钟（可选，用于同步逻辑）
    .rst_n      (button_rst_n)      ,       // 异步复位信号


    .pixclk_in  (pclk_in_out)          ,   // 与下采样器/DDR写端共用同一时钟链
    .vs_in_test (hdmi_vs_in)     ,  // 输入帧同步
    .href_in_test(hdmi_href_in)  ,  // 输入行有效
    .de_in_test (hdmi_de_in)       ,  // 输入数据使能
    .i_rgb565   (hdmi_rgb565)     ,    // 输入像素数据


    .vs_out     (out_vs)          ,      // 输出帧同步
    .href_out   (out_href)        ,      // 输出行有效
    .de_out     (out_de)          ,      // 输出数据使能
    .o_rgb565   (out_rgb)            ,    // 输出像素数据
    .pixclk_out (out_clk)      // 输出像素时钟（通常透传）
);

wire [15:0]out_rgb;

video_channel_selector u_video_channel_selector (
    .clk            (sys_clk),
    .rst_n          (button_rst_n),
    .key_in         (key1),
    .remote_select  (rk_video_ctrl[0]),
    .remote_update_toggle(rk_video_update_toggle),
    // 测试通道 (HDMI+下采样)
    .pixclk_in      (out_clk),
    .hdmi_vs_in     (out_vs),
    .hdmi_href_in   (out_href),
    .hdmi_de_in     (out_de),
    .hdmi_rgb565    (out_rgb),
    // 摄像头通道
    .cam_clk        (clk_50m),
    .cam_vs         (mosaic_vs),
    .cam_href       (de_o),
    .cam_de         (de_o),
    .cam_data       (o_rgb565),
    // 复用输出
    .mux_pclk       (mux_pclk),
    .mux_vs         (raw_mux_vs),
    .mux_href       (raw_mux_href),
    .mux_de         (raw_mux_de),
    .mux_rgb565     (raw_mux_rgb565),
    .video_set_led  (current_video_select)
);

// =========================================================================
// Part F.1: plate localization and frame-safe processing bypass
// =========================================================================
localparam VP_DATA_WIDTH = 11;

wire        vip_post_vsync;
wire        vip_post_href;
wire        vip_post_de;
wire [23:0] vip_post_rgb;
wire        process_pipeline_enable;
wire        process_pipeline_rst_n;
wire        process_enable_active;
wire        processing_path_req;
reg         preprocess_meta;
reg         preprocess_sync;
reg         preprocess_active;
reg         box_draw_meta;
reg         box_draw_sync;
reg         box_draw_active;
reg         white_balance_meta;
reg         white_balance_sync;
reg         white_balance_active;
reg         dehaze_meta;
reg         dehaze_sync;
reg         dehaze_active;
reg         feature_vsync_d;

// Synchronize all processing controls into the selected pixel-clock domain.
// While a processed path is on screen, changes are applied at an input frame
// boundary so no control can alter a module in the middle of a line or frame.
always @(posedge mux_pclk or negedge pll_rst_n) begin
    if (!pll_rst_n) begin
        preprocess_meta       <= 1'b0;
        preprocess_sync       <= 1'b0;
        preprocess_active     <= 1'b0;
        box_draw_meta          <= 1'b0;
        box_draw_sync          <= 1'b0;
        box_draw_active        <= 1'b0;
        white_balance_meta   <= 1'b0;
        white_balance_sync   <= 1'b0;
        white_balance_active <= 1'b0;
        dehaze_meta           <= 1'b0;
        dehaze_sync           <= 1'b0;
        dehaze_active         <= 1'b0;
        feature_vsync_d       <= 1'b0;
    end else begin
        preprocess_meta    <= rk_preprocess_ctrl[0];
        preprocess_sync    <= preprocess_meta;
        box_draw_meta       <= rk_box_draw_ctrl[0];
        box_draw_sync       <= box_draw_meta;
        white_balance_meta <= rk_white_balance_ctrl[0];
        white_balance_sync <= white_balance_meta;
        dehaze_meta         <= rk_dehaze_ctrl[0];
        dehaze_sync         <= dehaze_meta;
        feature_vsync_d     <= raw_mux_vs;

        if (!process_enable_active || (raw_mux_vs && !feature_vsync_d)) begin
            preprocess_active     <= preprocess_sync;
            box_draw_active        <= box_draw_sync;
            white_balance_active <= white_balance_sync;
            dehaze_active         <= dehaze_sync;
        end
    end
end

// Plate localization only needs to run when box drawing is enabled. The VIP
// and box modules have explicit bypasses, so reset is not used as a selector.
// HDMI is a hard bypass: do not activate the image-processing pipeline while
// that source is selected, even if its control registers are still enabled.
assign process_pipeline_rst_n = pll_rst_n & ~current_video_select &
                                (box_draw_sync | box_draw_active);
assign processing_path_req = ~current_video_select &
                             (preprocess_sync | box_draw_sync);

vip #(
    .IMG_HDISP (IMG_WIDTH),
    .IMG_VDISP (IMG_HEIGHT)
) u_vip (
    .clk              (mux_pclk),
    .rst_n            (pll_rst_n & ~current_video_select),
    .vip_crtl         ({preprocess_active,
                        dehaze_active,
                        white_balance_active}),
    .pre_frame_vsync  (raw_mux_vs),
    .pre_frame_href   (raw_mux_href),
    .pre_frame_de     (raw_mux_de),
    .pre_rgb          (raw_mux_rgb565),
    .post_frame_vsync (vip_post_vsync),
    .post_frame_href  (vip_post_href),
    .post_frame_de    (vip_post_de),
    .post_rgb         (vip_post_rgb)
);

wire                         license_plate_exist_flag;
wire [3:0]                   license_plate_count;
wire                         license_read_en;
wire [3:0]                   license_read_addr;
wire [VP_DATA_WIDTH*4-1:0]   license_plate_data;

vdp #(
    .IMG_HDISP   (IMG_WIDTH),
    .IMG_VDISP   (IMG_HEIGHT),
    .DATA_WIDTH  (VP_DATA_WIDTH),
    .NUM_CHANNELS(8)
) u_vdp (
    .clk                      (mux_pclk),
    .rst_n                    (process_pipeline_rst_n),
    .per_frame_vsync          (vip_post_vsync),
    .per_frame_href           (vip_post_href),
    .per_frame_clken          (vip_post_de),
    .img_rgb                  (vip_post_rgb),
    .license_plate_exist_flag (license_plate_exist_flag),
    .license_plate_count      (license_plate_count),
    .license_read_en          (license_read_en),
    .license_read_addr        (license_read_addr),
    .license_plate_data       (license_plate_data)
);

wire        box_draw_vsync;
wire        box_draw_href;
wire        box_draw_de;
wire [23:0] box_draw_rgb888;
wire        processed_vsync;
wire        processed_href;
wire        processed_de;
wire [23:0] processed_rgb888;
wire [15:0] processed_rgb565;

vdp_box_draw #(
    .IMG_HDISP   (IMG_WIDTH),
    .IMG_VDISP   (IMG_HEIGHT),
    .DATA_WIDTH  (VP_DATA_WIDTH),
    .MAX_BOXES   (8),
    .BORDER_WIDTH(2),
    .BOX_RGB     (24'hFF0000)
) u_vdp_box_draw (
    .clk                      (mux_pclk),
    .rst_n                    (process_pipeline_rst_n),
    .pre_frame_vsync          (vip_post_vsync),
    .pre_frame_href           (vip_post_href),
    .pre_frame_clken          (vip_post_de),
    .pre_rgb                  (vip_post_rgb),
    .license_plate_exist_flag (license_plate_exist_flag),
    .license_plate_count      (license_plate_count),
    .license_plate_data       (license_plate_data),
    .license_read_en          (license_read_en),
    .license_read_addr        (license_read_addr),
    .post_frame_vsync         (box_draw_vsync),
    .post_frame_href          (box_draw_href),
    .post_frame_clken         (box_draw_de),
    .post_rgb                 (box_draw_rgb888)
);

assign processed_vsync  = box_draw_active ? box_draw_vsync  : vip_post_vsync;
assign processed_href   = box_draw_active ? box_draw_href   : vip_post_href;
assign processed_de     = box_draw_active ? box_draw_de     : vip_post_de;
assign processed_rgb888 = box_draw_active ? box_draw_rgb888 : vip_post_rgb;

assign processed_rgb565 = {
    processed_rgb888[23:19],
    processed_rgb888[15:10],
    processed_rgb888[7:3]
};

video_process_selector u_video_process_selector (
    .pclk                  (mux_pclk),
    .rst_n                 (video_source_rst_n),
    .process_enable_req    (processing_path_req),
    .raw_vsync             (raw_mux_vs),
    .raw_href              (raw_mux_href),
    .raw_de                (raw_mux_de),
    .raw_rgb565            (raw_mux_rgb565),
    .processed_vsync       (processed_vsync),
    .processed_href        (processed_href),
    .processed_de          (processed_de),
    .processed_rgb565      (processed_rgb565),
    .out_vsync             (mux_vs),
    .out_href              (mux_href),
    .out_de                (mux_de),
    .out_rgb565            (mux_rgb565),
    .process_enable_active (process_enable_active),
    .processing_pipeline_enable(process_pipeline_enable)
);

// =========================================================================
// Part G: 四路独立DDR3帧缓存与2x2拼接
// =========================================================================
// AXI总线互联信号
wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr;
wire                        axi_awuser_ap;
wire [3:0]                  axi_awuser_id;
wire [3:0]                  axi_awlen;
wire                        axi_awready;
wire                        axi_awvalid;
wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata;
wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb;
wire                        axi_wready;
wire [3:0]                  axi_wusero_id;
wire                        axi_wusero_last;
wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr;
wire                        axi_aruser_ap;
wire [3:0]                  axi_aruser_id;
wire [3:0]                  axi_arlen;
wire                        axi_arready;
wire                        axi_arvalid;
wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata  /* synthesis syn_keep = 1 */;
wire                        axi_rvalid /* synthesis syn_keep = 1 */;
wire [3:0]                  axi_rid;
wire                        axi_rlast;
assign axi_awuser_ap = 1'b0;
assign axi_aruser_ap = 1'b0;

quad_ddr3_framebuffer #(.ADDR_WIDTH(CTRL_ADDR_WIDTH),.CAMERA_ENABLE(CAMERA_ENABLE)) u_quad_ddr3 (
    .ddr_clk        (core_clk),
    .ddr_rstn       (ddr_init_done),
    // 写端口 (视频输入)
    .in_clk         (camera_clk),
    .in_vs          (camera_store_vs),
    .in_de          (camera_store_de),
    .in_data        (camera_small_data),
    // 读端口 (视频输出)
    .pix_clk        (clk_50m),
    .out_vs         (vs_reg),
    .out_active     (rd_en),
    .out_x          (act_x),
    .out_y          (act_y),
    .out_de         (de_o),
    .out_data       (o_rgb565),
    // AXI总线
    .axi_awaddr     (axi_awaddr),
    .axi_awid       (axi_awuser_id),
    .axi_awlen      (axi_awlen),
    .axi_awready    (axi_awready),
    .axi_awvalid    (axi_awvalid),
    .axi_wdata      (axi_wdata),
    .axi_wstrb      (axi_wstrb),
    .axi_wlast      (axi_wusero_last),
    .axi_wvalid     (),
    .axi_wready     (axi_wready),
    .axi_araddr     (axi_araddr),
    .axi_arid       (axi_aruser_id),
    .axi_arlen      (axi_arlen),
    .axi_arvalid    (axi_arvalid),
    .axi_arready    (axi_arready),
    .axi_rready     (),
    .axi_rdata      (axi_rdata),
    .axi_rvalid     (axi_rvalid),
    .axi_rlast      (axi_rlast),
    .axi_rid        (axi_rid)
);

// 读端口输出信号
wire [15:0] o_rgb565;        // DDR读出的RGB565数据
wire        de_o;            // DDR读出的数据有效

// =========================================================================
// Part H: 摄像头拼接光栅时序生成 (1280x720)
// =========================================================================
wire        vs_reg;          // 时序生成器场同步输出
wire        hs_reg;          // 时序生成器行同步输出
wire        rd_en;           // 时序生成器数据有效 (读DDR使能)
wire [X_WIDTH-1:0] act_x;    // 当前像素X坐标
wire [Y_WIDTH-1:0] act_y;    // 当前像素Y坐标

sync_vg #(
    .X_BITS     (X_WIDTH),
    .Y_BITS     (Y_WIDTH),
    .V_TOTAL    (V_TOTAL),
    .V_FP       (V_FP),
    .V_BP       (V_BP),
    .V_SYNC     (V_SYNC),
    .V_ACT      (V_ACT),
    .H_TOTAL    (H_TOTAL),
    .H_FP       (H_FP),
    .H_BP       (H_BP),
    .H_SYNC     (H_SYNC),
    .H_ACT      (H_ACT)
) u_sync_vg (
    .clk        (clk_50m),
    .rstn       (ddr_init_done & ~current_video_select),
    .vs_out     (vs_reg),
    .hs_out     (hs_reg),
    .de_out     (rd_en),
    .x_act      (act_x),
    .y_act      (act_y)
);

// =========================================================================
// Part I: DDR3 控制器
// =========================================================================
ddr3 #(
    .MEM_ROW_WIDTH      (MEM_ROW_WIDTH),
    .MEM_COLUMN_WIDTH   (MEM_COLUMN_WIDTH),
    .MEM_BANK_WIDTH     (MEM_BANK_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH),
    .MEM_DM_WIDTH       (MEM_DQS_WIDTH),
    .MEM_DQS_WIDTH      (MEM_DQS_WIDTH),
    .CTRL_ADDR_WIDTH    (CTRL_ADDR_WIDTH)
) I_ips_ddr_top (
    .ref_clk                (sys_clk),
    .resetn                 (button_rst_n),
    .core_clk               (core_clk),
    .pll_lock               (),
    .phy_pll_lock           (),
    .gpll_lock              (),
    .rst_gpll_lock          (),
    .ddrphy_cpd_lock        (),
    .ddr_init_done          (ddr_init_done),
    // AXI写地址通道
    .axi_awaddr             (axi_awaddr),
    .axi_awuser_ap          (axi_awuser_ap),
    .axi_awuser_id          (axi_awuser_id),
    .axi_awlen              (axi_awlen),
    .axi_awready            (axi_awready),
    .axi_awvalid            (axi_awvalid),
    // AXI写数据通道
    .axi_wdata              (axi_wdata),
    .axi_wstrb              (axi_wstrb),
    .axi_wready             (axi_wready),
    .axi_wusero_id          (axi_wusero_id),
    .axi_wusero_last        (axi_wusero_last),
    // AXI读地址通道
    .axi_araddr             (axi_araddr),
    .axi_aruser_ap          (axi_aruser_ap),
    .axi_aruser_id          (axi_aruser_id),
    .axi_arlen              (axi_arlen),
    .axi_arready            (axi_arready),
    .axi_arvalid            (axi_arvalid),
    // AXI读数据通道
    .axi_rdata              (axi_rdata),
    .axi_rid                (axi_rid),
    .axi_rlast              (axi_rlast),
    .axi_rvalid             (axi_rvalid),
    // APB (未使用)
    .apb_clk                (1'b0),
    .apb_rst_n              (1'b0),
    .apb_sel                (1'b0),
    .apb_enable             (1'b0),
    .apb_addr               (8'd0),
    .apb_write              (1'b0),
    .apb_ready              (),
    .apb_wdata              (16'd0),
    .apb_rdata              (),
    // DDR3物理接口
    .mem_cs_n               (mem_cs_n),
    .mem_rst_n              (mem_rst_n),
    .mem_ck                 (mem_ck),
    .mem_ck_n               (mem_ck_n),
    .mem_cke                (mem_cke),
    .mem_ras_n              (mem_ras_n),
    .mem_cas_n              (mem_cas_n),
    .mem_we_n               (mem_we_n),
    .mem_odt                (mem_odt),
    .mem_a                  (mem_a),
    .mem_ba                 (mem_ba),
    .mem_dqs                (mem_dqs),
    .mem_dqs_n              (mem_dqs_n),
    .mem_dq                 (mem_dq),
    .mem_dm                 (mem_dm),
    // Debug (未使用)
    .dbg_gate_start         (1'b0),
    .dbg_cpd_start          (1'b0),
    .dbg_ddrphy_rst_n       (1'b1),
    .dbg_gpll_scan_rst      (1'b0),
    .samp_position_dyn_adj  (1'b0),
    .init_samp_position_even(16'd0),
    .init_samp_position_odd (16'd0),
    .wrcal_position_dyn_adj (1'b0),
    .init_wrcal_position    (16'd0),
    .force_read_clk_ctrl    (1'b0),
    .init_slip_step         (8'd0),
    .init_read_clk_ctrl     (6'd0),
    .debug_calib_ctrl       (),
    .dbg_dll_upd_state      (),
    .dbg_slice_status       (),
    .dbg_slice_state        (),
    .debug_data             (),
    .debug_gpll_dps_phase   (),
    .dbg_rst_dps_state      (),
    .dbg_tran_err_rst_cnt   (),
    .dbg_ddrphy_init_fail   (),
    .debug_cpd_offset_adj   (1'b0),
    .debug_cpd_offset_dir   (1'b0),
    .debug_cpd_offset       (10'd0),
    .debug_dps_cnt_dir0     (),
    .debug_dps_cnt_dir1     (),
    .ck_dly_en              (1'b0),
    .init_ck_dly_step       (8'd0),
    .ck_dly_set_bin         (),
    .align_error            (),
    .debug_rst_state        (),
    .debug_cpd_state        ()
);

// =========================================================================
// Part J: DDR3 心跳灯 (core_clk域, ~1Hz闪烁)
// =========================================================================
reg [26:0] cnt;

always @(posedge core_clk or negedge ddr_init_done) begin
    if (!ddr_init_done)
        cnt <= 27'd0;
    else if (cnt >= TH_1S)
        cnt <= 27'd0;
    else
        cnt <= cnt + 27'd1;
end

always @(posedge core_clk or negedge ddr_init_done) begin
    if (!ddr_init_done)
        heart_beat_led <= 1'd1;
    else if (cnt >= TH_1S)
        heart_beat_led <= ~heart_beat_led;
end

// =========================================================================
// Part K: 视频DMA打包与发送控制
// =========================================================================
wire        w_video_crtl_de   /* synthesis PAP_MARK_DEBUG="true" */;
wire        w_video_crtl_vs;
wire [15:0] w_video_crtl_data /* synthesis PAP_MARK_DEBUG="true" */;
wire        w_start_flag      /* synthesis PAP_MARK_DEBUG="true" */;
wire        w_dma_rd_en       /* synthesis PAP_MARK_DEBUG="true" */;
wire [127:0] w_dma_rd_data;
wire        dma_tx_done       /* synthesis PAP_MARK_DEBUG="true" */;

// 视频流控制: 保证采集时从完整帧开始
video_crtl #(
    .PPC            (PPC),
    .DATA_WIDTH     (16),
    .IMG_WIDTH      (IMG_WIDTH),
    .IMG_HEIGHT     (IMG_HEIGHT)
) u_video_crtl (
    .i_video_clk            (mux_pclk),
    .i_rst_n                (video_source_rst_n),
    .i_start_dma_tx_flag    (w_start_flag),
    // HDMI goes directly to PCIe after scaling/halving. Camera mode uses
    // the DDR mosaic, optionally passed through the existing processing path.
    .i_video_data           (current_video_select ? raw_mux_rgb565 : mux_rgb565),
    .i_video_vs             (current_video_select ? raw_mux_vs     : mux_vs),
    .i_video_de             (current_video_select ? raw_mux_de     : mux_de),
    .o_video_crtl_data      (w_video_crtl_data),
    .o_video_crtl_vs        (w_video_crtl_vs),
    .o_video_crtl_de        (w_video_crtl_de)
);



// DMA控制信号
wire        video_dma_req;
wire [63:0] video_dma_addr      /* synthesis PAP_MARK_DEBUG="true" */;
wire        dma_cmd_rdy;
wire [9:0]  video_dma_len;
wire        set_dma_config_en;
wire [63:0] dma_base_addr;
wire [63:0] dma_base_addr2;
wire [63:0] dma_base_addr3;
wire [63:0] dma_base_addr4;

assign video_dma_len = 10'd640;    // 单次DMA传输DW数

// =========================================================================
// Part L: PCIe TX 组包模块
// =========================================================================
wire [1:0]  o_dma_wr_index;
wire        o_dma_wr_done;

pcie_tx_fun #(
    .IMG_WIDTH          (1280),
    .IMG_HEIGHT         (720),
    .DMA_ADDR_WIDTH     (64),
    .VIDEO_DATA_WIDTH   (16),
    .PCIE_DATA_WIDTH    (128),
    .PPC                (1),
    .PIXCEL_BYTES       (2),
    .DMA_LEN            (2560)
) u_pcie_tx_fun (
    .i_pcie_clk         (pclk_div2),
    .i_pcie_rst_n       (core_rst_n),
    .i_dma_base_addr    (dma_base_addr),
    .i_dma_base_addr2   (dma_base_addr2),
    .i_dma_base_addr3   (dma_base_addr3),
    .i_dma_base_addr4   (dma_base_addr4),
    .i_dma_set_en       (set_dma_config_en),
    .o_dma_wr_index     (o_dma_wr_index),
    .o_dma_wr_done      (o_dma_wr_done),
    .i_start_tx_flag    (w_start_flag),
    .i_dma_cmd_rdy      (dma_cmd_rdy),
    .o_dma_req          (video_dma_req),
    .o_dma_addr         (video_dma_addr),
    .i_dma_tx_done      (dma_tx_done),
    .i_video_clk        (mux_pclk),
    .i_video_rst_n      (video_source_rst_n),
    .i_video_data       (w_video_crtl_data),
    .i_video_vs         (w_video_crtl_vs),
    .i_video_de         (w_video_crtl_de),
    .o_dma_rd_data      (w_dma_rd_data),
    .i_dma_rd_en        (w_dma_rd_en)
);

// =========================================================================
// Part M: PCIe PIO 控制 (BAR空间访问)
// =========================================================================
wire        pio_wr_en;
wire [9:0]  pio_wr_addr;
wire [31:0] pio_wr_data;
wire        pio_rd_en;
wire [9:0]  pio_rd_addr;
wire [31:0] pio_rd_data;

pio_crtl u_pio_crtl (
    .pcie_clk           (pclk_div2),
    .rst_n              (core_rst_n),
    .start_flag         (w_start_flag),
    .set_dma_config_en  (set_dma_config_en),
    .o_base_addr        (dma_base_addr),
    .o_base_addr2       (dma_base_addr2),
    .o_base_addr3       (dma_base_addr3),
    .o_base_addr4       (dma_base_addr4),
    .i_wr_frame_done    (o_dma_wr_done),
    .i_wr_index         (o_dma_wr_index),
    .pio_wr_en          (pio_wr_en),
    .pio_wr_addr        (pio_wr_addr),
    .pio_wr_data        (pio_wr_data),
    .pio_rd_en          (pio_rd_en),
    .pio_rd_addr        (pio_rd_addr),
    .pio_rd_data        (pio_rd_data)
);

// =========================================================================
// Part N: PCIe DMA 控制器
// =========================================================================
// APB 总线
wire [3:0]  p_strb;
wire [15:0] p_addr;
wire [31:0] p_wdata;
wire        p_ce;
wire        p_we;

// APB Mux选择信号
wire        p_sel_pcie;          // 0~5:hsstlp, 6:Reserved, 7:pcie
wire        p_sel_cfg;           // 8: cfg
wire        p_sel_dma;           // 9: dma

wire [31:0] p_rdata_pcie;
wire [31:0] p_rdata_cfg;
wire [31:0] p_rdata_dma;

wire        p_rdy_pcie;
wire        p_rdy_cfg;
wire        p_rdy_dma;

// PCIe配置信号
wire [7:0]  cfg_pbus_num;
wire [4:0]  cfg_pbus_dev_num;
wire [2:0]  cfg_max_rd_req_size;
wire [2:0]  cfg_max_payload_size;

ips2l_pcie_dma #(
    .DEVICE_TYPE        (DEVICE_TYPE),
    .AXIS_SLAVE_NUM     (AXIS_SLAVE_NUM)
) u_ips2l_pcie_dma (
    .clk                    (pclk_div2),
    .rst_n                  (core_rst_n),
    // 视频DMA请求
    .i_video_dma_req        (video_dma_req),
    .i_video_dma_addr       (video_dma_addr),
    .i_video_dma_len        (video_dma_len),
    .i_dma_32or64           (1'b0),              // 64bit地址
    .o_dma_cmd_rdy          (dma_cmd_rdy),
    .o_dma_tx_done          (dma_tx_done),
    .o_ch0_base_addr        (),
    // DMA数据接口
    .o_dma_rd_en            (w_dma_rd_en),
    .i_dma_rd_data          (w_dma_rd_data),
    .o_dma_wr_en            (),
    .o_dma_wr_data          (),
    // PIO接口
    .o_pio_wr_en            (pio_wr_en),
    .o_pio_wr_addr          (pio_wr_addr),
    .o_pio_wr_data          (pio_wr_data),
    .o_pio_rd_en            (pio_rd_en),
    .o_pio_rd_addr          (pio_rd_addr),
    .o_pio_rd_data          (pio_rd_data),
    // PCIe 配置
    .i_cfg_pbus_num         (cfg_pbus_num),
    .i_cfg_pbus_dev_num     (cfg_pbus_dev_num),
    .i_cfg_max_rd_req_size  (cfg_max_rd_req_size),
    .i_cfg_max_payload_size (cfg_max_payload_size),
    // AXIS Master
    .i_axis_master_tvld     (axis_master_tvalid_mem),
    .o_axis_master_trdy     (axis_master_tready_mem),
    .i_axis_master_tdata    (axis_master_tdata_mem),
    .i_axis_master_tkeep    (axis_master_tkeep_mem),
    .i_axis_master_tlast    (axis_master_tlast_mem),
    .i_axis_master_tuser    (axis_master_tuser_mem),
    // AXIS Slave 0
    .i_axis_slave0_trdy     (axis_slave0_tready),
    .o_axis_slave0_tvld     (dma_axis_slave0_tvalid),
    .o_axis_slave0_tdata    (dma_axis_slave0_tdata),
    .o_axis_slave0_tlast    (dma_axis_slave0_tlast),
    .o_axis_slave0_tuser    (dma_axis_slave0_tuser),
    // AXIS Slave 1
    .i_axis_slave1_trdy     (axis_slave1_tready),
    .o_axis_slave1_tvld     (axis_slave1_tvalid),
    .o_axis_slave1_tdata    (axis_slave1_tdata),
    .o_axis_slave1_tlast    (axis_slave1_tlast),
    .o_axis_slave1_tuser    (axis_slave1_tuser),
    // AXIS Slave 2
    .i_axis_slave2_trdy     (axis_slave2_tready),
    .o_axis_slave2_tvld     (axis_slave2_tvalid),
    .o_axis_slave2_tdata    (axis_slave2_tdata),
    .o_axis_slave2_tlast    (axis_slave2_tlast),
    .o_axis_slave2_tuser    (axis_slave2_tuser),
    // PCIe Credit配置
    .i_cfg_ido_req_en       (1'b0),
    .i_cfg_ido_cpl_en       (1'b0),
    .i_xadm_ph_cdts         (8'b0),
    .i_xadm_pd_cdts         (12'b0),
    .i_xadm_nph_cdts        (8'b0),
    .i_xadm_npd_cdts        (12'b0),
    .i_xadm_cplh_cdts       (8'b0),
    .i_xadm_cpld_cdts       (12'b0),
    // APB接口
    .i_apb_psel             (p_sel_dma),
    .i_apb_paddr            (p_addr[8:0]),
    .i_apb_pwdata           (p_wdata),
    .i_apb_pstrb            (p_strb),
    .i_apb_pwrite           (p_we),
    .i_apb_penable          (p_ce),
    .o_apb_prdy             (p_rdy_dma),
    .o_apb_prdata           (p_rdata_dma),
    .o_cross_4kb_boundary   ()
);

// =========================================================================
// Part O: PCIe Config 控制器 / AXIS Mux (仅RC模式)
// =========================================================================
wire        pcie_cfg_ctrl_en;
wire        axis_master_tready_cfg;
wire        cfg_axis_slave0_tvalid;
wire [127:0] cfg_axis_slave0_tdata;
wire        cfg_axis_slave0_tlast;
wire        cfg_axis_slave0_tuser;

// AXIS 主从通道信号
wire        axis_master_tvalid/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_master_tready/* synthesis PAP_MARK_DEBUG="true" */;
wire [127:0] axis_master_tdata/* synthesis PAP_MARK_DEBUG="true" */;
wire [3:0]  axis_master_tkeep/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_master_tlast/* synthesis PAP_MARK_DEBUG="true" */;
wire [7:0]  axis_master_tuser/* synthesis PAP_MARK_DEBUG="true" */;

// AXIS Mux信号
wire        axis_master_tready_mem;
wire        axis_master_tvalid_mem;
wire [127:0] axis_master_tdata_mem;
wire [3:0]  axis_master_tkeep_mem;
wire        axis_master_tlast_mem;
wire [7:0]  axis_master_tuser_mem;

// DMA AXIS Slave通道
wire        dma_axis_slave0_tvalid;
wire [127:0] dma_axis_slave0_tdata;
wire        dma_axis_slave0_tlast;
wire        dma_axis_slave0_tuser;

// AXIS Slave通道 (送往PCIe)
wire        axis_slave0_tready/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave0_tvalid/* synthesis PAP_MARK_DEBUG="true" */;
wire [127:0] axis_slave0_tdata/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave0_tlast/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave0_tuser/* synthesis PAP_MARK_DEBUG="true" */;

wire        axis_slave1_tready/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave1_tvalid/* synthesis PAP_MARK_DEBUG="true" */;
wire [127:0] axis_slave1_tdata/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave1_tlast/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave1_tuser/* synthesis PAP_MARK_DEBUG="true" */;

wire        axis_slave2_tready/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave2_tvalid/* synthesis PAP_MARK_DEBUG="true" */;
wire [127:0] axis_slave2_tdata/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave2_tlast/* synthesis PAP_MARK_DEBUG="true" */;
wire        axis_slave2_tuser/* synthesis PAP_MARK_DEBUG="true" */;

generate
    if (DEVICE_TYPE == 3'd4) begin : rc
        // CFG TLP TX/RX     BASE ADDR = 0x9000
        pcie_cfg_ctrl u_pcie_cfg_ctrl (
            .pclk_div2              (pclk_div2),
            .apb_rst_n              (core_rst_n),
            .p_sel                  (p_sel_cfg),
            .p_strb                 (p_strb),
            .p_addr                 (p_addr[7:0]),
            .p_wdata                (p_wdata),
            .p_ce                   (p_ce),
            .p_we                   (p_we),
            .p_rdy                  (p_rdy_cfg),
            .p_rdata                (p_rdata_cfg),
            .pcie_cfg_ctrl_en       (pcie_cfg_ctrl_en),
            // AXIS Slave
            .axis_slave_tready      (axis_slave0_tready),
            .axis_slave_tvalid      (cfg_axis_slave0_tvalid),
            .axis_slave_tlast       (cfg_axis_slave0_tlast),
            .axis_slave_tuser       (cfg_axis_slave0_tuser),
            .axis_slave_tdata       (cfg_axis_slave0_tdata),
            // AXIS Master
            .axis_master_tready     (axis_master_tready_cfg),
            .axis_master_tvalid     (axis_master_tvalid),
            .axis_master_tlast      (axis_master_tlast),
            .axis_master_tkeep      (axis_master_tkeep),
            .axis_master_tdata      (axis_master_tdata)
        );

        // AXIS Slave Mux: CFG vs DMA
        assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

        // AXIS Master Mux: CFG vs DMA
        assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
        assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
        assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
        assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
        assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;
        assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
    end else begin : ep
        // EP模式: 直通DMA
        assign p_rdy_cfg               = 1'b0;
        assign p_rdata_cfg             = 32'b0;

        assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = dma_axis_slave0_tdata;

        assign axis_master_tvalid_mem  = axis_master_tvalid;
        assign axis_master_tdata_mem   = axis_master_tdata;
        assign axis_master_tkeep_mem   = axis_master_tkeep;
        assign axis_master_tlast_mem   = axis_master_tlast;
        assign axis_master_tuser_mem   = axis_master_tuser;
        assign axis_master_tready      = axis_master_tready_mem;
    end
endgenerate

// =========================================================================
// Part P: PCIe 硬核 Wrapper
// =========================================================================
wire            pclk_div2;           // PCIe用户时钟 (gen1:62.5MHz, gen2:125MHz)
wire            pclk;                // PCIe接口时钟
wire            ref_clk;             // PCIe参考时钟

// UART-APB桥接 (32bit)
wire            uart_p_sel;
wire [3:0]      uart_p_strb;
wire [15:0]     uart_p_addr;
wire [31:0]     uart_p_wdata;
wire            uart_p_ce;
wire            uart_p_we;
wire            uart_p_rdy;
wire [31:0]     uart_p_rdata;

pcie_test u_ips2l_pcie_wrap (
    .button_rst_n               (sync_button_rst_n),
    .power_up_rst_n             (sync_perst_n),
    .perst_n                    (sync_perst_n),
    // 时钟与复位
    .pclk                       (pclk),
    .pclk_div2                  (pclk_div2),
    .ref_clk                    (ref_clk),
    .ref_clk_n                  (ref_clk_n),
    .ref_clk_p                  (ref_clk_p),
    .core_rst_n                 (core_rst_n),
    // APB to DBI
    .p_sel                      (p_sel_pcie),
    .p_strb                     (uart_p_strb),
    .p_addr                     (uart_p_addr),
    .p_wdata                    (uart_p_wdata),
    .p_ce                       (uart_p_ce),
    .p_we                       (uart_p_we),
    .p_rdy                      (p_rdy_pcie),
    .p_rdata                    (p_rdata_pcie),
    // PHY差分信号
    .rxn                        (rxn),
    .rxp                        (rxp),
    .txn                        (txn),
    .txp                        (txp),
    .pcs_nearend_loop           ({2{1'b0}}),
    .pma_nearend_ploop          ({2{1'b0}}),
    .pma_nearend_sloop          ({2{1'b0}}),
    // AXIS Master
    .axis_master_tvalid         (axis_master_tvalid),
    .axis_master_tready         (axis_master_tready),
    .axis_master_tdata          (axis_master_tdata),
    .axis_master_tkeep          (axis_master_tkeep),
    .axis_master_tlast          (axis_master_tlast),
    .axis_master_tuser          (axis_master_tuser),
    // AXIS Slave 0
    .axis_slave0_tready         (axis_slave0_tready),
    .axis_slave0_tvalid         (axis_slave0_tvalid),
    .axis_slave0_tdata          (axis_slave0_tdata),
    .axis_slave0_tlast          (axis_slave0_tlast),
    .axis_slave0_tuser          (axis_slave0_tuser),
    // AXIS Slave 1
    .axis_slave1_tready         (axis_slave1_tready),
    .axis_slave1_tvalid         (axis_slave1_tvalid),
    .axis_slave1_tdata          (axis_slave1_tdata),
    .axis_slave1_tlast          (axis_slave1_tlast),
    .axis_slave1_tuser          (axis_slave1_tuser),
    // AXIS Slave 2
    .axis_slave2_tready         (axis_slave2_tready),
    .axis_slave2_tvalid         (axis_slave2_tvalid),
    .axis_slave2_tdata          (axis_slave2_tdata),
    .axis_slave2_tlast          (axis_slave2_tlast),
    .axis_slave2_tuser          (axis_slave2_tuser),
    // Power Management
    .pm_xtlh_block_tlp          (),
    // Error signals
    .cfg_send_cor_err_mux       (),
    .cfg_send_nf_err_mux        (),
    .cfg_send_f_err_mux         (),
    .cfg_sys_err_rc             (),
    .cfg_aer_rc_err_mux         (),
    .radm_cpl_timeout           (),
    // PCIe配置输出
    .cfg_max_rd_req_size        (cfg_max_rd_req_size),
    .cfg_bus_master_en          (),
    .cfg_max_payload_size       (cfg_max_payload_size),
    .cfg_ext_tag_en             (),
    .cfg_rcb                    (),
    .cfg_mem_space_en           (),
    .cfg_pm_no_soft_rst         (),
    .cfg_crs_sw_vis_en          (),
    .cfg_no_snoop_en            (),
    .cfg_relax_order_en         (),
    .cfg_tph_req_en             (),
    .cfg_pf_tph_st_mode         (),
    .rbar_ctrl_update           (),
    .cfg_atomic_req_en          (),
    .cfg_pbus_num               (cfg_pbus_num),
    .cfg_pbus_dev_num           (cfg_pbus_dev_num),
    // Debug
    .radm_idle                  (),
    .radm_q_not_empty           (),
    .radm_qoverflow             (),
    .diag_ctrl_bus              (2'b0),
    .cfg_link_auto_bw_mux       (),
    .cfg_bw_mgt_mux             (),
    .cfg_pme_mux                (),
    .app_ras_des_sd_hold_ltssm  (1'b0),
    .app_ras_des_tba_ctrl       (2'b0),
    .dyn_debug_info_sel         (4'b0),
    .debug_info_mux             (),
    // 系统状态
    .smlh_link_up               (smlh_link_up),
    .rdlh_link_up               (rdlh_link_up),
    .smlh_ltssm_state           ()
);

// =========================================================================
// Part Q: 复位同步与消抖
// =========================================================================
// 按键复位消抖 (ref_clk域)
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10)
    `else
    .RST_CNTR_VALUE     (16'hC000)
    `endif
) u_refclk_buttonrstn_debounce (
    .clk        (ref_clk),
    .rstn_in    (button_rst_n),
    .rstn_out   (sync_button_rst_n)
);

// PCIe复位消抖 (ref_clk域)
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10)
    `else
    .RST_CNTR_VALUE     (16'hC000)
    `endif
) u_refclk_perstn_debounce (
    .clk        (ref_clk),
    .rstn_in    (perst_n),
    .rstn_out   (sync_perst_n)
);

// core_rst_n -> ref_clk域同步
hsst_rst_sync_v1_0 u_ref_core_rstn_sync (
    .clk        (ref_clk),
    .rst_n      (core_rst_n),
    .sig_async  (1'b1),
    .sig_synced (ref_core_rst_n)
);

// core_rst_n -> pclk域同步
hsst_rst_sync_v1_0 u_pclk_core_rstn_sync (
    .clk        (pclk),
    .rst_n      (core_rst_n),
    .sig_async  (1'b1),
    .sig_synced (s_pclk_rstn)
);

// =========================================================================
// Part R: LED 指示 (ref_clk域 / pclk域)
// =========================================================================
reg [22:0] ref_led_cnt;
reg [26:0] pclk_led_cnt;

// ref_clk域: ref_led闪烁
always @(posedge ref_clk or negedge sync_perst_n) begin
    if (!sync_perst_n)
        ref_led_cnt <= 23'd0;
    else
        ref_led_cnt <= ref_led_cnt + 23'd1;
end

always @(posedge ref_clk or negedge sync_perst_n) begin
    if (!sync_perst_n)
        ref_led <= 1'b1;
    else if (&ref_led_cnt)
        ref_led <= ~ref_led;
end

// pclk域: pclk_led闪烁
always @(posedge pclk or negedge s_pclk_rstn) begin
    if (!s_pclk_rstn)
        pclk_led_cnt <= 27'd0;
    else
        pclk_led_cnt <= pclk_led_cnt + 27'd1;
end

always @(posedge pclk or negedge s_pclk_rstn) begin
    if (!s_pclk_rstn)
        pclk_led <= 1'b1;
    else if (&pclk_led_cnt)
        pclk_led <= ~pclk_led;
end

endmodule
