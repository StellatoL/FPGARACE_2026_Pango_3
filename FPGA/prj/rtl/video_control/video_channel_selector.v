// =============================================================================
// Module     : video_channel_selector
// Description: 视频源通道选择器
//              - 通过按键切换HDMI测试源和OV5640摄像头源
//              - 20ms按键消抖 + 下降沿检测
//              - video_set=1: 测试通道(HDMI) / video_set=0: 摄像头通道
// =============================================================================

module video_channel_selector (
    input  wire         clk,              // 系统时钟 (25MHz/27MHz)
    input  wire         rst_n,            // 异步复位, 低有效
    input  wire         key_in,           // 按键输入 (按下为低电平)
    input  wire         remote_select,    // ARM I2C 0x01[0]: 0=摄像头, 1=HDMI
    input  wire         remote_update_toggle, // I2C每次写0x01时翻转

    // ---- 测试通道输入 (HDMI + 下采样) ----
    input  wire         pixclk_in,        // 测试通道像素时钟
    input  wire         hdmi_vs_in,       // 测试通道场同步
    input  wire         hdmi_href_in,     // HDMI line active
    input  wire         hdmi_de_in,       // 测试通道数据有效
    input  wire [15:0]  hdmi_rgb565,      // 测试通道RGB565数据

    // ---- 摄像头通道输入 (OV5640) ----
    input  wire         cam_clk,          // 摄像头像素时钟
    input  wire         cam_vs,           // 摄像头场同步
    input  wire         cam_href,         // camera line active
    input  wire         cam_de,           // 摄像头数据有效
    input  wire [15:0]  cam_data,         // 摄像头RGB565数据

    // ---- 多路复用输出 ----
    output wire         mux_pclk,         // 输出像素时钟
    output wire         mux_vs,           // 输出场同步
    output wire         mux_href,         // output line active
    output wire         mux_de,           // 输出数据有效
    output wire [15:0]  mux_rgb565,       // 输出RGB565数据

    // ---- 状态指示 ----
    output wire         video_set_led     // 通道指示LED (1=测试, 0=摄像头)
);

// =============================================================================
// 参数定义
// =============================================================================
localparam DELAY_20MS = 19'd500_000;       // 20ms消抖 (25MHz * 0.02s)

// =============================================================================
// 消抖与状态寄存器
// =============================================================================
reg [18:0]  delay_cnt;                     // 消抖计数器
reg         key_stable;                    // 消抖后的稳定状态
reg         key_stable_d0;                 // 边沿检测用延迟
reg         video_set;                     // 通道选择: 0=摄像头, 1=HDMI测试
reg         remote_select_meta;
reg         remote_select_sync;
reg         remote_toggle_meta;
reg         remote_toggle_sync;
reg         remote_toggle_sync_d;

// I2C控制寄存器位于50 MHz域。选择值与更新toggle分别做两级同步，
// toggle到达时数据已经稳定，因此不会丢失重复写入命令。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        remote_select_meta  <= 1'b0;
        remote_select_sync  <= 1'b0;
        remote_toggle_meta  <= 1'b0;
        remote_toggle_sync  <= 1'b0;
        remote_toggle_sync_d <= 1'b0;
    end else begin
        remote_select_meta   <= remote_select;
        remote_select_sync   <= remote_select_meta;
        remote_toggle_meta   <= remote_update_toggle;
        remote_toggle_sync   <= remote_toggle_meta;
        remote_toggle_sync_d <= remote_toggle_sync;
    end
end

// =============================================================================
// 第一部分: 按键消抖 (20ms窗口)
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        delay_cnt  <= 19'd0;
        key_stable <= 1'b1;                // 默认未按下
    end else if (key_in != key_stable) begin
        delay_cnt <= delay_cnt + 1'b1;
        if (delay_cnt >= DELAY_20MS) begin
            key_stable <= key_in;
            delay_cnt  <= 19'd0;
        end
    end else begin
        delay_cnt <= 19'd0;
    end
end

// =============================================================================
// 第二部分: 下降沿检测与状态翻转
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key_stable_d0 <= 1'b1;
        video_set     <= 1'b0;             // 默认摄像头通道
    end else begin
        key_stable_d0 <= key_stable;
        // ARM命令优先；没有新ARM命令时仍保留本地按键切换。
        if (remote_toggle_sync != remote_toggle_sync_d)
            video_set <= remote_select_sync;
        else if (key_stable_d0 == 1'b1 && key_stable == 1'b0)
            video_set <= ~video_set;
    end
end

// =============================================================================
// 第三部分: 视频通道复用 (数据信号)
// =============================================================================
assign mux_vs       = video_set ? hdmi_vs_in  : cam_vs;
assign mux_href     = video_set ? hdmi_href_in : cam_href;
assign mux_de       = video_set ? hdmi_de_in  : cam_de;
assign mux_rgb565   = video_set ? hdmi_rgb565 : cam_data;

assign video_set_led = video_set;

// =============================================================================
// 第四部分: 时钟复用 (使用紫光同创 GTP_CLKBUFGMUX 原语)
// =============================================================================
GTP_CLKBUFGMUX #(
    .TRIGGER_MODE("NEGEDGE"), // 在输入时钟低电平期间切换，避免窄脉冲/毛刺
    .SIM_DEVICE  ("LOGOS2")
) u_mux_pclk (
    .CLKOUT (mux_pclk),      // 输出全局时钟
    .CLKIN0 (cam_clk),       // 当 SEL=0 时的输入时钟
    .CLKIN1 (pixclk_in),     // 当 SEL=1 时的输入时钟
    .SEL    (video_set)      // 通道选择控制信号
);

endmodule
