module light_enhance #(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP = 1024,
    parameter IMG_VDISP = 600,
    parameter DEPTH = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    // 输入视频流同步信号
    input  wire        per_frame_vsync,
    input  wire        per_frame_href,
    input  wire        per_frame_clken,
    input  wire [DATA_WIDTH - 1:0]  img_y_in,
    input  wire [DATA_WIDTH - 1:0]  img_cb_in,
    input  wire [DATA_WIDTH - 1:0]  img_cr_in,

    // 输出视频流同步信号
    output wire        post_frame_vsync,
    output wire        post_frame_href,
    output wire        post_frame_clken,
    output wire [DATA_WIDTH - 1:0]  img_y_out,
    output wire [DATA_WIDTH - 1:0]  img_cb_out,
    output wire [DATA_WIDTH - 1:0]  img_cr_out
);

// 5x5均值滤波
wire         mean_frame_vsync;
wire         mean_frame_href;
wire         mean_frame_clken;
wire [DATA_WIDTH - 1:0] mean_data_y;
wire [DATA_WIDTH - 1:0] delay_img_y_d3;

vip_mean_filter_5x5 #(
    .DATA_WIDTH(DATA_WIDTH),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
)u_vip_mean_filter_5x5(
    .clk            (clk),
    .rst_n          (rst_n),

    .pre_frame_vsync(per_frame_vsync),
    .pre_frame_href (per_frame_href),
    .pre_frame_clken(per_frame_clken),
    .pre_frame_data (img_y_in),

    .pos_frame_vsync(mean_frame_vsync),
    .pos_frame_href (mean_frame_href),
    .pos_frame_clken(mean_frame_clken),
    .pos_frame_data (mean_data_y),
    .median_data    (delay_img_y_d3)
);

// 只有亮度数据经过均值滤波，对色度数据进行延迟处理，以保持同步
// 1次5x5加3拍
wire [DATA_WIDTH - 1:0]      delay_img_cb_d0;
wire [DATA_WIDTH - 1:0]      delay_img_cr_d0;

vip_matrix_generate_5x5_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_delay_cb(
    .clk            (clk),
    .rst_n          (rst_n),

    .per_frame_vsync(per_frame_vsync),
    .per_frame_href (per_frame_href),
    .per_frame_clken(per_frame_clken),
    .per_img_y      (img_cb_in),

    .matrix_frame_vsync(),
    .matrix_frame_href (),
    .matrix_frame_clken(),
    .matrix_p11       (),
    .matrix_p12       (),
    .matrix_p13       (),
    .matrix_p14       (),
    .matrix_p15       (),
    .matrix_p21       (),
    .matrix_p22       (),
    .matrix_p23       (),
    .matrix_p24       (),
    .matrix_p25       (),
    .matrix_p31       (),
    .matrix_p32       (),
    .matrix_p33       (delay_img_cb_d0),
    .matrix_p34       (),
    .matrix_p35       (),
    .matrix_p41       (),
    .matrix_p42       (),
    .matrix_p43       (),
    .matrix_p44       (),
    .matrix_p45       (),
    .matrix_p51       (),
    .matrix_p52       (),
    .matrix_p53       (),
    .matrix_p54       (),
    .matrix_p55       ()
);

vip_matrix_generate_5x5_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_delay_cr(
    .clk            (clk),
    .rst_n          (rst_n),

    .per_frame_vsync(per_frame_vsync),
    .per_frame_href (per_frame_href),
    .per_frame_clken(per_frame_clken),
    .per_img_y      (img_cr_in),

    .matrix_frame_vsync(),
    .matrix_frame_href (),
    .matrix_frame_clken(),
    .matrix_p11       (),
    .matrix_p12       (),
    .matrix_p13       (),
    .matrix_p14       (),
    .matrix_p15       (),
    .matrix_p21       (),
    .matrix_p22       (),
    .matrix_p23       (),
    .matrix_p24       (),
    .matrix_p25       (),
    .matrix_p31       (),
    .matrix_p32       (),
    .matrix_p33       (delay_img_cr_d0),
    .matrix_p34       (),
    .matrix_p35       (),
    .matrix_p41       (),
    .matrix_p42       (),
    .matrix_p43       (),
    .matrix_p44       (),
    .matrix_p45       (),
    .matrix_p51       (),
    .matrix_p52       (),
    .matrix_p53       (),
    .matrix_p54       (),
    .matrix_p55       ()
);

reg [DATA_WIDTH - 1:0]      delay_img_cb_d1;
reg [DATA_WIDTH - 1:0]      delay_img_cr_d1;
reg [DATA_WIDTH - 1:0]      delay_img_cb_d2;
reg [DATA_WIDTH - 1:0]      delay_img_cr_d2;
reg [DATA_WIDTH - 1:0]      delay_img_cb_d3;
reg [DATA_WIDTH - 1:0]      delay_img_cr_d3;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        delay_img_cb_d1 <= {DATA_WIDTH{1'b0}};
        delay_img_cb_d2 <= {DATA_WIDTH{1'b0}};
        delay_img_cb_d3 <= {DATA_WIDTH{1'b0}};
        delay_img_cr_d1 <= {DATA_WIDTH{1'b0}};
        delay_img_cr_d2 <= {DATA_WIDTH{1'b0}};
        delay_img_cr_d3 <= {DATA_WIDTH{1'b0}};
    end
    else begin
        delay_img_cb_d1 <= delay_img_cb_d0;
        delay_img_cb_d2 <= delay_img_cb_d1;
        delay_img_cb_d3 <= delay_img_cb_d2;

        delay_img_cr_d1 <= delay_img_cr_d0;
        delay_img_cr_d2 <= delay_img_cr_d1;
        delay_img_cr_d3 <= delay_img_cr_d2;
    end
end

// 亮度增强模块
wire [DATA_WIDTH - 1:0] bright_img_y;
wire [DATA_WIDTH - 1:0] bright_img_cb;
wire [DATA_WIDTH - 1:0] bright_img_cr;
wire                    bright_img_vsync;
wire                    bright_img_href;
wire                    bright_img_clken;

adaptive_light_enhancement #(
    .DATA_WIDTH(DATA_WIDTH)
)u_adaptive_light_enhancement(
    .clk        (clk),
    .rst_n      (rst_n),

    .per_frame_vsync(mean_frame_vsync),
    .per_frame_href (mean_frame_href),
    .per_frame_clken(mean_frame_clken),
    .img_y_in      (delay_img_y_d3),
    .img_cb_in     (delay_img_cb_d3),
    .img_cr_in     (delay_img_cr_d3),
    .local_y_mean  (mean_data_y),

    .post_frame_vsync(bright_img_vsync),
    .post_frame_href (bright_img_href),
    .post_frame_clken(bright_img_clken),
    .img_y_out      (bright_img_y),
    .img_cb_out     (bright_img_cb),
    .img_cr_out     (bright_img_cr)
);

// Disabled is a true zero-latency wire-through at the module boundary.
assign post_frame_vsync = enable ? bright_img_vsync : per_frame_vsync;
assign post_frame_href  = enable ? bright_img_href  : per_frame_href;
assign post_frame_clken = enable ? bright_img_clken : per_frame_clken;
assign img_y_out        = enable ? bright_img_y     : img_y_in;
assign img_cb_out       = enable ? bright_img_cb    : img_cb_in;
assign img_cr_out       = enable ? bright_img_cr    : img_cr_in;

endmodule
