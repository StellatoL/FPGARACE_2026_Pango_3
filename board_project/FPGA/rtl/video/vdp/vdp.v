module vdp#(
    parameter IMG_HDISP = 1280,
    parameter IMG_VDISP = 720,
    parameter DATA_WIDTH = $clog2(IMG_HDISP > IMG_VDISP ? IMG_HDISP : IMG_VDISP),
    parameter NUM_CHANNELS = 8
)(
    input                           clk,
    input                           rst_n,

    input                           per_frame_vsync,    // 输入的场同步信号
    input                           per_frame_href,     // 输入的行同步信号
    input                           per_frame_clken,    // 输入的数据使能信号
    input        [23:0]             img_rgb,            // 输入的RGB图像数据

    output                          license_plate_exist_flag, // 车牌存在标志位
    output       [3:0]              license_plate_count,      // 当前帧发现了几个车牌
    input                           license_read_en,          // 外部下游模块请求读数据
    input        [3:0]              license_read_addr,        // 外部下游模块请求读第几个车牌
    output       [DATA_WIDTH*4-1:0] license_plate_data        // 车牌坐标数据包 (x_min, x_max, y_min, y_max)
);

// step 1 边缘检测与颜色过滤
wire edge_frame_vsync;
wire edge_frame_href;
wire edge_frame_clken;
wire [7:0] edge_img_data;

edge_top #(
    .DATA_WIDTH(24),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) edge_top_inst (
    .clk              (clk            ),
    .rst_n            (rst_n          ),
    .per_frame_vsync  (per_frame_vsync),
    .per_frame_href   (per_frame_href ),
    .per_frame_clken  (per_frame_clken),
    .img_rgb          (img_rgb        ),
    .edge_frame_vsync (edge_frame_vsync),
    .edge_frame_href  (edge_frame_href ),
    .edge_frame_clken (edge_frame_clken),
    .edge_img_data    (edge_img_data   )
);

// step2 中值滤波消除噪点与二值化
wire median_frame_vsync;
wire median_frame_href;
wire median_frame_clken;
wire [7:0] median_img_data;

vip_gray_median_filter #(
    .DATA_WIDTH(8),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) vip_gray_median_filter_inst (
    .clk             (clk               ),
    .rst_n           (rst_n             ),
    .pre_frame_vsync (edge_frame_vsync  ),
    .pre_frame_href  (edge_frame_href   ),
    .pre_frame_clken (edge_frame_clken  ),
    .pre_img_y       (edge_img_data     ),
    .pos_frame_vsync (median_frame_vsync),
    .pos_frame_href  (median_frame_href ),
    .pos_frame_clken (median_frame_clken),
    .pos_img_y       (median_img_data   )
);

wire bin_frame_vsync;
wire bin_frame_href;
wire bin_frame_clken;
wire bin_img_bit;

binarization binarization_inst (
    .clk             (clk               ),
    .rst_n           (rst_n             ),
    .binary_threshold(8'd88            ), // 二值化阈值
    .ycbcr_vsync     (median_frame_vsync),
    .ycbcr_href      (median_frame_href ),
    .ycbcr_de        (median_frame_clken ),
    .luminance       (median_img_data   ),
    .post_vsync      (bin_frame_vsync   ),
    .post_href       (bin_frame_href    ),
    .post_de         (bin_frame_clken   ),
    .monoc           (bin_img_bit       )
);
// guass_bin #(
//     .DATA_WIDTH(8),
//     .IMG_WIDTH(IMG_HDISP),
//     .IMG_HEIGHT(IMG_VDISP),
//     .THRESHOLD_OFFSET(8)
// ) guass_bin_inst (
//     .clk             (clk               ),
//     .rst_n           (rst_n             ),
//     .ycbcr_vsync     (median_frame_vsync),
//     .ycbcr_href      (median_frame_href ),
//     .ycbcr_de        (median_frame_clken),
//     .luminance       (median_img_data   ),
//     .post_vsync      (bin_frame_vsync   ),
//     .post_href       (bin_frame_href    ),
//     .post_de         (bin_frame_clken   ),
//     .monoc           (bin_img_bit       )
// );

// step3 形态学处理
wire morph_frame_vsync;
wire morph_frame_href;
wire morph_frame_clken;
wire morph_img_bit;
morph_pipeline #(
    .DATA_WIDTH(1),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) morph_pipeline_inst (
    .clk              (clk               ),
    .rst_n            (rst_n             ),
    .pre_frame_vsync  (bin_frame_vsync   ),
    .pre_frame_href   (bin_frame_href    ),
    .pre_frame_clken  (bin_frame_clken   ),
    .pre_img_bit      (bin_img_bit       ),
    .post_frame_vsync (morph_frame_vsync ),
    .post_frame_href  (morph_frame_href  ),
    .post_frame_clken (morph_frame_clken ),
    .post_img_bit     (morph_img_bit     )
);

// =========================================================================
// Step 4: 水平与垂直投影 (级联多目标定位)
// =========================================================================

wire        proj_frame_vsync;
wire        proj_frame_href;
wire        proj_frame_clken;
wire        proj_img_bit;

wire        top_fifo_empty;
wire        top_fifo_rd_en;
wire [DATA_WIDTH * 4-1:0] top_fifo_rd_data;

VIP_projection_top #(
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP),
    .DATA_WIDTH(DATA_WIDTH),
    .NUM_CHANNELS(NUM_CHANNELS),
    .H_MIN_HEIGHT(20),
    .H_THRESHOLD(45),
    .V_THRESHOLD(15)
) VIP_projection_top_inst (
    .clk               (clk),
    .rst_n             (rst_n),
    
    // 输入形态学处理后的二值流
    .per_frame_vsync   (morph_frame_vsync),
    .per_frame_href    (morph_frame_href),
    .per_frame_clken   (morph_frame_clken),
    .per_img_Bit       (morph_img_bit),
    
    // 同步输出流 (供后续显示或处理使用)
    .post_frame_vsync  (proj_frame_vsync),
    .post_frame_href   (proj_frame_href),
    .post_frame_clken  (proj_frame_clken),
    .post_img_Bit      (proj_img_bit),
    
    // 聚合坐标输出接口
    .top_fifo_rd_en    (top_fifo_rd_en),
    .top_fifo_rd_data  (top_fifo_rd_data),
    .top_fifo_empty    (top_fifo_empty)
);

// =========================================================================
// Step 5: geometry filter and multi-plate pingpong buffer
// =========================================================================

wire                  license_found_pulse;
wire [DATA_WIDTH-1:0] plate_x_min;
wire [DATA_WIDTH-1:0] plate_x_max;
wire [DATA_WIDTH-1:0] plate_y_min;
wire [DATA_WIDTH-1:0] plate_y_max;

VIP_multi_plate_filter #(
    .DATA_WIDTH(DATA_WIDTH),
    .MIN_W(50),
    .MIN_H(20),
    .MAX_W(IMG_HDISP - (IMG_HDISP >> 4)),
    .MAX_H(IMG_VDISP)
) VIP_multi_plate_filter_inst (
    .clk             (clk),
    .rst_n           (rst_n),
    .per_frame_vsync (proj_frame_vsync),
    .fifo_empty      (top_fifo_empty),
    .fifo_data       (top_fifo_rd_data),
    .fifo_rd_en      (top_fifo_rd_en),
    .plate_valid     (license_found_pulse),
    .plate_left      (plate_x_min),
    .plate_right     (plate_x_max),
    .plate_up        (plate_y_min),
    .plate_down      (plate_y_max)
);

wire [3:0]              svm_plate_count;
wire [DATA_WIDTH*4-1:0] svm_plate_data;

VIP_plate_pingpong_buffer #(
    .DATA_WIDTH(DATA_WIDTH),
    .MAX_PLATES(NUM_CHANNELS)
) VIP_plate_pingpong_buffer_inst (
    .clk              (clk),
    .rst_n            (rst_n),
    .per_frame_vsync  (proj_frame_vsync),
    .plate_valid      (license_found_pulse),
    .plate_x_min      (plate_x_min),
    .plate_x_max      (plate_x_max),
    .plate_y_min      (plate_y_min),
    .plate_y_max      (plate_y_max),
    .stored_plate_num (svm_plate_count),
    .read_en          (license_read_en),
    .read_addr        (license_read_addr),
    .read_data        (svm_plate_data)
);

// =========================================================================
// Step 7: 顶层输出映射
// =========================================================================
assign license_plate_exist_flag = (svm_plate_count > 0) ? 1'b1 : 1'b0;
assign license_plate_count      = svm_plate_count;
assign license_plate_data       = svm_plate_data;
endmodule