module vip_mean_filter_3x3 #(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP  = 1024,
    parameter IMG_VDISP  = 600,
    parameter DEPTH      = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input                      clk,
    input                      rst_n,
        
    input                      pre_frame_vsync,
    input                      pre_frame_href,
    input                      pre_frame_clken,
    input  [DATA_WIDTH - 1:0]  pre_frame_data,

    output                     pos_frame_vsync,
    output                     pos_frame_href,
    output                     pos_frame_clken,
    output [DATA_WIDTH - 1:0]  pos_frame_data,

    output [DATA_WIDTH - 1:0]  median_data 
);

// 5x5 矩阵输出
wire        matrix_frame_vsync;
wire        matrix_frame_href;
wire        matrix_frame_clken;
wire [DATA_WIDTH - 1:0]  matrix_p11, matrix_p12, matrix_p13;
wire [DATA_WIDTH - 1:0]  matrix_p21, matrix_p22, matrix_p23;
wire [DATA_WIDTH - 1:0]  matrix_p31, matrix_p32, matrix_p33;

vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_mat_gen_3x3(
    .clk                 (clk),    
    .rst_n               (rst_n),
    .pre_frame_vsync     (pre_frame_vsync), 
    .pre_frame_href      (pre_frame_href),  
    .pre_frame_clken     (pre_frame_clken), 
    .pre_img_y           (pre_frame_data),
    .matrix_frame_vsync  (matrix_frame_vsync),
    .matrix_frame_href   (matrix_frame_href),
    .matrix_frame_clken  (matrix_frame_clken),
    .matrix_p11 (matrix_p11), .matrix_p12 (matrix_p12), .matrix_p13 (matrix_p13),
    .matrix_p21 (matrix_p21), .matrix_p22 (matrix_p22), .matrix_p23 (matrix_p23),
    .matrix_p31 (matrix_p31), .matrix_p32 (matrix_p32), .matrix_p33 (matrix_p33)
);

// 计算 3x3 平均值
wire [DATA_WIDTH - 1:0] mean_value;
wire                    post_frame_vsync;
wire                    post_frame_href;
wire                    post_frame_clken;

// 延迟为3拍
mean_filter_3x3 #(
    .DATA_WIDTH(DATA_WIDTH),
    .IMG_HDISP(IMG_HDISP),
    .IMG_VDISP(IMG_VDISP)
) u_mean_filter_3x3 (
    .clk            (clk),
    .rst_n          (rst_n),
    .mean_frame_vsync(matrix_frame_vsync),
    .mean_frame_href (matrix_frame_href),
    .mean_frame_clken(matrix_frame_clken),
    .data11 (matrix_p11), .data12 (matrix_p12), .data13 (matrix_p13),
    .data21 (matrix_p21), .data22 (matrix_p22), .data23 (matrix_p23),
    .data31 (matrix_p31), .data32 (matrix_p32), .data33 (matrix_p33),
    .pos_median_vsync(post_frame_vsync),
    .pos_median_href (post_frame_href),
    .pos_median_clken(post_frame_clken),
    .target_data     (mean_value)
);

// 对中间结果进行延迟处理，保持与原始数据同步
reg [DATA_WIDTH - 1:0] delay_img_y_d0;
reg [DATA_WIDTH - 1:0] delay_img_y_d1;
reg [DATA_WIDTH - 1:0] delay_img_y_d2;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        delay_img_y_d0 <= 0;
        delay_img_y_d1 <= 0;
        delay_img_y_d2 <= 0;
    end
    else begin
        if (matrix_frame_clken) begin
            delay_img_y_d0 <= matrix_p33;
            delay_img_y_d1 <= delay_img_y_d0;
            delay_img_y_d2 <= delay_img_y_d1;
        end
    end
end

assign pos_frame_vsync = post_frame_vsync;
assign pos_frame_href = post_frame_href;
assign pos_frame_clken = post_frame_clken;
assign pos_frame_data = mean_value;
assign median_data = delay_img_y_d2;

endmodule