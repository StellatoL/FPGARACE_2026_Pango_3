module edge_top#(
    parameter DATA_WIDTH      = 24,   //输入图像数据位宽
    parameter IMG_HDISP       = 1024,//图像水平分辨率
    parameter IMG_VDISP       = 600, //图像垂直分辨率
    parameter DEPTH           = (IMG_HDISP>IMG_VDISP)?IMG_HDISP:IMG_VDISP
)(
    input           clk,
    input           rst_n,
    input           per_frame_vsync,
    input           per_frame_href,
    input           per_frame_clken,
    input   [23:0]  img_rgb,

    output          edge_frame_vsync,
    output          edge_frame_href,
    output          edge_frame_clken,
    output  [ 7:0]  edge_img_data  
);

// step1 生成3x3矩阵
wire matrix_frame_vsync;
wire matrix_frame_href;
wire matrix_frame_clken;
wire [DATA_WIDTH - 1:0] matrix_p11;
wire [DATA_WIDTH - 1:0] matrix_p12;
wire [DATA_WIDTH - 1:0] matrix_p13;
wire [DATA_WIDTH - 1:0] matrix_p21;
wire [DATA_WIDTH - 1:0] matrix_p22;
wire [DATA_WIDTH - 1:0] matrix_p23;
wire [DATA_WIDTH - 1:0] matrix_p31;
wire [DATA_WIDTH - 1:0] matrix_p32;
wire [DATA_WIDTH - 1:0] matrix_p33;

//3x3矩阵
vip_matrix_generate_3x3_8bit #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DEPTH)
)u_vip_matrix_generate_3x3_24bit(
    .clk                 (clk),    
    .rst_n               (rst_n),
    //预处理数据
    .pre_frame_vsync     (per_frame_vsync), 
    .pre_frame_href      (per_frame_href),  
    .pre_frame_clken     (per_frame_clken), 
    .pre_img_y           (img_rgb),       
    
    //处理后的数据
    .matrix_frame_vsync  (matrix_frame_vsync), 
    .matrix_frame_href   (matrix_frame_href),  
    .matrix_frame_clken  (matrix_frame_clken), 
    .matrix_p11          (matrix_p11), 
    .matrix_p12          (matrix_p12), 
    .matrix_p13          (matrix_p13), //输出 3X3 矩阵
    .matrix_p21          (matrix_p21), 
    .matrix_p22          (matrix_p22),  
    .matrix_p23          (matrix_p23),
    .matrix_p31          (matrix_p31), 
    .matrix_p32          (matrix_p32),  
    .matrix_p33          (matrix_p33)
);



//step2 sobel边缘检测

wire sobel_frame_vsync;
wire sobel_frame_href;
wire sobel_frame_clken;
wire [7:0] sobel_img_r;
wire [7:0] sobel_img_g;
wire [7:0] sobel_img_b;

vip_sobel_edge_detector vip_sobel_edge_detector_inst_r(
    .clk             (clk            ), 
    .rst_n           (rst_n          ), 
    .pre_frame_vsync (matrix_frame_vsync), 
    .pre_frame_href  (matrix_frame_href ), 
    .pre_frame_clken (matrix_frame_clken),
    .matrix_p11 (matrix_p11[23:16]), .matrix_p12 (matrix_p12[23:16]), .matrix_p13 (matrix_p13[23:16]),
    .matrix_p21 (matrix_p21[23:16]), .matrix_p22 (matrix_p22[23:16]), .matrix_p23 (matrix_p23[23:16]),
    .matrix_p31 (matrix_p31[23:16]), .matrix_p32 (matrix_p32[23:16]), .matrix_p33 (matrix_p33[23:16]),
    .post_frame_vsync(sobel_frame_vsync), 
    .post_frame_href (sobel_frame_href ), 
    .post_frame_clken(sobel_frame_clken),
    .post_img_y      (sobel_img_r    )
);

vip_sobel_edge_detector vip_sobel_edge_detector_inst_g(
    .clk             (clk            ), 
    .rst_n           (rst_n          ), 
    .pre_frame_vsync (matrix_frame_vsync), 
    .pre_frame_href  (matrix_frame_href ), 
    .pre_frame_clken (matrix_frame_clken), 
    .matrix_p11 (matrix_p11[15:8]), .matrix_p12 (matrix_p12[15:8]), .matrix_p13 (matrix_p13[15:8]),
    .matrix_p21 (matrix_p21[15:8]), .matrix_p22 (matrix_p22[15:8]), .matrix_p23 (matrix_p23[15:8]),
    .matrix_p31 (matrix_p31[15:8]), .matrix_p32 (matrix_p32[15:8]), .matrix_p33 (matrix_p33[15:8]),
    .post_frame_vsync(), 
    .post_frame_href (), 
    .post_frame_clken(),
    .post_img_y      (sobel_img_g)    
);

vip_sobel_edge_detector vip_sobel_edge_detector_inst_b(
    .clk             (clk            ), 
    .rst_n           (rst_n          ), 
    .pre_frame_vsync (matrix_frame_vsync), 
    .pre_frame_href  (matrix_frame_href ), 
    .pre_frame_clken (matrix_frame_clken), 
    .matrix_p11 (matrix_p11[7:0]), .matrix_p12 (matrix_p12[7:0]), .matrix_p13 (matrix_p13[7:0]),
    .matrix_p21 (matrix_p21[7:0]), .matrix_p22 (matrix_p22[7:0]), .matrix_p23 (matrix_p23[7:0]),
    .matrix_p31 (matrix_p31[7:0]), .matrix_p32 (matrix_p32[7:0]), .matrix_p33 (matrix_p33[7:0]),
    .post_frame_vsync(), 
    .post_frame_href (), 
    .post_frame_clken(),
    .post_img_y      (sobel_img_b)
);

//step2 计算图像掩膜 共消耗7个周期的流水线延迟
wire [1:0] color_mask_bit;

color_mask color_mask_inst(
    .clk             (clk            ), 
    .rst_n           (rst_n          ), 
    .per_frame_vsync (matrix_frame_vsync), 
    .per_frame_href  (matrix_frame_href ), 
    .per_frame_clken (matrix_frame_clken), 
    .matrix_p11 (matrix_p11), .matrix_p12 (matrix_p12), .matrix_p13 (matrix_p13),
    .matrix_p21 (matrix_p21), .matrix_p22 (matrix_p22), .matrix_p23 (matrix_p23),
    .matrix_p31 (matrix_p31), .matrix_p32 (matrix_p32), .matrix_p33 (matrix_p33),
    .color_mask_frame_vsync(), 
    .color_mask_frame_href (), 
    .color_mask_frame_clken(),
    .color_mask_img_mask  (color_mask_bit)
);
// 将颜色掩膜数据延迟对齐到 Sobel 输出
// 延迟为15-7=8个周期
reg [1:0] color_mask_d [0:7];
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 8; i = i + 1) begin
            color_mask_d[i] <= 2'd0;
        end
    end else begin
        // 移位寄存器打拍，对其进行流水线同步
        color_mask_d[0] <= color_mask_bit;
        for (i = 1; i < 8; i = i + 1) begin
            color_mask_d[i] <= color_mask_d[i-1];
        end
    end
end

wire [1:0] aligned_color_mask = color_mask_d[7];

// step3 计算最终边缘图像输出

reg [7:0] final_edge_data;

// 为了和 final_edge_data 对齐，把由 Sobel 输出的同步信号再打一拍
reg edge_vsync_d;
reg edge_href_d;
reg edge_clken_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        final_edge_data <= 8'd0;
        edge_vsync_d    <= 1'b0;
        edge_href_d     <= 1'b0;
        edge_clken_d    <= 1'b0;
    end else begin
        // 同步信号打拍
        edge_vsync_d <= sobel_frame_vsync;
        edge_href_d  <= sobel_frame_href;
        edge_clken_d <= sobel_frame_clken;
        
        // 只有在数据有效时才进行边缘选择
        if (sobel_frame_clken) begin
            case (aligned_color_mask)
                2'd1: // 标签 1 (蓝色车牌): max(R, G)
                    final_edge_data <= (sobel_img_r > sobel_img_g) ? sobel_img_r : sobel_img_g;
                
                2'd2: // 标签 2 (绿色车牌): max(R, B)
                    final_edge_data <= (sobel_img_r > sobel_img_b) ? sobel_img_r : sobel_img_b;
                
                2'd3: // 标签 3 (黄色车牌): 黑字黄底，避开 R 和 G (黄底反光)，提取 max(B, G)
                    final_edge_data <= (sobel_img_b > sobel_img_g) ? sobel_img_b : sobel_img_g;
                
                default: // 非车牌区域直接置 0，这一步相当于极强的背景降噪！
                    final_edge_data <= 8'd0; 
            endcase
        end else begin
            final_edge_data <= 8'd0;
        end
    end
end

// 输出最终边缘图像数据和同步信号
assign edge_frame_vsync = edge_vsync_d;
assign edge_frame_href  = edge_href_d;
assign edge_frame_clken = edge_clken_d;
assign edge_img_data    = final_edge_data;

endmodule