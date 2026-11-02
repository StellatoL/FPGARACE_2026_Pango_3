// 自动白平衡算法
// Auto white balance
module image_awb #(
    parameter IMG_HDISP   = 1280,
    parameter IMG_VDISP   = 720,
    parameter RECIP_SHIFT = 16
)(
    input  wire        i_clk       ,
    input  wire        i_rst_n     ,
    input  wire        i_enable    ,

    input  wire        i_hs        ,
    input  wire        i_vs        ,
    input  wire        i_de        ,
    input  wire        i_pic_valid ,
    input  wire [7:0]  i_r         ,
    input  wire [7:0]  i_g         ,
    input  wire [7:0]  i_b         ,

    output wire        o_hs        ,
    output wire        o_vs        ,
    output wire        o_de        ,
    output wire        o_pic_valid ,
    output wire [7:0]  o_r         ,
    output wire [7:0]  o_g         ,
    output wire [7:0]  o_b
);
localparam IMG_PIXELS = IMG_HDISP * IMG_VDISP; // 图像总像素点数
localparam SUM_WIDTH = $clog2(IMG_PIXELS * 256); // 像素总和的二进制位宽
localparam PIX_CNT_WIDTH = $clog2(IMG_PIXELS + 1);
localparam [SUM_WIDTH - 1:0] DIV_STAGE_LIST = {SUM_WIDTH{1'b1}};
localparam RECIP_WIDTH = RECIP_SHIFT + 1;
localparam SCALE_WIDTH = 16 + RECIP_WIDTH;
localparam [SUM_WIDTH - 1:0] IMG_PIXELS_DIVISOR = IMG_PIXELS;
localparam [SCALE_WIDTH - 1:0] RECIP_ROUND =
    {{(SCALE_WIDTH - 1){1'b0}}, 1'b1} << (RECIP_SHIFT - 1);
// RECIP_SHIFT must be in the range 1..30 for the 32-bit LUT initializer below.

// wire or reg define
reg  [SUM_WIDTH - 1:0] sum_r       ; // R通道像素值总和
reg  [SUM_WIDTH - 1:0] sum_g       ; // G通道像素值总和
reg  [SUM_WIDTH - 1:0] sum_b       ; // B通道像素值总和
reg  [PIX_CNT_WIDTH - 1:0] pixel_cnt;
reg         frame_sum_done;
reg  [SUM_WIDTH - 1:0] frame_sum_r;
reg  [SUM_WIDTH - 1:0] frame_sum_g;
reg  [SUM_WIDTH - 1:0] frame_sum_b;
reg         div_issue_active;
reg  [1:0]  div_issue_index;
reg  [1:0]  div_result_index;
reg  [7:0]  r_aver      ; // R通道像素平均值
reg  [7:0]  g_aver      ; // G通道像素平均值
reg  [7:0]  b_aver      ; // B通道像素平均值
reg         aver_flag;
reg  [7:0]  k_aver      ; // RGB平均灰度
reg  [RECIP_WIDTH - 1:0] r_reciprocal;
reg  [RECIP_WIDTH - 1:0] g_reciprocal;
reg  [RECIP_WIDTH - 1:0] b_reciprocal;
reg  [2:0]  coeff_valid_pipe;

reg  [7:0]  r_new       ;
reg  [7:0]  g_new       ;
reg  [7:0]  b_new       ;
wire [15:0] r_mul_ik    ;
wire [15:0] g_mul_ik    ;
wire [15:0] b_mul_ik    ;
wire [SCALE_WIDTH - 1:0] r_mul_all;
wire [SCALE_WIDTH - 1:0] g_mul_all;
wire [SCALE_WIDTH - 1:0] b_mul_all;
wire [16:0] r_scaled;
wire [16:0] g_scaled;
wire [16:0] b_scaled;
reg  [15:0] r_mul_ik_r  ; // 像素乘目标灰度的流水寄存器
reg  [15:0] g_mul_ik_r  ;
reg  [15:0] b_mul_ik_r  ;
reg  [RECIP_WIDTH - 1:0] r_reciprocal_r;
reg  [RECIP_WIDTH - 1:0] g_reciprocal_r;
reg  [RECIP_WIDTH - 1:0] b_reciprocal_r;
reg  [SCALE_WIDTH - 1:0] r_mul_all_r;
reg  [SCALE_WIDTH - 1:0] g_mul_all_r;
reg  [SCALE_WIDTH - 1:0] b_mul_all_r;

reg         hs_r        ;
reg         vs_r        ;
reg         de_r        ;
reg         pic_valid_r ;
reg         coeff_valid ;
reg  [7:0]  r_data_r    ;
reg  [7:0]  g_data_r    ;
reg  [7:0]  b_data_r    ;
reg         hs_r2       ;
reg         vs_r2       ;
reg         de_r2       ;
reg         pic_valid_r2;
reg  [7:0]  r_data_r2   ;
reg  [7:0]  g_data_r2   ;
reg  [7:0]  b_data_r2   ;
reg         hs_r3       ;
reg         vs_r3       ;
reg         de_r3       ;
reg         pic_valid_r3;
reg  [7:0]  r_data_r3   ;
reg  [7:0]  g_data_r3   ;
reg  [7:0]  b_data_r3   ;
wire        wb_hs       ;
wire        wb_vs       ;
wire        wb_de       ;
wire        wb_pic_valid;
wire [7:0]  wb_r        ;
wire [7:0]  wb_g        ;
wire [7:0]  wb_b        ;

// main code

// R、G、B各个通道像素值求和
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        sum_r          <= {SUM_WIDTH {1'b0}};
        sum_g          <= {SUM_WIDTH {1'b0}};
        sum_b          <= {SUM_WIDTH {1'b0}};
        pixel_cnt      <= {PIX_CNT_WIDTH {1'b0}};
        frame_sum_done <= 1'b0;
    end
    else if (frame_sum_done) begin
        sum_r          <= i_pic_valid ? {{(SUM_WIDTH - 8){1'b0}}, i_r} : {SUM_WIDTH {1'b0}};
        sum_g          <= i_pic_valid ? {{(SUM_WIDTH - 8){1'b0}}, i_g} : {SUM_WIDTH {1'b0}};
        sum_b          <= i_pic_valid ? {{(SUM_WIDTH - 8){1'b0}}, i_b} : {SUM_WIDTH {1'b0}};
        pixel_cnt      <= i_pic_valid ? {{(PIX_CNT_WIDTH - 1){1'b0}}, 1'b1} : {PIX_CNT_WIDTH {1'b0}};
        frame_sum_done <= 1'b0;
    end
    else if (i_pic_valid) begin // 在图像的有效区域
        sum_r <= sum_r + i_r;
        sum_g <= sum_g + i_g;
        sum_b <= sum_b + i_b;

        if (pixel_cnt == IMG_PIXELS - 1'b1) begin
            pixel_cnt      <= {PIX_CNT_WIDTH {1'b0}};
            frame_sum_done <= 1'b1;
        end
        else begin
            pixel_cnt      <= pixel_cnt + 1'b1;
            frame_sum_done <= 1'b0;
        end
    end
end

wire sum_flag; // 求和结束标志信号
assign sum_flag = frame_sum_done;

wire [SUM_WIDTH - 1:0] div_input_a;
wire [SUM_WIDTH - 1:0] aver_quo;
wire                   aver_ack;

assign div_input_a = (div_issue_index == 2'd0) ? frame_sum_r :
                     (div_issue_index == 2'd1) ? frame_sum_g : frame_sum_b;

// 三个通道依次复用同一除法器；帧间隔远大于除法器流水延迟。
divider #(
    .XLEN       (SUM_WIDTH),
    .STAGE_LIST (DIV_STAGE_LIST)
) divider_aver_rgb (
    .clk (i_clk)            ,
    .rst (~i_rst_n)         ,
    .a   (div_input_a)      ,
    .b   (IMG_PIXELS_DIVISOR),
    .vld (div_issue_active) ,
    .quo (aver_quo)         ,
    .rem ()                 ,
    .ack (aver_ack)
);

// 帧结束时锁存求和结果，避免后续帧覆盖除法器输入。
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        frame_sum_r      <= {SUM_WIDTH{1'b0}};
        frame_sum_g      <= {SUM_WIDTH{1'b0}};
        frame_sum_b      <= {SUM_WIDTH{1'b0}};
        div_issue_active <= 1'b0;
        div_issue_index  <= 2'd0;
        div_result_index <= 2'd0;
        r_aver           <= 8'd0;
        g_aver           <= 8'd0;
        b_aver           <= 8'd0;
        aver_flag        <= 1'b0;
    end
    else begin
        aver_flag <= 1'b0;

        if (sum_flag) begin
            frame_sum_r      <= sum_r;
            frame_sum_g      <= sum_g;
            frame_sum_b      <= sum_b;
            div_issue_active <= 1'b1;
            div_issue_index  <= 2'd0;
        end
        else if (div_issue_active) begin
            if (div_issue_index == 2'd2) begin
                div_issue_active <= 1'b0;
                div_issue_index  <= 2'd0;
            end
            else begin
                div_issue_index <= div_issue_index + 1'b1;
            end
        end

        if (aver_ack) begin
            case (div_result_index)
                2'd0: begin
                    r_aver           <= aver_quo[7:0];
                    div_result_index <= 2'd1;
                end
                2'd1: begin
                    g_aver           <= aver_quo[7:0];
                    div_result_index <= 2'd2;
                end
                default: begin
                    b_aver           <= aver_quo[7:0];
                    div_result_index <= 2'd0;
                    aver_flag        <= 1'b1;
                end
            endcase
        end
    end
end

// 倒数查找表存储 round(2^RECIP_SHIFT / average)。
reg [RECIP_WIDTH - 1:0] recip_lut [0:255];
integer lut_i;

initial begin
    recip_lut[0] = {RECIP_WIDTH{1'b0}};
    for (lut_i = 1; lut_i < 256; lut_i = lut_i + 1) begin
        recip_lut[lut_i] = ((1 << RECIP_SHIFT) + (lut_i >> 1)) / lut_i;
    end
end

wire [9:0] rgb_aver_sum;
wire [18:0] rgb_aver_mul_683;
assign rgb_aver_sum = {2'd0, r_aver} + {2'd0, g_aver} + {2'd0, b_aver};
// 输入范围为0..765时，(sum * 683) >> 11 等于精确的 floor(sum / 3)。
assign rgb_aver_mul_683 = {9'd0, rgb_aver_sum} * 10'd683;

// 三个通道均值全部就绪后，同时更新目标灰度和倒数系数。
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        k_aver       <= 8'd0;
        r_reciprocal <= {RECIP_WIDTH{1'b0}};
        g_reciprocal <= {RECIP_WIDTH{1'b0}};
        b_reciprocal <= {RECIP_WIDTH{1'b0}};
    end
    else if (aver_flag) begin
        k_aver       <= rgb_aver_mul_683[18:11];
        r_reciprocal <= recip_lut[r_aver];
        g_reciprocal <= recip_lut[g_aver];
        b_reciprocal <= recip_lut[b_aver];
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        coeff_valid <= 1'b0;
        coeff_valid_pipe <= 2'b000;
    end
    else begin
        coeff_valid_pipe <= {coeff_valid_pipe[1:0], aver_flag};
        if (coeff_valid_pipe[2])
            coeff_valid <= 1'b1;
    end
end

// 保留倒数小数位到像素乘法结束，避免增益过早量化。
assign r_mul_ik  = {8'd0, i_r} * {8'd0, k_aver};
assign g_mul_ik  = {8'd0, i_g} * {8'd0, k_aver};
assign b_mul_ik  = {8'd0, i_b} * {8'd0, k_aver};
assign r_mul_all = {{RECIP_WIDTH{1'b0}}, r_mul_ik_r} * {{16{1'b0}}, r_reciprocal_r};
assign g_mul_all = {{RECIP_WIDTH{1'b0}}, g_mul_ik_r} * {{16{1'b0}}, g_reciprocal_r};
assign b_mul_all = {{RECIP_WIDTH{1'b0}}, b_mul_ik_r} * {{16{1'b0}}, b_reciprocal_r};
assign r_scaled  = (r_mul_all_r + RECIP_ROUND) >> RECIP_SHIFT;
assign g_scaled  = (g_mul_all_r + RECIP_ROUND) >> RECIP_SHIFT;
assign b_scaled  = (b_mul_all_r + RECIP_ROUND) >> RECIP_SHIFT;

// 像素部分积与倒数系数同步流水，保证每个像素使用同一组参数。
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_mul_ik_r      <= 16'd0;
        g_mul_ik_r      <= 16'd0;
        b_mul_ik_r      <= 16'd0;
        r_reciprocal_r  <= {RECIP_WIDTH{1'b0}};
        g_reciprocal_r  <= {RECIP_WIDTH{1'b0}};
        b_reciprocal_r  <= {RECIP_WIDTH{1'b0}};
        r_mul_all_r     <= {SCALE_WIDTH{1'b0}};
        g_mul_all_r     <= {SCALE_WIDTH{1'b0}};
        b_mul_all_r     <= {SCALE_WIDTH{1'b0}};
    end
    else begin
        r_mul_ik_r <= r_mul_ik;
        g_mul_ik_r <= g_mul_ik;
        b_mul_ik_r <= b_mul_ik;
        r_reciprocal_r <= r_reciprocal;
        g_reciprocal_r <= g_reciprocal;
        b_reciprocal_r <= b_reciprocal;
        r_mul_all_r <= r_mul_all;
        g_mul_all_r <= g_mul_all;
        b_mul_all_r <= b_mul_all;
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_new <= 8'd0;
        g_new <= 8'd0;
        b_new <= 8'd0;
    end
    else if (pic_valid_r2) begin
        r_new <= (r_scaled > 17'd255) ? 8'd255 : r_scaled[7:0];
        g_new <= (g_scaled > 17'd255) ? 8'd255 : g_scaled[7:0];
        b_new <= (b_scaled > 17'd255) ? 8'd255 : b_scaled[7:0];
    end
end

// 输入时序延时
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        hs_r        <= 1'b0;
        vs_r        <= 1'b0;
        de_r        <= 1'b0;
        pic_valid_r <= 1'b0;
        r_data_r    <= 8'd0;
        g_data_r    <= 8'd0;
        b_data_r    <= 8'd0;
    end
    else begin
        hs_r        <= i_hs;
        vs_r        <= i_vs;
        de_r        <= i_de;
        pic_valid_r <= i_pic_valid;
        r_data_r    <= i_r;
        g_data_r    <= i_g;
        b_data_r    <= i_b;
    end
end

// 延迟视频时序和直通像素，使其与三级补偿数据通路对齐。
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        hs_r2        <= 1'b0;
        vs_r2        <= 1'b0;
        de_r2        <= 1'b0;
        pic_valid_r2 <= 1'b0;
        r_data_r2    <= 8'd0;
        g_data_r2    <= 8'd0;
        b_data_r2    <= 8'd0;
        hs_r3        <= 1'b0;
        vs_r3        <= 1'b0;
        de_r3        <= 1'b0;
        pic_valid_r3 <= 1'b0;
        r_data_r3    <= 8'd0;
        g_data_r3    <= 8'd0;
        b_data_r3    <= 8'd0;
    end
    else begin
        hs_r2        <= hs_r;
        vs_r2        <= vs_r;
        de_r2        <= de_r;
        pic_valid_r2 <= pic_valid_r;
        r_data_r2    <= r_data_r;
        g_data_r2    <= g_data_r;
        b_data_r2    <= b_data_r;
        hs_r3        <= hs_r2;
        vs_r3        <= vs_r2;
        de_r3        <= de_r2;
        pic_valid_r3 <= pic_valid_r2;
        r_data_r3    <= r_data_r2;
        g_data_r3    <= g_data_r2;
        b_data_r3    <= b_data_r2;
    end
end

// 系数有效前保持视频时序连续，并直通原始像素。
assign wb_hs        = hs_r3;
assign wb_vs        = vs_r3;
assign wb_de        = de_r3;
assign wb_pic_valid = pic_valid_r3;
assign wb_r         = (coeff_valid && pic_valid_r3) ? r_new : r_data_r3;
assign wb_g         = (coeff_valid && pic_valid_r3) ? g_new : g_data_r3;
assign wb_b         = (coeff_valid && pic_valid_r3) ? b_new : b_data_r3;

assign o_hs        = i_enable ? wb_hs        : i_hs       ;
assign o_vs        = i_enable ? wb_vs        : i_vs       ;
assign o_de        = i_enable ? wb_de        : i_de       ;
assign o_pic_valid = i_enable ? wb_pic_valid : i_pic_valid;
assign o_r         = i_enable ? wb_r         : i_r        ;
assign o_g         = i_enable ? wb_g         : i_g        ;
assign o_b         = i_enable ? wb_b         : i_b        ;

endmodule
