// 自动白平衡算法
// Auto white balance
module image_awb #(
    parameter IMG_HDISP   = 1280,
    parameter IMG_VDISP   = 720,
    parameter RECIP_SHIFT = 15
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

// wire or reg define
reg  [SUM_WIDTH - 1:0] sum_r       ; // R通道像素值总和
reg  [SUM_WIDTH - 1:0] sum_g       ; // G通道像素值总和
reg  [SUM_WIDTH - 1:0] sum_b       ; // B通道像素值总和
reg  [PIX_CNT_WIDTH - 1:0] pixel_cnt;
reg         frame_sum_done;
reg  [7:0]  r_aver      ; // R通道像素平均值
reg  [7:0]  g_aver      ; // G通道像素平均值
reg  [7:0]  b_aver      ; // B通道像素平均值
reg  [10:0] r_division  ; // 1024/r_aver
reg  [10:0] g_division  ; // 1024/g_aver
reg  [10:0] b_division  ; // 1024/b_aver
reg  [31:0] r_division_calc_s1;
reg  [31:0] g_division_calc_s1;
reg  [31:0] b_division_calc_s1;
reg         r_aver_zero_s1;
reg         g_aver_zero_s1;
reg         b_aver_zero_s1;
reg         division_calc_s1_valid;
reg  [19:0]  k_aver      ; // 低8位有效

reg  [7:0]  r_new       ;
reg  [7:0]  g_new       ;
reg  [7:0]  b_new       ;
wire [17:0] r_mul_ik    ;
wire [17:0] g_mul_ik    ;
wire [17:0] b_mul_ik    ;
wire [28:0] r_mul_all   ;
wire [28:0] g_mul_all   ;
wire [28:0] b_mul_all   ;
wire [18:0] r_scaled    ;
wire [18:0] g_scaled    ;
wire [18:0] b_scaled    ;

reg         hs_r        ;
reg         vs_r        ;
reg         de_r        ;
reg         pic_valid_r ;
reg         coeff_valid ;
reg  [7:0]  r_data_r    ;
reg  [7:0]  g_data_r    ;
reg  [7:0]  b_data_r    ;
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
    else begin
        sum_r          <= sum_r;
        sum_g          <= sum_g;
        sum_b          <= sum_b;
        pixel_cnt      <= pixel_cnt;
        frame_sum_done <= 1'b0;
    end
end

wire sum_flag; // 求和结束标志信号
assign sum_flag = frame_sum_done;

wire [SUM_WIDTH - 1:0] r_aver_quo;
wire [SUM_WIDTH - 1:0] g_aver_quo;
wire [SUM_WIDTH - 1:0] b_aver_quo;
wire [SUM_WIDTH - 1:0] r_aver_rem;
wire [SUM_WIDTH - 1:0] g_aver_rem;
wire [SUM_WIDTH - 1:0] b_aver_rem;
wire                   r_aver_ack;
wire                   g_aver_ack;
wire                   b_aver_ack;
wire                   aver_ack;

assign aver_ack = r_aver_ack & g_aver_ack & b_aver_ack;

// 对R、G、B三个通道求和后的数据求平均值
divider #(
    .XLEN       (SUM_WIDTH),
    .STAGE_LIST (DIV_STAGE_LIST)
) divider_aver_r (
    .clk (i_clk)            ,
    .rst (~i_rst_n)         ,
    .a   (sum_r)            ,
    .b   (IMG_PIXELS),
    .vld (sum_flag)         ,
    .quo (r_aver_quo)       ,
    .rem (r_aver_rem)       ,
    .ack (r_aver_ack)
);

divider #(
    .XLEN       (SUM_WIDTH),
    .STAGE_LIST (DIV_STAGE_LIST)
) divider_aver_g (
    .clk (i_clk)            ,
    .rst (~i_rst_n)         ,
    .a   (sum_g)            ,
    .b   (IMG_PIXELS),
    .vld (sum_flag)         ,
    .quo (g_aver_quo)       ,
    .rem (g_aver_rem)       ,
    .ack (g_aver_ack)
);

divider #(
    .XLEN       (SUM_WIDTH),
    .STAGE_LIST (DIV_STAGE_LIST)
) divider_aver_b (
    .clk (i_clk)            ,
    .rst (~i_rst_n)         ,
    .a   (sum_b)            ,
    .b   (IMG_PIXELS),
    .vld (sum_flag)         ,
    .quo (b_aver_quo)       ,
    .rem (b_aver_rem)       ,
    .ack (b_aver_ack)
);

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_aver <= 8'd0;
        g_aver <= 8'd0;
        b_aver <= 8'd0;
    end
    else if (aver_ack) begin
        r_aver <= r_aver_quo[7:0];
        g_aver <= g_aver_quo[7:0];
        b_aver <= b_aver_quo[7:0];
    end
    else begin
        r_aver <= r_aver;
        g_aver <= g_aver;
        b_aver <= b_aver;
    end
end

// 平均值的同步信号，对除法器ack打一拍
reg aver_flag;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        aver_flag <= 1'b0;
    else
        aver_flag <= aver_ack;
end

// 倒数查表的小数位数(Q0.RECIP_SHIFT)，用于近似完成1024/aver
reg [15:0] recip_lut [0:255];
integer lut_i;

initial begin
    recip_lut[0] = 16'd32767;
    for (lut_i = 1; lut_i < 256; lut_i = lut_i + 1) begin
        recip_lut[lut_i] = ((1 << RECIP_SHIFT) + (lut_i >> 1)) / lut_i;
    end
end

// 对得到的三个RGB像素通道灰度平均值做除法，1024/各通道的灰度平均值
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_division            <= 11'd0;
        g_division            <= 11'd0;
        b_division            <= 11'd0;
        r_division_calc_s1    <= 32'd0;
        g_division_calc_s1    <= 32'd0;
        b_division_calc_s1    <= 32'd0;
        r_aver_zero_s1        <= 1'b1;
        g_aver_zero_s1        <= 1'b1;
        b_aver_zero_s1        <= 1'b1;
        division_calc_s1_valid <= 1'b0;
    end
    else begin
        division_calc_s1_valid <= aver_flag;

        if (aver_flag) begin
            r_division_calc_s1 <= 32'd1024 * recip_lut[r_aver] + (32'd1 << (RECIP_SHIFT - 1));
            g_division_calc_s1 <= 32'd1024 * recip_lut[g_aver] + (32'd1 << (RECIP_SHIFT - 1));
            b_division_calc_s1 <= 32'd1024 * recip_lut[b_aver] + (32'd1 << (RECIP_SHIFT - 1));
            r_aver_zero_s1     <= (r_aver == 8'd0);
            g_aver_zero_s1     <= (g_aver == 8'd0);
            b_aver_zero_s1     <= (b_aver == 8'd0);
        end

        if (division_calc_s1_valid) begin
            r_division <= r_aver_zero_s1 ? 11'd0 : (r_division_calc_s1 >> RECIP_SHIFT);
            g_division <= g_aver_zero_s1 ? 11'd0 : (g_division_calc_s1 >> RECIP_SHIFT);
            b_division <= b_aver_zero_s1 ? 11'd0 : (b_division_calc_s1 >> RECIP_SHIFT);
        end
    end
end

// 三个通道的灰度平均值k_aver
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        k_aver <= 12'd0;
    else if (aver_flag)
        k_aver <= (({2'd0,r_aver} + {2'd0,g_aver} + {2'd0,b_aver}) * 10'd341) >> 4'd10;
    else
        k_aver <= k_aver;
end

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n)
        coeff_valid <= 1'b0;
    else if (division_calc_s1_valid)
        coeff_valid <= 1'b1;
    else
        coeff_valid <= coeff_valid;
end

// 输出更新的数据
assign r_mul_ik  = {10'd0, i_r} * {8'd0, k_aver};
assign g_mul_ik  = {10'd0, i_g} * {8'd0, k_aver};
assign b_mul_ik  = {10'd0, i_b} * {8'd0, k_aver};
assign r_mul_all = {11'd0, r_mul_ik} * {18'd0, r_division};
assign g_mul_all = {11'd0, g_mul_ik} * {18'd0, g_division};
assign b_mul_all = {11'd0, b_mul_ik} * {18'd0, b_division};
assign r_scaled  = r_mul_all[28:10];
assign g_scaled  = g_mul_all[28:10];
assign b_scaled  = b_mul_all[28:10];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_new <= 8'd0;
        g_new <= 8'd0;
        b_new <= 8'd0;
    end
    else if (i_pic_valid) begin
        r_new <= (r_scaled >= 19'd255) ? 8'd255 : r_scaled[7:0];
        g_new <= (g_scaled >= 19'd255) ? 8'd255 : g_scaled[7:0];
        b_new <= (b_scaled >= 19'd255) ? 8'd255 : b_scaled[7:0];
    end
    else begin
        r_new <= r_new;
        g_new <= g_new;
        b_new <= b_new;
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

// 定义第二张图片之后才开始输出
// 输出时序赋值
// Keep timing continuous; pass pixels through until valid AWB coefficients exist.
assign wb_hs        = hs_r;
assign wb_vs        = vs_r;
assign wb_de        = de_r;
assign wb_pic_valid = pic_valid_r;
assign wb_r         = (coeff_valid && pic_valid_r) ? r_new : r_data_r;
assign wb_g         = (coeff_valid && pic_valid_r) ? g_new : g_data_r;
assign wb_b         = (coeff_valid && pic_valid_r) ? b_new : b_data_r;

assign o_hs        = i_enable ? wb_hs        : i_hs       ;
assign o_vs        = i_enable ? wb_vs        : i_vs       ;
assign o_de        = i_enable ? wb_de        : i_de       ;
assign o_pic_valid = i_enable ? wb_pic_valid : i_pic_valid;
assign o_r         = i_enable ? wb_r         : i_r        ;
assign o_g         = i_enable ? wb_g         : i_g        ;
assign o_b         = i_enable ? wb_b         : i_b        ;

endmodule
