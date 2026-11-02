module bilateral_filter#(
    parameter DATA_WIDTH = 8
)(
    input           clk,
    input           rst_n,

    input           bilateral_frame_vsync,
    input           bilateral_frame_href,
    input           bilateral_frame_clken,
    input  [DATA_WIDTH - 1:0] bilateral_p11,
    input  [DATA_WIDTH - 1:0] bilateral_p12,
    input  [DATA_WIDTH - 1:0] bilateral_p13,
    input  [DATA_WIDTH - 1:0] bilateral_p21,
    input  [DATA_WIDTH - 1:0] bilateral_p22,
    input  [DATA_WIDTH - 1:0] bilateral_p23,
    input  [DATA_WIDTH - 1:0] bilateral_p31,
    input  [DATA_WIDTH - 1:0] bilateral_p32,
    input  [DATA_WIDTH - 1:0] bilateral_p33,

    input  [DATA_WIDTH - 1:0] bilateral_p11_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p12_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p13_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p21_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p22_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p23_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p31_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p32_cr,
    input  [DATA_WIDTH - 1:0] bilateral_p33_cr,

    input  [DATA_WIDTH - 1:0] bilateral_p11_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p12_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p13_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p21_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p22_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p23_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p31_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p32_cb,
    input  [DATA_WIDTH - 1:0] bilateral_p33_cb,

    output          pos_frame_vsync,
    output          pos_frame_href,
    output          pos_frame_clken,
    output [DATA_WIDTH - 1:0] pos_img_y,
    output [DATA_WIDTH - 1:0] pos_img_cr,
    output [DATA_WIDTH - 1:0] pos_img_cb

);

localparam WEIGHT_WIDTH = 10;    //权重数据宽度
localparam LATENCY      = 37;    //双边滤波器的处理延迟
localparam DIV_XLEN     = WEIGHT_WIDTH*2+DATA_WIDTH+4;
localparam [DIV_XLEN-1:0] DIV_STAGE_LIST = {DIV_XLEN{1'b1}};

// 行场同步信号打拍寄存器
reg [LATENCY-1:0] vsync_pipe;
reg [LATENCY-1:0] href_pipe;
reg [LATENCY-1:0] clken_pipe;

// 双边滤波数据数组
wire [DATA_WIDTH-1:0] bilateral_p [0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_reg1[0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_reg2[0:2][0:2];

assign bilateral_p[0][0] = bilateral_p11;
assign bilateral_p[0][1] = bilateral_p12;
assign bilateral_p[0][2] = bilateral_p13;
assign bilateral_p[1][0] = bilateral_p21;
assign bilateral_p[1][1] = bilateral_p22;
assign bilateral_p[1][2] = bilateral_p23;
assign bilateral_p[2][0] = bilateral_p31;
assign bilateral_p[2][1] = bilateral_p32;
assign bilateral_p[2][2] = bilateral_p33;

wire [DATA_WIDTH-1:0] bilateral_p_cr [0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_cr_reg1[0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_cr_reg2[0:2][0:2];

assign bilateral_p_cr[0][0] = bilateral_p11_cr;
assign bilateral_p_cr[0][1] = bilateral_p12_cr;
assign bilateral_p_cr[0][2] = bilateral_p13_cr;
assign bilateral_p_cr[1][0] = bilateral_p21_cr;
assign bilateral_p_cr[1][1] = bilateral_p22_cr;
assign bilateral_p_cr[1][2] = bilateral_p23_cr;
assign bilateral_p_cr[2][0] = bilateral_p31_cr;
assign bilateral_p_cr[2][1] = bilateral_p32_cr;
assign bilateral_p_cr[2][2] = bilateral_p33_cr;

wire [DATA_WIDTH-1:0] bilateral_p_cb [0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_cb_reg1[0:2][0:2];
reg [DATA_WIDTH-1:0] bilateral_p_cb_reg2[0:2][0:2];

assign bilateral_p_cb[0][0] = bilateral_p11_cb;
assign bilateral_p_cb[0][1] = bilateral_p12_cb;
assign bilateral_p_cb[0][2] = bilateral_p13_cb;
assign bilateral_p_cb[1][0] = bilateral_p21_cb;
assign bilateral_p_cb[1][1] = bilateral_p22_cb;
assign bilateral_p_cb[1][2] = bilateral_p23_cb;
assign bilateral_p_cb[2][0] = bilateral_p31_cb;
assign bilateral_p_cb[2][1] = bilateral_p32_cb;
assign bilateral_p_cb[2][2] = bilateral_p33_cb;

// 双边滤波器的权重
reg  [WEIGHT_WIDTH-1:0]   bilateral_range_weight [2:0] [2:0];  //双边滤波器的范围权重
wire [WEIGHT_WIDTH-1:0]   bilateral_space_weight [2:0] [2:0];  //双边滤波器的空间权重
reg  [WEIGHT_WIDTH*2-1:0] bilateral_mult_weight  [2:0] [2:0];  //双边滤波器的权重

reg  [WEIGHT_WIDTH*2-1+2:0] sum_weight_row [2:0]; //双边滤波器的权重行和 
reg  [WEIGHT_WIDTH*2-1+4:0] sum_weight; //双边滤波器的权重总和

// 高斯滤波器的空间权重
assign bilateral_space_weight[0][0] = 8'd77;
assign bilateral_space_weight[0][1] = 8'd127;
assign bilateral_space_weight[0][2] = 8'd77;
assign bilateral_space_weight[1][0] = 8'd127;
assign bilateral_space_weight[1][1] = 8'd209;
assign bilateral_space_weight[1][2] = 8'd127;
assign bilateral_space_weight[2][0] = 8'd77;
assign bilateral_space_weight[2][1] = 8'd127;
assign bilateral_space_weight[2][2] = 8'd77;

function automatic [WEIGHT_WIDTH*2-1:0] calc_space_weight_mul;
    input [WEIGHT_WIDTH-1:0] range_weight;
    input [WEIGHT_WIDTH-1:0] space_weight;
    reg   [WEIGHT_WIDTH*2-1:0] range_ext;
    begin
        range_ext = {{WEIGHT_WIDTH{1'b0}}, range_weight};
        case(space_weight)
            10'd77:  calc_space_weight_mul = (range_ext << 6) + (range_ext << 3) + (range_ext << 2) + range_ext;
            10'd127: calc_space_weight_mul = (range_ext << 7) - range_ext;
            10'd209: calc_space_weight_mul = (range_ext << 7) + (range_ext << 6) + (range_ext << 4) + range_ext;
            default:  calc_space_weight_mul = {WEIGHT_WIDTH*2{1'b0}};
        endcase
    end
endfunction

// 范围权重
reg [9:0] range_lut [0:63]; //范围权重查找表，输入为像素差值，输出为对应的权重值
initial begin
    range_lut[0]  = 10'd1023;
    range_lut[1]  = 10'd1021;
    range_lut[2]  = 10'd1015;
    range_lut[3]  = 10'd1006;
    range_lut[4]  = 10'd993;
    range_lut[5]  = 10'd976;
    range_lut[6]  = 10'd957;
    range_lut[7]  = 10'd934;
    range_lut[8]  = 10'd908;
    range_lut[9]  = 10'd880;
    range_lut[10] = 10'd849;
    range_lut[11] = 10'd817;
    range_lut[12] = 10'd782;
    range_lut[13] = 10'd747;
    range_lut[14] = 10'd710;
    range_lut[15] = 10'd673;
    range_lut[16] = 10'd635;
    range_lut[17] = 10'd597;
    range_lut[18] = 10'd559;
    range_lut[19] = 10'd522;
    range_lut[20] = 10'd486;
    range_lut[21] = 10'd450;
    range_lut[22] = 10'd415;
    range_lut[23] = 10'd382;
    range_lut[24] = 10'd350;
    range_lut[25] = 10'd319;
    range_lut[26] = 10'd290;
    range_lut[27] = 10'd263;
    range_lut[28] = 10'd237;
    range_lut[29] = 10'd214;
    range_lut[30] = 10'd191;
    range_lut[31] = 10'd171;
    range_lut[32] = 10'd152;
    range_lut[33] = 10'd135;
    range_lut[34] = 10'd119;
    range_lut[35] = 10'd104;
    range_lut[36] = 10'd91;
    range_lut[37] = 10'd80;
    range_lut[38] = 10'd69;
    range_lut[39] = 10'd60;
    range_lut[40] = 10'd52;
    range_lut[41] = 10'd45;
    range_lut[42] = 10'd38;
    range_lut[43] = 10'd30;
    range_lut[44] = 10'd24;
    range_lut[45] = 10'd20;
    range_lut[46] = 10'd20;
    range_lut[47] = 10'd17;
    range_lut[48] = 10'd14;
    range_lut[49] = 10'd12;
    range_lut[50] = 10'd10;
    range_lut[51] = 10'd8;
    range_lut[52] = 10'd7;
    range_lut[53] = 10'd5;
    range_lut[54] = 10'd4;
    range_lut[55] = 10'd4;
    range_lut[56] = 10'd3;
    range_lut[57] = 10'd2;
    range_lut[58] = 10'd2;
    range_lut[59] = 10'd2;
    range_lut[60] = 10'd1;
    range_lut[61] = 10'd1;
    range_lut[62] = 10'd1;
    range_lut[63] = 10'd1;
end


reg  [DATA_WIDTH-1:0]   vip_martrix_gray_diff  [2:0] [2:0];      //输入图像的M*N矩阵与中心像素的灰度差值
reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1:0] sum_weight_pixel [2:0] [2:0]; // 加权像素值
reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1+4:0] sum_weight_data; // 加权像素值总和，9项累加预留4位

reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1:0] sum_weight_cr_pixel [2:0] [2:0]; // 加权像素值
reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1:0] sum_weight_cb_pixel [2:0] [2:0]; // 加权像素值

reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1+4:0] sum_weight_cr_data; // 加权像素值总和，9项累加预留4位
reg  [WEIGHT_WIDTH*2+DATA_WIDTH-1+4:0] sum_weight_cb_data; // 加权像素值总和，9项累加预留4位

genvar i, j;
generate
    for(i = 0; i < 3; i = i + 1) begin
        for(j = 0; j < 3; j = j + 1) begin
            // step1: 计算输入图像的M*N矩阵与中心像素的灰度差值
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    vip_martrix_gray_diff[i][j] <= {DATA_WIDTH{1'b0}};
                end else begin
                    vip_martrix_gray_diff[i][j] <= (bilateral_p[i][j] > bilateral_p[1][1]) ? (bilateral_p[i][j] - bilateral_p[1][1]) : (bilateral_p[1][1] - bilateral_p[i][j]);
                end
            end

            // step1: 计算双边滤波器的范围权重（组合逻辑）
            always @( *) begin
                if(vip_martrix_gray_diff[i][j]<64) begin
                    bilateral_range_weight[i][j] = range_lut[vip_martrix_gray_diff[i][j][5:0]];
                end else begin
                    bilateral_range_weight[i][j] = 10'd0;
                end
            end

            // step2: 计算双边滤波的总权重
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    bilateral_mult_weight[i][j] <= {(WEIGHT_WIDTH*2){1'b0}};
                end else begin
                    bilateral_mult_weight[i][j] <= calc_space_weight_mul(bilateral_range_weight[i][j], bilateral_space_weight[i][j]);
                end
            end

            // step1-2: 输入数据打两拍
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    bilateral_p_reg1[i][j] <= {DATA_WIDTH{1'b0}};
                    bilateral_p_reg2[i][j] <= {DATA_WIDTH{1'b0}};

                    bilateral_p_cr_reg1[i][j] <= {DATA_WIDTH{1'b0}};
                    bilateral_p_cr_reg2[i][j] <= {DATA_WIDTH{1'b0}};

                    bilateral_p_cb_reg1[i][j] <= {DATA_WIDTH{1'b0}};
                    bilateral_p_cb_reg2[i][j] <= {DATA_WIDTH{1'b0}};
                end else begin
                    bilateral_p_reg1[i][j] <= bilateral_p[i][j];
                    bilateral_p_reg2[i][j] <= bilateral_p_reg1[i][j];

                    bilateral_p_cr_reg1[i][j] <= bilateral_p_cr[i][j];
                    bilateral_p_cr_reg2[i][j] <= bilateral_p_cr_reg1[i][j];

                    bilateral_p_cb_reg1[i][j] <= bilateral_p_cb[i][j];
                    bilateral_p_cb_reg2[i][j] <= bilateral_p_cb_reg1[i][j];
                end
            end


            // step3: 计算双边滤波器的像素加权值(未除以总权重)，结果位宽为WEIGHT_WIDTH*2+DATA_WIDTH
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    sum_weight_pixel[i][j] <= {WEIGHT_WIDTH*2+DATA_WIDTH{1'b0}};
                    sum_weight_cr_pixel[i][j] <= {WEIGHT_WIDTH*2+DATA_WIDTH{1'b0}};
                    sum_weight_cb_pixel[i][j] <= {WEIGHT_WIDTH*2+DATA_WIDTH{1'b0}};
                end else begin
                    sum_weight_pixel[i][j] <= bilateral_p_reg2[i][j] * bilateral_mult_weight[i][j];
                    sum_weight_cr_pixel[i][j] <= bilateral_p_cr_reg2[i][j] * bilateral_mult_weight[i][j];
                    sum_weight_cb_pixel[i][j] <= bilateral_p_cb_reg2[i][j] * bilateral_mult_weight[i][j];
                end
            end
        end
    end
endgenerate

// step3-4：计算双边滤波器的权重总和，结果位宽为WEIGHT_WIDTH*2+4
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sum_weight <= {(WEIGHT_WIDTH*2+4){1'b0}};
    end
    else begin
        sum_weight_row[0] <= {2'd0, bilateral_mult_weight[0][0]} + {2'd0, bilateral_mult_weight[0][1]} + {2'd0, bilateral_mult_weight[0][2]};
        sum_weight_row[1] <= {2'd0, bilateral_mult_weight[1][0]} + {2'd0, bilateral_mult_weight[1][1]} + {2'd0, bilateral_mult_weight[1][2]};
        sum_weight_row[2] <= {2'd0, bilateral_mult_weight[2][0]} + {2'd0, bilateral_mult_weight[2][1]} + {2'd0, bilateral_mult_weight[2][2]};

        sum_weight  <= {2'd0, sum_weight_row[0]} + {2'd0, sum_weight_row[1]} + {2'd0, sum_weight_row[2]};

    end
end

// step4: 计算双边滤波器的像素加权值总和，结果位宽为WEIGHT_WIDTH*2+DATA_WIDTH+4
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sum_weight_data <= {(WEIGHT_WIDTH*2+DATA_WIDTH+4){1'b0}};
        sum_weight_cr_data <= {(WEIGHT_WIDTH*2+DATA_WIDTH+4){1'b0}};
        sum_weight_cb_data <= {(WEIGHT_WIDTH*2+DATA_WIDTH+4){1'b0}};
    end
    else begin
        sum_weight_data <= {4'd0, sum_weight_pixel[0][0]} + {4'd0, sum_weight_pixel[0][1]} + {4'd0, sum_weight_pixel[0][2]} +
                           {4'd0, sum_weight_pixel[1][0]} + {4'd0, sum_weight_pixel[1][1]} + {4'd0, sum_weight_pixel[1][2]} +
                           {4'd0, sum_weight_pixel[2][0]} + {4'd0, sum_weight_pixel[2][1]} + {4'd0, sum_weight_pixel[2][2]};
        sum_weight_cr_data <= {4'd0, sum_weight_cr_pixel[0][0]} + {4'd0, sum_weight_cr_pixel[0][1]} + {4'd0, sum_weight_cr_pixel[0][2]} +
                              {4'd0, sum_weight_cr_pixel[1][0]} + {4'd0, sum_weight_cr_pixel[1][1]} + {4'd0, sum_weight_cr_pixel[1][2]} +
                              {4'd0, sum_weight_cr_pixel[2][0]} + {4'd0, sum_weight_cr_pixel[2][1]} + {4'd0, sum_weight_cr_pixel[2][2]};
        sum_weight_cb_data <= {4'd0, sum_weight_cb_pixel[0][0]} + {4'd0, sum_weight_cb_pixel[0][1]} + {4'd0, sum_weight_cb_pixel[0][2]} +
                              {4'd0, sum_weight_cb_pixel[1][0]} + {4'd0, sum_weight_cb_pixel[1][1]} + {4'd0, sum_weight_cb_pixel[1][2]} +
                              {4'd0, sum_weight_cb_pixel[2][0]} + {4'd0, sum_weight_cb_pixel[2][1]} + {4'd0, sum_weight_cb_pixel[2][2]};
    end
end

wire [WEIGHT_WIDTH*2+DATA_WIDTH+4-1:0] bilateral_filter_result; //双边滤波器的输出结果
wire [WEIGHT_WIDTH*2+DATA_WIDTH+4-1:0] sum_weight_safe;
wire [DATA_WIDTH-1:0] bilateral_filter_sat;

wire [WEIGHT_WIDTH*2+DATA_WIDTH+4-1:0] bilateral_filter_result_cr; //双边滤波器的输出结果
wire [WEIGHT_WIDTH*2+DATA_WIDTH+4-1:0] bilateral_filter_result_cb; //双边滤波器的输出结果
wire [DATA_WIDTH-1:0] bilateral_filter_sat_cr;
wire [DATA_WIDTH-1:0] bilateral_filter_sat_cb;


assign sum_weight_safe = (sum_weight == {(WEIGHT_WIDTH*2-1+4+1){1'b0}}) ? {{(WEIGHT_WIDTH*2+DATA_WIDTH+4-1){1'b0}},1'b1} : sum_weight;
assign bilateral_filter_sat = (bilateral_filter_result > {DATA_WIDTH{1'b1}}) ? {DATA_WIDTH{1'b1}} : bilateral_filter_result[DATA_WIDTH-1 : 0];
assign bilateral_filter_sat_cr = (bilateral_filter_result_cr > {DATA_WIDTH{1'b1}}) ? {DATA_WIDTH{1'b1}} : bilateral_filter_result_cr[DATA_WIDTH-1 : 0];
assign bilateral_filter_sat_cb = (bilateral_filter_result_cb > {DATA_WIDTH{1'b1}}) ? {DATA_WIDTH{1'b1}} : bilateral_filter_result_cb[DATA_WIDTH-1 : 0];

// step5-37: 权重归一化（与控制路径对齐的全流水除法器）
divider #(
    .XLEN(DIV_XLEN), //被除数位宽
    .STAGE_LIST(DIV_STAGE_LIST)
)u_divider(
    .clk  (clk),
    .rst  (~rst_n),

    .vld  (clken_pipe[3]), //在第5拍开始计算归一化，与sum_weight_data/sum_weight对齐
    .a    (sum_weight_data),
    .b    (sum_weight_safe),
    .quo  (bilateral_filter_result),
    .rem  (),
    .ack  ()
);
// step5-37: 权重归一化（与控制路径对齐的全流水除法器）
divider #(
    .XLEN(DIV_XLEN), //被除数位宽
    .STAGE_LIST(DIV_STAGE_LIST)
)u_divider_cr(
    .clk  (clk),
    .rst  (~rst_n),

    .vld  (clken_pipe[3]), //在第5拍开始计算归一化，与sum_weight_data/sum_weight对齐
    .a    (sum_weight_cr_data),
    .b    (sum_weight_safe),
    .quo  (bilateral_filter_result_cr),
    .rem  (),
    .ack  ()
);
// step5-37: 权重归一化（与控制路径对齐的全流水除法器）
divider #(
    .XLEN(DIV_XLEN), //被除数位宽
    .STAGE_LIST(DIV_STAGE_LIST)
)u_divider_cb(
    .clk  (clk),
    .rst  (~rst_n),

    .vld  (clken_pipe[3]), //在第5拍开始计算归一化，与sum_weight_data/sum_weight对齐
    .a    (sum_weight_cb_data),
    .b    (sum_weight_safe),
    .quo  (bilateral_filter_result_cb),
    .rem  (),
    .ack  ()
);

// step1-37: 行场同步信号打拍
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vsync_pipe <= {LATENCY{1'b0}};
        href_pipe  <= {LATENCY{1'b0}};
        clken_pipe <= {LATENCY{1'b0}};
    end else begin
        vsync_pipe <= {vsync_pipe[LATENCY-2:0], bilateral_frame_vsync};
        href_pipe  <= {href_pipe[LATENCY-2:0], bilateral_frame_href};
        clken_pipe <= {clken_pipe[LATENCY-2:0], bilateral_frame_clken};
    end
end

assign pos_frame_vsync = vsync_pipe[LATENCY-1];
assign pos_frame_href = href_pipe[LATENCY-1];
assign pos_frame_clken = clken_pipe[LATENCY-1];

assign pos_img_y = bilateral_filter_sat;
assign pos_img_cr = bilateral_filter_sat_cr;
assign pos_img_cb = bilateral_filter_sat_cb;

endmodule