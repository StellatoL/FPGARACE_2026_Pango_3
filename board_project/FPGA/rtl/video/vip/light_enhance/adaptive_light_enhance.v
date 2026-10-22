module adaptive_light_enhancement #(
    parameter DATA_WIDTH = 8
)(
    input  wire        clk,
    input  wire        rst_n,

    // 输入视频流同步信号
    input  wire        per_frame_vsync,
    input  wire        per_frame_href,
    input  wire        per_frame_clken,
    input  wire [DATA_WIDTH-1:0]  img_y_in,
    input  wire [DATA_WIDTH-1:0]  img_cb_in,
    input  wire [DATA_WIDTH-1:0]  img_cr_in,

    // 均值滤波得到的局部亮度 (0~255)
    input  wire [DATA_WIDTH-1:0]  local_y_mean,

    // 输出视频流同步信号
    output wire        post_frame_vsync,
    output wire        post_frame_href,
    output wire        post_frame_clken,
    output wire [DATA_WIDTH-1:0]  img_y_out,
    output wire [DATA_WIDTH-1:0]  img_cb_out,
    output wire [DATA_WIDTH-1:0]  img_cr_out
);

//---------------------------------------------------
// 寄存器定义 (流水线级数：3级)
//---------------------------------------------------

// Stage 1: 查表与数据缓存
reg  [DATA_WIDTH-1:0]  gain_val;         // 从 ROM 取出的增益系数
(* rom_style = "block" *) reg [DATA_WIDTH-1:0] gain_lut [0:255];
reg  [DATA_WIDTH-1:0]  y_d1, cb_d1, cr_d1;
reg  [2:0]  sync_vsync_r;     // 同步信号移位寄存器 [2]:stage3, [1]:stage2, [0]:stage1
reg  [2:0]  sync_href_r;
reg  [2:0]  sync_clken_r;

// Stage 2: 乘法器阶段
reg  [15:0] y_mult;           // 乘法结果: 8bit * 8bit = 16bit
reg  [DATA_WIDTH-1:0]  cb_d2, cr_d2;

// Stage 3: 钳位与输出输出
reg  [DATA_WIDTH-1:0]  y_out_reg;
reg  [DATA_WIDTH-1:0]  cb_out_reg, cr_out_reg;

// 连线输出
assign post_frame_vsync = sync_vsync_r[2];
assign post_frame_href  = sync_href_r[2];
assign post_frame_clken = sync_clken_r[2];
assign img_y_out  = y_out_reg;
assign img_cb_out = cb_out_reg;
assign img_cr_out = cr_out_reg;

//======================================================================
//  增益 LUT：纯硬编码初始化，直接写在 RTL 里
//======================================================================
initial begin
    gain_lut[  0] = 8'd230;  gain_lut[  1] = 8'd229;  gain_lut[  2] = 8'd228;  gain_lut[  3] = 8'd228;  gain_lut[  4] = 8'd227;  gain_lut[  5] = 8'd226;  gain_lut[  6] = 8'd226;  gain_lut[  7] = 8'd225;
    gain_lut[  8] = 8'd224;  gain_lut[  9] = 8'd223;  gain_lut[ 10] = 8'd223;  gain_lut[ 11] = 8'd222;  gain_lut[ 12] = 8'd221;  gain_lut[ 13] = 8'd220;  gain_lut[ 14] = 8'd220;  gain_lut[ 15] = 8'd219;
    gain_lut[ 16] = 8'd218;  gain_lut[ 17] = 8'd217;  gain_lut[ 18] = 8'd217;  gain_lut[ 19] = 8'd216;  gain_lut[ 20] = 8'd215;  gain_lut[ 21] = 8'd215;  gain_lut[ 22] = 8'd214;  gain_lut[ 23] = 8'd213;
    gain_lut[ 24] = 8'd212;  gain_lut[ 25] = 8'd212;  gain_lut[ 26] = 8'd211;  gain_lut[ 27] = 8'd210;  gain_lut[ 28] = 8'd210;  gain_lut[ 29] = 8'd209;  gain_lut[ 30] = 8'd208;  gain_lut[ 31] = 8'd207;
    gain_lut[ 32] = 8'd207;  gain_lut[ 33] = 8'd206;  gain_lut[ 34] = 8'd205;  gain_lut[ 35] = 8'd205;  gain_lut[ 36] = 8'd204;  gain_lut[ 37] = 8'd203;  gain_lut[ 38] = 8'd202;  gain_lut[ 39] = 8'd202;
    gain_lut[ 40] = 8'd201;  gain_lut[ 41] = 8'd200;  gain_lut[ 42] = 8'd200;  gain_lut[ 43] = 8'd199;  gain_lut[ 44] = 8'd198;  gain_lut[ 45] = 8'd198;  gain_lut[ 46] = 8'd197;  gain_lut[ 47] = 8'd196;
    gain_lut[ 48] = 8'd196;  gain_lut[ 49] = 8'd195;  gain_lut[ 50] = 8'd194;  gain_lut[ 51] = 8'd194;  gain_lut[ 52] = 8'd193;  gain_lut[ 53] = 8'd192;  gain_lut[ 54] = 8'd192;  gain_lut[ 55] = 8'd191;
    gain_lut[ 56] = 8'd190;  gain_lut[ 57] = 8'd190;  gain_lut[ 58] = 8'd189;  gain_lut[ 59] = 8'd188;  gain_lut[ 60] = 8'd188;  gain_lut[ 61] = 8'd187;  gain_lut[ 62] = 8'd186;  gain_lut[ 63] = 8'd186;
    gain_lut[ 64] = 8'd185;  gain_lut[ 65] = 8'd184;  gain_lut[ 66] = 8'd184;  gain_lut[ 67] = 8'd183;  gain_lut[ 68] = 8'd182;  gain_lut[ 69] = 8'd182;  gain_lut[ 70] = 8'd181;  gain_lut[ 71] = 8'd180;
    gain_lut[ 72] = 8'd180;  gain_lut[ 73] = 8'd179;  gain_lut[ 74] = 8'd179;  gain_lut[ 75] = 8'd178;  gain_lut[ 76] = 8'd177;  gain_lut[ 77] = 8'd177;  gain_lut[ 78] = 8'd176;  gain_lut[ 79] = 8'd175;
    gain_lut[ 80] = 8'd175;  gain_lut[ 81] = 8'd174;  gain_lut[ 82] = 8'd174;  gain_lut[ 83] = 8'd173;  gain_lut[ 84] = 8'd172;  gain_lut[ 85] = 8'd172;  gain_lut[ 86] = 8'd171;  gain_lut[ 87] = 8'd170;
    gain_lut[ 88] = 8'd170;  gain_lut[ 89] = 8'd169;  gain_lut[ 90] = 8'd169;  gain_lut[ 91] = 8'd168;  gain_lut[ 92] = 8'd167;  gain_lut[ 93] = 8'd167;  gain_lut[ 94] = 8'd166;  gain_lut[ 95] = 8'd166;
    gain_lut[ 96] = 8'd165;  gain_lut[ 97] = 8'd164;  gain_lut[ 98] = 8'd164;  gain_lut[ 99] = 8'd163;  gain_lut[100] = 8'd163;  gain_lut[101] = 8'd162;  gain_lut[102] = 8'd161;  gain_lut[103] = 8'd161;
    gain_lut[104] = 8'd160;  gain_lut[105] = 8'd160;  gain_lut[106] = 8'd159;  gain_lut[107] = 8'd159;  gain_lut[108] = 8'd158;  gain_lut[109] = 8'd157;  gain_lut[110] = 8'd157;  gain_lut[111] = 8'd156;
    gain_lut[112] = 8'd156;  gain_lut[113] = 8'd155;  gain_lut[114] = 8'd155;  gain_lut[115] = 8'd154;  gain_lut[116] = 8'd154;  gain_lut[117] = 8'd153;  gain_lut[118] = 8'd152;  gain_lut[119] = 8'd152;
    gain_lut[120] = 8'd151;  gain_lut[121] = 8'd151;  gain_lut[122] = 8'd150;  gain_lut[123] = 8'd150;  gain_lut[124] = 8'd149;  gain_lut[125] = 8'd149;  gain_lut[126] = 8'd148;  gain_lut[127] = 8'd148;
    gain_lut[128] = 8'd147;  gain_lut[129] = 8'd146;  gain_lut[130] = 8'd146;  gain_lut[131] = 8'd145;  gain_lut[132] = 8'd145;  gain_lut[133] = 8'd144;  gain_lut[134] = 8'd144;  gain_lut[135] = 8'd143;
    gain_lut[136] = 8'd143;  gain_lut[137] = 8'd142;  gain_lut[138] = 8'd142;  gain_lut[139] = 8'd141;  gain_lut[140] = 8'd141;  gain_lut[141] = 8'd140;  gain_lut[142] = 8'd140;  gain_lut[143] = 8'd139;
    gain_lut[144] = 8'd139;  gain_lut[145] = 8'd138;  gain_lut[146] = 8'd138;  gain_lut[147] = 8'd137;  gain_lut[148] = 8'd137;  gain_lut[149] = 8'd136;  gain_lut[150] = 8'd136;  gain_lut[151] = 8'd135;
    gain_lut[152] = 8'd135;  gain_lut[153] = 8'd134;  gain_lut[154] = 8'd134;  gain_lut[155] = 8'd133;  gain_lut[156] = 8'd133;  gain_lut[157] = 8'd132;  gain_lut[158] = 8'd132;  gain_lut[159] = 8'd132;
    gain_lut[160] = 8'd131;  gain_lut[161] = 8'd131;  gain_lut[162] = 8'd130;  gain_lut[163] = 8'd130;  gain_lut[164] = 8'd129;  gain_lut[165] = 8'd129;  gain_lut[166] = 8'd128;  gain_lut[167] = 8'd128;
    gain_lut[168] = 8'd128;  gain_lut[169] = 8'd127;  gain_lut[170] = 8'd127;  gain_lut[171] = 8'd126;  gain_lut[172] = 8'd126;  gain_lut[173] = 8'd125;  gain_lut[174] = 8'd125;  gain_lut[175] = 8'd124;
    gain_lut[176] = 8'd124;  gain_lut[177] = 8'd124;  gain_lut[178] = 8'd123;  gain_lut[179] = 8'd123;  gain_lut[180] = 8'd122;  gain_lut[181] = 8'd122;  gain_lut[182] = 8'd122;  gain_lut[183] = 8'd121;
    gain_lut[184] = 8'd121;  gain_lut[185] = 8'd120;  gain_lut[186] = 8'd120;  gain_lut[187] = 8'd120;  gain_lut[188] = 8'd119;  gain_lut[189] = 8'd119;  gain_lut[190] = 8'd118;  gain_lut[191] = 8'd118;
    gain_lut[192] = 8'd118;  gain_lut[193] = 8'd117;  gain_lut[194] = 8'd117;  gain_lut[195] = 8'd117;  gain_lut[196] = 8'd116;  gain_lut[197] = 8'd116;  gain_lut[198] = 8'd116;  gain_lut[199] = 8'd115;
    gain_lut[200] = 8'd115;  gain_lut[201] = 8'd114;  gain_lut[202] = 8'd114;  gain_lut[203] = 8'd114;  gain_lut[204] = 8'd113;  gain_lut[205] = 8'd113;  gain_lut[206] = 8'd113;  gain_lut[207] = 8'd112;
    gain_lut[208] = 8'd112;  gain_lut[209] = 8'd112;  gain_lut[210] = 8'd111;  gain_lut[211] = 8'd111;  gain_lut[212] = 8'd111;  gain_lut[213] = 8'd111;  gain_lut[214] = 8'd110;  gain_lut[215] = 8'd110;
    gain_lut[216] = 8'd110;  gain_lut[217] = 8'd109;  gain_lut[218] = 8'd109;  gain_lut[219] = 8'd109;  gain_lut[220] = 8'd109;  gain_lut[221] = 8'd108;  gain_lut[222] = 8'd108;  gain_lut[223] = 8'd108;
    gain_lut[224] = 8'd107;  gain_lut[225] = 8'd107;  gain_lut[226] = 8'd107;  gain_lut[227] = 8'd107;  gain_lut[228] = 8'd106;  gain_lut[229] = 8'd106;  gain_lut[230] = 8'd106;  gain_lut[231] = 8'd106;
    gain_lut[232] = 8'd105;  gain_lut[233] = 8'd105;  gain_lut[234] = 8'd105;  gain_lut[235] = 8'd105;  gain_lut[236] = 8'd105;  gain_lut[237] = 8'd104;  gain_lut[238] = 8'd104;  gain_lut[239] = 8'd104;
    gain_lut[240] = 8'd104;  gain_lut[241] = 8'd104;  gain_lut[242] = 8'd103;  gain_lut[243] = 8'd103;  gain_lut[244] = 8'd103;  gain_lut[245] = 8'd103;  gain_lut[246] = 8'd103;  gain_lut[247] = 8'd103;
    gain_lut[248] = 8'd103;  gain_lut[249] = 8'd102;  gain_lut[250] = 8'd102;  gain_lut[251] = 8'd102;  gain_lut[252] = 8'd102;  gain_lut[253] = 8'd102;  gain_lut[254] = 8'd102;  gain_lut[255] = 8'd102;
end

//---------------------------------------------------
// 核心流水线逻辑
//---------------------------------------------------

// 【阶段 1】：利用 local_y_mean 查表获取增益，并缓存原始数据
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gain_val <= 8'd0;
        y_d1 <= 8'd0;
        cb_d1 <= 8'd0;
        cr_d1 <= 8'd0;
        sync_vsync_r[0] <= 1'b0;
        sync_href_r[0] <= 1'b0;
        sync_clken_r[0] <= 1'b0;
    end else begin
        // 地址为 local_y_mean，输出为增益系数
        gain_val <= gain_lut[local_y_mean];
        
        // 缓存中心像素数据和同步信号，等待查表结果
        y_d1  <= img_y_in;
        cb_d1 <= img_cb_in;
        cr_d1 <= img_cr_in;
        sync_vsync_r[0] <= per_frame_vsync;
        sync_href_r[0]  <= per_frame_href;
        sync_clken_r[0] <= per_frame_clken;
    end
end

// 【阶段 2】：中心像素亮度乘以增益
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y_mult <= 16'd0;
        cb_d2 <= 8'd0;
        cr_d2 <= 8'd0;
        sync_vsync_r[1] <= 1'b0;
        sync_href_r[1] <= 1'b0;
        sync_clken_r[1] <= 1'b0;
    end else begin
        // Y 亮度进行增益运算
        y_mult <= y_d1 * gain_val; 
        
        // 色度 Cb/Cr 不做处理，单纯打拍延迟
        cb_d2 <= cb_d1;
        cr_d2 <= cr_d1;
        
        // 同步信号打拍
        sync_vsync_r[1] <= sync_vsync_r[0];
        sync_href_r[1]  <= sync_href_r[0];
        sync_clken_r[1] <= sync_clken_r[0];
    end
end

// 【阶段 3】：定点还原与饱和钳位 (防过曝核心)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y_out_reg <= 8'd0;
        cb_out_reg <= 8'd0;
        cr_out_reg <= 8'd0;
        sync_vsync_r[2] <= 1'b0;
        sync_href_r[2] <= 1'b0;
        sync_clken_r[2] <= 1'b0;
    end else begin
        // 增益是放大了 128 倍的 (Q7格式)，所以乘完之后要右移 7 位 (相当于除以 128)
        // 使用一个中间变量判断是否越界
        if ((y_mult >> 7) > 16'd255) begin
            y_out_reg <= 8'd255;  // 过曝钳位
        end else begin
            y_out_reg <= y_mult[14:7]; // 正常赋值
        end

        // 色度与同步信号打最后一次拍，完成对齐
        cb_out_reg <= cb_d2;
        cr_out_reg <= cr_d2;
        sync_vsync_r[2] <= sync_vsync_r[1];
        sync_href_r[2]  <= sync_href_r[1];
        sync_clken_r[2] <= sync_clken_r[1];
    end
end

endmodule
