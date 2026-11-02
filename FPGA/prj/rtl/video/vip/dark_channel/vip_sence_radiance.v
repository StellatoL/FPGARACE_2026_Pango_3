// 恢复场景辐射
`timescale 1ns/1ns
module VIP_Scene_radiance #(
    parameter   IMG_HDISP = 640,
    parameter   IMG_VDISP = 640
)
(
    //global clock
    input               clk,
    input               rst_n,

    input               per_frame_vsync,
    input               per_frame_href,
    input               per_frame_clken,

    input       [7:0]   per_img_red  ,
    input       [7:0]   per_img_green,
    input       [7:0]   per_img_blue ,

    input       [7:0]   pre_transmission,   //透射率
    input       [7:0]   atmospheric_light,  //大气光强度

    output              post_frame_vsync,
    output              post_frame_href,
    output              post_frame_clken,
    output wire [7:0]   post_img_red,
    output wire [7:0]   post_img_green,
    output wire [7:0]   post_img_blue
);

wire   [7:0]   minus_trans;
assign minus_trans = 8'd255 - pre_transmission;

reg    [15:0]      fog_mult_255_r    ;
reg    [15:0]      fog_mult_255_g    ;
reg    [15:0]      fog_mult_255_b    ;
reg    [15:0]      trans_mult_atmos  ;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        fog_mult_255_r    <= 16'd0;
        fog_mult_255_g    <= 16'd0;
        fog_mult_255_b    <= 16'd0;
        trans_mult_atmos  <= 16'd0;
    end
    else begin
        fog_mult_255_r    <= {per_img_red   ,8'd0} - per_img_red;
        fog_mult_255_g    <= {per_img_green ,8'd0} - per_img_green;
        fog_mult_255_b    <= {per_img_blue  ,8'd0} - per_img_blue;
        trans_mult_atmos  <= minus_trans * atmospheric_light;
    end
end

reg    [15:0]      numerator_r;
reg    [15:0]      numerator_g;
reg    [15:0]      numerator_b;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        numerator_r <= 16'd0;
        numerator_g <= 16'd0;
        numerator_b <= 16'd0;
    end
    else begin
        if(fog_mult_255_r > trans_mult_atmos)
            numerator_r <= fog_mult_255_r - trans_mult_atmos;
        else
            numerator_r <= 16'd0;
        if(fog_mult_255_g > trans_mult_atmos)
            numerator_g <= fog_mult_255_g - trans_mult_atmos;
        else
            numerator_g <= 16'd0;
        if(fog_mult_255_b > trans_mult_atmos)
            numerator_b <= fog_mult_255_b - trans_mult_atmos;
        else
            numerator_b <= 16'd0;
    end
end

reg    [7:0]      transmission_reg1;
reg    [7:0]      transmission_reg2;
wire   [7:0]      transmission_reg2_safe;

assign transmission_reg2_safe = (transmission_reg2 == 8'd0) ? 8'd1 : transmission_reg2;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        transmission_reg1 <= 8'd0;
        transmission_reg2 <= 8'd0;
    end
    else begin
        transmission_reg1 <= pre_transmission;
        transmission_reg2 <= transmission_reg1;
    end
end

reg    [15:0]      scene_radiance_r;
reg    [15:0]      scene_radiance_g;
reg    [15:0]      scene_radiance_b;

// 倒数查表的小数位数(Q0.RECIP_SHIFT)
// 提高到Q15以提升除法近似精度，同时保持倒数值可落在16位范围内
parameter RECIP_SHIFT = 15;

reg [15:0] recip_lut [0:255];
integer i;

initial begin
    recip_lut[0] = 16'd32767;
    for(i = 1; i < 256; i = i + 1) begin
        // recip(i) = round(2^RECIP_SHIFT / i)
        recip_lut[i] = ((1 << RECIP_SHIFT) + (i >> 1)) / i;
    end
end

reg [31:0] numerator_r_recip_lut, numerator_g_recip_lut, numerator_b_recip_lut;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        numerator_r_recip_lut <= 32'd0;
        numerator_g_recip_lut <= 32'd0;
        numerator_b_recip_lut <= 32'd0;
    end
    else begin
        // 使用倒数查表替代除法：x/div ~= (x*recip(div)+2^(N-1))>>N
        numerator_r_recip_lut <= numerator_r * recip_lut[transmission_reg2_safe];
        numerator_g_recip_lut <= numerator_g * recip_lut[transmission_reg2_safe];
        numerator_b_recip_lut <= numerator_b * recip_lut[transmission_reg2_safe];
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        scene_radiance_r <= 16'd0;
        scene_radiance_g <= 16'd0;
        scene_radiance_b <= 16'd0;
    end
    else begin
        // 使用倒数查表替代除法：x/div ~= (x*recip(div)+2^(N-1))>>N
        scene_radiance_r <= (numerator_r_recip_lut + (1 << (RECIP_SHIFT - 1))) >> RECIP_SHIFT;
        scene_radiance_g <= (numerator_g_recip_lut + (1 << (RECIP_SHIFT - 1))) >> RECIP_SHIFT;
        scene_radiance_b <= (numerator_b_recip_lut + (1 << (RECIP_SHIFT - 1))) >> RECIP_SHIFT;
    end
end

assign post_img_red   = (scene_radiance_r > 255) ? 8'd255 : scene_radiance_r[7:0];
assign post_img_green = (scene_radiance_g > 255) ? 8'd255 : scene_radiance_g[7:0];
assign post_img_blue  = (scene_radiance_b > 255) ? 8'd255 : scene_radiance_b[7:0];

reg    [3:0]      per_frame_vsync_r;
reg    [3:0]      per_frame_href_r;
reg    [3:0]      per_frame_clken_r;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r <= 4'd0;
        per_frame_href_r  <= 4'd0;
        per_frame_clken_r <= 4'd0;
    end
    else begin
        per_frame_vsync_r <= {per_frame_vsync_r[2:0], per_frame_vsync};
        per_frame_href_r  <= {per_frame_href_r[2:0], per_frame_href};
        per_frame_clken_r <= {per_frame_clken_r[2:0], per_frame_clken};
    end
end

assign post_frame_vsync = per_frame_vsync_r[3];
assign post_frame_href  = per_frame_href_r[3];
assign post_frame_clken = per_frame_clken_r[3];

endmodule