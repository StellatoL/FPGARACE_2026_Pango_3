//计算透射率率图，消耗四个时钟
`timescale 1ns/1ns
module VIP_Transmission_Map
(
    //global clock
    input               clk,
    input               rst_n,

    input               per_frame_vsync,
    input               per_frame_href,
    input               per_frame_clken,
    input       [7:0]   per_img_Dark,       //暗通道

    input       [7:0]   atmospheric_light,  //大气光强度

    output              post_frame_vsync,
    output              post_frame_href,
    output              post_frame_clken,
    output      [7:0]   post_transmission   //透射率
);

//w = 0.9
parameter W_MULT_255 = 230; // 0.9 * 255

// 透射率下限 64
parameter T_MIN      = 40;  //透射率最小值

//倒数查表的小数位数(Q0.RECIP_SHIFT)
// 提升到Q15以提高除法近似精度，同时保持16位倒数表可实现
parameter RECIP_SHIFT = 15;

//透射率计算公式
//t = 1 - w* (Dark/atmospheric_light)
//转化为
//post_transmission = t*255 = 255 - W_MULT_255 * Dark / atmospheric_light

//-------------------------------------------------------
//除法优化说明：
//原式：W_MULT_255*Dark/atmospheric_light
//改为：W_MULT_255*Dark*recip(atmospheric_light) >> RECIP_SHIFT
//其中 recip(x) ~= 2^RECIP_SHIFT / x（查表得到），避免实时除法器。

reg [15:0] recip_lut [0:255];
integer i;

initial begin
    recip_lut[0] = 16'd32767; // 保护项，实际应由上游保证大气光>=1
    for(i = 1; i < 256; i = i + 1) begin
        //四舍五入：recip = round(2^N / i)
        recip_lut[i] = ((1 << RECIP_SHIFT) + (i >> 1)) / i;
    end
end

//-------------------------------------------------------
//第一个时钟：计算 W_MULT_255*Dark，并对大气光打一拍用于同像素对齐

reg [15:0]  w255_mult_dark;
reg [7:0]   atmospheric_light_d1;

always@ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        w255_mult_dark <= 16'd0;
        atmospheric_light_d1 <= 8'd1;
    end
    else begin
        w255_mult_dark <= W_MULT_255 * per_img_Dark;
        atmospheric_light_d1 <= atmospheric_light;
    end
end

// //-------------------------------------------------------
// //第二个时钟 计算除法

// reg [15:0] w255xdark_div_atmos;

// always@ (posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         w255xdark_div_atmos <= 16'd0;
//     end
//     else begin
//         w255xdark_div_atmos <= w255_mult_dark / atmospheric_light;
//     end
// end

//-------------------------------------------------------
//第二个时钟：查表后先做乘法，降低单拍组合深度

reg [31:0] w255xdark_mult_recip;

always@ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        w255xdark_mult_recip <= 32'd0;
    end
    else begin
        w255xdark_mult_recip <= w255_mult_dark * recip_lut[atmospheric_light_d1];
    end
end

//-------------------------------------------------------
//第三个时钟：右移并四舍五入，得到除法近似值

reg [15:0] w255xdark_div_atmos;

always@ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        w255xdark_div_atmos <= 16'd0;
    end
    else begin
        // +2^(N-1) 实现右移前四舍五入，降低量化误差
        w255xdark_div_atmos <= (w255xdark_mult_recip + (1 << (RECIP_SHIFT - 1))) >> RECIP_SHIFT;
    end
end



//-------------------------------------------------------
//第四个时钟：完成 t*255 = 255 - W_MULT_255*Dark/A，并限制最小透射率

reg [15:0] transmission;

always@ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        transmission    <= 16'd0;
    end
    else begin
        if(w255xdark_div_atmos >= 255 - T_MIN)
            transmission    <= T_MIN;           //限定透射率最小值
        else
            transmission    <= 255 - w255xdark_div_atmos;
    end
end

assign post_transmission = transmission[7:0];

//-------------------------------------------------------
//lag 4 clocks signal sync
reg [3:0]   per_frame_vsync_r;
reg [3:0]   per_frame_href_r;
reg [3:0]   per_frame_clken_r;

always@ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r <= 0;
        per_frame_href_r  <= 0;
        per_frame_clken_r <= 0;
        end
    else begin
        per_frame_vsync_r <= {per_frame_vsync_r[2:0],   per_frame_vsync};
        per_frame_href_r  <= {per_frame_href_r[2:0],    per_frame_href };
        per_frame_clken_r <= {per_frame_clken_r[2:0],   per_frame_clken};
    end
end

assign post_frame_vsync = per_frame_vsync_r[3];
assign post_frame_href  = per_frame_href_r[3];
assign post_frame_clken = per_frame_clken_r[3];

endmodule