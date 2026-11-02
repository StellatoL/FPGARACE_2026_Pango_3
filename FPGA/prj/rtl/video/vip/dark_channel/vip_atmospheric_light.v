// Estimate atmospheric light for dark-channel dehaze.
`timescale 1ns/1ns

module VIP_Atmospheric_Light #(
    parameter IMG_HDISP = 640,
    parameter IMG_VDISP = 640
)(
    input              clk,
    input              rst_n,

    input              per_frame_vsync,
    input              per_frame_href,
    input              per_frame_clken,
    input      [7:0]   per_img_Dark,

    input      [7:0]   per_img_red,
    input      [7:0]   per_img_green,
    input      [7:0]   per_img_blue,

    output reg [7:0]   atmospheric_light,
    output reg [11:0]  atmospheric_pos_x,
    output reg [11:0]  atmospheric_pos_y
);

reg per_frame_vsync_r;
reg per_frame_href_r;
reg per_frame_clken_r;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r <= 1'b0;
        per_frame_href_r  <= 1'b0;
        per_frame_clken_r <= 1'b0;
    end
    else begin
        per_frame_vsync_r <= per_frame_vsync;
        per_frame_href_r  <= per_frame_href;
        per_frame_clken_r <= per_frame_clken;
    end
end

wire vsync_pos_flag = per_frame_vsync & ~per_frame_vsync_r;

reg [11:0] x_cnt;
reg [11:0] y_cnt;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        x_cnt <= 12'd0;
        y_cnt <= 12'd0;
    end
    else if(vsync_pos_flag) begin
        x_cnt <= 12'd0;
        y_cnt <= 12'd0;
    end
    else if(per_frame_clken) begin
        if(x_cnt < IMG_HDISP - 1) begin
            x_cnt <= x_cnt + 1'b1;
        end
        else begin
            x_cnt <= 12'd0;
            y_cnt <= y_cnt + 1'b1;
        end
    end
end

reg [7:0]  dark_max;
reg [7:0]  color_R;
reg [7:0]  color_G;
reg [7:0]  color_B;
reg [11:0] atmos_x;
reg [11:0] atmos_y;
reg        frame_valid_seen;

wire [7:0] color_rg  = (color_R > color_G) ? color_R : color_G;
wire [7:0] color_max = (color_rg > color_B) ? color_rg : color_B;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        dark_max         <= 8'd0;
        color_R          <= 8'd0;
        color_G          <= 8'd0;
        color_B          <= 8'd0;
        atmos_x          <= 12'd0;
        atmos_y          <= 12'd0;
        frame_valid_seen <= 1'b0;
    end
    else if(vsync_pos_flag) begin
        dark_max         <= 8'd0;
        color_R          <= 8'd0;
        color_G          <= 8'd0;
        color_B          <= 8'd0;
        atmos_x          <= 12'd0;
        atmos_y          <= 12'd0;
        frame_valid_seen <= 1'b0;
    end
    else if(per_frame_clken) begin
        frame_valid_seen <= 1'b1;
        if(per_img_Dark > dark_max) begin
            dark_max <= per_img_Dark;
            color_R  <= per_img_red;
            color_G  <= per_img_green;
            color_B  <= per_img_blue;
            atmos_x  <= x_cnt;
            atmos_y  <= y_cnt;
        end
    end
end

reg atmospheric_init_done;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        atmospheric_light     <= 8'd180;
        atmospheric_pos_x     <= 12'd0;
        atmospheric_pos_y     <= 12'd0;
        atmospheric_init_done <= 1'b0;
    end
    else if(vsync_pos_flag && frame_valid_seen) begin
        atmospheric_pos_x <= atmos_x;
        atmospheric_pos_y <= atmos_y;

        if(!atmospheric_init_done) begin
            atmospheric_light     <= color_max;
            atmospheric_init_done <= 1'b1;
        end
        else begin
            atmospheric_light <= (({2'b00, atmospheric_light} * 3) +
                                  {2'b00, color_max} + 2) >> 2;
        end
    end
end

endmodule
