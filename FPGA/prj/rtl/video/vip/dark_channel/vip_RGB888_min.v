module VIP_RGB888_MIN(
    input               clk,
    input               rst_n,

    input               per_frame_vsync,
    input               per_frame_href,
    input               per_frame_clken,
    input       [7:0]   per_img_red  ,
    input       [7:0]   per_img_green,
    input       [7:0]   per_img_blue ,

    output              post_frame_vsync,
    output              post_frame_href,
    output              post_frame_clken,
    output reg  [7:0]   post_RGB_MIN
);

wire [7:0] min_rg;
wire [7:0] min_rgb;

reg        per_frame_vsync_r;
reg        per_frame_href_r;
reg        per_frame_clken_r;

assign min_rg  = (per_img_red   < per_img_green) ? per_img_red   : per_img_green;
assign min_rgb = (min_rg        < per_img_blue ) ? min_rg        : per_img_blue;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        post_RGB_MIN <= 8'd0;
    end
    else begin
        post_RGB_MIN <= min_rgb;
    end
end

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

assign post_frame_vsync = per_frame_vsync_r;
assign post_frame_href  = per_frame_href_r;
assign post_frame_clken = per_frame_clken_r;

endmodule
