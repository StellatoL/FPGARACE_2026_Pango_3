// pclk 域采集 1280x720 RGB565；vsync 上升沿开始新帧，且该拍不能有 de。
// 每 8 像素打包一个 128 位数据拍，最低 16 位为最早像素。
// FIFO 满后丢弃本帧剩余数据，不产生尾标记；下一 SOF 使写端重新对齐。
// 输入必须为完整光栅帧（每行 1280 个 de，720 行），不做摄像头配置。
module cds_capture (
    input wire pclk,
    input wire rst_n,
    input wire vsync,
    input wire de,
    input wire [15:0] pixel,
    output wire fifo_valid,
    input wire fifo_ready,
    output wire [129:0] fifo_data,
    output reg overflow_sticky
);
    localparam [19:0] FRAME_PIXELS = 20'd921600;
    reg vsync_d;
    reg active;
    reg [19:0] pixel_cnt;
    reg [127:0] packed_pixels;
    wire frame_start = vsync && !vsync_d;
    assign fifo_valid = active && de && !frame_start && pixel_cnt[2:0] == 3'd7;
    assign fifo_data = {pixel_cnt == FRAME_PIXELS - 1'b1,
                        pixel_cnt == 20'd7, pixel, packed_pixels[111:0]};

    always @(posedge pclk) begin
        if (active && de) begin
            packed_pixels[pixel_cnt[2:0]*16 +: 16] <= pixel;
        end
    end
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d <= 1'b0;
            active <= 1'b0;
            pixel_cnt <= 20'd0;
            overflow_sticky <= 1'b0;
        end else begin
            vsync_d <= vsync;
            if (frame_start) begin
                active <= 1'b1;
                pixel_cnt <= 20'd0;
            end else if (active && de) begin
                pixel_cnt <= pixel_cnt + 1'b1;
                if (fifo_valid && !fifo_ready) begin
                    active <= 1'b0;
                    overflow_sticky <= 1'b1;
                end else if (pixel_cnt == FRAME_PIXELS - 1'b1) begin
                    active <= 1'b0;
                end
            end
        end
    end
endmodule
