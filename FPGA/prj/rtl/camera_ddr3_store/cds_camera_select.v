// KEY2 低有效，50 MHz 时钟域同步与 20 ms 稳定消抖。
// 每次稳定按下切换一次，长按不连发，释放也必须经过消抖。
// 通道编码 0/1/2/3 对应板上摄像头 1/2/3/4；复位默认摄像头 1。
module cds_camera_select (
    input wire clk,
    input wire rst_n,
    input wire key2_n,
    output reg [1:0] camera_sel
);
    localparam [19:0] DEBOUNCE_LAST = 20'd999999;
    (* ASYNC_REG = "TRUE" *) reg key_meta;
    (* ASYNC_REG = "TRUE" *) reg key_sync;
    reg key_stable;
    reg [19:0] stable_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
        end else begin
            key_meta <= key2_n;
            key_sync <= key_meta;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable <= 1'b1;
            stable_count <= 20'd0;
            camera_sel <= 2'd0;
        end else if (key_sync == key_stable) begin
            stable_count <= 20'd0;
        end else if (stable_count == DEBOUNCE_LAST) begin
            stable_count <= 20'd0;
            key_stable <= key_sync;
            if (!key_sync) begin
                camera_sel <= camera_sel + 1'b1;
            end
        end else begin
            stable_count <= stable_count + 1'b1;
        end
    end
endmodule
