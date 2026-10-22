`timescale 1ns/1ns

module VIP_multi_plate_filter #(
    parameter DATA_WIDTH = 11,
    // 几何限制参数
    parameter MIN_W = 40,
    parameter MIN_H = 15,
    parameter MAX_W = 800,
    parameter MAX_H = 500
)(
    input                       clk,
    input                       rst_n,

    // 视频同步信号
    input                       per_frame_vsync,

    // 来自前级聚合 FIFO (top_fifo) 的接口
    input                       fifo_empty,
    input      [4*DATA_WIDTH-1:0] fifo_data,
    output reg                  fifo_rd_en,

    // 最终车牌定位输出 (脉冲接口)
    // 每当检测到一个合格车牌，plate_valid 会高电平保持一个周期
    output reg                  plate_valid,
    output reg [DATA_WIDTH-1:0] plate_left,
    output reg [DATA_WIDTH-1:0] plate_right,
    output reg [DATA_WIDTH-1:0] plate_up,
    output reg [DATA_WIDTH-1:0] plate_down
);

    // --- 1. 内部读取逻辑与延迟对齐 ---
    reg fifo_rd_en_d1; // 延迟一拍以对齐 FIFO 的读数据延迟

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_rd_en    <= 1'b0;
            fifo_rd_en_d1 <= 1'b0;
        end else begin
            // 只要 FIFO 不为空，就持续读取
            // 为了防止读空，只有在 !fifo_empty 时拉高 rd_en
            fifo_rd_en    <= !fifo_empty && !fifo_rd_en; 
            fifo_rd_en_d1 <= fifo_rd_en;
        end
    end

    // --- 2. 坐标解析与几何计算 ---
    // 这些 wire 会随 fifo_rd_en_d1 对齐输出的数据进行计算
    wire [DATA_WIDTH-1:0] cur_x_min = fifo_data[4*DATA_WIDTH-1 : 3*DATA_WIDTH];
    wire [DATA_WIDTH-1:0] cur_x_max = fifo_data[3*DATA_WIDTH-1 : 2*DATA_WIDTH];
    wire [DATA_WIDTH-1:0] cur_y_min = fifo_data[2*DATA_WIDTH-1 :   DATA_WIDTH];
    wire [DATA_WIDTH-1:0] cur_y_max = fifo_data[  DATA_WIDTH-1 :            0];

    wire [DATA_WIDTH:0] w = cur_x_max - cur_x_min;
    wire [DATA_WIDTH:0] h = cur_y_max - cur_y_min;
    wire [DATA_WIDTH+4:0] ratio_w5 = ({4'b0, w} << 2) + {4'b0, w};
    wire [DATA_WIDTH+4:0] ratio_h6 = ({4'b0, h} << 2) + ({4'b0, h} << 1);
    wire [DATA_WIDTH+4:0] ratio_h7 = ({4'b0, h} << 2) + ({4'b0, h} << 1) + {4'b0, h};

    // --- 3. 核心流式过滤逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            plate_valid <= 1'b0;
            plate_left  <= 0; plate_right <= 0;
            plate_up    <= 0; plate_down  <= 0;
        end 
        else if (fifo_rd_en_d1) begin
            // 对当前弹出的数据进行几何校验
            if (w >= MIN_W && h >= MIN_H && w <= MAX_W && h <= MAX_H) begin
                // 长宽比过滤 (1.2 < W/H < 7)
                if ((ratio_w5 > ratio_h6) && ({4'b0, w} < ratio_h7)) begin
                    // 校验通过，产生一个周期的有效信号
                    plate_valid <= 1'b1;
                    plate_left  <= cur_x_min;
                    plate_right <= cur_x_max;
                    plate_up    <= cur_y_min;
                    plate_down  <= cur_y_max;
                end else begin
                    plate_valid <= 1'b0;
                end
            end else begin
                plate_valid <= 1'b0;
            end
        end else begin
            plate_valid <= 1'b0;
        end
    end

endmodule