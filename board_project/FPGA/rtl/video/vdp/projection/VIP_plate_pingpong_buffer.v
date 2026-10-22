`timescale 1ns/1ns

module VIP_plate_pingpong_buffer #(
    parameter DATA_WIDTH = 11,
    parameter MAX_PLATES = 8  // 一帧内最多保存的车牌数量
)(
    input                       clk,
    input                       rst_n,
    input                       per_frame_vsync,

    // 输入接口（来自上一级的 filter 脉冲）
    input                       plate_valid,
    input [DATA_WIDTH-1:0]      plate_x_min,
    input [DATA_WIDTH-1:0]      plate_x_max,
    input [DATA_WIDTH-1:0]      plate_y_min,
    input [DATA_WIDTH-1:0]      plate_y_max,

    // 输出接口（供下一帧 SVM 或 OSD 使用）
    output reg [3:0]            stored_plate_num, // 上一帧共发现多少个车牌
    
    // [新增与修改] 带使能的同步读取接口
    input                       read_en,          // 读取使能
    input      [3:0]            read_addr,        // 读取地址
    output reg [4*DATA_WIDTH-1:0] read_data       // 读取到的十字坐标包 (改为 reg)
);

    // --- 1. 乒乓切换逻辑 ---
    reg vsync_d1;
    always @(posedge clk) vsync_d1 <= per_frame_vsync;
    wire vsync_pos = per_frame_vsync & ~vsync_d1;

    reg ping_pong_sel; // 0: 写 Buffer0/读 Buffer1; 1: 写 Buffer1/读 Buffer0
    reg [3:0] write_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ping_pong_sel <= 1'b0;
            write_cnt <= 0;
            stored_plate_num <= 0;
        end else if (vsync_pos) begin
            ping_pong_sel <= ~ping_pong_sel;   // 切换读写缓存
            stored_plate_num <= write_cnt;     // 锁存上一帧的检测数量
            write_cnt <= 0;                    // 清零计数器准备新一帧写入
        end else if (plate_valid && write_cnt < MAX_PLATES) begin
            write_cnt <= write_cnt + 1'b1;
        end
    end

    // --- 2. 存储实体 ---
    reg [4*DATA_WIDTH-1:0] buffer0 [0:MAX_PLATES-1];
    reg [4*DATA_WIDTH-1:0] buffer1 [0:MAX_PLATES-1];

    wire [4*DATA_WIDTH-1:0] write_data_pack = {plate_x_min, plate_x_max, plate_y_min, plate_y_max};

    // 写入逻辑 (同步写入)
    always @(posedge clk) begin
        if (plate_valid && write_cnt < MAX_PLATES) begin
            if (ping_pong_sel == 1'b0)
                buffer0[write_cnt] <= write_data_pack;
            else
                buffer1[write_cnt] <= write_data_pack;
        end
    end

    // --- 3. [已修改] 同步读取逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_data <= 0;
        end else if (read_en) begin
            // 只有当 read_en 为高时，才根据地址更新输出数据
            if (ping_pong_sel == 1'b1)
                read_data <= buffer0[read_addr];
            else
                read_data <= buffer1[read_addr];
        end
    end

endmodule