`timescale 1ns/1ns

module tb_sqrt();

    // ==========================================
    // 信号定义
    // ==========================================
    reg         clk;
    reg         rst;
    reg         i_vaild;
    reg  [21:0] data_i;

    wire        o_vaild;
    wire [10:0] data_o;
    wire [11:0] data_r;

    // ==========================================
    // 模块例化
    // ==========================================
    sqrt #(
        .d_width(22),
        .q_width(11),
        .r_width(12)
    ) u_sqrt (
        .clk     (clk    ),
        .rst     (rst    ),
        .i_vaild (i_vaild),
        .data_i  (data_i ),
        .o_vaild (o_vaild),
        .data_o  (data_o ),
        .data_r  (data_r )
    );

    // ==========================================
    // 时钟生成 (50MHz)
    // ==========================================
    initial begin
        clk = 0;
        forever #10 clk = ~clk; 
    end

    // ==========================================
    // 为了方便打印，将输入数据也延迟 12 拍以对齐输出
    // ==========================================
    reg [21:0] data_i_d [0:11];
    integer i;
    always @(posedge clk) begin
        data_i_d[0] <= data_i;
        for (i = 1; i < 12; i = i + 1) begin
            data_i_d[i] <= data_i_d[i-1];
        end
    end

    // ==========================================
    // 激励生成
    // ==========================================
    initial begin
        // 初始化
        rst     = 1;
        i_vaild = 0;
        data_i  = 0;
        
        // 释放复位
        #35;
        rst     = 0;
        #20;

        // 连续输入测试（测试流水线的吞吐能力）
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd0;       // 测试 0
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd9;       // 测试 9 (sqrt=3, r=0)
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd100;     // 测试 100 (sqrt=10, r=0)
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd105;     // 测试 105 (sqrt=10, r=5)
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd2080800; // 测试 Sobel 极限值 (sqrt=1442, r=1436)
        @(posedge clk);
        i_vaild <= 1; data_i <= 22'd4194303; // 测试 22bit 最大值 (sqrt=2047, r=4094)
        @(posedge clk);
        i_vaild <= 0; data_i <= 22'd0;       // 停止输入数据

        // 等待所有流水线数据输出完毕
        #300;
        $stop;
    end

    // ==========================================
    // 结果监测与文字输出
    // ==========================================
    always @(posedge clk) begin
        if (o_vaild) begin
            $display("Time: %0t | Input: %8d | Sqrt(Quotient): %4d | Remainder: %4d", 
                      $time, data_i_d[11], data_o, data_r);
            
            // 简单的自检逻辑：检查 商^2 + 余数 是否等于 原数据
            if ((data_o * data_o + data_r) != data_i_d[11]) begin
                $display("  -> [ERROR] Math mismatch!");
            end
        end
    end

endmodule