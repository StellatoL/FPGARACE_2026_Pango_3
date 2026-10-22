module frame_rate_halver (
    input  wire        clk_sys,     // 系统高速时钟（可选，用于同步逻辑�?
    input  wire        rst_n,       // 异步复位信号
    
    // 输入接口
    input  wire        pixclk_in,   // 输入像素时钟
    input  wire        vs_in_test,  // 输入帧同�?
    input  wire        href_in_test,// line active
    input  wire        de_in_test,  // 输入数据使能
    input  wire [15:0] i_rgb565,    // 输入像素数据
    
    // 输出接口
    output reg         vs_out,      // 输出帧同�?
    output reg         href_out,    // line active
    output reg         de_out,      // 输出数据使能
    output reg  [15:0] o_rgb565,    // 输出像素数据
    output wire        pixclk_out   // 输出像素时钟（通常透传�?
);

    reg vs_d0;
    reg frame_select;
    reg frame_keep;

    // 1. 检�?vs_in_test 的下降沿 (假设下降沿代表一帧结束或开�?
    always @(posedge pixclk_in or negedge rst_n) begin
        if (!rst_n) begin
            vs_d0 <= 1'b0;
            frame_select <= 1'b0;
            frame_keep <= 1'b0;
        end else begin
            vs_d0 <= vs_in_test;
            // Make one decision for both the VS pulse and its following
            // pixels. Never emit a VS pulse for a discarded frame.
            if (!vs_d0 && vs_in_test) begin
                frame_keep <= ~frame_select;
                frame_select <= ~frame_select;
            end
        end
    end

    // 2. 逻辑门控输出
    // 只有�?frame_select �?0 时，才允许信号输出，否则输出�?0
    always @(posedge pixclk_in or negedge rst_n) begin
        if (!rst_n) begin
            vs_out   <= 1'b0;
            href_out <= 1'b0;
            de_out   <= 1'b0;
            o_rgb565 <= 16'd0;
        end else begin
            if ((!vs_d0 && vs_in_test) ? ~frame_select : frame_keep) begin
                vs_out   <= vs_in_test;
                href_out <= href_in_test;
                de_out   <= de_in_test;
                o_rgb565 <= i_rgb565;
            end else begin
                vs_out   <= 1'b0; // 或者根据下游需求保持为�?�?
                href_out <= 1'b0;
                de_out   <= 1'b0;
                o_rgb565 <= 16'd0;
            end
        end
    end

    // 3. 时钟处理
    // 物理�?pixclk_in 依然存在，但由于 de_out 被拉低，下游模块不会采集有效像素
    assign pixclk_out = pixclk_in;

endmodule
