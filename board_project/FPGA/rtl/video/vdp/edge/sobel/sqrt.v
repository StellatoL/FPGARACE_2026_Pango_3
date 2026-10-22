module sqrt #(
    parameter d_width = 22, // 输入数据位宽
    parameter q_width = 11, // 开方结果位宽
    parameter r_width = 12  // 余数位宽，22 位输入时需要至少 12 位
)(
    input                  clk,
    input                  rst,     // 高电平复位 (对应主模块的 ~rst_n)
    input                  i_vaild, // 输入数据有效信号 (对应主模块拼写 i_vaild)
    input  [d_width-1:0]   data_i,  // 被开方数
    output                 o_vaild, // 输出数据有效信号
    output [q_width-1:0]   data_o,  // 开方结果
    output [r_width-1:0]   data_r   // 余数
);

    // 计算需要的流水线级数: 每级处理 2 bit，22 bit 需要 11 级
    localparam STAGES = d_width / 2;
    // 内部余数寄存器的安全位宽，防止移位时溢出
    localparam W = STAGES + 2;

    // 声明流水线寄存器阵列
    reg [d_width-1:0] d_pipe [0:STAGES];
    reg [q_width-1:0] q_pipe [0:STAGES];
    reg [W-1:0]       r_pipe [0:STAGES];
    reg [STAGES:0]    v_pipe;

    // 第0级流水线：输入数据打拍 (消耗 1 个时钟周期)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            d_pipe[0] <= 0;
            q_pipe[0] <= 0;
            r_pipe[0] <= 0;
            v_pipe[0] <= 0;
        end else begin
            d_pipe[0] <= data_i;
            q_pipe[0] <= 0;
            r_pipe[0] <= 0;
            v_pipe[0] <= i_vaild;
        end
    end

    // 第1~11级流水线：移位减迭代计算 (消耗 STAGES 个时钟周期)
    genvar i;
    generate
        for (i = 0; i < STAGES; i = i + 1) begin : gen_sqrt
            wire [W-1:0] temp_r;
            wire [W-1:0] temp_sub;

            // 提取被开方数的最高2位，追加到上一级余数的末尾
            assign temp_r = {r_pipe[i][W-3:0], d_pipe[i][d_width-1 : d_width-2]};
            
            // 构造减数：当前商左移 2 位后加上 2'b01 (即拼合 01)
            assign temp_sub = {q_pipe[i], 2'b01};

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    d_pipe[i+1] <= 0;
                    q_pipe[i+1] <= 0;
                    r_pipe[i+1] <= 0;
                    v_pipe[i+1] <= 0;
                end else begin
                    // 被开方数向左移 2 位，为下一级计算做准备
                    d_pipe[i+1] <= {d_pipe[i][d_width-3:0], 2'b00};
                    v_pipe[i+1] <= v_pipe[i];
                    
                    // 判断是否够减
                    if (temp_r >= temp_sub) begin
                        r_pipe[i+1] <= temp_r - temp_sub;
                        q_pipe[i+1] <= {q_pipe[i][q_width-2:0], 1'b1}; // 够减，当前位商 1
                    end else begin
                        r_pipe[i+1] <= temp_r;
                        q_pipe[i+1] <= {q_pipe[i][q_width-2:0], 1'b0}; // 不够减，当前位商 0
                    end
                end
            end
        end
    endgenerate

    // 组合输出结果
    assign o_vaild = v_pipe[STAGES];
    assign data_o  = q_pipe[STAGES];
    assign data_r  = r_pipe[STAGES][r_width-1:0];

endmodule