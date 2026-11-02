// 双口 RAM，2048×8bit，支持独立的读写接口
// 适配 blk_mem_gen_0 IP 核接口：分离读写时钟、读写复位、读写地址、读写数据
module dualport_ram #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 2048
)(
    // 写端口
    input wire                  wr_clk,
    input wire                  wr_rst,
    input wire [$clog2(DEPTH)-1:0] wr_addr,
    input wire [DATA_WIDTH-1:0] wr_data,
    input wire                  wr_en,

    // 读端口
    input wire                  rd_clk,
    input wire                  rd_rst,
    input wire [$clog2(DEPTH)-1:0] rd_addr,
    output reg [DATA_WIDTH-1:0] rd_data
);

// 地址位宽由深度自动推导
localparam ADDR_WIDTH = $clog2(DEPTH);

// RAM 存储阵列：2048×8
reg [DATA_WIDTH-1:0] ram_vector [DEPTH-1:0];

integer i;

// // 仿真初始化：避免读取未写地址时出现X扩散
// initial begin
//     for (i = 0; i < DEPTH; i = i + 1) begin
//         ram_vector[i] = {DATA_WIDTH{1'b0}};
//     end
// end

// 写逻辑：同步写，wr_en=1 时更新 RAM
always @(posedge wr_clk or posedge wr_rst) begin
    if (wr_rst) begin
        // 复位时无需清空 RAM 内容，只需确保写有效状态初始化
    end
    else if (wr_en) begin
        ram_vector[wr_addr] <= wr_data;
    end
end

// 读逻辑：同步读，输出有 1 拍读延迟（符合 BRAM 行为）
always @(posedge rd_clk or posedge rd_rst) begin
    if (rd_rst) begin
        rd_data <= {DATA_WIDTH{1'b0}};
    end
    else begin
        rd_data <= ram_vector[rd_addr];
    end
end

endmodule
