// 双时钟 FIFO：写端 ready/valid，读端同步 RAM 预取并保持输出。
// 两域必须共同复位；复位在各自时钟域同步释放。ADDR_BITS >= 2。
// Gray 指针同步器需在实现中添加 CDC/总线偏斜约束，RAM 推断需综合核对。
module cds_async_fifo #(
    parameter WIDTH = 130,
    parameter ADDR_BITS = 9
)(
    input wire wr_clk,
    input wire wr_rst_n,
    input wire wr_valid,
    output wire wr_ready,
    input wire [WIDTH-1:0] wr_data,
    input wire rd_clk,
    input wire rd_rst_n,
    output reg rd_valid,
    input wire rd_ready,
    output reg [WIDTH-1:0] rd_data
);
    reg [WIDTH-1:0] memory [0:(1 << ADDR_BITS)-1];
    reg [ADDR_BITS:0] wr_bin;
    reg [ADDR_BITS:0] wr_gray;
    reg [ADDR_BITS:0] rd_bin;
    reg [ADDR_BITS:0] rd_gray;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] rd_meta, rd_sync;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] wr_meta, wr_sync;
    wire full = wr_gray == {~rd_sync[ADDR_BITS:ADDR_BITS-1],
                            rd_sync[ADDR_BITS-2:0]};
    wire empty = rd_gray == wr_sync;
    wire [ADDR_BITS:0] wr_next = wr_bin + 1'b1;
    wire [ADDR_BITS:0] rd_next = rd_bin + 1'b1;
    assign wr_ready = !full;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_meta <= 0;
            rd_sync <= 0;
            wr_bin <= 0;
            wr_gray <= 0;
        end else begin
            rd_meta <= rd_gray;
            rd_sync <= rd_meta;
            if (wr_valid && wr_ready) begin
                wr_bin <= wr_next;
                wr_gray <= (wr_next >> 1) ^ wr_next;
            end
        end
    end
    always @(posedge wr_clk) begin
        if (wr_valid && wr_ready && wr_rst_n) begin
            memory[wr_bin[ADDR_BITS-1:0]] <= wr_data;
        end
    end
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_meta <= 0;
            wr_sync <= 0;
            rd_bin <= 0;
            rd_gray <= 0;
            rd_valid <= 1'b0;
        end else begin
            wr_meta <= wr_gray;
            wr_sync <= wr_meta;
            if (!rd_valid || rd_ready) begin
                rd_valid <= !empty;
                if (!empty) begin
                    rd_bin <= rd_next;
                    rd_gray <= (rd_next >> 1) ^ rd_next;
                end
            end
        end
    end
    always @(posedge rd_clk) begin
        if ((!rd_valid || rd_ready) && !empty) begin
            rd_data <= memory[rd_bin[ADDR_BITS-1:0]];
        end
    end
endmodule
