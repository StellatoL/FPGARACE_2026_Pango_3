`timescale 1ns/1ns
module fifo_ram#(
    parameter DATA_WIDTH = 8,
    parameter DATA_DEPTH = 640
)
(
    input                               clk,
    //write port
    input                               wr_en,
    input       [DATA_WIDTH - 1 : 0]    wr_data,
    output                              wr_full,
    //read port
    input                               rd_en,
    output reg  [DATA_WIDTH - 1 : 0]    rd_data,
    output                              rd_empty
);
    //define ram
    (*ram_style = "block" *) reg [DATA_WIDTH - 1 : 0] fifo_buffer[DATA_DEPTH - 1 : 0];

    integer i;
    initial begin
        for(i = 0;i<DATA_DEPTH;i = i + 1)begin
            fifo_buffer[i] <=  0;
        end
    end

    reg     [$clog2(DATA_DEPTH) - 1 : 0]    wr_pointer = 0;
    reg     [$clog2(DATA_DEPTH) - 1 : 0]    rd_pointer = 0;
    
    // [新增] 内部数据量计数器，用于判断空/满
    reg     [$clog2(DATA_DEPTH) : 0]        fifo_count = 0; 
    wire    [DATA_WIDTH - 1 : 0]            rd_data_out;

// keep track of data count for empty/full flags
always @(posedge clk) begin
    case ({wr_en, rd_en})
        2'b10: if (!wr_full)  fifo_count <= fifo_count + 1'b1;
        2'b01: if (!rd_empty) fifo_count <= fifo_count - 1'b1;
        2'b11: fifo_count <= fifo_count; // 同时读写，数量不变
        default: fifo_count <= fifo_count;
    endcase
end

assign wr_full  = (fifo_count == DATA_DEPTH);
assign rd_empty = (fifo_count == 0);

//keep track of the write  pointer
always @(posedge clk) begin
    if (wr_en && !wr_full) begin
        if (wr_pointer == DATA_DEPTH - 1) 
            wr_pointer <= 0; 
        else 
            wr_pointer <= wr_pointer + 1;
    end
end

//keep track of the read pointer 
always @(posedge clk) begin
    if (rd_en && !rd_empty) begin
        if (rd_pointer == DATA_DEPTH - 1) 
            rd_pointer <= 0;
        else 
            rd_pointer <= rd_pointer + 1;
    end
end

//write data into fifo when wr_en is asserted
always @(posedge clk) begin
    if (wr_en && !wr_full) begin
        fifo_buffer[wr_pointer] <= wr_data;
    end
end

assign rd_data_out = rd_en ? fifo_buffer[rd_pointer] : 0;
    
always @(posedge clk) begin
    rd_data <=  rd_data_out; // 延迟一拍输出
end

endmodule