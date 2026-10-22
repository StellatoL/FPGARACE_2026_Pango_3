module sort5#(
    parameter DATA_WIDTH = 8
)(
    input                       clk,
    input                       rst_n,
    input   [DATA_WIDTH-1:0]    data1,
    input   [DATA_WIDTH-1:0]    data2,
    input   [DATA_WIDTH-1:0]    data3,
    input   [DATA_WIDTH-1:0]    data4,
    input   [DATA_WIDTH-1:0]    data5,

    output reg [DATA_WIDTH-1:0] sort_data0,
    output reg [DATA_WIDTH-1:0] sort_data1,
    output reg [DATA_WIDTH-1:0] sort_data2,
    output reg [DATA_WIDTH-1:0] sort_data3,
    output reg [DATA_WIDTH-1:0] sort_data4
);

reg [DATA_WIDTH-1:0] val0;
reg [DATA_WIDTH-1:0] val1;
reg [DATA_WIDTH-1:0] val2;
reg [DATA_WIDTH-1:0] val3;
reg [DATA_WIDTH-1:0] val4;
reg [DATA_WIDTH-1:0] swap_tmp;

always @(*) begin
    val0 = data1;
    val1 = data2;
    val2 = data3;
    val3 = data4;
    val4 = data5;

    if (val0 < val1) begin
        swap_tmp = val0; val0 = val1; val1 = swap_tmp;
    end
    if (val1 < val2) begin
        swap_tmp = val1; val1 = val2; val2 = swap_tmp;
    end
    if (val2 < val3) begin
        swap_tmp = val2; val2 = val3; val3 = swap_tmp;
    end
    if (val3 < val4) begin
        swap_tmp = val3; val3 = val4; val4 = swap_tmp;
    end

    if (val0 < val1) begin
        swap_tmp = val0; val0 = val1; val1 = swap_tmp;
    end
    if (val1 < val2) begin
        swap_tmp = val1; val1 = val2; val2 = swap_tmp;
    end
    if (val2 < val3) begin
        swap_tmp = val2; val2 = val3; val3 = swap_tmp;
    end

    if (val0 < val1) begin
        swap_tmp = val0; val0 = val1; val1 = swap_tmp;
    end
    if (val1 < val2) begin
        swap_tmp = val1; val1 = val2; val2 = swap_tmp;
    end

    if (val0 < val1) begin
        swap_tmp = val0; val0 = val1; val1 = swap_tmp;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sort_data0 <= {DATA_WIDTH{1'b0}};
        sort_data1 <= {DATA_WIDTH{1'b0}};
        sort_data2 <= {DATA_WIDTH{1'b0}};
        sort_data3 <= {DATA_WIDTH{1'b0}};
        sort_data4 <= {DATA_WIDTH{1'b0}};
    end
    else begin
        sort_data0 <= val0;
        sort_data1 <= val1;
        sort_data2 <= val2;
        sort_data3 <= val3;
        sort_data4 <= val4;
    end
end

endmodule
