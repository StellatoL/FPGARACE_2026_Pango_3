module sort3#(
    parameter DATA_WIDTH = 8
)(
    input            clk,
    input            rst_n,
    input [DATA_WIDTH-1:0]      data1, 
    input [DATA_WIDTH-1:0]      data2, 
    input [DATA_WIDTH-1:0]      data3,
    
    output reg [DATA_WIDTH-1:0] max_data, 
    output reg [DATA_WIDTH-1:0] mid_data, 
    output reg [DATA_WIDTH-1:0] min_data
);

//-----------------------------------
//对三个数据进行排序
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        max_data <= {DATA_WIDTH{1'b0}};
        mid_data <= {DATA_WIDTH{1'b0}};
        min_data <= {DATA_WIDTH{1'b0}};
    end
    else begin
        //取最大值
        if(data1 >= data2 && data1 >= data3)
            max_data <= data1;
        else if(data2 >= data1 && data2 >= data3)
            max_data <= data2;
        else//(data3 >= data1 && data3 >= data2)
            max_data <= data3;
        //取中值
        if((data1 >= data2 && data1 <= data3) || (data1 >= data3 && data1 <= data2))
            mid_data <= data1;
        else if((data2 >= data1 && data2 <= data3) || (data2 >= data3 && data2 <= data1))
            mid_data <= data2;
        else//((data3 >= data1 && data3 <= data2) || (data3 >= data2 && data3 <= data1))
            mid_data <= data3;
        //取最小值
        if(data1 <= data2 && data1 <= data3)
            min_data <= data1;
        else if(data2 <= data1 && data2 <= data3)
            min_data <= data2;
        else//(data3 <= data1 && data3 <= data2)
            min_data <= data3;
        
     end
end

endmodule 