module minimum_filter_5x5(
    input       clk,
    input       rst_n,
    input       minimum_frame_vsync,
    input       minimum_frame_href,
    input       minimum_frame_clken,
   
    input [7:0]  data11, 
    input [7:0]  data12, 
    input [7:0]  data13,
    input [7:0]  data14,
    input [7:0]  data15,
    input [7:0]  data21, 
    input [7:0]  data22, 
    input [7:0]  data23,
    input [7:0]  data24,
    input [7:0]  data25,
    input [7:0]  data31, 
    input [7:0]  data32, 
    input [7:0]  data33,
    input [7:0]  data34,
    input [7:0]  data35,
    input [7:0]  data41,
    input [7:0]  data42,
    input [7:0]  data43,
    input [7:0]  data44,
    input [7:0]  data45,
    input [7:0]  data51,
    input [7:0]  data52,
    input [7:0]  data53,
    input [7:0]  data54,
    input [7:0]  data55,
   
    output [7:0] target_data,
    output       pos_minimum_vsync,
    output       pos_minimum_href,
    output       pos_minimum_clken
);

//--------------------------------------------------------------------------------------
//FPGA minimum Filter Sort order

//reg define
reg [1:0]   minimum_frame_vsync_r;
reg [1:0]   minimum_frame_href_r;
reg [1:0]   minimum_frame_clken_r;
//wire define 
wire [7:0] min_data1;
wire [7:0] min_data2;
wire [7:0] min_data3;
wire [7:0] min_data4;
wire [7:0] min_data5;

//*****************************************************
//**                    main code
//*****************************************************

assign pos_minimum_vsync = minimum_frame_vsync_r[1];
assign pos_minimum_href  = minimum_frame_href_r[1];
assign pos_minimum_clken = minimum_frame_clken_r[1];

//Step1 对stor5进行五次例化操作
sort5  u_sort5_1(     //第一行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
    
    .data1    (data11), 
    .data2    (data12), 
    .data3    (data13),
    .data4    (data14),
    .data5    (data15),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (min_data1)
);

sort5  u_sort5_2(      //第二行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data21), 
    .data2    (data22), 
    .data3    (data23),
    .data4    (data24),
    .data5    (data25),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (min_data2)
);

sort5  u_sort5_3(      //第三行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data31), 
    .data2    (data32), 
    .data3    (data33),
    .data4    (data34),
    .data5    (data35),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (min_data3)
);

sort5 u_sort5_4(      //第四行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data41), 
    .data2    (data42), 
    .data3    (data43),
    .data4    (data44),
    .data5    (data45),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (min_data4)
);

sort5 u_sort5_5(      //第五行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data51), 
    .data2    (data52), 
    .data3    (data53),
    .data4    (data54),
    .data5    (data55),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (min_data5)
);

//step2 将step1 中得到的五个值，再次取最小值
sort5 u_sort5_inst(
    .clk      (clk),
    .rst_n    (rst_n),
          
    .data1    (min_data1), 
    .data2    (min_data2), 
    .data3    (min_data3),
    .data4    (min_data4),
    .data5    (min_data5),
    
    .sort_data0 (),
    .sort_data1 (),
    .sort_data2 (),
    .sort_data3 (),
    .sort_data4 (target_data)
);

//延迟二个周期进行同步
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        minimum_frame_vsync_r <= 0;
        minimum_frame_href_r  <= 0;
        minimum_frame_clken_r <= 0;
    end
    else begin
        minimum_frame_vsync_r <= {minimum_frame_vsync_r[0],minimum_frame_vsync};
        minimum_frame_href_r  <= {minimum_frame_href_r [0], minimum_frame_href};
        minimum_frame_clken_r <= {minimum_frame_clken_r[0],minimum_frame_clken};
    end
end

endmodule 