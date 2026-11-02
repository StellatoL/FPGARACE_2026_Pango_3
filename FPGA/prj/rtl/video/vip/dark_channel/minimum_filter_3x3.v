module minimum_filter_3x3(
    input       clk,
    input       rst_n,
    input       minimum_frame_vsync,
    input       minimum_frame_href,
    input       minimum_frame_clken,
   
    input [7:0]  data11, 
    input [7:0]  data12, 
    input [7:0]  data13,
    input [7:0]  data21, 
    input [7:0]  data22, 
    input [7:0]  data23,
    input [7:0]  data31, 
    input [7:0]  data32, 
    input [7:0]  data33,
   
    output [7:0] target_data,
    output       pos_minimum_vsync,
    output       pos_minimum_href,
    output       pos_minimum_clken
);

//--------------------------------------------------------------------------------------
//FPGA minimum Filter Sort order
//       Pixel -- Sort1 -- Sort2 -- Sort3
// [ P1  P2  P3 ]	[   Max1  Mid1   Min1 ]
// [ P4  P5  P6 ]	[   Max2  Mid2   Min2 ]	[Min1, Min2, Min3]	Min_valid
// [ P7  P8  P9 ]	[   Max3  Mid3   Min3 ]

//reg define
reg [1:0]   minimum_frame_vsync_r;
reg [1:0]   minimum_frame_href_r;
reg [1:0]   minimum_frame_clken_r;
//wire define
wire [7:0] max_data1; 
wire [7:0] mid_data1; 
wire [7:0] min_data1;
wire [7:0] max_data2; 
wire [7:0] mid_data2; 
wire [7:0] min_data2;
wire [7:0] max_data3; 
wire [7:0] mid_data3; 
wire [7:0] min_data3;

//*****************************************************
//**                    main code
//*****************************************************

assign pos_minimum_vsync = minimum_frame_vsync_r[1];
assign pos_minimum_href  = minimum_frame_href_r[1];
assign pos_minimum_clken = minimum_frame_clken_r[1];

//Step1 对stor3进行三次例化操作
sort3  u_sort3_1(     //第一行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
    
    .data1    (data11), 
    .data2    (data12), 
    .data3    (data13),
    
    .max_data (max_data1),
    .mid_data (mid_data1),
    .min_data (min_data1)
);

sort3  u_sort3_2(      //第二行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data21), 
    .data2    (data22), 
    .data3    (data23),
    
    .max_data (max_data2),
    .mid_data (mid_data2),
    .min_data (min_data2)
);

sort3  u_sort3_3(      //第三行数据排序
    .clk      (clk),
    .rst_n    (rst_n),
        
    .data1    (data31), 
    .data2    (data32), 
    .data3    (data33),
    
    .max_data (max_data3),
    .mid_data (mid_data3),
    .min_data (min_data3)
);

//step2 将step1 中得到的三个值，再次取最小值
sort3 u_sort3_7(
    .clk      (clk),
    .rst_n    (rst_n),
          
    .data1    (min_data1), 
    .data2    (min_data2), 
    .data3    (min_data3),
    
    .max_data (),
    .mid_data (),
    .min_data (target_data)
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