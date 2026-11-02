module mean_filter_5x5#(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP = 1024,
    parameter IMG_VDISP = 600
)(
    input       clk,
    input       rst_n,
    input       mean_frame_vsync,
    input       mean_frame_href,
    input       mean_frame_clken,
    
    input  [DATA_WIDTH - 1:0]  data11,
    input  [DATA_WIDTH - 1:0]  data12,
    input  [DATA_WIDTH - 1:0]  data13,
    input  [DATA_WIDTH - 1:0]  data14,
    input  [DATA_WIDTH - 1:0]  data15,
    input  [DATA_WIDTH - 1:0]  data21,
    input  [DATA_WIDTH - 1:0]  data22,
    input  [DATA_WIDTH - 1:0]  data23,
    input  [DATA_WIDTH - 1:0]  data24,
    input  [DATA_WIDTH - 1:0]  data25,
    input  [DATA_WIDTH - 1:0]  data31,
    input  [DATA_WIDTH - 1:0]  data32,
    input  [DATA_WIDTH - 1:0]  data33,
    input  [DATA_WIDTH - 1:0]  data34,
    input  [DATA_WIDTH - 1:0]  data35,
    input  [DATA_WIDTH - 1:0]  data41,
    input  [DATA_WIDTH - 1:0]  data42,
    input  [DATA_WIDTH - 1:0]  data43,
    input  [DATA_WIDTH - 1:0]  data44,
    input  [DATA_WIDTH - 1:0]  data45,
    input  [DATA_WIDTH - 1:0]  data51,
    input  [DATA_WIDTH - 1:0]  data52,
    input  [DATA_WIDTH - 1:0]  data53,
    input  [DATA_WIDTH - 1:0]  data54,
    input  [DATA_WIDTH - 1:0]  data55, 
   
    output [DATA_WIDTH - 1:0] target_data,
    output       pos_median_vsync,
    output       pos_median_href,
    output       pos_median_clken
);

reg [DATA_WIDTH + 2:0] target_data_row1;
reg [DATA_WIDTH + 2:0] target_data_row2;
reg [DATA_WIDTH + 2:0] target_data_row3;
reg [DATA_WIDTH + 2:0] target_data_row4;
reg [DATA_WIDTH + 2:0] target_data_row5;

reg [DATA_WIDTH + 5:0] target_data_sum;

// step1-2: 计算 5x5 矩阵中每一行的像素值之和，并计算总和
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        target_data_row1 <= 0;
        target_data_row2 <= 0;
        target_data_row3 <= 0;
        target_data_row4 <= 0;
        target_data_row5 <= 0;
        target_data_sum  <= 0;
    end
    else begin
        target_data_row1 <= {3'd0, data11} + {3'd0, data12} + {3'd0, data13} + {3'd0, data14} + {3'd0, data15};
        target_data_row2 <= {3'd0, data21} + {3'd0, data22} + {3'd0, data23} + {3'd0, data24} + {3'd0, data25};
        target_data_row3 <= {3'd0, data31} + {3'd0, data32} + {3'd0, data33} + {3'd0, data34} + {3'd0, data35};
        target_data_row4 <= {3'd0, data41} + {3'd0, data42} + {3'd0, data43} + {3'd0, data44} + {3'd0, data45};
        target_data_row5 <= {3'd0, data51} + {3'd0, data52} + {3'd0, data53} + {3'd0, data54} + {3'd0, data55};
        target_data_sum  <= {3'd0, target_data_row1} + {3'd0, target_data_row2} + {3'd0, target_data_row3} + {3'd0, target_data_row4} + {3'd0, target_data_row5};
    end
end

// step3：利用移位计算平均值
reg [19:0] target_data_mult;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        target_data_mult <= 0;
    end else begin
        target_data_mult <= ({7'd0, target_data_sum} << 7) + 
                            ({7'd0, target_data_sum} << 5) + 
                            ({7'd0, target_data_sum} << 2);
    end
end


reg mean_vsync_d0;
reg mean_vsync_d1;
reg mean_vsync_d2;
reg mean_href_d0;
reg mean_href_d1;
reg mean_href_d2;
reg mean_clken_d0;
reg mean_clken_d1;
reg mean_clken_d2;

// step1-3: 将计算结果同步化输出(打3拍)
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        mean_vsync_d0 <= 1'b0;
        mean_vsync_d1 <= 1'b0;
        mean_vsync_d2 <= 1'b0;
        mean_href_d0  <= 1'b0;
        mean_href_d1  <= 1'b0;
        mean_href_d2  <= 1'b0;
        mean_clken_d0 <= 1'b0;
        mean_clken_d1 <= 1'b0;
        mean_clken_d2 <= 1'b0;
    end
    else begin
        mean_vsync_d0 <= mean_frame_vsync;
        mean_vsync_d1 <= mean_vsync_d0;
        mean_vsync_d2 <= mean_vsync_d1;

        mean_href_d0  <= mean_frame_href;
        mean_href_d1  <= mean_href_d0;
        mean_href_d2  <= mean_href_d1;

        mean_clken_d0 <= mean_frame_clken;
        mean_clken_d1 <= mean_clken_d0;
        mean_clken_d2 <= mean_clken_d1;
    end
end

assign pos_median_vsync = mean_vsync_d2;
assign pos_median_href = mean_href_d2;
assign pos_median_clken = mean_clken_d2;
// step3: 计算平均值，除以25
assign target_data = target_data_mult[19:12];

endmodule 