module mean_filter_3x3#(
    parameter DATA_WIDTH = 8,
    parameter IMG_HDISP = 1024,
    parameter IMG_VDISP = 720
)(
    input       clk,
    input       rst_n,
    input       mean_frame_vsync,
    input       mean_frame_href,
    input       mean_frame_clken,
    
    input  [DATA_WIDTH - 1:0]  data11,
    input  [DATA_WIDTH - 1:0]  data12,
    input  [DATA_WIDTH - 1:0]  data13,
    input  [DATA_WIDTH - 1:0]  data21,
    input  [DATA_WIDTH - 1:0]  data22,
    input  [DATA_WIDTH - 1:0]  data23,
    input  [DATA_WIDTH - 1:0]  data31,
    input  [DATA_WIDTH - 1:0]  data32,
    input  [DATA_WIDTH - 1:0]  data33, 
   
    output [DATA_WIDTH - 1:0] target_data,
    output       pos_median_vsync,
    output       pos_median_href,
    output       pos_median_clken
);

reg [DATA_WIDTH + 1:0] target_data_row1;
reg [DATA_WIDTH + 1:0] target_data_row2;
reg [DATA_WIDTH + 1:0] target_data_row3;

reg [DATA_WIDTH + 3:0] target_data_sum;

// step1-2: 计算 3x3 矩阵中每一行的像素值之和，并计算总和
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        target_data_row1 <= 0;
        target_data_row2 <= 0;
        target_data_row3 <= 0;
        target_data_sum  <= 0;
    end
    else begin
        target_data_row1 <= {2'd0, data11} + {2'd0, data12} + {2'd0, data13};
        target_data_row2 <= {2'd0, data21} + {2'd0, data22} + {2'd0, data23};
        target_data_row3 <= {2'd0, data31} + {2'd0, data32} + {2'd0, data33};
        target_data_sum  <= {2'd0, target_data_row1} + {2'd0, target_data_row2} + {2'd0, target_data_row3};
    end
end

// step3：利用移位计算平均值
reg [19:0] target_data_mult;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        target_data_mult <= 0;
    end else begin
        target_data_mult <= ({8'd0, target_data_sum} << 8) +
                            ({8'd0, target_data_sum} << 7) +
                            ({8'd0, target_data_sum} << 6) +
                            ({8'd0, target_data_sum} << 2) +
                            ({8'd0, target_data_sum} << 1) +
                            {8'd0, target_data_sum} + 20'd2048;
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
// step3: 计算平均值，除以9
assign target_data = target_data_mult[19:12];

endmodule 