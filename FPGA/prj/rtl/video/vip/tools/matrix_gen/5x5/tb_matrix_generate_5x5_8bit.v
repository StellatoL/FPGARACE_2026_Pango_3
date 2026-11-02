`timescale 1ns/1ps

module tb_matrix_generate_5x5_8bit;

    localparam DATA_WIDTH = 8;
    localparam DEPTH      = 16;
    localparam IMG_W      = 8;
    localparam IMG_H      = 8;

    reg                     clk;
    reg                     rst_n;
    reg                     per_frame_vsync;
    reg                     per_frame_href;
    reg                     per_frame_clken;
    reg  [DATA_WIDTH-1:0]   per_img_y;

    wire                    matrix_frame_vsync;
    wire                    matrix_frame_href;
    wire                    matrix_frame_clken;
    wire [DATA_WIDTH-1:0]   matrix_p11;
    wire [DATA_WIDTH-1:0]   matrix_p12;
    wire [DATA_WIDTH-1:0]   matrix_p13;
    wire [DATA_WIDTH-1:0]   matrix_p14;
    wire [DATA_WIDTH-1:0]   matrix_p15;
    wire [DATA_WIDTH-1:0]   matrix_p21;
    wire [DATA_WIDTH-1:0]   matrix_p22;
    wire [DATA_WIDTH-1:0]   matrix_p23;
    wire [DATA_WIDTH-1:0]   matrix_p24;
    wire [DATA_WIDTH-1:0]   matrix_p25;
    wire [DATA_WIDTH-1:0]   matrix_p31;
    wire [DATA_WIDTH-1:0]   matrix_p32;
    wire [DATA_WIDTH-1:0]   matrix_p33;
    wire [DATA_WIDTH-1:0]   matrix_p34;
    wire [DATA_WIDTH-1:0]   matrix_p35;
    wire [DATA_WIDTH-1:0]   matrix_p41;
    wire [DATA_WIDTH-1:0]   matrix_p42;
    wire [DATA_WIDTH-1:0]   matrix_p43;
    wire [DATA_WIDTH-1:0]   matrix_p44;
    wire [DATA_WIDTH-1:0]   matrix_p45;
    wire [DATA_WIDTH-1:0]   matrix_p51;
    wire [DATA_WIDTH-1:0]   matrix_p52;
    wire [DATA_WIDTH-1:0]   matrix_p53;
    wire [DATA_WIDTH-1:0]   matrix_p54;
    wire [DATA_WIDTH-1:0]   matrix_p55;

    integer row;
    integer col;
    integer pixel_value;
    integer cycle_cnt;

    vip_matrix_generate_5x5_8bit #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH     (DEPTH)
    ) dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .per_frame_vsync   (per_frame_vsync),
        .per_frame_href    (per_frame_href),
        .per_frame_clken   (per_frame_clken),
        .per_img_y         (per_img_y),
        .matrix_frame_vsync(matrix_frame_vsync),
        .matrix_frame_href (matrix_frame_href),
        .matrix_frame_clken(matrix_frame_clken),
        .matrix_p11        (matrix_p11),
        .matrix_p12        (matrix_p12),
        .matrix_p13        (matrix_p13),
        .matrix_p14        (matrix_p14),
        .matrix_p15        (matrix_p15),
        .matrix_p21        (matrix_p21),
        .matrix_p22        (matrix_p22),
        .matrix_p23        (matrix_p23),
        .matrix_p24        (matrix_p24),
        .matrix_p25        (matrix_p25),
        .matrix_p31        (matrix_p31),
        .matrix_p32        (matrix_p32),
        .matrix_p33        (matrix_p33),
        .matrix_p34        (matrix_p34),
        .matrix_p35        (matrix_p35),
        .matrix_p41        (matrix_p41),
        .matrix_p42        (matrix_p42),
        .matrix_p43        (matrix_p43),
        .matrix_p44        (matrix_p44),
        .matrix_p45        (matrix_p45),
        .matrix_p51        (matrix_p51),
        .matrix_p52        (matrix_p52),
        .matrix_p53        (matrix_p53),
        .matrix_p54        (matrix_p54),
        .matrix_p55        (matrix_p55)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst_n           = 1'b0;
        per_frame_vsync = 1'b0;
        per_frame_href  = 1'b0;
        per_frame_clken = 1'b0;
        per_img_y       = {DATA_WIDTH{1'b0}};
        cycle_cnt       = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        repeat (2) @(posedge clk);
        send_one_frame;
        repeat (8) @(posedge clk);

        $display("Simulation finished at %0t.", $time);
        $finish;
    end

    always @(posedge clk) begin
        cycle_cnt <= cycle_cnt + 1;
        $display("cycle=%0d time=%0t in[vsync=%0b href=%0b clken=%0b pix=%0d] out[vsync=%0b href=%0b clken=%0b]",
                 cycle_cnt, $time, per_frame_vsync, per_frame_href, per_frame_clken, per_img_y,
                 matrix_frame_vsync, matrix_frame_href, matrix_frame_clken);
        $display("    [%0d %0d %0d %0d %0d]", matrix_p11, matrix_p12, matrix_p13, matrix_p14, matrix_p15);
        $display("    [%0d %0d %0d %0d %0d]", matrix_p21, matrix_p22, matrix_p23, matrix_p24, matrix_p25);
        $display("    [%0d %0d %0d %0d %0d]", matrix_p31, matrix_p32, matrix_p33, matrix_p34, matrix_p35);
        $display("    [%0d %0d %0d %0d %0d]", matrix_p41, matrix_p42, matrix_p43, matrix_p44, matrix_p45);
        $display("    [%0d %0d %0d %0d %0d]", matrix_p51, matrix_p52, matrix_p53, matrix_p54, matrix_p55);
    end

    task send_one_frame;
        begin
            @(posedge clk);
            per_frame_vsync <= 1'b1;

            for (row = 0; row < IMG_H; row = row + 1) begin
                @(posedge clk);
                per_frame_href  <= 1'b1;
                per_frame_clken <= 1'b1;

                for (col = 0; col < IMG_W; col = col + 1) begin
                    pixel_value <= row * IMG_W + col;
                    per_img_y   <= row * IMG_W + col;
                    @(posedge clk);
                end

                per_frame_href  <= 1'b0;
                per_frame_clken <= 1'b0;
                per_img_y       <= {DATA_WIDTH{1'b0}};
                @(posedge clk);
            end

            @(posedge clk);
            per_frame_vsync <= 1'b0;
            per_frame_href  <= 1'b0;
            per_frame_clken <= 1'b0;
            per_img_y       <= {DATA_WIDTH{1'b0}};
        end
    endtask

endmodule
