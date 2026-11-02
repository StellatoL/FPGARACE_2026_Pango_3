// Histogram equalization module
module histogram #(
    parameter DATA_WIDTH   = 8,
    parameter IMG_HDISP    = 1280,
    parameter IMG_VDISP    = 720,
    parameter IMG_TOTAL_PIXEL = IMG_HDISP * IMG_VDISP,
    parameter IMG_TOTAL_PIXEL_WIDTH = $clog2(IMG_TOTAL_PIXEL + 1)
) (
    input           rst_n,

    input           cam_clk,
    input           cam_vsync,
    input           cam_href,
    input           cam_valid,
    input  [DATA_WIDTH-1:0]    cam_gray,
    input  [DATA_WIDTH-1:0]    cam_cb,
    input  [DATA_WIDTH-1:0]    cam_cr,

    output          post_frame_vsync,
    output          post_frame_href,
    output          post_frame_clken,
    output [DATA_WIDTH-1:0]    post_img_Y,
    output [DATA_WIDTH-1:0]    post_img_Cb,
    output [DATA_WIDTH-1:0]    post_img_Cr,

    input           vga_clk,
    input           vga_rd_en,
    input  [DATA_WIDTH-1:0]    vga_rd_addr,
    output [IMG_TOTAL_PIXEL_WIDTH-1:0]   vga_rd_data
);

localparam DIV_XLEN = IMG_TOTAL_PIXEL_WIDTH + 8;
localparam [DIV_XLEN-1:0] DIV_STAGE_LIST = {DIV_XLEN{1'b1}};
localparam PIPELINE_LATENCY = DIV_XLEN + 3;

wire        clk;
wire        per_frame_vsync;
wire        per_frame_href;
wire        per_frame_clken;
wire [DATA_WIDTH-1:0]  per_img_Y;
wire [DATA_WIDTH-1:0]  per_img_Cb;
wire [DATA_WIDTH-1:0]  per_img_Cr;

assign clk             = vga_clk;
assign per_frame_vsync = cam_vsync;
assign per_frame_href  = cam_href;
assign per_frame_clken = cam_valid;
assign per_img_Y       = cam_gray;
assign per_img_Cb      = cam_cb;
assign per_img_Cr      = cam_cr;

wire [IMG_TOTAL_PIXEL_WIDTH-1:0] rd_data;
reg         wr_en;
reg  [DATA_WIDTH-1:0]  cam_gray_d;
reg         vsync_delay;

wire        vsync_negedge;
wire        vsync_posedge;
wire        vga_vsync_negedge;
wire        vga_vsync_posedge;
reg  [1:0]  vga_vsync_r;

reg  [DATA_WIDTH-1:0]  ram_addr_b;
reg         ram_rd_b;
wire [DATA_WIDTH-1:0]  ram_rd_addr_b;
wire [IMG_TOTAL_PIXEL_WIDTH-1:0] ram_rd_q_b;

reg         hist_clr_busy;
reg  [DATA_WIDTH-1:0]  hist_clr_addr;
wire        hist_wr_en;
wire [DATA_WIDTH-1:0]  hist_wr_addr;
wire [IMG_TOTAL_PIXEL_WIDTH-1:0] hist_wr_data;

wire        ram_his_rd_b;
wire [DATA_WIDTH-1:0]  ram_his_addr_b;
wire [IMG_TOTAL_PIXEL_WIDTH-1:0] ram_his_rd_data_b;

reg         ram_accu_wr;
reg  [DATA_WIDTH-1:0]  ram_accu_wraddr;
wire [IMG_TOTAL_PIXEL_WIDTH-1:0] ram_accu_wrdata;
wire [IMG_TOTAL_PIXEL_WIDTH-1:0] ram_accu_rddata;
reg  [1:0]  data_valid_r;

assign ram_his_rd_b      = ram_rd_b;
assign ram_his_addr_b    = ram_addr_b;
assign ram_his_rd_data_b = ram_rd_q_b;

// Camera gray data is read first, then the delayed address is written back +1.
always @(posedge cam_clk or negedge rst_n) begin
    if(!rst_n) begin
        wr_en      <= 1'b0;
        cam_gray_d <= {DATA_WIDTH{1'b0}};
    end
    else begin
        wr_en      <= cam_valid;
        cam_gray_d <= cam_gray;
    end
end

// Capture frame sync edges.
always @(posedge cam_clk or negedge rst_n) begin
    if(!rst_n)
        vsync_delay <= 1'b0;
    else
        vsync_delay <= cam_vsync;
end

assign vsync_negedge = vsync_delay & (!cam_vsync);
assign vsync_posedge = (!vsync_delay) & cam_vsync;

always @(posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        vga_vsync_r <= 2'b00;
    else
        vga_vsync_r <= {vga_vsync_r[0], cam_vsync};
end

assign vga_vsync_negedge = vga_vsync_r[1] & (!vga_vsync_r[0]);
assign vga_vsync_posedge = (!vga_vsync_r[1]) & vga_vsync_r[0];

// Clear the histogram RAM through the same write port before the next frame.
always @(posedge cam_clk or negedge rst_n) begin
    if(!rst_n) begin
        hist_clr_busy <= 1'b0;
        hist_clr_addr <= {DATA_WIDTH{1'b0}};
    end
    else begin
        if(vsync_negedge) begin
            hist_clr_busy <= 1'b1;
            hist_clr_addr <= {DATA_WIDTH{1'b0}};
        end
        else if(hist_clr_busy) begin
            hist_clr_addr <= hist_clr_addr + 1'b1;
            if(hist_clr_addr == {DATA_WIDTH{1'b1}})
                hist_clr_busy <= 1'b0;
        end
    end
end

assign hist_wr_en   = hist_clr_busy ? 1'b1                          : wr_en;
assign hist_wr_addr = hist_clr_busy ? hist_clr_addr                 : cam_gray_d;
assign hist_wr_data = hist_clr_busy ? {IMG_TOTAL_PIXEL_WIDTH{1'b0}} : rd_data + 1'b1;

// The project RAM has one write port and one read port. Two mirrored RAMs keep
// the camera read-modify-write path and the VGA/accumulation readout path apart.
dualport_ram #(
    .DATA_WIDTH (IMG_TOTAL_PIXEL_WIDTH),
    .DEPTH      ( 256 )
) u_histogram_cam_ram (
    .wr_clk  ( cam_clk ),
    .wr_rst  ( !rst_n ),
    .wr_addr ( hist_wr_addr ),
    .wr_data ( hist_wr_data ),
    .wr_en   ( hist_wr_en ),

    .rd_clk  ( cam_clk ),
    .rd_rst  ( !rst_n ),
    .rd_addr ( cam_gray ),
    .rd_data ( rd_data )
);

assign ram_rd_addr_b = vga_rd_en ? vga_rd_addr : ram_addr_b;

dualport_ram #(
    .DATA_WIDTH (IMG_TOTAL_PIXEL_WIDTH),
    .DEPTH      ( 256 )
) u_histogram_vga_ram (
    .wr_clk  ( cam_clk ),
    .wr_rst  ( !rst_n ),
    .wr_addr ( hist_wr_addr ),
    .wr_data ( hist_wr_data ),
    .wr_en   ( hist_wr_en ),

    .rd_clk  ( vga_clk ),
    .rd_rst  ( !rst_n ),
    .rd_addr ( ram_rd_addr_b ),
    .rd_data ( ram_rd_q_b )
);

// A vsync rising edge means the previous frame histogram is complete.
always @(posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        ram_rd_b <= 1'b0;
    else begin
        if(vga_vsync_posedge)
            ram_rd_b <= 1'b1;
        else if(ram_addr_b == 8'd255)
            ram_rd_b <= 1'b0;
    end
end

// Traverse the histogram RAM address space during readout.
always @(posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        ram_addr_b <= {DATA_WIDTH{1'b0}};
    else begin
        if(ram_rd_b)
            ram_addr_b <= ram_addr_b + 1'b1;
        else
            ram_addr_b <= {DATA_WIDTH{1'b0}};
    end
end

// Delay one clock after histogram RAM readout to write cumulative results.
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        ram_accu_wr <= 1'b0;
    else
        ram_accu_wr <= ram_his_rd_b;
end

reg [IMG_TOTAL_PIXEL_WIDTH-1:0] his_sum;

// Accumulate the histogram data.
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        his_sum <= {IMG_TOTAL_PIXEL_WIDTH{1'b0}};
    else begin
        if(vga_vsync_negedge)
            his_sum <= {IMG_TOTAL_PIXEL_WIDTH{1'b0}};
        else if(ram_accu_wr)
            his_sum <= his_sum + ram_his_rd_data_b;
    end
end

assign ram_accu_wrdata = his_sum + ram_his_rd_data_b;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        ram_accu_wraddr <= {DATA_WIDTH{1'b0}};
    else
        ram_accu_wraddr <= ram_his_addr_b;
end

dualport_ram #(
    .DATA_WIDTH (IMG_TOTAL_PIXEL_WIDTH),
    .DEPTH      ( 256 )
) u_histogram_accumulate_ram (
    .wr_clk  ( clk ),
    .wr_rst  ( !rst_n ),
    .wr_addr ( ram_accu_wraddr ),
    .wr_data ( ram_accu_wrdata ),
    .wr_en   ( ram_accu_wr ),

    .rd_clk  ( clk ),
    .rd_rst  ( !rst_n ),
    .rd_addr ( per_img_Y ),
    .rd_data ( ram_accu_rddata )
);

reg [DIV_XLEN-1:0] data_mult;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        data_mult <= {DIV_XLEN{1'b0}};
    else
        data_mult <= ram_accu_rddata * {DATA_WIDTH{1'b1}};
end

wire [DIV_XLEN-1:0] data_div_quo;
wire               data_div_ack;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        data_valid_r <= 2'b00;
    else
        data_valid_r <= {data_valid_r[0], per_frame_clken};
end

divider #(
    .XLEN       ( DIV_XLEN ),
    .STAGE_LIST ( DIV_STAGE_LIST )
) u_histogram_divider (
    .clk ( clk ),
    .rst ( !rst_n ),
    .a   ( data_mult ),
    .b   ( {{8{1'b0}}, IMG_TOTAL_PIXEL} ),
    .vld ( data_valid_r[1] ),
    .quo ( data_div_quo ),
    .rem ( ),
    .ack ( data_div_ack )
);

// Delay control and chroma channels to match the pipelined Y equalization path.
reg [PIPELINE_LATENCY-1:0] per_frame_vsync_r;
reg [PIPELINE_LATENCY-1:0] per_frame_href_r;
reg [PIPELINE_LATENCY-1:0] per_frame_clken_r;
reg [DATA_WIDTH*PIPELINE_LATENCY-1:0] per_img_Cb_pipe;
reg [DATA_WIDTH*PIPELINE_LATENCY-1:0] per_img_Cr_pipe;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        per_frame_vsync_r <= {PIPELINE_LATENCY{1'b0}};
        per_frame_href_r  <= {PIPELINE_LATENCY{1'b0}};
        per_frame_clken_r <= {PIPELINE_LATENCY{1'b0}};
        per_img_Cb_pipe   <= {(DATA_WIDTH*PIPELINE_LATENCY){1'b0}};
        per_img_Cr_pipe   <= {(DATA_WIDTH*PIPELINE_LATENCY){1'b0}};
    end
    else begin
        per_frame_vsync_r <= {per_frame_vsync_r[PIPELINE_LATENCY-2:0], per_frame_vsync};
        per_frame_href_r  <= {per_frame_href_r[PIPELINE_LATENCY-2:0],  per_frame_href};
        per_frame_clken_r <= {per_frame_clken_r[PIPELINE_LATENCY-2:0], per_frame_clken};
        per_img_Cb_pipe   <= {per_img_Cb_pipe[DATA_WIDTH*(PIPELINE_LATENCY-1)-1:0], per_img_Cb};
        per_img_Cr_pipe   <= {per_img_Cr_pipe[DATA_WIDTH*(PIPELINE_LATENCY-1)-1:0], per_img_Cr};
    end
end

assign post_frame_vsync = per_frame_vsync_r[PIPELINE_LATENCY-1];
assign post_frame_href  = per_frame_href_r[PIPELINE_LATENCY-1];
assign post_frame_clken = per_frame_clken_r[PIPELINE_LATENCY-1];
assign post_img_Y       = data_div_quo[7:0];
assign post_img_Cb      = per_img_Cb_pipe[DATA_WIDTH*PIPELINE_LATENCY-1 -: DATA_WIDTH];
assign post_img_Cr      = per_img_Cr_pipe[DATA_WIDTH*PIPELINE_LATENCY-1 -: DATA_WIDTH];

assign vga_rd_data      = ram_rd_q_b;

endmodule
