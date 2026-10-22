`timescale 1ns/1ns

module vdp_box_draw #(
    parameter IMG_HDISP    = 1280,
    parameter IMG_VDISP    = 720,
    parameter DATA_WIDTH   = $clog2((IMG_HDISP > IMG_VDISP) ? IMG_HDISP : IMG_VDISP),
    parameter MAX_BOXES    = 8,
    parameter BORDER_WIDTH = 2,
    parameter BOX_RGB      = 24'hFF0000
)(
    input                           clk,
    input                           rst_n,

    input                           pre_frame_vsync,
    input                           pre_frame_href,
    input                           pre_frame_clken,
    input        [23:0]             pre_rgb,

    input                           license_plate_exist_flag,
    input        [3:0]              license_plate_count,
    input        [4*DATA_WIDTH-1:0] license_plate_data,
    output                          license_read_en,
    output       [3:0]              license_read_addr,

    output reg                      post_frame_vsync,
    output reg                      post_frame_href,
    output reg                      post_frame_clken,
    output reg   [23:0]             post_rgb
);

localparam [3:0] BOX_LIMIT = (MAX_BOXES > 15) ? 4'd15 : MAX_BOXES;

localparam FETCH_IDLE    = 2'd0;
localparam FETCH_ISSUE   = 2'd1;
localparam FETCH_WAIT    = 2'd2;
localparam FETCH_CAPTURE = 2'd3;

reg [1:0] fetch_state;
reg [3:0] fetch_idx;
reg [3:0] fetch_limit;
reg [3:0] active_box_count;
reg       read_en_reg;

assign license_read_en   = read_en_reg;
assign license_read_addr = fetch_idx;

reg [DATA_WIDTH-1:0] box_x_min [0:MAX_BOXES-1];
reg [DATA_WIDTH-1:0] box_x_max [0:MAX_BOXES-1];
reg [DATA_WIDTH-1:0] box_y_min [0:MAX_BOXES-1];
reg [DATA_WIDTH-1:0] box_y_max [0:MAX_BOXES-1];

reg vsync_d1;
reg href_d1;

wire vsync_pos = pre_frame_vsync & ~vsync_d1;
wire href_neg  = ~pre_frame_href & href_d1;

integer init_i;
integer hit_i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_d1 <= 1'b0;
        href_d1  <= 1'b0;
    end else begin
        vsync_d1 <= pre_frame_vsync;
        href_d1  <= pre_frame_href;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fetch_state      <= FETCH_IDLE;
        fetch_idx        <= 4'd0;
        fetch_limit      <= 4'd0;
        active_box_count <= 4'd0;
        read_en_reg      <= 1'b0;
        for (init_i = 0; init_i < MAX_BOXES; init_i = init_i + 1) begin
            box_x_min[init_i] <= {DATA_WIDTH{1'b0}};
            box_x_max[init_i] <= {DATA_WIDTH{1'b0}};
            box_y_min[init_i] <= {DATA_WIDTH{1'b0}};
            box_y_max[init_i] <= {DATA_WIDTH{1'b0}};
        end
    end else begin
        case (fetch_state)
            FETCH_IDLE: begin
                read_en_reg <= 1'b0;
                if (vsync_pos) begin
                    if (license_plate_exist_flag && (license_plate_count != 4'd0)) begin
                        fetch_idx   <= 4'd0;
                        fetch_limit <= (license_plate_count > BOX_LIMIT) ? BOX_LIMIT : license_plate_count;
                        fetch_state <= FETCH_ISSUE;
                    end else begin
                        fetch_limit      <= 4'd0;
                        active_box_count <= 4'd0;
                    end
                end
            end

            FETCH_ISSUE: begin
                read_en_reg <= 1'b1;
                fetch_state <= FETCH_WAIT;
            end

            FETCH_WAIT: begin
                read_en_reg <= 1'b0;
                fetch_state <= FETCH_CAPTURE;
            end

            FETCH_CAPTURE: begin
                box_x_min[fetch_idx] <= license_plate_data[4*DATA_WIDTH-1:3*DATA_WIDTH];
                box_x_max[fetch_idx] <= license_plate_data[3*DATA_WIDTH-1:2*DATA_WIDTH];
                box_y_min[fetch_idx] <= license_plate_data[2*DATA_WIDTH-1:  DATA_WIDTH];
                box_y_max[fetch_idx] <= license_plate_data[  DATA_WIDTH-1:0];

                if ((fetch_idx + 1'b1) < fetch_limit) begin
                    fetch_idx   <= fetch_idx + 1'b1;
                    fetch_state <= FETCH_ISSUE;
                end else begin
                    active_box_count <= fetch_limit;
                    fetch_state      <= FETCH_IDLE;
                end
            end

            default: begin
                read_en_reg <= 1'b0;
                fetch_state <= FETCH_IDLE;
            end
        endcase
    end
end

reg [DATA_WIDTH-1:0] x_cnt;
reg [DATA_WIDTH-1:0] y_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_cnt <= {DATA_WIDTH{1'b0}};
        y_cnt <= {DATA_WIDTH{1'b0}};
    end else if (vsync_pos) begin
        x_cnt <= {DATA_WIDTH{1'b0}};
        y_cnt <= {DATA_WIDTH{1'b0}};
    end else if (href_neg) begin
        x_cnt <= {DATA_WIDTH{1'b0}};
        if (y_cnt == IMG_VDISP - 1)
            y_cnt <= {DATA_WIDTH{1'b0}};
        else
            y_cnt <= y_cnt + 1'b1;
    end else if (pre_frame_clken) begin
        if (x_cnt == IMG_HDISP - 1)
            x_cnt <= {DATA_WIDTH{1'b0}};
        else
            x_cnt <= x_cnt + 1'b1;
    end
end

reg draw_hit;

always @(*) begin
    draw_hit = 1'b0;
    for (hit_i = 0; hit_i < MAX_BOXES; hit_i = hit_i + 1) begin
        if ((hit_i < active_box_count) &&
            (x_cnt >= box_x_min[hit_i]) && (x_cnt <= box_x_max[hit_i]) &&
            (y_cnt >= box_y_min[hit_i]) && (y_cnt <= box_y_max[hit_i])) begin
            if (((x_cnt - box_x_min[hit_i]) < BORDER_WIDTH) ||
                ((box_x_max[hit_i] - x_cnt) < BORDER_WIDTH) ||
                ((y_cnt - box_y_min[hit_i]) < BORDER_WIDTH) ||
                ((box_y_max[hit_i] - y_cnt) < BORDER_WIDTH)) begin
                draw_hit = 1'b1;
            end
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        post_frame_vsync <= 1'b0;
        post_frame_href  <= 1'b0;
        post_frame_clken <= 1'b0;
        post_rgb         <= 24'd0;
    end else begin
        post_frame_vsync <= pre_frame_vsync;
        post_frame_href  <= pre_frame_href;
        post_frame_clken <= pre_frame_clken;
        post_rgb         <= (pre_frame_clken && draw_hit) ? BOX_RGB : pre_rgb;
    end
end

endmodule
