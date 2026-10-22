`timescale 1ns/1ns

module VIP_projection_top #(
    parameter IMG_HDISP = 1600,
    parameter IMG_VDISP = 1200,
    parameter DATA_WIDTH = $clog2((IMG_HDISP > IMG_VDISP) ? IMG_HDISP : IMG_VDISP), // $clog2(1600)
    
    // Core Parameter: Max concurrent license plates supported
    parameter NUM_CHANNELS = 4, 
    
    // Passthrough Thresholds
    parameter H_MIN_HEIGHT = 20,
    parameter H_THRESHOLD  = 45,
    parameter V_THRESHOLD  = 15
)(
    input                       clk,
    input                       rst_n,

    // Raw input video stream
    input                       per_frame_vsync,    
    input                       per_frame_href,     
    input                       per_frame_clken,    
    input                       per_img_Bit,        
    
    // Delayed sync video stream output (aligned with bounding box modules)
    output                      post_frame_vsync,   
    output                      post_frame_href,    
    output                      post_frame_clken,   
    output                      post_img_Bit,       
    
    // Top-level Aggregation FIFO (Contains complete Cross Coordinates)
    input                       top_fifo_rd_en,
    // [43:33] X_start, [32:22] X_end, [21:11] Y_start, [10:0] Y_end
    output [4*DATA_WIDTH-1:0]   top_fifo_rd_data, 
    output                      top_fifo_empty
);

localparam CH_BITS = (NUM_CHANNELS <= 2) ? 1 : $clog2(NUM_CHANNELS);

//====================================================================
// 1. Horizontal Projection Instantiation (Frame N computes Y coords)
//====================================================================
wire                      h2v_vsync, h2v_href, h2v_clken, h2v_img_Bit;
wire                      h_fifo_rd_en;
wire [2*DATA_WIDTH-1:0]   h_fifo_rd_data;
wire                      h_fifo_empty;

VIP_horizon_projection_mult #(
    .IMG_HDISP(IMG_HDISP), .IMG_VDISP(IMG_VDISP), .DATA_WIDTH(DATA_WIDTH),
    .MIN_HEIGHT(H_MIN_HEIGHT), .PROJ_THRESHOLD(H_THRESHOLD)
) u_horizon_proj (
    .clk                (clk),
    .rst_n              (rst_n),
    .per_frame_vsync    (per_frame_vsync),
    .per_frame_href     (per_frame_href),
    .per_frame_clken    (per_frame_clken),
    .per_img_Bit        (per_img_Bit),
    
    // Seamlessly pass video stream to vertical projection
    .post_frame_vsync   (h2v_vsync),
    .post_frame_href    (h2v_href),
    .post_frame_clken   (h2v_clken),
    .post_img_Bit       (h2v_img_Bit),
    
    .horizon_start      ({DATA_WIDTH{1'b0}}), // Scan full screen width
    .horizon_end        (IMG_HDISP - 1),
    
    .fifo_rd_en         (h_fifo_rd_en),
    .fifo_rd_data       (h_fifo_rd_data),
    .fifo_empty         (h_fifo_empty)
);

//====================================================================
// 2. Channel Allocation FSM (Distribute H_FIFO data to channels)
//====================================================================
reg [DATA_WIDTH-1:0] roi_y_st [0:NUM_CHANNELS-1];
reg [DATA_WIDTH-1:0] roi_y_ed [0:NUM_CHANNELS-1];
reg                  roi_valid[0:NUM_CHANNELS-1];

reg [CH_BITS-1:0] ch_alloc_cnt;
reg [CH_BITS:0]   read_issued_cnt; // Tracks issued reads to prevent overflow
reg       h_rd_en_reg;
reg       h_rd_en_d1;

assign h_fifo_rd_en = h_rd_en_reg;

// Extract VSYNC Positive Edge (True start of a new frame)
reg h2v_vsync_d1;
always @(posedge clk) h2v_vsync_d1 <= h2v_vsync;
wire vsync_pos_edge = h2v_vsync & ~h2v_vsync_d1;

localparam ALLOC_IDLE = 1'b0;
localparam ALLOC_LOAD = 1'b1;
reg alloc_state;

integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        alloc_state     <= ALLOC_IDLE;
        ch_alloc_cnt    <= 0;
        read_issued_cnt <= 0;
        h_rd_en_reg     <= 0;
        h_rd_en_d1      <= 0;
        for (i=0; i<NUM_CHANNELS; i=i+1) begin
            roi_y_st[i] <= 0; roi_y_ed[i] <= 0; roi_valid[i] <= 0;
        end
    end else begin
        h_rd_en_d1 <= h_rd_en_reg;
        
        // Delay 1 cycle to safely capture FIFO data
        if (h_rd_en_d1) begin
            roi_y_st[ch_alloc_cnt]  <= h_fifo_rd_data[2*DATA_WIDTH-1 : DATA_WIDTH];
            roi_y_ed[ch_alloc_cnt]  <= h_fifo_rd_data[DATA_WIDTH-1 : 0];
            roi_valid[ch_alloc_cnt] <= 1'b1;
            ch_alloc_cnt <= ch_alloc_cnt + 1'b1;
        end
        
        case (alloc_state)
            ALLOC_IDLE: begin
                h_rd_en_reg <= 1'b0;
                // On the rising edge of VSYNC, clear old ROIs and start loading new ones
                if (vsync_pos_edge) begin
                    ch_alloc_cnt    <= 0;
                    read_issued_cnt <= 0;
                    for (i=0; i<NUM_CHANNELS; i=i+1) roi_valid[i] <= 0;
                    alloc_state <= ALLOC_LOAD;
                end
            end
            
            ALLOC_LOAD: begin
                // If FIFO has data, and we haven't reached channel limit, issue a read
                if (!h_fifo_empty && !h_rd_en_reg && read_issued_cnt < NUM_CHANNELS) begin
                    h_rd_en_reg <= 1'b1;
                    read_issued_cnt <= read_issued_cnt + 1'b1;
                end else begin
                    h_rd_en_reg <= 1'b0;
                    // Exit load state when FIFO is empty or channels are full (ensure pending reads finish)
                    if ((h_fifo_empty || read_issued_cnt == NUM_CHANNELS) && !h_rd_en_reg && !h_rd_en_d1) begin
                        alloc_state <= ALLOC_IDLE;
                    end
                end
            end
        endcase
    end
end

//====================================================================
// 3. Vertical Projection Concurrent Array (Generated instances)
//====================================================================
wire                      v_fifo_rd_en [0:NUM_CHANNELS-1];
wire [2*DATA_WIDTH-1:0]   v_fifo_rd_data [0:NUM_CHANNELS-1];
wire                      v_fifo_empty [0:NUM_CHANNELS-1];

wire v_post_vsync [0:NUM_CHANNELS-1];
wire v_post_href  [0:NUM_CHANNELS-1];
wire v_post_clken [0:NUM_CHANNELS-1];
wire v_post_img   [0:NUM_CHANNELS-1];

genvar gen_i;
generate
    for (gen_i = 0; gen_i < NUM_CHANNELS; gen_i = gen_i + 1) begin : V_PROJ_CH
        VIP_vertical_projection_mult #(
            .IMG_HDISP(IMG_HDISP), .IMG_VDISP(IMG_VDISP), 
            .X_BITS(DATA_WIDTH), .Y_BITS(DATA_WIDTH),
            .PROJ_THRESHOLD(V_THRESHOLD)
        ) u_vertical_proj (
            .clk                (clk),
            .rst_n              (rst_n),
            // All vertical modules share the same video bus from horizontal module
            .per_frame_vsync    (h2v_vsync),
            .per_frame_href     (h2v_href),
            .per_frame_clken    (h2v_clken),
            .per_img_Bit        (h2v_img_Bit),
            
            .post_frame_vsync   (v_post_vsync[gen_i]),
            .post_frame_href    (v_post_href[gen_i]),
            .post_frame_clken   (v_post_clken[gen_i]),
            .post_img_Bit       (v_post_img[gen_i]),
            
            // Dynamically assigned ROI (Pass 0 if channel is invalid)
            .vertical_start     (roi_valid[gen_i] ? roi_y_st[gen_i] : {DATA_WIDTH{1'b0}}),
            .vertical_end       (roi_valid[gen_i] ? roi_y_ed[gen_i] : {DATA_WIDTH{1'b0}}),
            
            .fifo_rd_en         (v_fifo_rd_en[gen_i]),
            .fifo_rd_data       (v_fifo_rd_data[gen_i]),
            .fifo_empty         (v_fifo_empty[gen_i])
        );
    end
endgenerate

// Output video stream (Only need signals from CH0 since all delays are identical)
assign post_frame_vsync = v_post_vsync[0];
assign post_frame_href  = v_post_href[0];
assign post_frame_clken = v_post_clken[0];
assign post_img_Bit     = v_post_img[0];

//====================================================================
// 4. Aggregation FSM (Poll all V_FIFOs and merge into Top FIFO)
//====================================================================
reg [CH_BITS-1:0] poll_idx;
reg [1:0] poll_step; 
reg       top_wr_en;
reg [4*DATA_WIDTH-1:0] top_wr_data;

wire top_fifo_full;

reg v_fifo_rd_en_reg [0:NUM_CHANNELS-1];

generate
    for (gen_i = 0; gen_i < NUM_CHANNELS; gen_i = gen_i + 1) begin : V_FIFO_RD_BIND
        assign v_fifo_rd_en[gen_i] = v_fifo_rd_en_reg[gen_i];
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        poll_idx  <= 0;
        poll_step <= 0;
        top_wr_en <= 0;
        top_wr_data <= 0;
        for(i=0; i<NUM_CHANNELS; i=i+1) v_fifo_rd_en_reg[i] <= 0;
    end else begin
        top_wr_en <= 0;
        case (poll_step)
            0: begin
                // If the polled V_FIFO is not empty and channel is valid, issue read
                if (!v_fifo_empty[poll_idx] && roi_valid[poll_idx] && !top_fifo_full) begin
                    v_fifo_rd_en_reg[poll_idx] <= 1'b1;
                    poll_step <= 1;
                end else begin
                    // Otherwise switch to the next channel
                    if (poll_idx == NUM_CHANNELS - 1) poll_idx <= 0;
                    else poll_idx <= poll_idx + 1;
                end
            end
            1: begin
                // Close read enable, wait 1 cycle for data
                v_fifo_rd_en_reg[poll_idx] <= 1'b0;
                poll_step <= 2;
            end
            2: begin
                // Data is ready, concatenate X and Y coords and push to TOP_FIFO
                top_wr_en   <= 1'b1;
                top_wr_data <= {v_fifo_rd_data[poll_idx], roi_y_st[poll_idx], roi_y_ed[poll_idx]};
                // Return to step 0 to check if this channel has more boxes
                poll_step   <= 0;
            end
        endcase
    end
end

//====================================================================
// 5. Final Coordinates Output FIFO
//====================================================================
fifo_ram #(
    .DATA_WIDTH ( 4 * DATA_WIDTH ), // {X_start, X_end, Y_start, Y_end}
    .DATA_DEPTH ( 32 )              
) u_top_coord_fifo (
    .clk        ( clk ),
    .wr_en      ( top_wr_en ),
    .wr_data    ( top_wr_data ),
    .wr_full    ( top_fifo_full ),
    .rd_en      ( top_fifo_rd_en ),
    .rd_data    ( top_fifo_rd_data ),
    .rd_empty   ( top_fifo_empty )
);

endmodule
