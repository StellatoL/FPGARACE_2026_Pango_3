// Selects between the raw source and the preprocessing/box result.
// The asynchronous I2C request is synchronized into the selected pixel-clock
// domain. A change waits for the current path's next frame boundary, blanks
// the hand-over interval, and starts the target path on its next frame boundary.
module video_process_selector (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        process_enable_req,

    input  wire        raw_vsync,
    input  wire        raw_href,
    input  wire        raw_de,
    input  wire [15:0] raw_rgb565,

    input  wire        processed_vsync,
    input  wire        processed_href,
    input  wire        processed_de,
    input  wire [15:0] processed_rgb565,

    output wire        out_vsync,
    output wire        out_href,
    output wire        out_de,
    output wire [15:0] out_rgb565,
    output wire        process_enable_active,
    output wire        processing_pipeline_enable
);

reg process_enable_meta;
reg process_enable_sync;
reg process_enable_frame;
reg raw_vsync_d;
reg processed_vsync_d;
reg switch_wait;

wire raw_vsync_pos       = raw_vsync       & ~raw_vsync_d;
wire processed_vsync_pos = processed_vsync & ~processed_vsync_d;

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        process_enable_meta  <= 1'b0;
        process_enable_sync  <= 1'b0;
        process_enable_frame <= 1'b0;
        raw_vsync_d          <= 1'b0;
        processed_vsync_d    <= 1'b0;
        switch_wait          <= 1'b0;
    end else begin
        process_enable_meta <= process_enable_req;
        process_enable_sync <= process_enable_meta;
        raw_vsync_d         <= raw_vsync;
        processed_vsync_d   <= processed_vsync;

        if (!switch_wait) begin
            if (process_enable_sync != process_enable_frame) begin
                if ((!process_enable_frame && raw_vsync_pos) ||
                    ( process_enable_frame && processed_vsync_pos))
                    switch_wait <= 1'b1;
            end
        end else if ((!process_enable_sync && raw_vsync_pos) ||
                     ( process_enable_sync && processed_vsync_pos)) begin
            process_enable_frame <= process_enable_sync;
            switch_wait          <= 1'b0;
        end
    end
end

assign out_vsync  = switch_wait ? 1'b0 : (process_enable_frame ? processed_vsync  : raw_vsync);
assign out_href   = switch_wait ? 1'b0 : (process_enable_frame ? processed_href   : raw_href);
assign out_de     = switch_wait ? 1'b0 : (process_enable_frame ? processed_de     : raw_de);
assign out_rgb565 = switch_wait ? 16'd0 : (process_enable_frame ? processed_rgb565 : raw_rgb565);
assign process_enable_active = process_enable_frame;
// Release the complete processing pipeline before switching to it, and keep
// it alive until the output has safely returned to the raw selector path.
assign processing_pipeline_enable = process_enable_sync | process_enable_frame;

endmodule
