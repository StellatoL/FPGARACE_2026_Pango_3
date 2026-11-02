// Four independent 1280x720 RGB565 frame slots -> one 1280x720 mosaic.
// Camera 0/1/2/3 occupy TL/TR/BL/BR. Keep even source rows and columns.
// All ports are synchronous to clk (DDR core clock). Output supports backpressure.
// Lock every available source at frame start; missing sources stay black for this frame.
module cds_quad_reader (
    input wire clk, rst_n,
    input wire seq_valid, output wire seq_ready,
    output reg seq_done, seq_error,
    output wire pixel_valid, input wire pixel_ready,
    output wire [15:0] pixel,
    output wire pixel_sof, pixel_eol, pixel_eof,
    input wire [3:0] latest_valid, latest_slot,
    output wire [3:0] acquire, release_slot,
    output wire mem_req, output wire [27:0] mem_addr,
    output wire [3:0] mem_len,
    input wire mem_done, mem_error, input wire [2047:0] mem_data
);
    localparam IDLE=2'd0, REQUEST=2'd1, STREAM=2'd2;
    reg [1:0] state;
    reg [3:0] valid_snapshot, slot_snapshot;
    reg [10:0] out_x;
    reg [9:0] out_y;
    reg [5:0] sample_index;
    reg [2047:0] buffer_data;
    wire [1:0] camera = {out_y >= 10'd360, out_x >= 11'd640};
    wire [10:0] local_x = out_x >= 11'd640 ? out_x-11'd640 : out_x;
    wire [9:0] local_y = out_y >= 10'd360 ? out_y-10'd360 : out_y;
    wire take = seq_valid && seq_ready;
    wire finish = pixel_valid && pixel_ready && pixel_eof;
    wire failed = state==REQUEST && mem_done && mem_error;
    assign seq_ready = state==IDLE;
    assign acquire = take ? latest_valid : 4'd0;
    assign release_slot = finish || failed ? valid_snapshot : 4'd0;
    assign mem_req = state==REQUEST && valid_snapshot[camera];
    assign mem_len = 4'd15;
    // Address unit: RGB565 words, NOT bytes. 2*local_y*1280 + 2*local_x.
    // REQUEST is entered only at 64-output-pixel boundaries (128 source pixels).
    assign mem_addr = {5'd0,camera,slot_snapshot[camera],20'd0}
        + ({18'd0,local_y} << 11) + ({18'd0,local_y} << 9)
        + ({17'd0,local_x} << 1);
    assign pixel_valid = state==STREAM;
    assign pixel = valid_snapshot[camera] ? buffer_data[{sample_index,5'b0} +: 16] : 16'd0;
    assign pixel_sof = out_x==0 && out_y==0;
    assign pixel_eol = out_x==11'd1279;
    assign pixel_eof = pixel_eol && out_y==10'd719;

    // Do not reset the data buffer; only STREAM with a valid source can expose it.
    always @(posedge clk) begin
        if (mem_req && mem_done && !mem_error) buffer_data <= mem_data;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; valid_snapshot<=0; slot_snapshot<=0;
            out_x<=0; out_y<=0; sample_index<=0;
            seq_done<=0; seq_error<=0;
        end else begin
            seq_done<=0; seq_error<=0;
            case (state)
                IDLE: if (take) begin
                    valid_snapshot<=latest_valid; slot_snapshot<=latest_slot;
                    out_x<=0; out_y<=0; sample_index<=0;
                    state<=REQUEST;
                end
                REQUEST: begin
                    if (!valid_snapshot[camera]) state<=STREAM;
                    else if (mem_done) begin
                        if (mem_error) begin
                            state<=IDLE; seq_done<=1; seq_error<=1;
                        end else state<=STREAM;
                    end
                end
                STREAM: if (pixel_ready) begin
                    if (finish) begin
                        state<=IDLE; seq_done<=1;
                    end else begin
                        out_x<=pixel_eol ? 11'd0 : out_x+1'b1;
                        if (pixel_eol) out_y<=out_y+1'b1;
                        sample_index<=sample_index+1'b1;
                        if (sample_index==6'd63) state<=REQUEST;
                    end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
