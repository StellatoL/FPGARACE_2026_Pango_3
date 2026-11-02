// 单一读会话：mode=0 顺序读完整帧，mode=1 坐标读。模式仅在空闲时生效。
// 请求接受时锁存通道/槽/帧序号，直至末像素或响应被消费才解锁。
// 坐标请求之间可以换帧；多坐标同帧的批量会话需由后续模块扩展。
// 所有接口属于 core_clk；输出不是 HDMI 固定像素时序。
module cds_reader #(
    parameter TAG_WIDTH = 16
)(
    input wire clk,
    input wire rst_n,
    input wire mode,
    input wire seq_valid,
    output wire seq_ready,
    input wire [1:0] source_sel,
    output reg seq_done,
    output reg seq_error,
    output wire pixel_valid,
    input wire pixel_ready,
    output wire [15:0] pixel,
    output wire pixel_sof,
    output wire pixel_eol,
    output wire pixel_eof,
    input wire req_valid,
    output wire req_ready,
    input wire [1:0] camera_id,
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [TAG_WIDTH-1:0] req_tag,
    output wire rsp_valid,
    input wire rsp_ready,
    output reg [15:0] rsp_pixel,
    output reg [TAG_WIDTH-1:0] rsp_tag,
    output reg rsp_error,
    output reg [31:0] selected_sequence,
    input wire [3:0] latest_valid,
    input wire [3:0] latest_slot,
    input wire [127:0] frame_sequences,
    output wire [3:0] acquire,
    output wire [3:0] release_slot,
    output wire mem_req,
    output reg [27:0] mem_addr,
    output wire [3:0] mem_len,
    input wire mem_done,
    input wire mem_error,
    input wire [2047:0] mem_data
);
    localparam IDLE = 3'd0;
    localparam REQUEST = 3'd1;
    localparam STREAM = 3'd2;
    localparam RESPONSE = 3'd3;
    reg [2:0] state;
    reg random_mode;
    reg locked;
    reg [1:0] camera;
    reg [2:0] lane;
    reg [6:0] buffer_index;
    reg [19:0] pixel_offset;
    reg [10:0] column;
    reg [2047:0] buffer_data;
    wire take_seq = seq_valid && seq_ready;
    wire take_random = req_valid && req_ready;
    wire [1:0] chosen_camera = take_random ? camera_id : source_sel;
    wire coordinates_ok = x < 11'd1280 && y < 10'd720;
    wire can_lock = latest_valid[chosen_camera] && (!take_random || coordinates_ok);
    wire [27:0] chosen_base = {5'd0, chosen_camera, latest_slot[chosen_camera], 20'd0};
    wire [27:0] row_offset = ({18'd0, y} << 10) + ({18'd0, y} << 8);
    wire release_now = locked &&
        ((state == RESPONSE && rsp_ready) ||
         (state == STREAM && pixel_ready && pixel_eof) ||
         (state == REQUEST && mem_done && mem_error && !random_mode));
    assign seq_ready = state == IDLE && !mode;
    assign req_ready = state == IDLE && mode;
    assign acquire = (take_seq || take_random) && can_lock ?
                     (4'b0001 << chosen_camera) : 4'b0000;
    assign release_slot = release_now ? (4'b0001 << camera) : 4'b0000;
    assign mem_req = state == REQUEST;
    assign mem_len = random_mode ? 4'd0 : 4'd15;
    assign pixel_valid = state == STREAM;
    assign pixel = buffer_data[buffer_index*16 +: 16];
    assign pixel_sof = pixel_offset == 20'd0;
    assign pixel_eol = column == 11'd1279;
    assign pixel_eof = pixel_offset == 20'd921599;
    assign rsp_valid = state == RESPONSE;

    always @(posedge clk) begin
        if (state == REQUEST && mem_done) begin
            buffer_data <= mem_data;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            random_mode <= 1'b0;
            locked <= 1'b0;
            camera <= 2'd0;
            lane <= 3'd0;
            buffer_index <= 7'd0;
            pixel_offset <= 20'd0;
            column <= 11'd0;
            mem_addr <= 28'd0;
            seq_done <= 1'b0;
            seq_error <= 1'b0;
            rsp_pixel <= 16'd0;
            rsp_tag <= 0;
            rsp_error <= 1'b0;
            selected_sequence <= 32'd0;
        end else begin
            seq_done <= 1'b0;
            seq_error <= 1'b0;
            if (release_now) begin
                locked <= 1'b0;
            end
            case (state)
                IDLE: begin
                    if (take_seq || take_random) begin
                        camera <= chosen_camera;
                        random_mode <= take_random;
                        locked <= can_lock;
                        selected_sequence <= can_lock ?
                            frame_sequences[chosen_camera*32 +: 32] : 32'd0;
                        pixel_offset <= 20'd0;
                        column <= 11'd0;
                        buffer_index <= 7'd0;
                        lane <= x[2:0];
                        rsp_tag <= req_tag;
                        rsp_pixel <= 16'd0;
                        rsp_error <= !can_lock;
                        mem_addr <= chosen_base + (take_random ?
                            row_offset + {17'd0, x[10:3], 3'd0} : 28'd0);
                        if (can_lock) begin
                            state <= REQUEST;
                        end else if (take_random) begin
                            state <= RESPONSE;
                        end else begin
                            seq_done <= 1'b1;
                            seq_error <= 1'b1;
                        end
                    end
                end
                REQUEST: begin
                    if (mem_done) begin
                        if (random_mode) begin
                            rsp_pixel <= mem_error ? 16'd0 : mem_data[lane*16 +: 16];
                            rsp_error <= mem_error;
                            state <= RESPONSE;
                        end else if (mem_error) begin
                            seq_done <= 1'b1;
                            seq_error <= 1'b1;
                            state <= IDLE;
                        end else begin
                            buffer_index <= 7'd0;
                            state <= STREAM;
                        end
                    end
                end
                STREAM: begin
                    if (pixel_ready) begin
                        pixel_offset <= pixel_offset + 1'b1;
                        column <= pixel_eol ? 11'd0 : column + 1'b1;
                        buffer_index <= buffer_index + 1'b1;
                        if (pixel_eof) begin
                            seq_done <= 1'b1;
                            state <= IDLE;
                        end else if (buffer_index == 7'd127) begin
                            mem_addr <= mem_addr + 28'd128;
                            state <= REQUEST;
                        end
                    end
                end
                RESPONSE: begin
                    if (rsp_ready) begin
                        state <= IDLE;
                    end
                end
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
