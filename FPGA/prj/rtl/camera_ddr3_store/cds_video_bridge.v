// core_clk 可背压像素流 -> video_clk 固定光栅，固定 1280x720。
// 两域共用 reset_n，内部同步释放。每个 VS 上升沿最多请求一帧。
// 相机选择作为 bundled-data 与请求 toggle 传递，在本帧结束前保持不变。
// FIFO 带 SOF/EOL/EOF 和 abort 标记；欠流后本帧余下输出黑色并排空到帧尾，
// 避免后续像素错位。没有完整帧或未赶上本次扫描时输出黑帧。
// FIFO_ADDR_BITS >= 2，默认 4096 像素缓冲。不能承诺任意 DDR 停顿下无欠流。
module cds_video_bridge #(
    parameter FIFO_ADDR_BITS = 12
)(
    input wire core_clk,
    input wire video_clk,
    input wire reset_n,
    input wire [2:0] mode_pending,
    output reg [2:0] frame_mode,
    output reg [2:0] active_mode,
    input wire raster_vs,
    input wire raster_de,
    input wire [11:0] raster_x,
    input wire [11:0] raster_y,
    output reg video_vs,
    output reg video_de,
    output reg [15:0] video_pixel,
    output reg underflow_sticky,
    output reg seq_valid,
    input wire seq_ready,
    output reg [1:0] source_sel,
    input wire seq_done,
    input wire seq_error,
    input wire pixel_valid,
    output wire pixel_ready,
    input wire [15:0] pixel,
    input wire pixel_sof,
    input wire pixel_eol,
    input wire pixel_eof,
    // video_clk 域诊断，计数器按 32 位回绕；abort 同时包含无帧和 DDR 读错误。
    output reg [31:0] abort_count,
    output reg [31:0] fifo_empty_count,
    output reg [31:0] marker_error_count,
    output reg [31:0] drain_frame_count,
    // 首次异常快照：{有效, state[1:0], FIFO有效, abort/EOF/EOL/SOF, x[11:0], y[11:0]}。
    output reg [31:0] first_fault
);
    localparam IDLE = 2'd0;
    localparam WAIT_FRAME = 2'd1;
    localparam RUN = 2'd2;
    localparam DRAIN = 2'd3;
    (* ASYNC_REG = "TRUE" *) reg [1:0] core_reset_pipe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] video_reset_pipe;
    wire core_rst_n = core_reset_pipe[1];
    wire video_rst_n = video_reset_pipe[1];
    reg request_toggle;
    reg [2:0] held_mode;
    reg complete_toggle;
    (* ASYNC_REG="TRUE" *) reg [1:0] complete_sync;
    reg session_active;
    reg session_toggle;
    (* ASYNC_REG = "TRUE" *) reg request_meta;
    (* ASYNC_REG = "TRUE" *) reg request_sync;
    reg request_seen;
    reg abort_pending;
    wire fifo_wr_ready;
    wire fifo_rd_valid;
    wire fifo_rd_ready;
    wire [19:0] fifo_rd_data;
    wire fifo_abort = fifo_rd_data[19];
    wire fifo_eof = fifo_rd_data[18];
    wire fifo_eol = fifo_rd_data[17];
    wire fifo_sof = fifo_rd_data[16];
    reg raster_vs_d;
    reg [1:0] state;
    wire frame_start = raster_vs && !raster_vs_d;
    wire first_pixel = raster_x == 12'd0 && raster_y == 12'd0;
    wire last_pixel = raster_x == 12'd1279 && raster_y == 12'd719;
    wire marker_match = !fifo_abort && fifo_sof == first_pixel &&
                        fifo_eol == (raster_x == 12'd1279) && fifo_eof == last_pixel;
    wire consuming_frame = state == WAIT_FRAME || state == RUN;
    wire abort_event = consuming_frame && fifo_rd_valid && fifo_abort &&
                       (state == WAIT_FRAME || raster_de);
    wire empty_event = consuming_frame && raster_de && !fifo_rd_valid;
    wire marker_event = consuming_frame && raster_de && fifo_rd_valid &&
                        !fifo_abort && !marker_match;

    assign pixel_ready = fifo_wr_ready && !abort_pending;
    assign fifo_rd_ready = state == DRAIN ||
                           (state == WAIT_FRAME && fifo_abort && fifo_rd_valid) ||
                           ((state == WAIT_FRAME || state == RUN) && raster_de);

    // 每次输出失败只在离开 WAIT_FRAME/RUN 时记录一次，排空时不累计假欠流。
    // DRAIN 中的 abort 也计入：DDR 读错误可能发生在输出已经欠流之后。
    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            abort_count <= 32'd0;
            fifo_empty_count <= 32'd0;
            marker_error_count <= 32'd0;
            drain_frame_count <= 32'd0;
            first_fault <= 32'd0;
        end else begin
            if (abort_event || (state == DRAIN && fifo_rd_valid && fifo_abort)) begin
                abort_count <= abort_count + 1'b1;
            end
            if (empty_event) begin
                fifo_empty_count <= fifo_empty_count + 1'b1;
            end
            if (marker_event) begin
                marker_error_count <= marker_error_count + 1'b1;
            end
            if (state == DRAIN && frame_start) begin
                drain_frame_count <= drain_frame_count + 1'b1;
            end
            if (!first_fault[31] && (abort_event || empty_event || marker_event)) begin
                first_fault <= {1'b1, state, fifo_rd_valid,
                                (fifo_rd_valid ? fifo_rd_data[19:16] : 4'd0),
                                raster_x, raster_y};
            end
        end
    end

    always @(posedge core_clk or negedge reset_n) begin
        if (!reset_n) begin
            core_reset_pipe <= 2'b00;
        end else begin
            core_reset_pipe <= {core_reset_pipe[0], 1'b1};
        end
    end
    always @(posedge video_clk or negedge reset_n) begin
        if (!reset_n) begin
            video_reset_pipe <= 2'b00;
        end else begin
            video_reset_pipe <= {video_reset_pipe[0], 1'b1};
        end
    end

    // 请求在视频帧起点产生；held_mode 在返回帧尾/abort 消费前不再改变。
    // 两级 toggle 同步后再采样该稳定总线，不对二进制选择值直接逐位同步。
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            request_meta <= 1'b0;
            request_sync <= 1'b0;
            request_seen <= 1'b0;
            seq_valid <= 1'b0;
            source_sel <= 2'd0; frame_mode<=0; complete_toggle<=0; session_active<=0; session_toggle<=0;
            abort_pending <= 1'b0;
        end else begin
            request_meta <= request_toggle;
            request_sync <= request_meta;
            if (request_sync != request_seen) begin
                request_seen <= request_sync;
                // Mode 0 uses the quad reader; modes 1..4 select cameras 0..3.
                source_sel <= held_mode>=1 && held_mode<=4 ? held_mode[1:0]-2'd1 : 2'd0;
                frame_mode<=held_mode; session_toggle<=request_sync;
                seq_valid <= 1'b1;
            end else if (seq_valid && seq_ready) begin
                seq_valid <= 1'b0; session_active<=1;
            end
            if (abort_pending && fifo_wr_ready) begin
                abort_pending <= 1'b0;
            end
            if(seq_done && session_active) begin
                complete_toggle<=session_toggle; session_active<=0;
            end
            if (seq_done && seq_error) begin
                abort_pending <= 1'b1;
            end
        end
    end

    cds_async_fifo #(
        .WIDTH(20),
        .ADDR_BITS(FIFO_ADDR_BITS)
    ) u_video_fifo (
        .wr_clk(core_clk),
        .wr_rst_n(core_rst_n),
        .wr_valid(abort_pending || pixel_valid),
        .wr_ready(fifo_wr_ready),
        .wr_data(abort_pending ? 20'h80000 :
                 {1'b0, pixel_eof, pixel_eol, pixel_sof, pixel}),
        .rd_clk(video_clk),
        .rd_rst_n(video_rst_n),
        .rd_valid(fifo_rd_valid),
        .rd_ready(fifo_rd_ready),
        .rd_data(fifo_rd_data)
    );

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            raster_vs_d <= 1'b0;
            state <= IDLE;
            held_mode <= 3'd0; active_mode<=0; complete_sync<=0;
            request_toggle <= 1'b0;
            video_vs <= 1'b0;
            video_de <= 1'b0;
            video_pixel <= 16'd0;
            underflow_sticky <= 1'b0;
        end else begin
            complete_sync<={complete_sync[0],complete_toggle};
            raster_vs_d <= raster_vs;
            video_vs <= raster_vs;
            video_de <= raster_de;
            video_pixel <= 16'd0;
            case (state)
                IDLE: begin
                    if (frame_start && complete_sync[1]==request_toggle) begin
                        held_mode <= mode_pending; active_mode<=mode_pending;
                        request_toggle <= !request_toggle;
                        state <= WAIT_FRAME;
                    end
                end
                WAIT_FRAME, RUN: begin
                    if (state == WAIT_FRAME && fifo_rd_valid && fifo_abort) begin
                        state <= IDLE;
                    end else if (raster_de) begin
                        if (fifo_rd_valid && marker_match) begin
                            video_pixel <= fifo_rd_data[15:0];
                            state <= last_pixel ? IDLE : RUN;
                        end else begin
                            underflow_sticky <= 1'b1;
                            if (fifo_rd_valid && (fifo_eof || fifo_abort)) begin
                                state <= IDLE;
                            end else begin
                                state <= DRAIN;
                            end
                        end
                    end
                end
                DRAIN: begin
                    if (fifo_rd_valid && (fifo_eof || fifo_abort)) begin
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
