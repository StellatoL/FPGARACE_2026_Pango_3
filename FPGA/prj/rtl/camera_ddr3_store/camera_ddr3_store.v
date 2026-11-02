// 四路 1280x720 RGB565 双帧槽存储顶层，固定 28 位 Pango 字地址接口。
// 摄像头接口分别属于 cam_clk；所有读取/状态/DDR 接口属于 core_clk。
// reset_n 必须同时复位整个子系统和 DDR 命令上下文；释放在各域同步。
// reset_n 应包含 DDR 初始化完成条件。摄像头需连续提供时钟。
// cam_vsync 上升沿位于有效图像之前，cam_de 为原始全分辨率像素有效。
// 单路顺序读与坐标读互斥；read_mode 只在读会话空闲时生效。
module camera_ddr3_store #(
    parameter TAG_WIDTH = 16,
    // 保留以 128 像素突发计容量的参数语义；内部按 8 像素一拍，地址宽度加 4。
    parameter FIFO_ADDR_BITS = 5
)(
    input wire core_clk,
    input wire reset_n,
    input wire [3:0] cam_clk,
    input wire [3:0] cam_vsync,
    input wire [3:0] cam_de,
    input wire [63:0] cam_pixel,
    input wire [2:0] display_mode,
    input wire read_mode,
    input wire seq_valid,
    output wire seq_ready,
    input wire [1:0] source_sel,
    output wire seq_done,
    output wire seq_error,
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
    output wire [15:0] rsp_pixel,
    output wire [TAG_WIDTH-1:0] rsp_tag,
    output wire rsp_error,
    output wire [31:0] selected_sequence,
    output wire [3:0] frame_valid,
    output wire [127:0] frame_sequences,
    output wire [3:0] capture_overflow,
    output wire [3:0] write_drop,
    output wire protocol_error,
    output wire [31:0] write_beats,
    output wire [31:0] read_beats,
    output wire [31:0] busy_cycles,
    output wire [159:0] request_wait_cycles,
    output wire [27:0] axi_awaddr,
    output wire [3:0] axi_awlen,
    output wire axi_awvalid,
    input wire axi_awready,
    output wire [127:0] axi_wdata,
    output wire [15:0] axi_wstrb,
    input wire axi_wready,
    input wire axi_wlast,
    output wire [27:0] axi_araddr,
    output wire [3:0] axi_arlen,
    output wire axi_arvalid,
    input wire axi_arready,
    input wire [127:0] axi_rdata,
    input wire axi_rvalid,
    input wire axi_rlast
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] core_reset_pipe;
    wire core_rst_n = core_reset_pipe[1];
    wire [3:0] latest_slot;
    wire [3:0] acquire;
    wire [3:0] release_slot;
    wire [3:0] wr_req;
    wire [111:0] wr_addr;
    wire [8191:0] wr_data;
    wire [3:0] wr_done;
    wire transaction_error;
    wire rd_req;
    wire [27:0] rd_addr;
    wire [3:0] rd_len;
    wire rd_done;
    wire [2047:0] rd_data;

    always @(posedge core_clk or negedge reset_n) begin
        if (!reset_n) begin
            core_reset_pipe <= 2'b00;
        end else begin
            core_reset_pipe <= {core_reset_pipe[0], 1'b1};
        end
    end
    genvar channel;
    generate
        for (channel = 0; channel < 4; channel = channel + 1) begin : cameras
            localparam [27:0] BASE = channel * 28'h0200000;
            (* ASYNC_REG = "TRUE" *) reg [1:0] cam_reset_pipe;
            (* ASYNC_REG = "TRUE" *) reg overflow_meta;
            (* ASYNC_REG = "TRUE" *) reg overflow_sync;
            wire cam_rst_n = cam_reset_pipe[1];
            wire overflow;
            wire capture_valid;
            wire capture_ready;
            wire [129:0] capture_data;
            wire fifo_valid;
            wire fifo_ready;
            wire [129:0] fifo_data;
            wire slot_start;
            wire slot_grant;
            wire alloc_slot;
            wire commit;
            assign capture_overflow[channel] = overflow_sync;

            always @(posedge cam_clk[channel] or negedge reset_n) begin
                if (!reset_n) begin
                    cam_reset_pipe <= 2'b00;
                end else begin
                    cam_reset_pipe <= {cam_reset_pipe[0], 1'b1};
                end
            end
            always @(posedge core_clk or negedge core_rst_n) begin
                if (!core_rst_n) begin
                    overflow_meta <= 1'b0;
                    overflow_sync <= 1'b0;
                end else begin
                    overflow_meta <= overflow;
                    overflow_sync <= overflow_meta;
                end
            end
            cds_capture u_capture (
                .pclk(cam_clk[channel]),
                .rst_n(cam_rst_n),
                .vsync(cam_vsync[channel]),
                .de(cam_de[channel]),
                .pixel(cam_pixel[channel*16 +: 16]),
                .fifo_valid(capture_valid),
                .fifo_ready(capture_ready),
                .fifo_data(capture_data),
                .overflow_sticky(overflow)
            );
            cds_async_fifo #(
                .WIDTH(130),
                .ADDR_BITS(FIFO_ADDR_BITS + 4)
            ) u_fifo (
                .wr_clk(cam_clk[channel]),
                .wr_rst_n(cam_rst_n),
                .wr_valid(capture_valid),
                .wr_ready(capture_ready),
                .wr_data(capture_data),
                .rd_clk(core_clk),
                .rd_rst_n(core_rst_n),
                .rd_valid(fifo_valid),
                .rd_ready(fifo_ready),
                .rd_data(fifo_data)
            );
            cds_slots u_slots (
                .clk(core_clk),
                .rst_n(core_rst_n),
                .start(slot_start),
                .grant(slot_grant),
                .alloc_slot(alloc_slot),
                .commit(commit),
                .acquire(acquire[channel]),
                .release_slot(release_slot[channel]),
                .latest_valid(frame_valid[channel]),
                .latest_slot(latest_slot[channel]),
                .frame_sequence(frame_sequences[channel*32 +: 32])
            );
            cds_writer #(
                .CHANNEL_BASE(BASE)
            ) u_writer (
                .clk(core_clk),
                .rst_n(core_rst_n),
                .fifo_valid(fifo_valid),
                .fifo_ready(fifo_ready),
                .fifo_data(fifo_data),
                .slot_start(slot_start),
                .slot_grant(slot_grant),
                .alloc_slot(alloc_slot),
                .commit(commit),
                .req(wr_req[channel]),
                .addr(wr_addr[channel*28 +: 28]),
                .data(wr_data[channel*2048 +: 2048]),
                .done(wr_done[channel]),
                .error(transaction_error),
                .drop_sticky(write_drop[channel])
            );
        end
    endgenerate

    wire [2:0] route;
    wire owner_idle;
    wire old_seq_ready, old_seq_done, old_seq_error, old_pixel_valid;
    wire old_pixel_sof, old_pixel_eol, old_pixel_eof, old_req_ready, old_rsp_valid;
    wire [15:0] old_pixel;
    wire [3:0] old_acquire, old_release_slot, old_mem_len;
    wire old_mem_req;
    wire [27:0] old_mem_addr;
    wire q_ready, q_done, q_error, q_valid, q_sof, q_eol, q_eof, q_mem_req;
    wire [15:0] q_pixel;
    wire [3:0] q_acquire, q_release, q_mem_len;
    wire [27:0] q_mem_addr;
    assign seq_ready=owner_idle && (route==4 ? q_ready : route==1 && old_seq_ready);
    assign seq_done=route==4 ? q_done : route==1 && old_seq_done;
    assign seq_error=route==4 ? q_error : route==1 && old_seq_error;
    assign pixel_valid=route==4 ? q_valid : route==1 && old_pixel_valid;
    assign pixel=route==4 ? q_pixel : old_pixel;
    assign pixel_sof=route==4 ? q_sof : old_pixel_sof;
    assign pixel_eol=route==4 ? q_eol : old_pixel_eol;
    assign pixel_eof=route==4 ? q_eof : old_pixel_eof;
    assign req_ready=owner_idle && route==3 && old_req_ready;
    assign rsp_valid=route==3 && old_rsp_valid;
    assign acquire=route==4 ? q_acquire : route!=0 ? old_acquire : 4'd0;
    assign release_slot=route==4 ? q_release : route!=0 ? old_release_slot : 4'd0;
    assign rd_req=route==4 ? q_mem_req : route!=0 && old_mem_req;
    assign rd_addr=route==4 ? q_mem_addr : old_mem_addr;
    assign rd_len=route==4 ? q_mem_len : old_mem_len;
    stereo_read_mux ownership (
        .clk(core_clk),.rst_n(core_rst_n),.seq_valid(seq_valid),.seq_ready(seq_ready),.display_mode(display_mode),
        .random_mode(read_mode),.req_valid(req_valid),.req_ready(req_ready),.single_done(old_seq_done),
        .quad_done(q_done),.rsp_valid(old_rsp_valid),.rsp_ready(rsp_ready),.route(route),.idle(owner_idle));
    cds_quad_reader quad_reader (
        .clk(core_clk),.rst_n(core_rst_n),
        .seq_valid(seq_valid && owner_idle && route==4),.seq_ready(q_ready),
        .seq_done(q_done),.seq_error(q_error),
        .pixel_valid(q_valid),.pixel_ready(pixel_ready && route==4),.pixel(q_pixel),
        .pixel_sof(q_sof),.pixel_eol(q_eol),.pixel_eof(q_eof),
        .latest_valid(frame_valid),.latest_slot(latest_slot),
        .acquire(q_acquire),.release_slot(q_release),
        .mem_req(q_mem_req),.mem_addr(q_mem_addr),.mem_len(q_mem_len),
        .mem_done(rd_done && route==4),.mem_error(transaction_error),.mem_data(rd_data));
    cds_reader #(
        .TAG_WIDTH(TAG_WIDTH)
    ) u_reader (
        .clk(core_clk),
        .rst_n(core_rst_n),
        .mode(route==3),
        .seq_valid(seq_valid && owner_idle && route==1),
        .seq_ready(old_seq_ready),
        .source_sel(source_sel),
        .seq_done(old_seq_done),
        .seq_error(old_seq_error),
        .pixel_valid(old_pixel_valid),
        .pixel_ready(pixel_ready && route==1),
        .pixel(old_pixel),
        .pixel_sof(old_pixel_sof),
        .pixel_eol(old_pixel_eol),
        .pixel_eof(old_pixel_eof),
        .req_valid(req_valid && owner_idle && route==3),
        .req_ready(old_req_ready),
        .camera_id(camera_id),
        .x(x),
        .y(y),
        .req_tag(req_tag),
        .rsp_valid(old_rsp_valid),
        .rsp_ready(rsp_ready && route==3),
        .rsp_pixel(rsp_pixel),
        .rsp_tag(rsp_tag),
        .rsp_error(rsp_error),
        .selected_sequence(selected_sequence),
        .latest_valid(frame_valid),
        .latest_slot(latest_slot),
        .frame_sequences(frame_sequences),
        .acquire(old_acquire),
        .release_slot(old_release_slot),
        .mem_req(old_mem_req),
        .mem_addr(old_mem_addr),
        .mem_len(old_mem_len),
        .mem_done(rd_done && (route==1 || route==3)),
        .mem_error(transaction_error),
        .mem_data(rd_data)
    );
    cds_ddr_port u_ddr_port (
        .clk(core_clk),
        .rst_n(core_rst_n),
        .wr_req(wr_req),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .wr_done(wr_done),
        .transaction_error(transaction_error),
        .rd_req(rd_req),
        .rd_addr(rd_addr),
        .rd_len(rd_len),
        .rd_done(rd_done),
        .rd_data(rd_data),
        .axi_awaddr(axi_awaddr),
        .axi_awlen(axi_awlen),
        .axi_awvalid(axi_awvalid),
        .axi_awready(axi_awready),
        .axi_wdata(axi_wdata),
        .axi_wstrb(axi_wstrb),
        .axi_wready(axi_wready),
        .axi_wlast(axi_wlast),
        .axi_araddr(axi_araddr),
        .axi_arlen(axi_arlen),
        .axi_arvalid(axi_arvalid),
        .axi_arready(axi_arready),
        .axi_rdata(axi_rdata),
        .axi_rvalid(axi_rvalid),
        .axi_rlast(axi_rlast),
        .protocol_error_sticky(protocol_error),
        .write_beats(write_beats),
        .read_beats(read_beats),
        .busy_cycles(busy_cycles),
        .request_wait_cycles(request_wait_cycles)
    );
endmodule
