// 适配当前 Pango DDR 原生端口，不是通用 AXI4：无 WVALID/RREADY/B 通道。
// AW 握手后，每个 WREADY 表示消费一拍数据；WLAST 由 DDR IP 返回。
// 读返回不可背压，内部一次完整缓存 16 拍。全端口最多一个未完成事务。
// 四写一读轮询，以突发为单位保持所有权；done 是完成脉冲而非命令接收。
// 若控制器永久不返回 last，本模块保持等待，必须系统级复位恢复。
module cds_ddr_port (
    input wire clk,
    input wire rst_n,
    input wire [3:0] wr_req,
    input wire [111:0] wr_addr,
    input wire [8191:0] wr_data,
    output reg [3:0] wr_done,
    output reg transaction_error,
    input wire rd_req,
    input wire [27:0] rd_addr,
    input wire [3:0] rd_len,
    output reg rd_done,
    output reg [2047:0] rd_data,
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
    input wire axi_rlast,
    output reg protocol_error_sticky,
    output reg [31:0] write_beats,
    output reg [31:0] read_beats,
    output reg [31:0] busy_cycles,
    output wire [159:0] request_wait_cycles
);
    localparam IDLE = 3'd0;
    localparam WRITE_ADDR = 3'd1;
    localparam WRITE_DATA = 3'd2;
    localparam READ_ADDR = 3'd3;
    localparam READ_DATA = 3'd4;
    localparam FINISH = 3'd5;
    localparam RETIRE = 3'd6;
    localparam LOAD_WRITE = 3'd7;
    reg [2:0] state;
    reg [2:0] owner;
    reg [2:0] round_robin;
    reg [3:0] beat;
    reg [3:0] length;
    reg [27:0] address;
    reg [2047:0] write_buffer;
    reg burst_error;
    reg found;
    reg [2:0] winner;
    integer scan;
    integer candidate;
    wire [4:0] requests = {rd_req, wr_req};
    assign axi_awaddr = address;
    assign axi_araddr = address;
    assign axi_awlen = 4'd15;
    assign axi_arlen = length;
    assign axi_awvalid = state == WRITE_ADDR;
    assign axi_arvalid = state == READ_ADDR;
    assign axi_wdata = write_buffer[beat*128 +: 128];
    assign axi_wstrb = 16'hffff;

    // 各请求源从 req 到 done 的累计等待周期（含实际传输），32 位回绕。
    genvar source;
    generate
        for (source = 0; source < 5; source = source + 1) begin : wait_statistics
            reg [31:0] cycles;
            assign request_wait_cycles[source*32 +: 32] = cycles;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cycles <= 32'd0;
                end else if (requests[source]) begin
                    cycles <= cycles + 1'b1;
                end
            end
        end
    endgenerate

    always @(*) begin
        found = 1'b0;
        winner = round_robin;
        candidate = 0;
        for (scan = 0; scan < 5; scan = scan + 1) begin
            candidate = round_robin + scan;
            if (candidate >= 5) begin
                candidate = candidate - 5;
            end
            if (!found && requests[candidate]) begin
                found = 1'b1;
                winner = candidate[2:0];
            end
        end
    end
    always @(posedge clk) begin
        // owner 已寄存，组合轮询不再直接驱动 2048 位数据选择和装载使能。
        // 写端必须将 req/addr/data 保持到 done，新增这一拍不会改变源数据。
        if (state == LOAD_WRITE) begin
            write_buffer <= wr_data[owner*2048 +: 2048];
        end
        if (state == READ_DATA && axi_rvalid) begin
            rd_data[beat*128 +: 128] <= axi_rdata;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            owner <= 3'd0;
            round_robin <= 3'd0;
            beat <= 4'd0;
            length <= 4'd0;
            address <= 28'd0;
            wr_done <= 4'd0;
            rd_done <= 1'b0;
            transaction_error <= 1'b0;
            burst_error <= 1'b0;
            protocol_error_sticky <= 1'b0;
            write_beats <= 32'd0;
            read_beats <= 32'd0;
            busy_cycles <= 32'd0;
        end else begin
            wr_done <= 4'd0;
            rd_done <= 1'b0;
            if (state != IDLE) begin
                busy_cycles <= busy_cycles + 1'b1;
            end
            case (state)
                IDLE: begin
                    if (found) begin
                        owner <= winner;
                        beat <= 4'd0;
                        burst_error <= 1'b0;
                        if (winner == 3'd4) begin
                            address <= rd_addr;
                            length <= rd_len;
                            state <= READ_ADDR;
                        end else begin
                            length <= 4'd15;
                            state <= LOAD_WRITE;
                        end
                    end
                end
                LOAD_WRITE: begin
                    address <= wr_addr[owner*28 +: 28];
                    state <= WRITE_ADDR;
                end
                WRITE_ADDR: begin
                    if (axi_awready) begin
                        state <= WRITE_DATA;
                    end
                end
                WRITE_DATA: begin
                    if (axi_wready) begin
                        write_beats <= write_beats + 1'b1;
                        beat <= beat + 1'b1;
                        if (axi_wlast != (beat == 4'd15)) begin
                            burst_error <= 1'b1;
                            protocol_error_sticky <= 1'b1;
                        end
                        if (axi_wlast) begin
                            state <= FINISH;
                        end
                    end
                end
                READ_ADDR: begin
                    if (axi_arready) begin
                        state <= READ_DATA;
                    end
                end
                READ_DATA: begin
                    if (axi_rvalid) begin
                        read_beats <= read_beats + 1'b1;
                        beat <= beat + 1'b1;
                        if (axi_rlast != (beat == length)) begin
                            burst_error <= 1'b1;
                            protocol_error_sticky <= 1'b1;
                        end
                        if (axi_rlast) begin
                            state <= FINISH;
                        end
                    end
                end
                FINISH: begin
                    // 完成后保留一拍，源端在下一时钟沿撤销 req。
                    transaction_error <= burst_error;
                    if (owner == 3'd4) begin
                        rd_done <= 1'b1;
                    end else begin
                        wr_done[owner] <= 1'b1;
                    end
                    round_robin <= owner == 3'd4 ? 3'd0 : owner + 1'b1;
                    state <= RETIRE;
                end
                RETIRE: begin
                    state <= IDLE;
                end
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
