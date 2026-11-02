// core_clk 域先收齐 16 个 128 位拍再发起 DDR 突发；req 保持到实际完成 done。
// 非完整帧永不发布；下一 SOF 丢弃前帧上下文。地址单位为 16 位字。
module cds_writer #(
    parameter [27:0] CHANNEL_BASE = 28'h0000000
)(
    input wire clk,
    input wire rst_n,
    input wire fifo_valid,
    output wire fifo_ready,
    input wire [129:0] fifo_data,
    output wire slot_start,
    input wire slot_grant,
    input wire alloc_slot,
    output wire commit,
    output reg req,
    output reg [27:0] addr,
    output reg [2047:0] data,
    input wire done,
    input wire error,
    output reg drop_sticky
);
    reg active;
    reg slot;
    reg last_chunk;
    reg [12:0] chunk_cnt;
    reg [3:0] beat_cnt;
    wire first = fifo_data[128];
    wire last = fifo_data[129];
    wire expected_last = chunk_cnt == 13'd7199 && beat_cnt == 4'd15;
    assign fifo_ready = !req;
    assign slot_start = fifo_valid && fifo_ready && first;
    assign commit = req && done && !error && last_chunk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req <= 1'b0;
            addr <= 28'd0;
            active <= 1'b0;
            slot <= 1'b0;
            last_chunk <= 1'b0;
            chunk_cnt <= 13'd0;
            beat_cnt <= 4'd0;
            drop_sticky <= 1'b0;
        end else begin
            if (req && done) begin
                req <= 1'b0;
                if (error || last_chunk) begin
                    active <= 1'b0;
                end
                if (error) begin
                    drop_sticky <= 1'b1;
                end
            end
            if (fifo_valid && fifo_ready) begin
                if (first) begin
                    if (active || !slot_grant || last) begin
                        drop_sticky <= 1'b1;
                    end
                    active <= slot_grant && !last;
                    slot <= alloc_slot;
                    chunk_cnt <= 13'd0;
                    beat_cnt <= 4'd1;
                    last_chunk <= 1'b0;
                end else if (active) begin
                    if (last != expected_last) begin
                        active <= 1'b0;
                        drop_sticky <= 1'b1;
                        beat_cnt <= 4'd0;
                    end else if (beat_cnt == 4'd15) begin
                        addr <= CHANNEL_BASE + (slot ? 28'h0100000 : 28'd0)
                                + {8'd0, chunk_cnt, 7'd0};
                        chunk_cnt <= chunk_cnt + 1'b1;
                        beat_cnt <= 4'd0;
                        req <= 1'b1;
                        last_chunk <= last;
                    end else begin
                        beat_cnt <= beat_cnt + 1'b1;
                    end
                end
            end
        end
    end
    always @(posedge clk) begin
        if (fifo_valid && fifo_ready) begin
            // 请求持有期间 fifo_ready=0，组装寄存器不会被覆盖。
            if (first) begin
                data[127:0] <= fifo_data[127:0];
            end else if (active) begin
                data[beat_cnt*128 +: 128] <= fifo_data[127:0];
            end
        end
    end
endmodule
