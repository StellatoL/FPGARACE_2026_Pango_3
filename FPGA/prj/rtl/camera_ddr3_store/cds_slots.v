// 单路双槽管理，全部信号属于 core_clk 域。
// 单一读会话可锁一槽；同拍 acquire 优先于写分配，防止新读者撞上覆盖。
// start 每个输入 SOF 一次；commit 只能在最后写事务完成且整帧无错时产生。
module cds_slots (
    input wire clk,
    input wire rst_n,
    input wire start,
    output wire grant,
    output wire alloc_slot,
    input wire commit,
    input wire acquire,
    input wire release_slot,
    output wire latest_valid,
    output reg latest_slot,
    output reg [31:0] frame_sequence
);
    reg [1:0] valid_slots;
    reg [1:0] locked;
    reg write_slot;
    wire [1:0] acquire_mask = acquire && latest_valid ?
                              (latest_slot ? 2'b10 : 2'b01) : 2'b00;
    wire [1:0] protected_slots = locked | acquire_mask;
    wire preferred = latest_valid ? !latest_slot : 1'b0;
    assign latest_valid = valid_slots[latest_slot];
    // 始终保留 latest_slot 的完整帧，只允许复用另一槽。
    // 若另一槽仍被旧读会话锁定，丢弃本次输入帧；不能退回覆盖 latest_slot，
    // 否则旧读会话结束到新写帧完成之间会出现无可读帧的窗口。
    // 尚未发布首帧时允许使用槽0；写失败后下一 SOF 仍可复用未完成槽。
    assign alloc_slot = preferred;
    assign grant = !protected_slots[preferred];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_slots <= 2'b00;
            locked <= 2'b00;
            write_slot <= 1'b0;
            latest_slot <= 1'b0;
            frame_sequence <= 32'd0;
        end else begin
            if (release_slot) begin
                locked <= 2'b00;
            end else if (acquire && latest_valid) begin
                locked <= locked | acquire_mask;
            end
            if (start && grant) begin
                write_slot <= alloc_slot;
                valid_slots[alloc_slot] <= 1'b0;
            end
            if (commit) begin
                valid_slots[write_slot] <= 1'b1;
                latest_slot <= write_slot;
                frame_sequence <= frame_sequence + 1'b1;
            end
        end
    end
endmodule
