// KEY1: quad -> camera1 -> camera2 -> camera3 -> camera4 -> quad.
module stereo_mode_select #(parameter CLK_HZ=50000000)(
    input wire clk, input wire rst_n, input wire key_n,
    output reg [2:0] mode_pending
);
    localparam DEBOUNCE=CLK_HZ/50;
    (* ASYNC_REG="TRUE" *) reg [1:0] sync_key;
    reg stable_key;
    reg [31:0] count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sync_key<=3; stable_key<=1; count<=0; mode_pending<=0; end
        else begin
            sync_key<={sync_key[0],key_n};
            if (sync_key[1]==stable_key) count<=0;
            else if (count==DEBOUNCE-1) begin
                count<=0; stable_key<=sync_key[1];
                if (!sync_key[1]) mode_pending<=mode_pending==4 ? 3'd0 : mode_pending+1'b1;
            end else count<=count+1'b1;
        end
    end
endmodule
