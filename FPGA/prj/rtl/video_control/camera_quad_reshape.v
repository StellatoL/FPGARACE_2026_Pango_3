// Convert one 1280x720 RGB565 camera stream to a 640x360 frame by keeping
// every other pixel and line. The output is suitable for a 640x360 DDR frame.
module camera_quad_reshape (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [15:0] in_data,
    output reg         out_vs,
    output reg         out_de,
    output reg  [15:0] out_data
);
    reg de_d;
    reg [11:0] x;
    reg [11:0] y;
    reg [15:0] line_mem [0:639];
    reg [9:0] emit_x;
    reg emitting;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d     <= 0;
            x        <= 0;
            y        <= 0;
            emit_x   <= 0;
            emitting <= 0;
            out_vs   <= 0;
            out_de   <= 0;
            out_data <= 0;
        end else begin
            de_d   <= in_de;
            out_vs <= in_vs;
            out_de <= emitting;

            if (emitting) begin
                out_data <= line_mem[emit_x];

                if (emit_x == 639) begin
                    emitting <= 0;
                end else begin
                    emit_x <= emit_x + 1'b1;
                end
            end

            if (in_vs) begin
                x        <= 0;
                y        <= 0;
                emitting <= 0;
                out_de   <= 0;
            end else if (in_de) begin
                if (x == 1279) begin
                    x <= 0;
                end else begin
                    x <= x + 1'b1;
                end
            end

            if (!in_vs && in_de && !x[0] && !y[0]) begin
                line_mem[x[10:1]] <= in_data;
            end

            if (!in_vs && !in_de && de_d) begin
                if (!y[0]) begin
                    emitting <= 1;
                    emit_x   <= 0;
                end

                if (y != 719) begin
                    y <= y + 1'b1;
                end
            end
        end
    end
endmodule
