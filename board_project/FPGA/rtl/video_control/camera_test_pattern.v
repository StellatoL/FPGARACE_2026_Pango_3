// RGB565 color bars clocked from the known camera stream. Reuse the source
// VS/DE exactly so the test pattern follows the real camera's active timing.
module camera_test_pattern (
    input wire clk,
    input wire rst_n,
    input wire in_vs,
    input wire in_de,
    output reg out_vs,
    output reg out_de,
    output reg [15:0] out_data
);
    reg in_vs_d;
    reg de_d;
    reg [10:0] x;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_vs_d <= 1'b0;
            de_d <= 1'b0;
            out_vs <= 1'b0;
            out_de <= 1'b0;
            out_data <= 16'd0;
            x <= 11'd0;
        end else begin
            in_vs_d <= in_vs;
            de_d <= in_de;
            out_vs <= in_vs;
            out_de <= in_de;
            if (in_de) begin
                case (x[10:7])
                    4'd0: out_data <= 16'hF800; // red
                    4'd1: out_data <= 16'hFD20; // orange
                    4'd2: out_data <= 16'hFFE0; // yellow
                    4'd3: out_data <= 16'h07E0; // green
                    4'd4: out_data <= 16'h07FF; // cyan
                    4'd5: out_data <= 16'h001F; // blue
                    4'd6: out_data <= 16'hF81F; // magenta
                    4'd7: out_data <= 16'hFFFF; // white
                    4'd8: out_data <= 16'h8410; // gray
                    default: out_data <= 16'h0000;
                endcase
                x <= x + 1'b1;
            end else begin
                out_data <= 16'd0;
                if (in_vs && !in_vs_d) x <= 11'd0;
                else if (de_d) x <= 11'd0;
            end
        end
    end
endmodule
