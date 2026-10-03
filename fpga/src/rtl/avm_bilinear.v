// RGB565 expansion and bilinear sampling. Three independently elastic stages.
module avm_bilinear #(parameter TAG_W = 128)(
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready,
    input wire [63:0] s_pixels, input wire [7:0] s_fx, s_fy,
    input wire [TAG_W-1:0] s_tag,
    output wire m_valid, input wire m_ready,
    output reg [23:0] m_rgb, output reg [TAG_W-1:0] m_tag
);
    reg v1, v2, v3;
    wire r3 = !v3 || m_ready;
    wire r2 = !v2 || r3;
    wire r1 = !v1 || r2;
    assign s_ready = r1;
    assign m_valid = v3;
    reg [15:0] h0 [0:2];
    reg [15:0] h1 [0:2];
    reg [23:0] sums [0:2];
    reg [7:0] fy1;
    reg [TAG_W-1:0] tag1, tag2;
    integer c;
    function [7:0] expand;
        input [15:0] p;
        input integer channel;
        begin
            case (channel)
                0: expand = {p[15:11],p[15:13]};
                1: expand = {p[10:5],p[10:9]};
                default: expand = {p[4:0],p[4:2]};
            endcase
        end
    endfunction
    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 0; v2 <= 0; v3 <= 0;
            m_rgb <= 0; m_tag <= 0;
        end else begin
            if (r1) begin
                v1 <= s_valid;
                if (s_valid) begin
                    for (c=0; c<3; c=c+1) begin
                        h0[c] <= expand(s_pixels[15:0],c)*(9'd256-{1'b0,s_fx})
                               + expand(s_pixels[31:16],c)*{1'b0,s_fx};
                        h1[c] <= expand(s_pixels[47:32],c)*(9'd256-{1'b0,s_fx})
                               + expand(s_pixels[63:48],c)*{1'b0,s_fx};
                    end
                    fy1 <= s_fy; tag1 <= s_tag;
                end
            end
            if (r2) begin
                v2 <= v1;
                if (v1) begin
                    for (c=0; c<3; c=c+1)
                        sums[c] <= h0[c]*(9'd256-{1'b0,fy1}) + h1[c]*{1'b0,fy1};
                    tag2 <= tag1;
                end
            end
            if (r3) begin
                v3 <= v2;
                if (v2) begin
                    for (c=0; c<3; c=c+1)
                        m_rgb[23-c*8 -: 8] <= (sums[c]+24'd32768)>>16;
                    m_tag <= tag2;
                end
            end
        end
    end
endmodule
