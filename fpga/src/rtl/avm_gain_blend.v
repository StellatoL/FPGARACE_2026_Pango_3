// Two elastic stages: saturating per-channel Q4.12 gain, then complementary blend.
module avm_gain_blend #(parameter TAG_W = 128)(
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready,
    input wire [23:0] s_rgb_a, s_rgb_b,
    input wire [47:0] s_gain_a, s_gain_b,
    input wire [8:0] s_alpha,
    input wire s_valid_a, s_valid_b,
    input wire [TAG_W-1:0] s_tag,
    output wire m_valid, input wire m_ready,
    output reg [15:0] m_pixel, output reg [TAG_W-1:0] m_tag
);
    reg v1, v2;
    wire r2 = !v2 || m_ready;
    wire r1 = !v1 || r2;
    assign s_ready = r1;
    assign m_valid = v2;
    reg [23:0] gain_a, gain_b;
    reg [8:0] alpha1;
    reg [TAG_W-1:0] tag1;
    integer c;
    function [7:0] apply_gain;
        input [7:0] p;
        input [15:0] gain;
        reg [23:0] product;
        begin
            product = p*gain + 24'd2048;
            apply_gain = (product[23:12] > 12'd255) ? 8'd255 : product[19:12];
        end
    endfunction
    function [7:0] blend;
        input [7:0] a, b;
        input [8:0] alpha;
        reg [16:0] total;
        begin
            total = a*alpha + b*(9'd256-alpha) + 17'd128;
            blend = total[15:8];
        end
    endfunction
    wire [7:0] blend_r = blend(gain_a[23:16],gain_b[23:16],alpha1);
    wire [7:0] blend_g = blend(gain_a[15:8],gain_b[15:8],alpha1);
    wire [7:0] blend_b = blend(gain_a[7:0],gain_b[7:0],alpha1);
    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 0; v2 <= 0; m_pixel <= 0; m_tag <= 0;
        end else begin
            if (r1) begin
                v1 <= s_valid;
                if (s_valid) begin
                    for (c=0; c<3; c=c+1) begin
                        gain_a[23-c*8 -: 8] <= s_valid_a ? apply_gain(s_rgb_a[23-c*8 -: 8],s_gain_a[c*16 +: 16]) : 8'd0;
                        gain_b[23-c*8 -: 8] <= s_valid_b ? apply_gain(s_rgb_b[23-c*8 -: 8],s_gain_b[c*16 +: 16]) : 8'd0;
                    end
                    alpha1 <= !s_valid_b ? 9'd256 : (!s_valid_a ? 9'd0 : s_alpha);
                    tag1 <= s_tag;
                end
            end
            if (r2) begin
                v2 <= v1;
                if (v1) begin
                    m_pixel <= {blend_r[7:3],blend_g[7:2],blend_b[7:3]};
                    m_tag <= tag1;
                end
            end
        end
    end
endmodule
