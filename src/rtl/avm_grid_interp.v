// Q12.8 signed coordinate interpolation. Round half away from zero at EACH axis.
// Four elastic stages: horizontal total, horizontal round, vertical total, vertical round.
module avm_grid_interp #(parameter TAG_W = 128)(
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready,
    input wire signed [20:0] s_u00, s_u10, s_u01, s_u11,
    input wire signed [20:0] s_v00, s_v10, s_v01, s_v11,
    input wire [3:0] s_xfrac, s_yfrac,
    input wire [2:0] s_step_log2, input wire [TAG_W-1:0] s_tag,
    output wire m_valid, input wire m_ready,
    output reg signed [20:0] m_u, m_v,
    output reg [TAG_W-1:0] m_tag
);
    reg v1, v2, v3, v4;
    wire r4 = !v4 || m_ready;
    wire r3 = !v3 || r4;
    wire r2 = !v2 || r3;
    wire r1 = !v1 || r2;
    assign s_ready = r1;
    assign m_valid = v4;

    reg signed [26:0] htu0, htu1, htv0, htv1;
    reg signed [20:0] hu0, hu1, hv0, hv1;
    reg signed [26:0] vtu, vtv;
    reg [3:0] yf1, yf2;
    reg [2:0] log1, log2, log3;
    reg [TAG_W-1:0] tag1, tag2, tag3;

    function signed [26:0] lerp_total;
        input signed [20:0] a, b;
        input [3:0] fraction;
        input [2:0] log_step;
        reg signed [21:0] delta;
        reg signed [4:0] weight;
        reg signed [26:0] base, product;
        begin
            // Explicit sign extension preserves the full 22-bit difference.
            delta = $signed({b[20],b}) - $signed({a[20],a});
            weight = $signed({1'b0,fraction});
            base = $signed({{6{a[20]}},a}) <<< log_step;
            product = delta * weight;
            lerp_total = base + product;
        end
    endfunction

    function signed [20:0] round_total;
        input signed [26:0] total;
        input [2:0] log_step;
        reg signed [4:0] bias;
        reg signed [26:0] rounded;
        begin
            // floor((total+half-sign(total))/step) rounds signed ties away
            // from zero. For step=1, fraction=0 and rounding is identity.
            case (log_step)
                3'd1: bias = total[26] ? 5'sd0 : 5'sd1;
                3'd2: bias = total[26] ? 5'sd1 : 5'sd2;
                3'd3: bias = total[26] ? 5'sd3 : 5'sd4;
                3'd4: bias = total[26] ? 5'sd7 : 5'sd8;
                default: bias = 5'sd0;
            endcase
            rounded = (total + bias) >>> log_step;
            round_total = rounded[20:0];
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0;
            m_u <= 0; m_v <= 0; m_tag <= 0;
        end else begin
            if (r1) begin
                v1 <= s_valid;
                if (s_valid) begin
                    htu0 <= lerp_total(s_u00,s_u10,s_xfrac,s_step_log2);
                    htu1 <= lerp_total(s_u01,s_u11,s_xfrac,s_step_log2);
                    htv0 <= lerp_total(s_v00,s_v10,s_xfrac,s_step_log2);
                    htv1 <= lerp_total(s_v01,s_v11,s_xfrac,s_step_log2);
                    yf1 <= s_yfrac; log1 <= s_step_log2; tag1 <= s_tag;
                end
            end
            if (r2) begin
                v2 <= v1;
                if (v1) begin
                    hu0 <= round_total(htu0,log1);
                    hu1 <= round_total(htu1,log1);
                    hv0 <= round_total(htv0,log1);
                    hv1 <= round_total(htv1,log1);
                    yf2 <= yf1; log2 <= log1; tag2 <= tag1;
                end
            end
            if (r3) begin
                v3 <= v2;
                if (v2) begin
                    vtu <= lerp_total(hu0,hu1,yf2,log2);
                    vtv <= lerp_total(hv0,hv1,yf2,log2);
                    log3 <= log2; tag3 <= tag2;
                end
            end
            if (r4) begin
                v4 <= v3;
                if (v3) begin
                    m_u <= round_total(vtu,log3);
                    m_v <= round_total(vtv,log3);
                    m_tag <= tag3;
                end
            end
        end
    end
endmodule
