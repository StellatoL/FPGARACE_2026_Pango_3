// =============================================================================
// Module     : downscaler_1080p_to_720p_ddr
// Description: Pipelined 1920x1080 to 1280x720 3:2 polyphase scaler.
//
// Horizontal, for each input triplet p0,p1,p2:
//   o0 = (3*p0 + p1 + 2) / 4
//   o1 = (p1 + 3*p2 + 2) / 4
// Vertical, for each input-line triplet L0,L1,L2:
//   O0 = (3*L0 + L1 + 2) / 4
//   O1 = (L1 + 3*L2 + 2) / 4
//
// Filtering is performed in RGB888.  Quantization to RGB565 happens only at
// the final output.  A register stage after the line RAMs removes the BRAM to
// arithmetic critical path that prevented the previous interpolating version
// from meeting the 148.5 MHz pixel clock.
// =============================================================================

module downscaler_1080p_to_720p_ddr (
    input  wire         rst_n,
    input  wire         pclk_in,
    input  wire         vs_in,
    input  wire         hs_in,
    input  wire         de_in,
    input  wire [23:0]  rgb_in,

    output wire         pclk_in_out,
    output reg          vs_in_out,
    output reg          href_in_out,
    output reg          de_in_out,
    output reg  [15:0]  rgb565_out
);

localparam OUT_WIDTH  = 1280;
localparam ADDR_WIDTH = 11;

reg [1:0] h_phase;
reg [1:0] v_phase;
reg       de_in_d1;
reg       vs_in_d1;
reg       hs_in_d1;
reg [23:0] h_pix0;
reg [23:0] h_pix1;

reg [ADDR_WIDTH-1:0] wr_addr;
reg [ADDR_WIDTH-1:0] rd_addr;
reg [ADDR_WIDTH:0]   rd_issue_cnt;
reg                  rd_issue_active;
reg                  rd_mode;

reg                  rd_valid_d1;
reg                  rd_valid_d2;
reg                  rd_mode_d1;
reg                  rd_mode_d2;
reg [23:0]           ram0_data_d1;
reg [23:0]           ram1_data_d1;
reg [23:0]           ram2_data_d1;
reg                  vs_pipe_d1;
reg                  vs_pipe_d2;

wire [23:0] ram0_rd_data;
wire [23:0] ram1_rd_data;
wire [23:0] ram2_rd_data;

wire vs_pos   = vs_in & ~vs_in_d1;
wire hs_pos   = hs_in & ~hs_in_d1;
wire line_end = de_in_d1 & ~de_in;

wire h_wr_en = de_in & ((h_phase == 2'd1) | (h_phase == 2'd2));
wire [23:0] h_wr_data = (h_phase == 2'd1) ? rgb888_mix_3_1(h_pix0, rgb_in) :
                                           rgb888_mix_1_3(h_pix1, rgb_in);
wire ram0_wr_en = h_wr_en & (v_phase == 2'd0);
wire ram1_wr_en = h_wr_en & (v_phase == 2'd1);
wire ram2_wr_en = h_wr_en & (v_phase == 2'd2);

wire start_read    = line_end & (v_phase != 2'd0);
wire issue_rd_addr = start_read |
                     (rd_issue_active & (rd_issue_cnt < OUT_WIDTH));
wire issue_rd_mode = start_read ? (v_phase == 2'd2) : rd_mode;

assign pclk_in_out = pclk_in;

function [23:0] rgb888_mix_3_1;
    input [23:0] a;
    input [23:0] b;
    reg [9:0] r;
    reg [9:0] g;
    reg [9:0] bl;
    begin
        r  = ({2'b00, a[23:16]} << 1) + {2'b00, a[23:16]} +
             {2'b00, b[23:16]} + 10'd2;
        g  = ({2'b00, a[15:8]} << 1) + {2'b00, a[15:8]} +
             {2'b00, b[15:8]} + 10'd2;
        bl = ({2'b00, a[7:0]} << 1) + {2'b00, a[7:0]} +
             {2'b00, b[7:0]} + 10'd2;
        rgb888_mix_3_1 = {r[9:2], g[9:2], bl[9:2]};
    end
endfunction

function [23:0] rgb888_mix_1_3;
    input [23:0] a;
    input [23:0] b;
    reg [9:0] r;
    reg [9:0] g;
    reg [9:0] bl;
    begin
        r  = {2'b00, a[23:16]} + ({2'b00, b[23:16]} << 1) +
             {2'b00, b[23:16]} + 10'd2;
        g  = {2'b00, a[15:8]} + ({2'b00, b[15:8]} << 1) +
             {2'b00, b[15:8]} + 10'd2;
        bl = {2'b00, a[7:0]} + ({2'b00, b[7:0]} << 1) +
             {2'b00, b[7:0]} + 10'd2;
        rgb888_mix_1_3 = {r[9:2], g[9:2], bl[9:2]};
    end
endfunction

function [15:0] rgb888_mix_3_1_to_565;
    input [23:0] a;
    input [23:0] b;
    reg [9:0] r;
    reg [9:0] g;
    reg [9:0] bl;
    begin
        r  = ({2'b00, a[23:16]} << 1) + {2'b00, a[23:16]} +
             {2'b00, b[23:16]} + 10'd2;
        g  = ({2'b00, a[15:8]} << 1) + {2'b00, a[15:8]} +
             {2'b00, b[15:8]} + 10'd2;
        bl = ({2'b00, a[7:0]} << 1) + {2'b00, a[7:0]} +
             {2'b00, b[7:0]} + 10'd2;
        rgb888_mix_3_1_to_565 = {r[9:5], g[9:4], bl[9:5]};
    end
endfunction

function [15:0] rgb888_mix_1_3_to_565;
    input [23:0] a;
    input [23:0] b;
    reg [9:0] r;
    reg [9:0] g;
    reg [9:0] bl;
    begin
        r  = {2'b00, a[23:16]} + ({2'b00, b[23:16]} << 1) +
             {2'b00, b[23:16]} + 10'd2;
        g  = {2'b00, a[15:8]} + ({2'b00, b[15:8]} << 1) +
             {2'b00, b[15:8]} + 10'd2;
        bl = {2'b00, a[7:0]} + ({2'b00, b[7:0]} << 1) +
             {2'b00, b[7:0]} + 10'd2;
        rgb888_mix_1_3_to_565 = {r[9:5], g[9:4], bl[9:5]};
    end
endfunction

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n) begin
        de_in_d1 <= 1'b0;
        vs_in_d1 <= 1'b0;
        hs_in_d1 <= 1'b0;
    end else begin
        de_in_d1 <= de_in;
        vs_in_d1 <= vs_in;
        hs_in_d1 <= hs_in;
    end
end

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n) begin
        h_phase <= 2'd0;
        h_pix0  <= 24'd0;
        h_pix1  <= 24'd0;
    end else if (vs_pos || hs_pos || line_end) begin
        h_phase <= 2'd0;
        h_pix0  <= 24'd0;
        h_pix1  <= 24'd0;
    end else if (de_in) begin
        case (h_phase)
            2'd0: begin
                h_pix0  <= rgb_in;
                h_phase <= 2'd1;
            end
            2'd1: begin
                h_pix1  <= rgb_in;
                h_phase <= 2'd2;
            end
            default: h_phase <= 2'd0;
        endcase
    end
end

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n)
        v_phase <= 2'd0;
    else if (vs_pos)
        v_phase <= 2'd0;
    else if (line_end) begin
        if (v_phase == 2'd2)
            v_phase <= 2'd0;
        else
            v_phase <= v_phase + 1'b1;
    end
end

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n)
        wr_addr <= {ADDR_WIDTH{1'b0}};
    else if (vs_pos || line_end)
        wr_addr <= {ADDR_WIDTH{1'b0}};
    else if (h_wr_en && wr_addr != OUT_WIDTH - 1)
        wr_addr <= wr_addr + 1'b1;
end

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n) begin
        rd_addr         <= {ADDR_WIDTH{1'b0}};
        rd_issue_cnt    <= {(ADDR_WIDTH+1){1'b0}};
        rd_issue_active <= 1'b0;
        rd_mode         <= 1'b0;
    end else if (vs_pos) begin
        rd_addr         <= {ADDR_WIDTH{1'b0}};
        rd_issue_cnt    <= {(ADDR_WIDTH+1){1'b0}};
        rd_issue_active <= 1'b0;
        rd_mode         <= 1'b0;
    end else if (start_read) begin
        rd_addr         <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
        rd_issue_cnt    <= {{ADDR_WIDTH{1'b0}}, 1'b1};
        rd_issue_active <= 1'b1;
        rd_mode         <= (v_phase == 2'd2);
    end else if (rd_issue_active) begin
        if (rd_issue_cnt < OUT_WIDTH) begin
            rd_issue_cnt <= rd_issue_cnt + 1'b1;
            if (rd_issue_cnt == OUT_WIDTH - 1)
                rd_addr <= {ADDR_WIDTH{1'b0}};
            else
                rd_addr <= rd_addr + 1'b1;
        end else begin
            rd_addr         <= {ADDR_WIDTH{1'b0}};
            rd_issue_cnt    <= {(ADDR_WIDTH+1){1'b0}};
            rd_issue_active <= 1'b0;
        end
    end
end

// BRAM-output pipeline.  Data and mode are delayed together with validity.
always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n) begin
        rd_valid_d1 <= 1'b0;
        rd_valid_d2 <= 1'b0;
        rd_mode_d1  <= 1'b0;
        rd_mode_d2  <= 1'b0;
        ram0_data_d1 <= 24'd0;
        ram1_data_d1 <= 24'd0;
        ram2_data_d1 <= 24'd0;
        vs_pipe_d1   <= 1'b0;
        vs_pipe_d2   <= 1'b0;
    end else begin
        rd_valid_d1 <= issue_rd_addr;
        rd_valid_d2 <= rd_valid_d1;
        if (issue_rd_addr)
            rd_mode_d1 <= issue_rd_mode;
        if (rd_valid_d1)
            rd_mode_d2 <= rd_mode_d1;
        ram0_data_d1 <= ram0_rd_data;
        ram1_data_d1 <= ram1_rd_data;
        ram2_data_d1 <= ram2_rd_data;
        vs_pipe_d1   <= vs_in;
        vs_pipe_d2   <= vs_pipe_d1;
    end
end

always @(posedge pclk_in or negedge rst_n) begin
    if (!rst_n) begin
        vs_in_out   <= 1'b0;
        href_in_out <= 1'b0;
        de_in_out   <= 1'b0;
        rgb565_out  <= 16'd0;
    end else begin
        vs_in_out   <= vs_pipe_d2;
        href_in_out <= rd_valid_d2;
        de_in_out   <= rd_valid_d2;
        if (rd_valid_d2) begin
            if (rd_mode_d2)
                rgb565_out <= rgb888_mix_1_3_to_565(ram1_data_d1,
                                                    ram2_data_d1);
            else
                rgb565_out <= rgb888_mix_3_1_to_565(ram0_data_d1,
                                                    ram1_data_d1);
        end else begin
            rgb565_out <= 16'd0;
        end
    end
end

dualport_ram #(.DATA_WIDTH(24), .DEPTH(OUT_WIDTH)) u_line_ram0 (
    .wr_clk(pclk_in), .wr_rst(~rst_n), .wr_addr(wr_addr),
    .wr_data(h_wr_data), .wr_en(ram0_wr_en),
    .rd_clk(pclk_in), .rd_rst(~rst_n), .rd_addr(rd_addr),
    .rd_data(ram0_rd_data)
);

dualport_ram #(.DATA_WIDTH(24), .DEPTH(OUT_WIDTH)) u_line_ram1 (
    .wr_clk(pclk_in), .wr_rst(~rst_n), .wr_addr(wr_addr),
    .wr_data(h_wr_data), .wr_en(ram1_wr_en),
    .rd_clk(pclk_in), .rd_rst(~rst_n), .rd_addr(rd_addr),
    .rd_data(ram1_rd_data)
);

dualport_ram #(.DATA_WIDTH(24), .DEPTH(OUT_WIDTH)) u_line_ram2 (
    .wr_clk(pclk_in), .wr_rst(~rst_n), .wr_addr(wr_addr),
    .wr_data(h_wr_data), .wr_en(ram2_wr_en),
    .rd_clk(pclk_in), .rd_rst(~rst_n), .rd_addr(rd_addr),
    .rd_data(ram2_rd_data)
);

endmodule
