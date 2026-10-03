// Macro-major pixels within sequential 16-row stripes -> raster stream.
// A buffer belongs to the writer until all real pixels arrive, then to the
// reader until its final output handshake. Padded macro pixels MUST be omitted.
module avm_stripe_reorder #(parameter WIDTH=1280, parameter HEIGHT=720)(
    input wire clk, input wire rst_n, input wire start,
    input wire s_valid, output wire s_ready,
    input wire [15:0] s_pixel, input wire [11:0] s_x, s_y,
    output reg m_valid, input wire m_ready,
    output wire [15:0] m_pixel, output reg m_sof, m_eol,
    output reg done
);
    // Cap each inferred memory at 4096 words. A 1280x16 stripe then uses
    // five exact chunks rather than a single 20480-word array that ADS
    // rounds up to 32768 words. Preserve a smaller final chunk for other widths.
    localparam WORDS = WIDTH*16;
    localparam CHUNKS = (WORDS+4095)/4096;
    function integer clog2;
        input integer value;
        integer v;
        begin
            v=value-1;
            for (clog2=0; v>0; clog2=clog2+1) v=v>>1;
        end
    endfunction
    localparam CHUNK_BITS = CHUNKS > 1 ? clog2(CHUNKS) : 1;
    wire [CHUNKS*16-1:0] read_pixels0, read_pixels1;
    reg [CHUNK_BITS-1:0] out_chunk;
    reg out_bank, out_has_pixel;
    reg [1:0] full;
    reg active, input_complete, wr_bank, rd_bank, rd_last_issued;
    reg [31:0] wr_count, wr_base_y, rd_base_y, rd_index, rd_x;
    reg out_last, out_frame_last;
    wire [31:0] wr_rows = HEIGHT-wr_base_y >= 16 ? 16 : HEIGHT-wr_base_y;
    wire [31:0] rd_rows = HEIGHT-rd_base_y >= 16 ? 16 : HEIGHT-rd_base_y;
    wire [31:0] wr_address = s_y[3:0]*WIDTH+s_x;
    assign s_ready = active && !input_complete && !full[wr_bank];
    wire write_pixel = rst_n && !start && s_valid && s_ready;
    wire read_pixel = active && full[rd_bank] && !rd_last_issued && (!m_valid || m_ready);

    // The RAM outputs and their selector register update on the same edge,
    // so this retains the original one-cycle read and output hold behavior.
    // Reset only the visibility flag; resetting RAM output data can prevent
    // use of the embedded memory output register on some synthesis tools.
    assign m_pixel = !out_has_pixel ? 16'd0 :
                     out_bank ? read_pixels1[out_chunk*16 +: 16] :
                                read_pixels0[out_chunk*16 +: 16];
    genvar chunk;
    generate for (chunk=0; chunk<CHUNKS; chunk=chunk+1) begin: stripe_chunk
        localparam DEPTH = WORDS-chunk*4096 < 4096 ? WORDS-chunk*4096 : 4096;
        reg [15:0] memory0 [0:DEPTH-1];
        reg [15:0] memory1 [0:DEPTH-1];
        reg [15:0] q0, q1;
        assign read_pixels0[chunk*16 +: 16] = q0;
        assign read_pixels1[chunk*16 +: 16] = q1;
        always @(posedge clk) begin
            if (write_pixel && wr_address[31:12] == chunk) begin
                if (wr_bank) memory1[wr_address[11:0]] <= s_pixel;
                else memory0[wr_address[11:0]] <= s_pixel;
            end
            if (rst_n && !start && read_pixel && rd_index[31:12] == chunk) begin
                if (rd_bank) q1 <= memory1[rd_index[11:0]];
                else q0 <= memory0[rd_index[11:0]];
            end
        end
    end endgenerate
    always @(posedge clk) begin
        if (!rst_n || start) begin
            active <= rst_n && start;
            input_complete <= 0; full <= 0;
            wr_bank <= 0; rd_bank <= 0;
            wr_count <= 0; wr_base_y <= 0;
            rd_base_y <= 0; rd_index <= 0; rd_x <= 0;
            rd_last_issued <= 0;
            m_valid <= 0; m_sof <= 0; m_eol <= 0;
            out_chunk <= 0; out_bank <= 0; out_has_pixel <= 0;
            out_last <= 0; out_frame_last <= 0; done <= 0;
        end else begin
            done <= 0;
            if (s_valid && s_ready) begin
                if (wr_count == wr_rows*WIDTH-1) begin
                    full[wr_bank] <= 1;
                    wr_bank <= !wr_bank;
                    wr_count <= 0; wr_base_y <= wr_base_y+16;
                    if (wr_base_y+16 >= HEIGHT) input_complete <= 1;
                end else wr_count <= wr_count+1;
            end
            if (m_valid && m_ready) begin
                m_valid <= 0;
                if (out_last) begin
                    full[rd_bank] <= 0;
                    rd_bank <= !rd_bank;
                    rd_index <= 0; rd_x <= 0; rd_base_y <= rd_base_y+16;
                    rd_last_issued <= 0;
                    if (out_frame_last) begin done <= 1; active <= 0; end
                end
            end
            if (read_pixel) begin
                out_chunk <= rd_index[31:12];
                out_bank <= rd_bank; out_has_pixel <= 1;
                m_valid <= 1;
                m_sof <= rd_base_y == 0 && rd_index == 0;
                m_eol <= rd_x == WIDTH-1;
                out_last <= rd_index == rd_rows*WIDTH-1;
                out_frame_last <= rd_base_y+16 >= HEIGHT;
                if (rd_index == rd_rows*WIDTH-1) rd_last_issued <= 1;
                else begin
                    rd_index <= rd_index+1;
                    if (rd_x == WIDTH-1) rd_x <= 0;
                    else rd_x <= rd_x+1;
                end
            end
        end
    end
endmodule
