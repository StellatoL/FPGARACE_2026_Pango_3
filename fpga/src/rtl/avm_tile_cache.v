// 16 tiles of 32x32 RGB565. Four merged 4096x16 parity banks.
// Each bank has at most two writes per clock. Filling one DDR beat takes two
// clocks; accepted hit reads are synchronous and run at one sample per clock.
module avm_tile_cache #(parameter TAG_W=128, parameter PREFETCH_TILES=2) (
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready, input wire s_enable,
    input wire [1:0] s_cam, input wire [11:0] s_x, input wire [11:0] s_y,
    input wire [TAG_W-1:0] s_tag,
    input wire [127:0] camera_bases,
    input wire [11:0] src_width, input wire [11:0] src_height,
    input wire [15:0] src_stride, input wire invalidate,
    output reg m_valid, input wire m_ready, output reg [63:0] m_pixels,
    output reg [TAG_W-1:0] m_tag, output reg m_sample_valid,
    output wire req_valid, input wire req_ready,
    output wire [31:0] req_addr, output wire [4:0] req_len,
    input wire rsp_valid, output wire rsp_ready,
    input wire [127:0] rsp_data, input wire rsp_last,
    output reg [31:0] hit_count, output reg [31:0] miss_count,
    output reg [31:0] read_beats
);
    localparam IDLE=3'd0, REQUEST=3'd1, RECEIVE=3'd2, PREPARE=3'd3, DRAIN=3'd4, SET_LENGTH=3'd5;
    reg [2:0] state;
    reg [15:0] bank0[0:4095],bank1[0:4095],bank2[0:4095],bank3[0:4095];
    reg [15:0] valid_slots;
    reg [1:0] tag_cam[0:15];
    reg [6:0] tag_x[0:15],tag_y[0:15];
    reg [3:0] replace_next;
    reg [11:0] px[0:3],py[0:3];
    reg [3:0] slot[0:3];
    reg [3:0] present;
    reg [11:0] bank_addr[0:3];
    reg [11:0] pixel_addr;
    reg [3:0] victims[0:3];
    reg [15:0] chosen_slots;
    reg [3:0] group_resident;
    reg [6:0] prefetch_x;
    reg [2:0] prefetch_count;
    integer group_x,group_i,start_i,end_i,missing_i;
    reg run_open;
    reg [15:0] protected_mask,available_slots;
    reg [31:0] rotated_slots;
    reg [3:0] victim_offset;
    reg [1:0] missing_pixel;
    reg missing_found;
    integer i,j,k;
    // Balanced lowest-set-bit encoder. The rotated mask is nonempty because
    // at most four of sixteen slots are protected by a 2x2 neighborhood.
    function [3:0] first_available;
        input [15:0] mask;
        reg [7:0] half_mask;
        reg [3:0] quarter_mask;
        reg [1:0] pair_mask;
        begin
            first_available[3]=!(|mask[7:0]);
            half_mask=first_available[3]?mask[15:8]:mask[7:0];
            first_available[2]=!(|half_mask[3:0]);
            quarter_mask=first_available[2]?half_mask[7:4]:half_mask[3:0];
            first_available[1]=!(|quarter_mask[1:0]);
            pair_mask=first_available[1]?quarter_mask[3:2]:quarter_mask[1:0];
            first_available[0]=!pair_mask[0];
        end
    endfunction
    wire sample_ok=s_enable && ({1'b0,s_x}+13'd1<{1'b0,src_width})
                              && ({1'b0,s_y}+13'd1<{1'b0,src_height});
    wire all_present=&present;
    wire output_room=!m_valid || m_ready;
    assign s_ready=state==IDLE && !invalidate && output_room && (!sample_ok || all_present);

    // The source must hold s_* while valid and !ready. A miss is started from
    // that presented request, so no circular dependency on input readiness.
    always @* begin
        px[0]=s_x; py[0]=s_y; px[1]=s_x+12'd1; py[1]=s_y;
        px[2]=s_x; py[2]=s_y+12'd1; px[3]=s_x+12'd1; py[3]=s_y+12'd1;
        present=0;
        for(i=0;i<4;i=i+1) begin
            slot[i]=0; bank_addr[i]=0;
            for(j=0;j<16;j=j+1)
                if(valid_slots[j] && tag_cam[j]==s_cam && tag_x[j]==px[i][11:5] && tag_y[j]==py[i][11:5]) begin
                    present[i]=1'b1; slot[i]=j;
                end
        end
        pixel_addr=0;
        for(i=0;i<4;i=i+1) begin
            pixel_addr={slot[i],py[i][4:1],px[i][4:1]};
            bank_addr[{py[i][0],px[i][0]}]=pixel_addr;
        end
        missing_pixel=0; missing_found=0;
        for(i=0;i<4;i=i+1) if(!present[i] && !missing_found) begin
            missing_pixel=i; missing_found=1;
        end
        // Fetch a contiguous missing run within an aligned 1/2/4-tile group.
        // Already resident neighbors terminate the run instead of being read
        // again. Group alignment supports forward and reversed image scans.
        group_x=(px[missing_pixel][11:5]/PREFETCH_TILES)*PREFETCH_TILES;
        missing_i=px[missing_pixel][11:5]-group_x;
        group_resident=0;
        for(i=0;i<4;i=i+1) begin
            if(i>=PREFETCH_TILES || (group_x+i)*32>=src_width) group_resident[i]=1;
            for(j=0;j<16;j=j+1)
                if(valid_slots[j] && tag_cam[j]==s_cam && tag_y[j]==py[missing_pixel][11:5] && tag_x[j]==group_x+i)
                    group_resident[i]=1;
        end
        start_i=0;
        for(i=0;i<4;i=i+1) if(i<missing_i && group_resident[i]) start_i=i+1;
        end_i=missing_i;run_open=1;
        for(i=0;i<4;i=i+1) if(i>missing_i) begin
            if(group_resident[i]) run_open=0;
            if(run_open) end_i=i;
        end
        prefetch_x=group_x+start_i;
        prefetch_count=end_i-start_i+1;
        // Compare each physical tag once. Rotate the availability mask into
        // replacement order, then remove its lowest bit for each allocated
        // tile. This is the same round-robin choice as the nested scan but
        // avoids cascading four variable-index tag muxes and priority chains.
        protected_mask=0;
        for(i=0;i<16;i=i+1)
            for(k=0;k<4;k=k+1)
                if(valid_slots[i] && tag_cam[i]==s_cam && tag_x[i]==px[k][11:5] && tag_y[i]==py[k][11:5]) protected_mask[i]=1;
        rotated_slots={~protected_mask,~protected_mask}>>replace_next;
        available_slots=rotated_slots[15:0];chosen_slots=0;victim_offset=0;
        for(group_i=0;group_i<4;group_i=group_i+1) victims[group_i]=0;
        for(group_i=0;group_i<PREFETCH_TILES;group_i=group_i+1) begin
            victim_offset=first_available(available_slots);
            victims[group_i]=replace_next+victim_offset;
            if(group_i<prefetch_count) begin
                chosen_slots[victims[group_i]]=1;
                available_slots=available_slots&(available_slots-16'd1);
            end
        end
    end

    reg [15:0] sample_b0,sample_b1,sample_b2,sample_b3;
    reg [1:0] sample_parity;
    // Bank reads each use one registered port. Output reorder only permutes
    // those registered values, and therefore stays stable under backpressure.
    always @* begin
        case(sample_parity)
            2'b00: m_pixels={sample_b3,sample_b2,sample_b1,sample_b0};
            2'b01: m_pixels={sample_b2,sample_b3,sample_b0,sample_b1};
            2'b10: m_pixels={sample_b1,sample_b0,sample_b3,sample_b2};
            default: m_pixels={sample_b0,sample_b1,sample_b2,sample_b3};
        endcase
        if(!m_sample_valid) m_pixels=0;
    end

    reg [3:0] fill_slots[0:3];
    reg [2:0] fill_count;
    reg [1:0] fill_cam;
    reg [6:0] fill_x,fill_y;
    reg [5:0] fill_row,fill_rows;
    reg [4:0] fill_col,fill_row_beats;
    reg [31:0] fill_row_addr,burst_addr;
    reg [4:0] burst_len,burst_received;
    // Response elasticity keeps the shared DDR channel free to complete a
    // whole burst while dual-port RAM writes retire at one beat/two clocks.
    // Sixteen entries hold one maximum burst; requests reserve enough space
    // for every response beat, and subsequent row latency overlaps draining.
    // Keep the fall-through FIFO read asynchronous, including empty-to-write.
    reg [127:0] fifo_data[0:15] /* synthesis syn_ramstyle="lut_ram" */;
    reg [10:0] fifo_meta[0:15]; // slot[3:0], row[4:0], column beat[1:0]
    reg [3:0] fifo_wr,fifo_rd;
    reg [4:0] fifo_count;
    reg write_half;
    wire [10:0] head_meta=fifo_meta[fifo_rd];
    wire [127:0] head_data=fifo_data[fifo_rd];
    wire [11:0] write_addr={head_meta[10:7],head_meta[6:3],head_meta[1:0],2'b00}+(write_half?12'd2:12'd0);
    wire [63:0] write_four=write_half?head_data[127:64]:head_data[63:0];
    wire [31:0] next_burst_addr=burst_addr+{23'd0,burst_len,4'b0};
    wire [31:0] next_row_addr=fill_row_addr+src_stride;
    wire [12:0] remaining_x={1'b0,src_width}-{1'b0,prefetch_x,5'b0};
    wire [12:0] remaining_y={1'b0,src_height}-{1'b0,py[missing_pixel][11:5],5'b0};
    wire [4:0] initial_row_beats=(remaining_x>=prefetch_count*32)?{prefetch_count,2'b0}:((remaining_x+7)>>3);
    wire [31:0] prepared_row_addr=camera_bases[fill_cam*32+:32]+({20'd0,fill_y,5'b0}*src_stride)+{19'd0,fill_x,6'b0};
    function [4:0] clipped_len;
        input [31:0] addr; input [4:0] wanted;
        reg [8:0] to_page;
        begin
            to_page=(13'd4096-{1'b0,addr[11:0]})>>4;
            clipped_len=(wanted>to_page)?to_page[4:0]:wanted;
        end
    endfunction
    assign req_valid=state==REQUEST && ({1'b0,fifo_count}+{1'b0,burst_len}<=6'd16);
    assign req_addr=burst_addr;
    assign req_len=burst_len;
    assign rsp_ready=state==RECEIVE && fifo_count<16;

    wire enqueue=rsp_valid && rsp_ready;
    wire dequeue=fifo_count!=0 && write_half;
    wire fill_write=rst_n && fifo_count!=0;
    wire hit_read=s_valid && s_ready && sample_ok;
    wire bank_even_we=fill_write && !head_meta[2];
    wire bank_odd_we=fill_write && head_meta[2];
    // The read/write port must use ONE address expression for ADS to infer
    // a true dual-port DRM. Separate read/write subscripts look like three
    // logical ports even though filling and hit acceptance are exclusive.
    wire [11:0] bank0_port_a=bank_even_we?write_addr:bank_addr[0];
    wire [11:0] bank1_port_a=bank_even_we?write_addr:bank_addr[1];
    wire [11:0] bank2_port_a=bank_odd_we?write_addr:bank_addr[2];
    wire [11:0] bank3_port_a=bank_odd_we?write_addr:bank_addr[3];
    always @(posedge clk) begin
        if(enqueue) begin
            fifo_data[fifo_wr]<=rsp_data;
            fifo_meta[fifo_wr]<={fill_slots[fill_col[3:2]],fill_row[4:0],fill_col[1:0]};
        end
        if(!rst_n) begin fifo_wr<=0;fifo_rd<=0;fifo_count<=0;write_half<=0;end
        else begin
            if(enqueue) fifo_wr<=fifo_wr+1'b1;
            if(dequeue) fifo_rd<=fifo_rd+1'b1;
            case({enqueue,dequeue})
                2'b10:fifo_count<=fifo_count+1'b1;
                2'b01:fifo_count<=fifo_count-1'b1;
                default:begin end
            endcase
            if(fifo_count!=0) write_half<=!write_half;
            else write_half<=0;
        end
    end
    // Explicit two-port templates. Port A is shared between hit reads and
    // the first fill write, and port B performs the second fill write. The
    // RAM contents/read registers have no reset; validity guards the output.
    always @(posedge clk) begin
        if(bank_even_we) bank0[bank0_port_a]<=write_four[15:0];
        else if(hit_read) sample_b0<=bank0[bank0_port_a];
    end
    always @(posedge clk) begin
        if(fill_write && !head_meta[2]) bank0[write_addr+12'd1]<=write_four[47:32];
    end
    always @(posedge clk) begin
        if(bank_even_we) bank1[bank1_port_a]<=write_four[31:16];
        else if(hit_read) sample_b1<=bank1[bank1_port_a];
    end
    always @(posedge clk) begin
        if(fill_write && !head_meta[2]) bank1[write_addr+12'd1]<=write_four[63:48];
    end
    always @(posedge clk) begin
        if(bank_odd_we) bank2[bank2_port_a]<=write_four[15:0];
        else if(hit_read) sample_b2<=bank2[bank2_port_a];
    end
    always @(posedge clk) begin
        if(fill_write && head_meta[2]) bank2[write_addr+12'd1]<=write_four[47:32];
    end
    always @(posedge clk) begin
        if(bank_odd_we) bank3[bank3_port_a]<=write_four[31:16];
        else if(hit_read) sample_b3<=bank3[bank3_port_a];
    end
    always @(posedge clk) begin
        if(fill_write && head_meta[2]) bank3[write_addr+12'd1]<=write_four[63:48];
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE; valid_slots<=0; replace_next<=0; m_valid<=0;
            m_tag<=0; m_sample_valid<=0; sample_parity<=0;
            hit_count<=0; miss_count<=0; read_beats<=0;
            fill_count<=0;fill_slots[0]<=0;fill_slots[1]<=0;fill_slots[2]<=0;fill_slots[3]<=0;
            fill_cam<=0;fill_x<=0;fill_y<=0;fill_row<=0;fill_rows<=0;
            fill_col<=0;fill_row_beats<=0;fill_row_addr<=0;
            burst_addr<=0;burst_len<=0;burst_received<=0;
        end else begin
            if(m_valid && m_ready) m_valid<=0;
            if(s_valid && s_ready) begin
                m_valid<=1; m_tag<=s_tag; m_sample_valid<=sample_ok;
                sample_parity<={s_y[0],s_x[0]};
                if(sample_ok) hit_count<=hit_count+1'b1;
            end
            case(state)
                IDLE: if(invalidate) valid_slots<=0;
                else if(s_valid && sample_ok && !all_present) begin
                    fill_cam<=s_cam;fill_count<=prefetch_count;
                    fill_x<=prefetch_x; fill_y<=py[missing_pixel][11:5];
                    valid_slots<=valid_slots & ~chosen_slots;
                    fill_slots[0]<=victims[0];fill_slots[1]<=victims[1];
                    fill_slots[2]<=victims[2];fill_slots[3]<=victims[3];
                    replace_next<=victims[prefetch_count-1]+1'b1;
                    fill_row<=0; fill_col<=0;
                    fill_rows<=(remaining_y>=32)?6'd32:remaining_y[5:0];
                    fill_row_beats<=initial_row_beats;
                    fill_row_addr<=0; burst_addr<=0;
                    // Register the selected tile before source addressing,
                    // then register that address before the page comparison.
                    // Only a miss pays these two setup clocks; hits keep II=1.
                    burst_len<=0;
                    burst_received<=0; miss_count<=miss_count+prefetch_count; state<=PREPARE;
                end
                PREPARE: begin
                    fill_row_addr<=prepared_row_addr;burst_addr<=prepared_row_addr;
                    state<=SET_LENGTH;
                end
                SET_LENGTH: begin
                    burst_len<=clipped_len(burst_addr,fill_row_beats);
                    state<=REQUEST;
                end
                REQUEST: if(req_valid && req_ready) begin burst_received<=0; state<=RECEIVE; end
                RECEIVE: if(enqueue) begin
                    read_beats<=read_beats+1'b1;
                    fill_col<=fill_col+1'b1;
                    burst_received<=burst_received+1'b1;
                    if(burst_received+1'b1<burst_len) state<=RECEIVE;
                    else if(fill_col+1'b1<fill_row_beats) begin
                        burst_addr<=next_burst_addr;
                        burst_len<=clipped_len(next_burst_addr,fill_row_beats-fill_col-1'b1);
                        state<=REQUEST;
                    end else if(fill_row+1'b1<fill_rows) begin
                        fill_row<=fill_row+1'b1; fill_col<=0;
                        fill_row_addr<=next_row_addr; burst_addr<=next_row_addr;
                        burst_len<=clipped_len(next_row_addr,fill_row_beats);
                        state<=REQUEST;
                    end else state<=DRAIN;
                end
                DRAIN: if(fifo_count==0) begin
                        if(fill_count>=1) begin valid_slots[fill_slots[0]]<=1; tag_cam[fill_slots[0]]<=fill_cam;tag_x[fill_slots[0]]<=fill_x;tag_y[fill_slots[0]]<=fill_y;end
                        if(fill_count>=2) begin valid_slots[fill_slots[1]]<=1; tag_cam[fill_slots[1]]<=fill_cam;tag_x[fill_slots[1]]<=fill_x+1;tag_y[fill_slots[1]]<=fill_y;end
                        if(fill_count>=3) begin valid_slots[fill_slots[2]]<=1; tag_cam[fill_slots[2]]<=fill_cam;tag_x[fill_slots[2]]<=fill_x+2;tag_y[fill_slots[2]]<=fill_y;end
                        if(fill_count>=4) begin valid_slots[fill_slots[3]]<=1; tag_cam[fill_slots[3]]<=fill_cam;tag_x[fill_slots[3]]<=fill_x+3;tag_y[fill_slots[3]]<=fill_y;end
                        state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
