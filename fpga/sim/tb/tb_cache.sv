`timescale 1ns/1ps
module tb_cache;
    localparam W=70,H=67,STRIDE=144;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,s_valid=0,s_enable=1,invalidate=0;
    reg [1:0] s_cam=0;
    reg [11:0] s_x=0,s_y=0;
    reg [31:0] s_tag=0;
    wire s_ready,m_valid,m_sample_valid;
    reg m_ready=0;
    wire [63:0] m_pixels;
    wire [31:0] m_tag,hit_count,miss_count,cache_beats;
    wire cache_req_valid,cache_req_ready,cache_rsp_valid,cache_rsp_ready,cache_rsp_last;
    wire [31:0] cache_req_addr;
    wire [4:0] cache_req_len;
    wire [127:0] cache_rsp_data;
    wire [2:0] c_req_valid,c_req_ready,c_rsp_valid,c_rsp_ready,c_rsp_last;
    wire [95:0] c_req_addr;
    wire [14:0] c_req_len;
    wire [383:0] c_rsp_data;
    reg lut_valid=0,other_valid=0;
    reg [31:0] lut_addr=0,other_addr=0;
    reg [4:0] lut_len=0,other_len=0;
    wire mem_req_valid,mem_req_ready,mem_rsp_valid,mem_rsp_ready,mem_rsp_last;
    wire [31:0] mem_req_addr;
    wire [4:0] mem_req_len;
    wire [127:0] mem_rsp_data;
    reg [31:0] randstate=32'h1edc6f41;
    integer ready_mode=0;
    integer wr=0,rd=0,samples_total=0,cycles=0,lut_done=0,other_done=0;
    reg [63:0] expected_pixels[0:4095];
    reg [31:0] expected_tags[0:4095];
    reg expected_valid[0:4095];
    reg was_stalled=0;
    reg [96:0] stalled_payload;
    integer lut_index=0,other_index=0;
    integer cam,x,y,word,p,ii,t0,t1,miss_before;
    reg [127:0] bases;
    assign c_req_valid={other_valid,cache_req_valid,lut_valid};
    assign c_req_addr={other_addr,cache_req_addr,lut_addr};
    assign c_req_len={other_len,cache_req_len,lut_len};
    assign c_rsp_ready={randstate[9],cache_rsp_ready,randstate[7]};
    assign cache_req_ready=c_req_ready[1];
    assign cache_rsp_valid=c_rsp_valid[1];
    assign cache_rsp_data=c_rsp_data[255:128];
    assign cache_rsp_last=c_rsp_last[1];
    avm_tile_cache #(.TAG_W(32)) dut(
        .clk(clk),.rst_n(rst_n),.s_valid(s_valid),.s_ready(s_ready),.s_enable(s_enable),
        .s_cam(s_cam),.s_x(s_x),.s_y(s_y),.s_tag(s_tag),.camera_bases(bases),
        .src_width(12'd70),.src_height(12'd67),.src_stride(16'd144),.invalidate(invalidate),
        .m_valid(m_valid),.m_ready(m_ready),.m_pixels(m_pixels),.m_tag(m_tag),.m_sample_valid(m_sample_valid),
        .req_valid(cache_req_valid),.req_ready(cache_req_ready),.req_addr(cache_req_addr),.req_len(cache_req_len),
        .rsp_valid(cache_rsp_valid),.rsp_ready(cache_rsp_ready),.rsp_data(cache_rsp_data),.rsp_last(cache_rsp_last),
        .hit_count(hit_count),.miss_count(miss_count),.read_beats(cache_beats));
    avm_mem_arbiter arb(.clk(clk),.rst_n(rst_n),.c_req_valid(c_req_valid),.c_req_ready(c_req_ready),
        .c_req_addr(c_req_addr),.c_req_len(c_req_len),.c_rsp_valid(c_rsp_valid),.c_rsp_ready(c_rsp_ready),
        .c_rsp_data(c_rsp_data),.c_rsp_last(c_rsp_last),.req_valid(mem_req_valid),.req_ready(mem_req_ready),
        .req_addr(mem_req_addr),.req_len(mem_req_len),.rsp_valid(mem_rsp_valid),.rsp_ready(mem_rsp_ready),
        .rsp_data(mem_rsp_data),.rsp_last(mem_rsp_last));
    avm_memory_bfm #(.MEMORY_WORDS(16384),.CAPTURE_MBPS(221.184),.STRESS(1)) mem(
        .clk(clk),.rst_n(rst_n),.req_valid(mem_req_valid),.req_ready(mem_req_ready),
        .req_addr(mem_req_addr),.req_len(mem_req_len),.rsp_valid(mem_rsp_valid),.rsp_ready(mem_rsp_ready),
        .rsp_data(mem_rsp_data),.rsp_last(mem_rsp_last));
    function automatic [15:0] pixel(input integer camera,input integer xx,input integer yy);
        pixel=(camera<<14)^((yy*97+xx*13)&16'h3fff);
    endfunction
    function automatic [63:0] neighborhood(input integer camera,input integer xx,input integer yy);
        neighborhood={pixel(camera,xx+1,yy+1),pixel(camera,xx,yy+1),pixel(camera,xx+1,yy),pixel(camera,xx,yy)};
    endfunction
    always @(negedge clk) begin
        randstate<={randstate[30:0],randstate[31]^randstate[21]^randstate[1]^randstate[0]};
        if(ready_mode==0) m_ready<=randstate[2:0]!=0;
        else m_ready<=ready_mode==1;
    end
    always @(posedge clk) begin
        cycles<=cycles+1;
        if(cycles>3000000) $fatal(1,"cache deadlock watchdog wr=%0d rd=%0d state=%0d",wr,rd,dut.state);
        if(!rst_n) begin wr=0;rd=0;was_stalled=0;lut_done=0;other_done=0;end
        else begin
            if(was_stalled && (!m_valid || {m_sample_valid,m_tag,m_pixels}!==stalled_payload))
                $fatal(1,"cache output changed while stalled");
            was_stalled=m_valid&&!m_ready;
            stalled_payload={m_sample_valid,m_tag,m_pixels};
            if(s_valid&&s_ready) begin
                expected_valid[wr]=s_enable&&(s_x+1<W)&&(s_y+1<H);
                expected_tags[wr]=s_tag;
                expected_pixels[wr]=expected_valid[wr]?neighborhood(s_cam,s_x,s_y):64'd0;
                wr=wr+1;
            end
            if(m_valid&&m_ready) begin
                if(rd>=wr) $fatal(1,"cache produced unexpected output");
                if(m_pixels!==expected_pixels[rd] || m_tag!==expected_tags[rd] || m_sample_valid!==expected_valid[rd])
                    $fatal(1,"cache mismatch n=%0d tag=%h expected_tag=%h actual=%h expected=%h valid=%b/%b",rd,m_tag,expected_tags[rd],m_pixels,expected_pixels[rd],m_sample_valid,expected_valid[rd]);
                rd=rd+1;samples_total=samples_total+1;
            end
            if(c_req_valid[0]&&c_req_ready[0]) lut_index=lut_addr>>4;
            if(c_req_valid[2]&&c_req_ready[2]) other_index=other_addr>>4;
            if(c_rsp_valid[0]&&c_rsp_ready[0]) begin
                if(c_rsp_data[127:0]!==mem.memory[lut_index]) $fatal(1,"arbiter LUT response mismatch");
                lut_index=lut_index+1;if(c_rsp_last[0]) lut_done=lut_done+1;
            end
            if(c_rsp_valid[2]&&c_rsp_ready[2]) begin
                if(c_rsp_data[383:256]!==mem.memory[other_index]) $fatal(1,"arbiter B response mismatch");
                other_index=other_index+1;if(c_rsp_last[2]) other_done=other_done+1;
            end
        end
    end
    task automatic sample(input integer camera,input integer xx,input integer yy,input integer en=1);
        begin
            @(negedge clk);s_valid=1;s_enable=en;s_cam=camera;s_x=xx;s_y=yy;s_tag=s_tag+1;
            do @(posedge clk); while(!s_ready);
            @(negedge clk);s_valid=0;
        end
    endtask
    task automatic drain;
        begin wait(rd==wr);@(negedge clk);end
    endtask
    task automatic run_lut;
        integer n,goal;
        begin for(n=0;n<80;n=n+1) begin
            @(negedge clk);goal=lut_done+1;lut_addr=16'h1000+(n%32)*16;lut_len=1+(n%16);lut_valid=1;
            do @(posedge clk); while(!c_req_ready[0]);
            @(negedge clk);lut_valid=0;wait(lut_done==goal);
        end end
    endtask
    task automatic run_other;
        integer n,goal;
        begin for(n=0;n<80;n=n+1) begin
            @(negedge clk);goal=other_done+1;other_addr=16'h5000+(n%32)*16;other_len=1+(n%16);other_valid=1;
            do @(posedge clk); while(!c_req_ready[2]);
            @(negedge clk);other_valid=0;wait(other_done==goal);
        end end
    endtask
    initial begin
        bases={32'hcfc0,32'h8fc0,32'h4fc0,32'h0fc0};
        for(word=0;word<16384;word=word+1) mem.memory[word]=0;
        for(cam=0;cam<4;cam=cam+1)
            for(y=0;y<H;y=y+1)
                for(x=0;x<W;x=x+1) begin
                    p=16'h0fc0+cam*16'h4000+y*STRIDE+x*2;
                    mem.memory[p>>4][(p&15)*8+:16]=pixel(cam,x,y);
                end
        repeat(5) @(negedge clk);rst_n=1;
        // Presented miss must make forward progress with input ready low.
        sample(0,2,2);drain();
        // Hit path, all parities, uninterrupted input and output: II=1.
        ready_mode=1;
        @(negedge clk);t0=cycles;
        for(ii=0;ii<400;ii=ii+1) begin
            s_valid=1;s_enable=1;s_cam=0;s_x=2+(ii%16);s_y=2+((ii/16)%16);s_tag=s_tag+1;
            @(posedge clk);if(!s_ready) $fatal(1,"resident hit did not sustain II1");
            @(negedge clk);
        end
        t1=cycles;s_valid=0;drain();
        if(t1-t0!=400) $fatal(1,"II1 measurement wrong: %0d",t1-t0);
        ready_mode=0;
        fork
            run_lut();run_other();
            begin
                // Force four-tile neighborhoods, replacements, partial edge
                // rows, multiple camera tags, and bursts crossing page edges.
                for(cam=0;cam<4;cam=cam+1) begin
                    sample(cam,31,31);sample(cam,63,31);sample(cam,31,63);sample(cam,63,63);
                    sample(cam,68,65);sample(cam,0,0);sample(cam,32,32);
                end
                for(ii=0;ii<128;ii=ii+1) sample(ii%4,(ii*17)%69,(ii*29)%66);
            end
        join
        drain();
        sample(0,0,0,0);sample(0,69,1);sample(1,1,66);sample(3,4095,4095);drain();
        // Stall an accepted hit, then present a miss. Fill may run, but the
        // held output must remain exact and ready cannot accept a second item.
        sample(0,5,5);drain();ready_mode=2;
        sample(0,5,5);
        @(negedge clk);s_valid=1;s_cam=3;s_x=63;s_y=63;s_tag=s_tag+1;
        repeat(4000) @(negedge clk);
        if(!m_valid || s_ready) $fatal(1,"held output ownership failed");
        ready_mode=0;
        do @(posedge clk); while(!s_ready);
        @(negedge clk);s_valid=0;drain();
        miss_before=miss_count;
        @(negedge clk);invalidate=1;@(negedge clk);invalidate=0;
        sample(3,63,63);drain();
        if(miss_count<=miss_before) $fatal(1,"invalidate did not drop tags");
        if(mem.virtual_write_beats==0) $fatal(1,"capture load did not consume slots");
        $display("CACHE_PRE_RESET samples=%0d misses=%0d reads=%0d capture_beats=%0d requests=%0d",samples_total,miss_count,cache_beats,mem.virtual_write_beats,mem.request_count);
        // Reset during active fill; no stale sample may emerge afterward.
        @(negedge clk);invalidate=1;@(negedge clk);invalidate=0;
        @(negedge clk);s_valid=1;s_cam=1;s_x=10;s_y=10;s_tag=s_tag+1;
        wait(cache_req_valid);repeat(3) @(negedge clk);rst_n=0;s_valid=0;
        repeat(5) @(negedge clk);rst_n=1;
        sample(2,31,31);drain();
        repeat(20) @(negedge clk);
        $display("PASS tb_cache samples=%0d hit_II=1 arbiter_clients=3 stress=1 reset_during_fill=1",samples_total);
        $finish;
    end
endmodule
