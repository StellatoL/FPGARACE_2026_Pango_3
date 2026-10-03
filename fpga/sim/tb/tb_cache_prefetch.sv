`timescale 1ns/1ps
// Directed burst-prefetch checks supplement the randomized cache/arbiter test.
module tb_cache_prefetch #(parameter PREFETCH_TILES=4);
    localparam W=134,H=35,STRIDE=272;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,s_valid=0,s_enable=1,invalidate=0,m_ready=1;
    reg [1:0] s_cam=0;reg [11:0] s_x=0,s_y=0;reg[31:0]s_tag=0;
    wire s_ready,m_valid,m_sample_valid;wire[63:0]m_pixels;wire[31:0]m_tag;
    wire req_valid,req_ready,rsp_valid,rsp_ready,rsp_last;
    wire[31:0]req_addr,hit_count,miss_count,read_beats;
    wire[4:0]req_len;wire[127:0]rsp_data;
    reg[127:0]bases={32'h30fc0,32'h20fc0,32'h10fc0,32'h0fc0};
    integer camera,xx,yy,p,cycles=0,samples=0,max_burst=0;
    integer reads_before,miss_before;
    reg [63:0]want;
    avm_tile_cache #(.TAG_W(32),.PREFETCH_TILES(PREFETCH_TILES)) dut(
        .clk(clk),.rst_n(rst_n),.s_valid(s_valid),.s_ready(s_ready),.s_enable(s_enable),
        .s_cam(s_cam),.s_x(s_x),.s_y(s_y),.s_tag(s_tag),.camera_bases(bases),
        .src_width(12'd134),.src_height(12'd35),.src_stride(16'd272),.invalidate(invalidate),
        .m_valid(m_valid),.m_ready(m_ready),.m_pixels(m_pixels),.m_tag(m_tag),.m_sample_valid(m_sample_valid),
        .req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
        .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last),
        .hit_count(hit_count),.miss_count(miss_count),.read_beats(read_beats));
    avm_memory_bfm #(.MEMORY_WORDS(16384),.CAPTURE_MBPS(0),.STRESS(0)) mem(
        .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),
        .req_addr(req_addr),.req_len(req_len),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last));
    function automatic [15:0] pixel(input integer c,input integer x,input integer y);
        pixel=(c<<14)^((y*113+x*17)&16'h3fff);
    endfunction
    function automatic [63:0] expected(input integer c,input integer x,input integer y);
        expected={pixel(c,x+1,y+1),pixel(c,x,y+1),pixel(c,x+1,y),pixel(c,x,y)};
    endfunction
    task automatic sample(input integer c,input integer x,input integer y,input integer en=1);
        begin
            @(negedge clk);s_valid=1;s_enable=en;s_cam=c;s_x=x;s_y=y;s_tag=s_tag+1;
            do @(posedge clk);while(!s_ready);
            @(negedge clk);s_valid=0;
            if(!m_valid || m_pixels!==(en && x+1<W && y+1<H?expected(c,x,y):64'd0)
               || m_tag!==s_tag || m_sample_valid!==(en && x+1<W && y+1<H))
                $fatal(1,"prefetch mismatch c=%0d x=%0d y=%0d got=%h",c,x,y,m_pixels);
            samples=samples+1;@(negedge clk);
        end
    endtask
    always @(posedge clk) begin
        cycles<=cycles+1;if(cycles>500000)$fatal(1,"prefetch watchdog");
        if(req_valid&&req_ready&&req_len>max_burst)max_burst=req_len;
        if(rst_n&&dut.fifo_count>16)$fatal(1,"response FIFO overflow");
        if(rst_n&&dut.state==0&&dut.fifo_count!=0)$fatal(1,"tile published before writes retired");
    end
    initial begin
        for(p=0;p<16384;p=p+1)mem.memory[p]=0;
        for(camera=0;camera<4;camera=camera+1)
            for(yy=0;yy<H;yy=yy+1)
                for(xx=0;xx<W;xx=xx+1)begin
                    p=16'h0fc0+camera*65536+yy*STRIDE+xx*2;
                    mem.memory[p>>4][(p&15)*8+:16]=pixel(camera,xx,yy);
                end
        repeat(4)@(negedge clk);rst_n=1;
        sample(0,1,1);
        if(read_beats!=PREFETCH_TILES*128 || miss_count!=PREFETCH_TILES)
            $fatal(1,"wrong initial group reads=%0d tiles=%0d",read_beats,miss_count);
        reads_before=read_beats;
        sample(0,PREFETCH_TILES*32-2,2);
        if(read_beats!=reads_before)$fatal(1,"prefetched neighbor caused another read");
        // Four-way crossing, clipped six-pixel right edge, three-row bottom.
        sample(0,127,31);sample(0,132,33);sample(0,31,31);
        sample(0,133,1);sample(0,1,34);sample(0,4095,4095,0);
        for(camera=1;camera<4;camera=camera+1)begin
            sample(camera,127,31);sample(camera,32,2);sample(camera,132,33);
        end
        // A resident output survives writes to replacement slots.
        sample(3,31,2);m_ready=0;
        sample(3,31,2);want=m_pixels;
        @(negedge clk);s_valid=1;s_enable=1;s_cam=0;s_x=127;s_y=31;s_tag=s_tag+1;
        repeat(3000)begin @(negedge clk);if(!m_valid||m_pixels!==want)$fatal(1,"prefetch overwrote stalled output");end
        m_ready=1;
        do @(posedge clk);while(!s_ready);
        @(negedge clk);s_valid=0;
        if(!m_valid||m_pixels!==expected(0,127,31))$fatal(1,"post-stall request mismatch");
        @(negedge clk);invalidate=1;@(negedge clk);invalidate=0;
        miss_before=miss_count;sample(0,1,1);
        if(miss_count!=miss_before+PREFETCH_TILES)$fatal(1,"invalidate group miss count");
        if(max_burst!=PREFETCH_TILES*4)$fatal(1,"no expected long burst max=%0d",max_burst);
        // Reset while received beats are still queued for RAM retirement.
        @(negedge clk);invalidate=1;@(negedge clk);invalidate=0;
        @(negedge clk);s_valid=1;s_enable=1;s_cam=3;s_x=0;s_y=0;s_tag=s_tag+1;
        wait(dut.fifo_count>=2);@(negedge clk);rst_n=0;s_valid=0;
        repeat(4)@(negedge clk);rst_n=1;
        sample(1,127,31);sample(1,132,33);
        $display("PASS tb_cache_prefetch group=%0d samples=%0d max_burst=%0d reset_with_queued_writes=1",PREFETCH_TILES,samples,max_burst);
        $finish;
    end
endmodule
