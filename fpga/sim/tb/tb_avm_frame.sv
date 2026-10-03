`timescale 1ns/1ps
`include "fixture.vh"
`ifndef AVM_OUTPUT_SEED
 `define AVM_OUTPUT_SEED 32'h12345678
`endif
`ifndef AVM_MEMORY_SEED
 `define AVM_MEMORY_SEED 32'h51ac35a7
`endif
module tb_avm_frame;
 localparam W=`AVM_WIDTH,H=`AVM_HEIGHT,N=W*H,FRAMES=`AVM_FEEDBACK?2:1;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,cfg_valid=0;wire cfg_ready;
 reg[1:0]cfg_kind=0;reg[8:0]cfg_index=0;reg[31:0]cfg_data=0;
 wire busy,done,m_valid;reg m_ready=1;wire[15:0]m_pixel;wire m_sof,m_eol;
 wire[31:0]frame_id,config_version,cycles,errors,miss_a,miss_b;
 wire req_valid,req_ready,rsp_valid,rsp_ready,rsp_last;
 wire[31:0]req_addr;wire[4:0]req_len;wire[127:0]rsp_data;
 wire[767:0]stat_sums;wire[127:0]stat_counts;
 avm_core #(.WIDTH(W),.HEIGHT(H),.CACHE_PREFETCH_TILES(`AVM_PREFETCH)) dut(
  .clk(clk),.rst_n(rst_n),.start(start),.frame_id(32'd123),
  .camera_bases(`AVM_BASES),.src_width(12'd`AVM_SRC_WIDTH),.src_height(12'd`AVM_SRC_HEIGHT),
  .src_stride(16'd`AVM_STRIDE),.desc_base(32'd`AVM_DESC_BASE),.route_base(32'd`AVM_ROUTE_BASE),
  .cfg_valid(cfg_valid),.cfg_ready(cfg_ready),.cfg_kind(cfg_kind),.cfg_index(cfg_index),.cfg_data(cfg_data),
  .busy(busy),.done(done),.output_frame_id(frame_id),.config_version(config_version),
  .m_valid(m_valid),.m_ready(m_ready),.m_pixel(m_pixel),.m_sof(m_sof),.m_eol(m_eol),
  .req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last),
  .stat_sums(stat_sums),.stat_counts(stat_counts),.cache_misses_a(miss_a),.cache_misses_b(miss_b),
  .cycle_count(cycles),.error_flags(errors));
 avm_memory_bfm #(.MEMORY_WORDS(`AVM_MEMORY_WORDS),.CAPTURE_MBPS(`AVM_CAPTURE_MBPS),.STRESS(`AVM_STRESS),.SEED(`AVM_MEMORY_SEED)) memory(
  .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last));
 reg[15:0]expected[0:N*FRAMES-1];reg[191:0]gains=`AVM_GAINS,feedback_gains=`AVM_FEEDBACK_GAINS;
 reg[767:0]expected_sums=`AVM_STAT_SUMS;reg[127:0]expected_counts=`AVM_STAT_COUNTS;
 string expect_path,feedback_path,out_path,stats_path,wave_path;integer fd,sfd,i,count=0;
 reg[31:0]first_cycles;
 reg[63:0]frame_read_base=0,frame_capture_base=0,frame_request_base=0;
 reg[31:0]frame_miss_a_base=0,frame_miss_b_base=0;
 reg measuring=0;
 reg[63:0]cache_wait_a=0,cache_wait_b=0,cache_fill_a=0,cache_fill_b=0;
 reg[63:0]output_blocked=0,request_blocked=0;
 reg[63:0]service_base=0,latency_base=0;
 reg[31:0]source_a_base=0,source_b_base=0;
 integer watchdog=0;reg[31:0]rng=`AVM_OUTPUT_SEED;
 reg held=0;reg[17:0]held_pixel;
 reg request_held=0,response_held=0;
 reg[36:0]held_request;reg[128:0]held_response;
 task cfg_write(input[1:0]kind,input[8:0]idx,input[31:0]value);
  begin
   @(negedge clk);cfg_valid=1;cfg_kind=kind;cfg_index=idx;cfg_data=value;
   do @(posedge clk);while(!cfg_ready);
   @(negedge clk);cfg_valid=0;
  end
 endtask
 initial begin
  if(!$value$plusargs("EXPECTED=%s",expect_path))$fatal(1,"EXPECTED required");
  if(!$value$plusargs("OUTPUT=%s",out_path))$fatal(1,"OUTPUT required");
  if(!$value$plusargs("STATS=%s",stats_path))$fatal(1,"STATS required");
  $readmemh(expect_path,expected,0,N-1);
  if(`AVM_FEEDBACK)begin
   if(!$value$plusargs("FEEDBACK_EXPECTED=%s",feedback_path))$fatal(1,"feedback expected required");
   $readmemh(feedback_path,expected,N,N*2-1);
  end
  fd=$fopen(out_path,"w");if(!fd)$fatal(1,"output open");
  if($value$plusargs("VCD=%s",wave_path))begin
   $dumpfile(wave_path);
   $dumpvars(0,clk,rst_n,start,busy,done,req_addr,req_len,req_valid,req_ready,
             rsp_valid,rsp_ready,rsp_last,m_valid,m_ready,m_sof,m_eol,m_pixel,config_version,count);
  end
  repeat(5)@(negedge clk);rst_n=1;
  for(i=0;i<12;i=i+1)cfg_write(0,i,{16'd0,gains[i*16+:16]});
  cfg_write(2,0,7);
  @(negedge clk);start=1;@(negedge clk);start=0;
  // Pending changes cannot alter this active frame. The next commit blocks
  // further writes until the next frame boundary, while video keeps running.
  repeat(50)@(negedge clk);
  if(`AVM_FEEDBACK)begin
   for(i=0;i<12;i=i+1)cfg_write(0,i,{16'd0,feedback_gains[i*16+:16]});
  end else begin cfg_write(0,0,16'd8192);cfg_write(1,128,9'd17);end
  cfg_write(2,0,8);
  wait(done);@(negedge clk);
  if(count!=N)$fatal(1,"pixel count %0d expected %0d",count,N);
  if(errors!=0)$fatal(1,"core errors %h",errors);
  if(frame_id!=123||config_version!=7)$fatal(1,"frame/config version changed");
  first_cycles=cycles;
  if(`AVM_FEEDBACK)begin
   wait(!busy);@(negedge clk);start=1;@(negedge clk);start=0;
   wait(done);@(negedge clk);
   if(count!=N*2||errors!=0||config_version!=8)$fatal(1,"feedback frame/config failure");
  end
  if(stat_sums!==expected_sums)$fatal(1,"statistics sums mismatch got=%h expected=%h",stat_sums,expected_sums);
  if(stat_counts!==expected_counts)$fatal(1,"statistics counts mismatch got=%h expected=%h",stat_counts,expected_counts);
  sfd=$fopen(stats_path,"w");
  $fwrite(sfd,"{\"cycles\":%0d,\"processing_ms\":%0.6f,\"first_frame_cycles\":%0d,\"frames\":%0d,\"pixels\":%0d,\"timed_frame_pixels\":%0d,\"cache_misses\":[%0d,%0d],\"read_beats\":%0d,\"capture_beats\":%0d,\"requests\":%0d,\"statistics\":[",cycles,cycles/100000.0,first_cycles,FRAMES,count,N,miss_a-frame_miss_a_base,miss_b-frame_miss_b_base,memory.read_beats-frame_read_base,memory.virtual_write_beats-frame_capture_base,memory.request_count-frame_request_base);
  for(i=0;i<4;i=i+1)begin
   if(i!=0)$fwrite(sfd,",");
   $fwrite(sfd,"{\"cameras\":[%0d,%0d],\"count\":%0d,\"sums\":[[%0d,%0d,%0d],[%0d,%0d,%0d]]}",i/2,2+i%2,stat_counts[i*32+:32],stat_sums[(i*6)*32+:32],stat_sums[(i*6+1)*32+:32],stat_sums[(i*6+2)*32+:32],stat_sums[(i*6+3)*32+:32],stat_sums[(i*6+4)*32+:32],stat_sums[(i*6+5)*32+:32]);
  end
  $fwrite(sfd,"],\"cache_wait_cycles\":[%0d,%0d],\"cache_fill_cycles\":[%0d,%0d],\"output_blocked_cycles\":%0d,\"request_blocked_cycles\":%0d,\"ddr_service_cycles\":%0d,\"ddr_read_latency_cycles\":%0d,\"source_read_beats\":[%0d,%0d]}\n",cache_wait_a,cache_wait_b,cache_fill_a,cache_fill_b,output_blocked,request_blocked,memory.service_cycles-service_base,memory.latency_cycles-latency_base,dut.CACHE[0].cache.read_beats-source_a_base,dut.CACHE[1].cache.read_beats-source_b_base);$fclose(sfd);$fclose(fd);
  $display("PASS: tb_avm_frame pixels=%0d cycles=%0d ms=%0.6f misses=%0d/%0d reads=%0d capture=%0d",count,cycles,cycles/100000.0,miss_a,miss_b,memory.read_beats,memory.virtual_write_beats);
  $finish;
 end
 always @(negedge clk)if(rst_n)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  if(`AVM_STRESS)m_ready=(rng[2:0]!=0)&&!(watchdog%2000<150);else m_ready=1;
 end
 always @(posedge clk)begin
  if(rst_n)begin
   if(start&&!busy)begin
    frame_read_base<=memory.read_beats;frame_capture_base<=memory.virtual_write_beats;
    frame_request_base<=memory.request_count;frame_miss_a_base<=miss_a;frame_miss_b_base<=miss_b;
    measuring<=1;cache_wait_a<=0;cache_wait_b<=0;cache_fill_a<=0;cache_fill_b<=0;
    output_blocked<=0;request_blocked<=0;
    service_base<=memory.service_cycles;latency_base<=memory.latency_cycles;
    source_a_base<=dut.CACHE[0].cache.read_beats;source_b_base<=dut.CACHE[1].cache.read_beats;
   end else if(measuring)begin
    if(done)measuring<=0;
    if(dut.CACHE[0].cache.s_valid&&!dut.CACHE[0].cache.s_ready)cache_wait_a<=cache_wait_a+1;
    if(dut.CACHE[1].cache.s_valid&&!dut.CACHE[1].cache.s_ready)cache_wait_b<=cache_wait_b+1;
    if(dut.CACHE[0].cache.state!=0)cache_fill_a<=cache_fill_a+1;
    if(dut.CACHE[1].cache.state!=0)cache_fill_b<=cache_fill_b+1;
    if(m_valid&&!m_ready)output_blocked<=output_blocked+1;
    if(req_valid&&!req_ready)request_blocked<=request_blocked+1;
   end
   watchdog<=watchdog+1;
   if(watchdog>100000000)$fatal(1,"watchdog state=%0d macro=%0d issue=%0d written=%0d",dut.state,dut.macro_index,dut.issue_i,dut.written);
   if(held&&(m_valid!==1'b1||{m_sof,m_eol,m_pixel}!==held_pixel))$fatal(1,"unstable stalled output");
   held<=m_valid&&!m_ready;held_pixel<={m_sof,m_eol,m_pixel};
   if(request_held&&(req_valid!==1'b1||{req_addr,req_len}!==held_request))$fatal(1,"unstable stalled DDR request");
   if(response_held&&(rsp_valid!==1'b1||{rsp_data,rsp_last}!==held_response))$fatal(1,"unstable stalled DDR response");
   request_held<=req_valid&&!req_ready;held_request<={req_addr,req_len};
   response_held<=rsp_valid&&!rsp_ready;held_response<={rsp_data,rsp_last};
   if(m_valid&&m_ready)begin
    if(count>=N*FRAMES)$fatal(1,"too many pixels");
    if(m_pixel!==expected[count])$fatal(1,"pixel %0d xy=(%0d,%0d) got=%h expected=%h",count,count%W,count/W,m_pixel,expected[count]);
    if(m_sof!==(count%N==0)||m_eol!==(count%W==W-1))$fatal(1,"bad SOF/EOL %0d",count);
    $fwrite(fd,"%04h\n",m_pixel);count<=count+1;
   end
  end
 end
endmodule
