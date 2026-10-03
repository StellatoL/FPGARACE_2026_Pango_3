`timescale 1ns/1ps
`include "gate_fixture.vh"
// External-interface scoreboard for a fixed-size synthesis netlist. No DUT
// hierarchical names are used: PDS is free to flatten/rename internal logic.
module tb_avm_gate;
 localparam W=`AVM_WIDTH,H=`AVM_HEIGHT,N=W*H;
 localparam WATCHDOG_CYCLES=(N*100>2000000)?N*100:2000000;
 localparam [127:0] BASES=`AVM_BASES;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,cfg_valid=0;wire cfg_ready;
 reg [1:0]cfg_kind=0;reg [8:0]cfg_index=0;reg [31:0]cfg_data=0;
 wire busy,done,m_valid;reg m_ready=1;
 wire [15:0]m_pixel;wire m_sof,m_eol;
 wire [31:0]frame_id,config_version,cycles,errors,miss_a,miss_b;
 wire req_valid,req_ready,rsp_valid,rsp_ready,rsp_last;
 wire [31:0]req_addr;wire [4:0]req_len;wire [127:0]rsp_data;
 wire [767:0]stat_sums;wire [127:0]stat_counts;
 // Pango simulation primitives locate this named global reset instance.
 GTP_GRS GRS_INST(.GRS_N(rst_n));
 // The generated avm_core has no parameter declaration. Its dimensions must
 // match the fixture at synthesis time; the runner checks build provenance.
 avm_core dut(
  .clk(clk),.rst_n(rst_n),.start(start),.frame_id(32'd123),
  .camera_bases(BASES),.src_width(12'd`AVM_SRC_WIDTH),.src_height(12'd`AVM_SRC_HEIGHT),
  .src_stride(16'd`AVM_STRIDE),.desc_base(32'd`AVM_DESC_BASE),.route_base(32'd`AVM_ROUTE_BASE),
  .cfg_valid(cfg_valid),.cfg_ready(cfg_ready),.cfg_kind(cfg_kind),.cfg_index(cfg_index),.cfg_data(cfg_data),
  .busy(busy),.done(done),.output_frame_id(frame_id),.config_version(config_version),
  .m_valid(m_valid),.m_ready(m_ready),.m_pixel(m_pixel),.m_sof(m_sof),.m_eol(m_eol),
  .req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last),
  .stat_sums(stat_sums),.stat_counts(stat_counts),.cache_misses_a(miss_a),.cache_misses_b(miss_b),
  .cycle_count(cycles),.error_flags(errors));
 avm_memory_bfm #(.MEMORY_WORDS(`AVM_MEMORY_WORDS),.CAPTURE_MBPS(221.184),.STRESS(0)) memory(
  .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last));
 reg [15:0]expected[0:N-1];reg [191:0]gains=`AVM_GAINS;
 reg [767:0]expected_sums=`AVM_STAT_SUMS;reg [127:0]expected_counts=`AVM_STAT_COUNTS;
 string expect_path,out_path,stats_path;
 integer fd,sfd,i,camera,count=0,watchdog=0;
 integer requests=0,read_beats=0,source_requests=0,descriptor_requests=0,route_requests=0;
 integer output_stalls=0,request_stalls=0,configuration_writes=0;
 reg held=0,request_held=0;reg [17:0]held_pixel;reg [36:0]held_request;

 task cfg_write(input [1:0]kind,input [8:0]idx,input [31:0]value);
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
  $readmemh(expect_path,expected);
  fd=$fopen(out_path,"w");if(!fd)$fatal(1,"output open");
  repeat(10)@(negedge clk);rst_n=1;
  for(i=0;i<12;i=i+1)cfg_write(0,i,{16'd0,gains[i*16+:16]});
  cfg_write(2,0,7);
  @(negedge clk);start=1;@(negedge clk);start=0;
  // Non-unity fixture gains are applied at start. A pending change must not
  // affect the active frame's pixels or committed configuration version.
  repeat(50)@(negedge clk);
  cfg_write(0,0,16'd8192);cfg_write(1,128,9'd17);cfg_write(2,0,8);
  wait(done);@(negedge clk);
  if(count!=N)$fatal(1,"pixel count %0d expected %0d",count,N);
  if(errors!==32'd0)$fatal(1,"core errors %h",errors);
  if(frame_id!==32'd123||config_version!==32'd7)$fatal(1,"frame/config version changed");
  if(stat_sums!==expected_sums)$fatal(1,"statistics sums mismatch got=%h expected=%h",stat_sums,expected_sums);
  if(stat_counts!==expected_counts)$fatal(1,"statistics counts mismatch got=%h expected=%h",stat_counts,expected_counts);
  if(requests==0||read_beats==0||source_requests==0||descriptor_requests==0||route_requests==0)
   $fatal(1,"missing external DDR traffic req=%0d beats=%0d source=%0d desc=%0d route=%0d",requests,read_beats,source_requests,descriptor_requests,route_requests);
  if(configuration_writes!=16)$fatal(1,"configuration handshakes %0d",configuration_writes);
  if(output_stalls==0)$fatal(1,"output backpressure not covered");
  sfd=$fopen(stats_path,"w");if(!sfd)$fatal(1,"stats open");
  $fwrite(sfd,"{\"cycles\":%0d,\"pixels\":%0d,\"frame_id\":%0d,\"config_version\":%0d,\"configuration_writes\":%0d,\"cache_misses\":[%0d,%0d],\"requests\":%0d,\"read_beats\":%0d,\"source_requests\":%0d,\"descriptor_requests\":%0d,\"route_requests\":%0d,\"output_stalls\":%0d,\"request_stalls\":%0d,\"statistics\":[",cycles,count,frame_id,config_version,configuration_writes,miss_a,miss_b,requests,read_beats,source_requests,descriptor_requests,route_requests,output_stalls,request_stalls);
  for(i=0;i<4;i=i+1)begin
   if(i!=0)$fwrite(sfd,",");
   $fwrite(sfd,"{\"cameras\":[%0d,%0d],\"count\":%0d,\"sums\":[[%0d,%0d,%0d],[%0d,%0d,%0d]]}",i/2,2+i%2,stat_counts[i*32+:32],stat_sums[(i*6)*32+:32],stat_sums[(i*6+1)*32+:32],stat_sums[(i*6+2)*32+:32],stat_sums[(i*6+3)*32+:32],stat_sums[(i*6+4)*32+:32],stat_sums[(i*6+5)*32+:32]);
  end
  $fwrite(sfd,"]}\n");$fclose(sfd);$fclose(fd);
  $display("PASS: tb_avm_gate pixels=%0d cycles=%0d requests=%0d reads=%0d source=%0d output_stalls=%0d",count,cycles,requests,read_beats,source_requests,output_stalls);
  $finish;
 end

 // Exercise stalls with a deterministic duty cycle, without internal probes.
 always @(negedge clk)if(rst_n)m_ready=(watchdog%13!=0);
 always @(posedge clk)begin
  if(rst_n)begin
   watchdog<=watchdog+1;
   if(watchdog>WATCHDOG_CYCLES)$fatal(1,"gate watchdog pixels=%0d requests=%0d reads=%0d busy=%b errors=%h",count,requests,read_beats,busy,errors);
   if(cfg_valid&&cfg_ready)configuration_writes<=configuration_writes+1;
   if(held&&(!m_valid||{m_sof,m_eol,m_pixel}!==held_pixel))$fatal(1,"unstable stalled output");
   held<=m_valid&&!m_ready;held_pixel<={m_sof,m_eol,m_pixel};
   if(request_held&&(!req_valid||{req_addr,req_len}!==held_request))$fatal(1,"unstable stalled DDR request");
   request_held<=req_valid&&!req_ready;held_request<={req_addr,req_len};
   if(m_valid&&!m_ready)output_stalls<=output_stalls+1;
   if(req_valid&&!req_ready)request_stalls<=request_stalls+1;
   if(req_valid&&req_ready)begin
    requests<=requests+1;
    if(req_addr==`AVM_DESC_BASE)descriptor_requests<=descriptor_requests+1;
    if(req_addr==`AVM_ROUTE_BASE)route_requests<=route_requests+1;
    for(camera=0;camera<4;camera=camera+1)
     if(req_addr>=BASES[camera*32+:32]&&req_addr<BASES[camera*32+:32]+`AVM_SRC_HEIGHT*`AVM_STRIDE)
      source_requests<=source_requests+1;
   end
   if(rsp_valid&&rsp_ready)read_beats<=read_beats+1;
   if(m_valid&&m_ready)begin
    if(count>=N)$fatal(1,"too many pixels");
    if(m_pixel!==expected[count])$fatal(1,"pixel %0d xy=(%0d,%0d) got=%h expected=%h",count,count%W,count/W,m_pixel,expected[count]);
    if(m_sof!==(count==0)||m_eol!==(count%W==W-1))$fatal(1,"bad SOF/EOL %0d",count);
    $fwrite(fd,"%04h\n",m_pixel);count<=count+1;
   end
  end
 end
endmodule
