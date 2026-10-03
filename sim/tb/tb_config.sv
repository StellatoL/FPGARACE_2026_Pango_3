`timescale 1ns/1ps
// Real small frames use inactive descriptors and black routes. This isolates
// control semantics while still exercising the complete loader/stream/reorder.
module tb_config;
 localparam W=17,H=17;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,cfg_valid=0,m_ready=0;
 reg[31:0]frame_id=0,cfg_data=0;reg[1:0]cfg_kind=0;reg[8:0]cfg_index=0;
 wire cfg_ready,busy,done,m_valid,m_sof,m_eol;
 wire[15:0]m_pixel;wire[31:0]output_frame_id,config_version;
 wire req_valid,req_ready,rsp_ready;wire[31:0]req_addr;wire[4:0]req_len;
 reg rsp_valid=0,rsp_last=0;reg[127:0]rsp_data=0;
 wire[767:0]stat_sums;wire[127:0]stat_counts;
 wire[31:0]miss_a,miss_b,cycle_count,error_flags;
 avm_core #(.WIDTH(W),.HEIGHT(H)) dut(
  .clk(clk),.rst_n(rst_n),.start(start),.frame_id(frame_id),
  .camera_bases(128'b0),.src_width(12'd32),.src_height(12'd32),.src_stride(16'd64),
  .desc_base(32'hff0),.route_base(32'h2ff0),
  .cfg_valid(cfg_valid),.cfg_ready(cfg_ready),.cfg_kind(cfg_kind),.cfg_index(cfg_index),.cfg_data(cfg_data),
  .busy(busy),.done(done),.output_frame_id(output_frame_id),.config_version(config_version),
  .m_valid(m_valid),.m_ready(m_ready),.m_pixel(m_pixel),.m_sof(m_sof),.m_eol(m_eol),
  .req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last),
  .stat_sums(stat_sums),.stat_counts(stat_counts),.cache_misses_a(miss_a),.cache_misses_b(miss_b),
  .cycle_count(cycle_count),.error_flags(error_flags));

 // Single outstanding burst, variable request/response latency. All data zero.
 reg outstanding=0;integer wait_clocks=0,remaining=0,tick=0;
 integer desc_split=0,desc_resume=0,route_split=0,route_resume=0,requests=0;
 assign req_ready=rst_n&&!outstanding&&(tick%3!=0);
 always @(posedge clk)begin
  tick<=tick+1;
  if(!rst_n)begin outstanding<=0;rsp_valid<=0;rsp_last<=0;wait_clocks<=0;remaining<=0;end
  else begin
   if(req_valid&&req_ready)begin
    if(req_addr[3:0]!=0||req_len<1||req_len>16||req_addr[11:0]+req_len*16>4096)
     $fatal(1,"illegal burst address=%h length=%0d",req_addr,req_len);
    if(req_addr==32'hff0)begin if(req_len!=1)$fatal(1,"descriptor page split");desc_split++;end
    if(req_addr==32'h1000)begin if(req_len!=3)$fatal(1,"descriptor resume index");desc_resume++;end
    if(req_addr==32'h2ff0)begin if(req_len!=1)$fatal(1,"route page split");route_split++;end
    if(req_addr==32'h3000)begin if(req_len!=16)$fatal(1,"route resume index");route_resume++;end
    outstanding<=1;remaining<=req_len;wait_clocks<=2+(tick%5);requests++;
   end
   if(outstanding)begin
    if(wait_clocks>0)wait_clocks<=wait_clocks-1;
    else if(!rsp_valid)begin rsp_valid<=1;rsp_last<=remaining==1;rsp_data<=0;end
    else if(rsp_ready)begin
     rsp_valid<=0;remaining<=remaining-1;
     if(remaining==1)outstanding<=0;
     else wait_clocks<=tick%2;
    end
   end
  end
 end

 reg[191:0]expected_gains={12{16'd4096}};
 reg[8:0]expected_alpha[0:256];reg[31:0]expected_version=0,expected_frame=0;
 integer received=0,completed=0,frame_age=0,i;
 bit monitor=0,seen_done=0;
 always @(negedge clk)begin
  if(!rst_n)m_ready=0;
  else m_ready=(frame_age>150)&&($urandom_range(0,4)!=0);
 end
 always @(posedge clk)begin
  if(!rst_n||start)begin received=0;frame_age=0;seen_done=0;end
  else begin
   if(busy)frame_age++;
   if(m_valid&&m_ready)begin
    if(received>=W*H||m_pixel!==16'd0||m_sof!==(received==0)||m_eol!==(received%W==W-1))
     $fatal(1,"config frame output mismatch pixel=%0d",received);
    received++;
   end
   if(done)begin
    if(received!=W*H||seen_done)$fatal(1,"config frame done mismatch");
    if(stat_sums!==0||stat_counts!==0||error_flags!==0)$fatal(1,"unexpected frame status");
    seen_done=1;completed++;
   end
  end
  // Observe registers after the clock edge that performs an atomic commit.
  #1;
  if(rst_n&&monitor)begin
   if(dut.gains!==expected_gains||config_version!==expected_version)$fatal(1,"non-atomic gain/version update");
   for(integer n=0;n<257;n++)if(dut.alpha[n]!==expected_alpha[n])$fatal(1,"non-atomic alpha update index=%0d",n);
   if(busy&&output_frame_id!==expected_frame)$fatal(1,"frame ID was not latched");
  end
 end
 task automatic write_config(input integer kind,index,input reg[31:0]data);
  begin
   @(negedge clk);cfg_kind=kind;cfg_index=index;cfg_data=data;cfg_valid=1;
   @(posedge clk);while(!cfg_ready)@(posedge clk);
   @(negedge clk);cfg_valid=0;
  end
 endtask
 task automatic blocked_write;
  reg[191:0]saved;
  begin
   saved=dut.shadow_gains;
   @(negedge clk);cfg_kind=0;cfg_index=0;cfg_data=9999;cfg_valid=1;
   repeat(3)begin @(posedge clk);if(cfg_ready)$fatal(1,"pending config accepted mutation");end
   @(negedge clk);cfg_valid=0;
   if(dut.shadow_gains!==saved)$fatal(1,"blocked mutation reached shadow");
  end
 endtask
 task automatic launch(input integer id,input bit race_write);
  begin
   @(negedge clk);expected_frame=id;frame_id=id;start=1;
   if(race_write)begin cfg_valid=1;cfg_kind=0;cfg_index=0;cfg_data=9999;end
   @(posedge clk);if(cfg_ready)$fatal(1,"start did not protect configuration");
   #1;
   if(dut.gains!==expected_gains||config_version!==expected_version)$fatal(1,"configuration missing at launch edge");
   for(integer n=0;n<257;n++)if(dut.alpha[n]!==expected_alpha[n])$fatal(1,"alpha missing at launch edge");
   @(negedge clk);start=0;cfg_valid=0;
   #1;
   if(!busy)$fatal(1,"frame failed to launch");
   frame_id=id+1000; // The external input may change after the launch edge.
  end
 endtask
 task automatic finish_frame;
  integer timeout;
  begin
   timeout=0;
   while(busy)begin
    @(negedge clk);timeout++;
    if(timeout>10000)$fatal(1,"frame timeout state=%0d macro=%0d issue=%0d written=%0d recv=%0d req=%0d/%0d response=%0d/%0d pending=%0d remaining=%0d",dut.state,dut.macro_index,dut.issue_i,dut.written,received,req_valid,req_ready,rsp_valid,rsp_ready,outstanding,remaining);
   end
   if(!seen_done||received!=W*H)$fatal(1,"frame ended without complete raster");
  end
 endtask
 initial begin
  for(i=0;i<257;i++)expected_alpha[i]=i;
  repeat(3)@(negedge clk);rst_n=1;monitor=1;
  // Staging alone must not change an active frame, even when staged while idle.
  write_config(0,0,6144);write_config(0,11,2048);write_config(1,64,200);
  launch(10,1);
  if(dut.shadow_gains[15:0]!==16'd6144)$fatal(1,"start/config collision changed shadow");
  write_config(0,1,5000);write_config(1,128,16);
  finish_frame();

  // Idle commit installs all staged fields together at the next frame start.
  write_config(2,0,7);blocked_write();
  expected_gains[15:0]=6144;expected_gains[31:16]=5000;expected_gains[191:176]=2048;
  expected_alpha[64]=200;expected_alpha[128]=16;expected_version=7;
  // No clock is allowed between changing expected state and asserting launch.
  monitor=0;launch(11,0);monitor=1;
  if(dut.cfg_pending||!cfg_ready)$fatal(1,"launch did not release pending config");
  // Running-frame commit remains pending and cannot alter the current picture.
  write_config(0,0,2048);write_config(1,0,256);write_config(1,256,0);
  write_config(2,0,9);blocked_write();finish_frame();

  expected_gains[15:0]=2048;expected_alpha[0]=256;expected_alpha[256]=0;expected_version=9;
  monitor=0;launch(12,0);monitor=1;
  // Invalid addresses/data cannot change either staged or active valid entries.
  write_config(0,12,1);write_config(1,257,100);write_config(1,256,257);
  if(dut.shadow_alpha[256]!==9'd0)$fatal(1,"invalid alpha value accepted");
  write_config(0,2,7000);write_config(1,32,30);finish_frame();

  launch(13,0); // No commit: staged channel2/alpha32 still must not appear.
  write_config(2,0,11);
  if(cfg_ready||!dut.cfg_pending)$fatal(1,"running commit was not pending");
  // Reset an in-progress frame and its pending configuration/DDR transaction.
  @(negedge clk);monitor=0;rst_n=0;
  repeat(3)@(negedge clk);
  expected_gains={12{16'd4096}};expected_version=0;
  for(i=0;i<257;i++)expected_alpha[i]=i;
  rst_n=1;monitor=1;
  @(negedge clk);
  if(dut.cfg_pending||!cfg_ready||busy||dut.shadow_gains!==expected_gains)$fatal(1,"reset did not clear pending/frame/shadow");
  launch(14,0);finish_frame();
  if(completed!=4||desc_split<4||desc_resume<4||route_split<4||route_resume<4)$fatal(1,"config/page split coverage incomplete");
  $display("PASS config completed_frames=%0d requests=%0d idle/running staging, commit atomicity, no-commit hold, ready protection, reset; 4KiB descriptor/route splits=%0d/%0d",completed,requests,desc_split,route_split);
  $finish;
 end
 initial begin #600000;$fatal(1,"config suite timeout");end
endmodule
