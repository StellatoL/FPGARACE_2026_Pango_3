`timescale 1ns/1ps
// Regression for an invalid A route with valid B=camera0. Nonconstant step-2
// patches make the old dummy-A camera collision observable beyond pixel zero.
module tb_b_only;
 localparam W=17,H=2,SRC=32'h1000;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,cfg_valid=0,m_ready=0;
 reg[1:0]cfg_kind=0;reg[8:0]cfg_index=0;reg[31:0]cfg_data=0;
 wire cfg_ready,busy,done,m_valid,m_sof,m_eol;
 wire[15:0]m_pixel;wire[31:0]output_frame_id,config_version;
 wire req_valid,req_ready,rsp_ready;wire[31:0]req_addr;wire[4:0]req_len;
 reg rsp_valid=0,rsp_last=0;reg[127:0]rsp_data=0;
 wire[767:0]stat_sums;wire[127:0]stat_counts;
 wire[31:0]miss_a,miss_b,cycle_count,error_flags;
 avm_core #(.WIDTH(W),.HEIGHT(H)) dut(
  .clk(clk),.rst_n(rst_n),.start(start),.frame_id(32'd37),
  .camera_bases(128'h00000000000000000000000000001000),
  .src_width(12'd32),.src_height(12'd4),.src_stride(16'd64),.desc_base(32'd0),.route_base(32'h100),
  .cfg_valid(cfg_valid),.cfg_ready(cfg_ready),.cfg_kind(cfg_kind),.cfg_index(cfg_index),.cfg_data(cfg_data),
  .busy(busy),.done(done),.output_frame_id(output_frame_id),.config_version(config_version),
  .m_valid(m_valid),.m_ready(m_ready),.m_pixel(m_pixel),.m_sof(m_sof),.m_eol(m_eol),
  .req_valid(req_valid),.req_ready(req_ready),.req_addr(req_addr),.req_len(req_len),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_data(rsp_data),.rsp_last(rsp_last),
  .stat_sums(stat_sums),.stat_counts(stat_counts),.cache_misses_a(miss_a),.cache_misses_b(miss_b),
  .cycle_count(cycle_count),.error_flags(error_flags));
 reg[127:0]memory[0:511];
 function automatic[15:0] source_pixel(input integer x,y);
  integer r,g,b;
  begin r=(x+3*y)%32;g=(3*x+5*y+7)%64;b=(5*x+2*y+11)%32;source_pixel=(r<<11)|(g<<5)|b;end
 endfunction
 function automatic integer channel(input reg[15:0]p,input integer c);
  integer v;
  begin
   if(c==0)begin v=(p>>11)&31;channel=v*8+v/4;end
   else if(c==1)begin v=(p>>5)&63;channel=v*4+v/16;end
   else begin v=p&31;channel=v*8+v/4;end
  end
 endfunction
 function automatic[15:0] golden(input integer x,y);
  longint unsigned total;integer c,p,gain,result;
  begin
   result=0;
   // x=15 maps to u=-0.5; x=16 maps to u=31.25 in a 32-pixel source.
   if(x<15)for(c=0;c<3;c++)begin
    total=channel(source_pixel(x,y),c)*192*128+channel(source_pixel(x+1,y),c)*64*128
         +channel(source_pixel(x,y+1),c)*192*128+channel(source_pixel(x+1,y+1),c)*64*128;
    p=(total+32768)/65536;
    gain=c==0?6144:(c==1?3072:4096);p=(p*gain+2048)/4096;if(p>255)p=255;
    // B is the only valid camera: its full value must survive alpha LUT=200.
    if(c==1)result=(result<<6)|(p/4);else result=(result<<5)|(p/8);
   end
   golden=result;
  end
 endfunction
 reg outstanding=0;integer remaining=0,wait_clocks=0,word_address=0,tick=0;
 integer reads=0,source_reads=0;
 assign req_ready=rst_n&&!outstanding&&tick%4!=0;
 always @(posedge clk)begin
  tick<=tick+1;
  if(!rst_n)begin outstanding<=0;rsp_valid<=0;rsp_last<=0;remaining<=0;wait_clocks<=0;end
  else begin
   if(req_valid&&req_ready)begin
    if(req_addr[3:0]!=0||req_len<1||req_len>16||req_addr[11:0]+req_len*16>4096||req_addr+req_len*16>8192)
     $fatal(1,"invalid B-only DDR request %h/%0d",req_addr,req_len);
    outstanding<=1;remaining<=req_len;word_address<=req_addr/16;wait_clocks<=2+tick%3;
    reads++;if(req_addr>=SRC)source_reads++;
   end
   if(outstanding)begin
    if(wait_clocks>0)wait_clocks<=wait_clocks-1;
    else if(!rsp_valid)begin rsp_valid<=1;rsp_data<=memory[word_address];rsp_last<=remaining==1;end
    else if(rsp_ready)begin
     rsp_valid<=0;remaining<=remaining-1;word_address<=word_address+1;
     if(remaining==1)outstanding<=0;else wait_clocks<=tick%2;
    end
   end
  end
 end
 integer received=0,black=0,stalls=0;bit held=0,finished=0;
 reg[17:0]held_value;
 always @(negedge clk)m_ready=rst_n&&tick%5!=0;
 always @(posedge clk)begin
  if(!rst_n)begin received=0;black=0;held=0;finished=0;end
  else begin
   if(held&&(!m_valid||{m_pixel,m_sof,m_eol}!==held_value))$fatal(1,"B-only output changed under stall");
   held=m_valid&&!m_ready;held_value={m_pixel,m_sof,m_eol};if(held)stalls++;
   if(m_valid&&m_ready)begin
    if(received>=W*H||m_pixel!==golden(received%W,received/W))
     $fatal(1,"B-only camera0 mismatch pixel=%0d x=%0d y=%0d expected=%h got=%h",received,received%W,received/W,golden(received%W,received/W),m_pixel);
    if(m_sof!==(received==0)||m_eol!==(received%W==W-1))$fatal(1,"B-only raster marker mismatch");
    if(received%W>=15)begin if(m_pixel!==0)$fatal(1,"invalid coordinate not black");black++;end
    received++;
   end
   if(done)begin
    if(received!=W*H||black!=4||source_reads==0||miss_a!=0||miss_b==0||stalls==0)$fatal(1,"B-only coverage incomplete");
    if(config_version!=7||output_frame_id!=37||error_flags!==0||stat_sums!==0||stat_counts!==0)$fatal(1,"B-only status mismatch");
    finished=1;
   end
  end
 end
 task automatic config_write(input integer kind,index,value);
  begin
   @(negedge clk);cfg_kind=kind;cfg_index=index;cfg_data=value;cfg_valid=1;
   @(posedge clk);while(!cfg_ready)@(posedge clk);
   @(negedge clk);cfg_valid=0;
  end
 endtask
 integer i,macro,x,y,n,address,node_base;reg signed[20:0]u,v;
 initial begin
  for(i=0;i<512;i++)memory[i]=0;
  for(macro=0;macro<2;macro++)begin
   node_base=macro==0?32'h800:32'hb00;
   memory[macro*4][31:0]=node_base;memory[macro*4][34:32]=1;memory[macro*4][35]=1;
   for(n=0;n<256;n++)memory[(32'h100+macro*512+n*2)/16][(n%8)*16+:16]=16'h1823;
   for(y=0;y<9;y++)for(x=0;x<9;x++)begin
    if(macro==1)u=31*256+64;else if(x==8)u=-3904;else u=x*2*256+64;
    v=y*2*256+128;n=y*9+x;
    memory[(node_base+n*8)/16][(n%2)*64+:64]={22'd0,v,u};
   end
  end
  for(y=0;y<4;y++)for(x=0;x<32;x++)begin
   address=SRC+y*64+x*2;memory[address/16][(x%8)*16+:16]=source_pixel(x,y);
  end
  repeat(3)@(negedge clk);rst_n=1;
  config_write(0,0,6144);config_write(0,1,3072);config_write(0,2,4096);
  config_write(1,96,200);config_write(2,0,7);
  @(negedge clk);start=1;@(negedge clk);start=0;
  wait(finished);@(negedge clk);
  $display("PASS B-only camera0 pixels=%0d invalid_black=%0d DDR_requests=%0d source_requests=%0d step2/gain/alpha_override/backpressure",received,black,reads,source_reads);
  $finish;
 end
 initial begin #200000;$fatal(1,"B-only timeout state=%0d received=%0d",dut.state,received);end
endmodule
