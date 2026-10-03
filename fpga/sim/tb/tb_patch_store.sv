`timescale 1ns/1ps
// Independent memory scoreboard checks all parity permutations and both patch
// buffers, then checks the actual two grid pipelines under independent stalls.
module tb_patch_store;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,lwe=0,lwe_hi=0,lb=0,sv=0,sbuf=0;
 reg[1:0]lc,ca,cb;reg[4:0]lx,ly;reg[41:0]ln;
 reg[4:0]lx_hi,ly_hi;reg[41:0]ln_hi;
 reg[3:0]sx,sy;reg[2:0]sa,sb;reg[31:0]stag;
 wire sr,pv,pr;wire[167:0]na,nb;
 wire[2:0]ma,mb;wire[3:0]xa,ya,xb,yb;wire[31:0]ptag;
 avm_patch_store #(.TAG_W(32)) store(clk,rst_n,lwe,lb,lc,lx,ly,ln,
  sv,sr,sbuf,ca,cb,sx,sy,sa,sb,stag,pv,pr,na,nb,ma,mb,xa,ya,xb,yb,ptag,
  lwe_hi,lx_hi,ly_hi,ln_hi);
 wire[1:0]gsr,gv;reg[1:0]gr=0;wire[41:0]gu,gvo;wire[63:0]gt;
 assign pr=&gsr;
 genvar lane;
 generate for(lane=0;lane<2;lane++)begin:GRID
  wire[167:0]n=lane==0?na:nb;
  avm_grid_interp #(.TAG_W(32)) interp(
   .clk(clk),.rst_n(rst_n),.s_valid(pv&&gsr[1-lane]),.s_ready(gsr[lane]),
   .s_u00(n[20:0]),.s_u10(n[62:42]),.s_u01(n[104:84]),.s_u11(n[146:126]),
   .s_v00(n[41:21]),.s_v10(n[83:63]),.s_v01(n[125:105]),.s_v11(n[167:147]),
   .s_xfrac(lane==0?xa:xb),.s_yfrac(lane==0?ya:yb),.s_step_log2(lane==0?ma:mb),.s_tag(ptag),
   .m_valid(gv[lane]),.m_ready(gr[lane]),.m_u(gu[lane*21+:21]),.m_v(gvo[lane*21+:21]),.m_tag(gt[lane*32+:32]));
 end endgenerate

 reg[41:0]reference[0:1][0:3][0:16][0:16];
 reg[167:0]expected_a[0:20000],expected_b[0:20000];
 reg[21:0]expected_meta[0:20000];reg[31:0]expected_tag[0:20000];
 reg signed[20:0]exp_u[0:1][0:20000],exp_v[0:1][0:20000];
 integer accepted=0,patches=0,got[0:1],checked=0;
 integer log_coverage[0:1][0:4],camera_coverage[0:1][0:3],buffer_coverage[0:1];
 integer cycle=0,b,c,x,y,k,rxi,ryi,index,which,write_side;
 integer dual_writes=0,dual_wraps=0;
 reg[167:0]ea,eb;bit previous_accept=0,held=0;
 reg[389:0]hold_payload;
 function automatic[41:0] node(input integer version,bufnum,cam,xx,yy);
  reg signed[20:0]u,v;
  begin
   u=-50000+bufnum*25117+cam*7193+xx*xx*37+yy*91+version*3;
   v=47000-bufnum*23617-cam*6211-xx*97-yy*yy*29-version*7;
   node={v,u};
  end
 endfunction
 function automatic longint signed rounded(input longint signed n,input integer denominator);
  if(n<0)rounded=-((-n+denominator/2)/denominator);
  else rounded=(n+denominator/2)/denominator;
 endfunction
 function automatic signed[20:0] interp_ref(input reg[167:0]nodes,input integer channel,fx,fy,step);
  longint signed p0,p1,p2,p3,h0,h1;
  begin
   p0=$signed(nodes[channel*21+:21]);p1=$signed(nodes[42+channel*21+:21]);
   p2=$signed(nodes[84+channel*21+:21]);p3=$signed(nodes[126+channel*21+:21]);
   h0=rounded(p0*(step-fx)+p1*fx,step);h1=rounded(p2*(step-fx)+p3*fx,step);
   interp_ref=rounded(h0*(step-fy)+h1*fy,step);
  end
 endfunction
 always @(posedge clk)begin
  previous_accept=sv&&sr;
  if(lwe) reference[lb][lc][ly][lx]=ln;
  if(lwe_hi)begin
   if(lwe&&{ly[0],lx[0]}=={ly_hi[0],lx_hi[0]})$fatal(1,"dual node writes collide in one physical parity bank");
   reference[lb][lc][ly_hi][lx_hi]=ln_hi;
   dual_writes++;
   if(ly_hi!=ly)dual_wraps++;
  end
  if(!rst_n)begin accepted=0;patches=0;got[0]=0;got[1]=0;held=0;end
  else begin
   if(held&&(!pv||{na,nb,ma,mb,xa,ya,xb,yb,ptag}!==hold_payload)) $fatal(1,"patch changed under stall");
   held=pv&&!pr;hold_payload={na,nb,ma,mb,xa,ya,xb,yb,ptag};
   if(sv&&sr)begin
    rxi=sx>>sa;ryi=sy>>sa;
    ea={reference[sbuf][ca][ryi+1][rxi+1],reference[sbuf][ca][ryi+1][rxi],reference[sbuf][ca][ryi][rxi+1],reference[sbuf][ca][ryi][rxi]};
    rxi=sx>>sb;ryi=sy>>sb;
    eb={reference[sbuf][cb][ryi+1][rxi+1],reference[sbuf][cb][ryi+1][rxi],reference[sbuf][cb][ryi][rxi+1],reference[sbuf][cb][ryi][rxi]};
    expected_a[accepted]=ea;expected_b[accepted]=eb;expected_tag[accepted]=stag;
    // Explicit assignments avoid unsized-expression widths in packed metadata.
    expected_meta[accepted][21:19]=sa;expected_meta[accepted][18:16]=sb;
    expected_meta[accepted][15:12]=sx% (1<<sa);expected_meta[accepted][11:8]=sy%(1<<sa);
    expected_meta[accepted][7:4]=sx%(1<<sb);expected_meta[accepted][3:0]=sy%(1<<sb);
    exp_u[0][accepted]=interp_ref(ea,0,sx%(1<<sa),sy%(1<<sa),1<<sa);
    exp_v[0][accepted]=interp_ref(ea,1,sx%(1<<sa),sy%(1<<sa),1<<sa);
    exp_u[1][accepted]=interp_ref(eb,0,sx%(1<<sb),sy%(1<<sb),1<<sb);
    exp_v[1][accepted]=interp_ref(eb,1,sx%(1<<sb),sy%(1<<sb),1<<sb);
    log_coverage[0][sa]++;log_coverage[1][sb]++;
    camera_coverage[0][ca]++;camera_coverage[1][cb]++;buffer_coverage[sbuf]++;
    accepted++;
   end
   if(pv&&pr)begin
    if(patches>=accepted||na!==expected_a[patches]||nb!==expected_b[patches]||ptag!==expected_tag[patches])
     $fatal(1,"patch node/tag mismatch index=%0d tag=%0d expected_a=%h actual_a=%h expected_b=%h actual_b=%h",patches,ptag,expected_a[patches],na,expected_b[patches],nb);
    if({ma,mb,xa,ya,xb,yb}!==expected_meta[patches])$fatal(1,"patch step/fraction mismatch");
    patches++;checked++;
   end
   for(k=0;k<2;k++)if(gv[k]&&gr[k])begin
    index=got[k];
    if(index>=accepted||gu[k*21+:21]!==exp_u[k][index]||gvo[k*21+:21]!==exp_v[k][index]||gt[k*32+:32]!==expected_tag[index])
     $fatal(1,"patch -> grid mismatch lane=%0d index=%0d",k,index);
    got[k]++;
   end
  end
 end
 initial begin
  for(b=0;b<2;b++)begin
   buffer_coverage[b]=0;
   for(c=0;c<4;c++)camera_coverage[b][c]=0;
   for(c=0;c<5;c++)log_coverage[b][c]=0;
  end
  repeat(3)@(negedge clk);rst_n=1;
  for(b=0;b<2;b++)for(c=0;c<4;c++)for(y=0;y<=16;y++)for(x=0;x<=16;x++)begin
   @(negedge clk);lwe=1;lb=b;lc=c;lx=x;ly=y;ln=node(0,b,c,x,y);
  end
  @(negedge clk);lwe=0;
  for(cycle=0;cycle<12000;cycle++)begin
   @(negedge clk);
   if(cycle==4567)begin rst_n=0;sv=0;lwe=0;lwe_hi=0;gr=0;end
   if(cycle==4570)rst_n=1;
   if(rst_n)begin
    gr=cycle<512||cycle>=10000?2'b11:$urandom;
    if(cycle>=2000&&cycle<2300)gr=0;
    if(!sv||previous_accept)begin
     sv=cycle<10000&&(cycle<512||$urandom_range(0,4)!=0);
     sbuf=(cycle/1000)%2;
     ca=$urandom;cb=(ca+1+$urandom_range(0,2))%4;
     if(cycle<512)begin sx=cycle%16;sy=(cycle/16)%16;sa=0;sb=1;end
     else begin sx=$urandom;sy=$urandom;sa=cycle%5;sb=$urandom_range(0,4);end
     stag=cycle;
    end
    // Concurrent loading of the other patch must never corrupt this query.
    lwe=cycle<10000&&$urandom_range(0,3)==0;
    write_side=(1<<(cycle%5))+1;
    lb=!sbuf;lc=$urandom;lx=$urandom_range(0,write_side-1);ly=$urandom_range(0,write_side-1);
    ln=node(cycle,lb,lc,lx,ly);
    lwe_hi=lwe&&(lx!=write_side-1||ly!=write_side-1);
    lx_hi=lx==write_side-1?0:lx+1;ly_hi=lx==write_side-1?ly+1:ly;
    ln_hi=node(cycle,lb,lc,lx_hi,ly_hi);
   end
  end
  repeat(10)@(negedge clk);
  if(accepted!=patches||got[0]!=accepted||got[1]!=accepted)$fatal(1,"patch/grid pipeline not drained");
  if(checked<4000)$fatal(1,"insufficient patch coverage");
  if(dual_writes<1000||dual_wraps<100)$fatal(1,"insufficient dual-node write coverage");
  for(b=0;b<2;b++)begin
   if(buffer_coverage[b]<1000)$fatal(1,"buffer coverage");
   for(c=0;c<4;c++)if(camera_coverage[b][c]<100)$fatal(1,"camera coverage");
   for(c=0;c<5;c++)if(log_coverage[b][c]<100)$fatal(1,"step coverage");
  end
  $display("PASS patch_store patches=%0d dual_writes=%0d row_wraps=%0d all five steps/four cameras/two buffers, parity, parallel load, independent grid stalls, reset",checked,dual_writes,dual_wraps);
  $finish;
 end
 initial begin #300000;$fatal(1,"patch store timeout");end
endmodule
