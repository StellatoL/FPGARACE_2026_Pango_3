// Four camera stores, four parity banks per camera. One query/camera/cycle;
// each bank's write port loads the inactive patch while its read port samples
// the active patch. Up to two adjacent linear nodes arrive per clock, necessarily
// in different parity banks. Coordinates are signed Q*.8.
module avm_patch_camera (
 input wire clk, input wire we, input wire wbuffer,
 input wire [4:0] wx,wy, input wire [41:0] wnode,
 input wire re, input wire rbuffer, input wire [4:0] rx,ry,
 output wire [167:0] nodes,
 input wire we_hi,input wire[4:0]wx_hi,wy_hi,input wire[41:0]wnode_hi
);
 reg [41:0] b00[0:161],b10[0:161],b01[0:161],b11[0:161];
 reg [41:0] q00,q10,q01,q11;
 reg px,py;
 wire [7:0] wa=(wbuffer?8'd81:8'd0)+(wy>>1)*9+(wx>>1);
 wire [7:0] wa_hi=(wbuffer?8'd81:8'd0)+(wy_hi>>1)*9+(wx_hi>>1);
 wire [4:0] xe=rx[0]?rx+1'b1:rx, xo=rx[0]?rx:rx+1'b1;
 wire [4:0] ye=ry[0]?ry+1'b1:ry, yo=ry[0]?ry:ry+1'b1;
 wire [7:0] base=rbuffer?8'd81:8'd0;
 always @(posedge clk) begin
  // Adjacent linear nodes always land in different parity banks. These muxes
  // expose exactly ONE physical write port per bank, alongside its read port.
  if(we&&{wy[0],wx[0]}==0)b00[wa]<=wnode;
  else if(we_hi&&{wy_hi[0],wx_hi[0]}==0)b00[wa_hi]<=wnode_hi;
  if(we&&{wy[0],wx[0]}==1)b10[wa]<=wnode;
  else if(we_hi&&{wy_hi[0],wx_hi[0]}==1)b10[wa_hi]<=wnode_hi;
  if(we&&{wy[0],wx[0]}==2)b01[wa]<=wnode;
  else if(we_hi&&{wy_hi[0],wx_hi[0]}==2)b01[wa_hi]<=wnode_hi;
  if(we&&{wy[0],wx[0]}==3)b11[wa]<=wnode;
  else if(we_hi&&{wy_hi[0],wx_hi[0]}==3)b11[wa_hi]<=wnode_hi;
  if(re) begin
   q00<=b00[base+(ye>>1)*9+(xe>>1)];
   q10<=b10[base+(ye>>1)*9+(xo>>1)];
   q01<=b01[base+(yo>>1)*9+(xe>>1)];
   q11<=b11[base+(yo>>1)*9+(xo>>1)];
   px<=rx[0]; py<=ry[0];
  end
 end
 wire [41:0] n00=py?(px?q11:q01):(px?q10:q00);
 wire [41:0] n10=py?(px?q01:q11):(px?q00:q10);
 wire [41:0] n01=py?(px?q10:q00):(px?q11:q01);
 wire [41:0] n11=py?(px?q00:q10):(px?q01:q11);
 assign nodes={n11,n01,n10,n00};
endmodule

module avm_patch_store #(parameter TAG_W=128)(
 input wire clk,rst_n,
 input wire load_we,load_buffer,input wire[1:0]load_cam,
 input wire[4:0]load_x,load_y,input wire[41:0]load_node,
 input wire s_valid,output wire s_ready,input wire s_buffer,
 input wire[1:0]s_cam_a,s_cam_b,input wire[3:0]s_x,s_y,
 input wire[2:0]s_step_a,s_step_b,input wire[TAG_W-1:0]s_tag,
 output reg m_valid,input wire m_ready,
 output wire[167:0]m_nodes_a,m_nodes_b,
 output reg[2:0]m_step_a,m_step_b,
 output reg[3:0]m_xfrac_a,m_yfrac_a,m_xfrac_b,m_yfrac_b,
 output reg[TAG_W-1:0]m_tag,
 input wire load_we_hi,input wire[4:0]load_x_hi,load_y_hi,
 input wire[41:0]load_node_hi
);
 wire advance=!m_valid||m_ready;
 assign s_ready=advance;
 reg[1:0]qa,qb;
 wire[671:0]all_nodes;
 genvar c;
 generate for(c=0;c<4;c=c+1)begin:CAMERA
  wire select_a=(s_cam_a==c);
  wire[2:0]step=select_a?s_step_a:s_step_b;
  avm_patch_camera bank(
   .clk(clk),.we(load_we&&load_cam==c),.wbuffer(load_buffer),
   .wx(load_x),.wy(load_y),.wnode(load_node),
   .we_hi(load_we_hi&&load_cam==c),.wx_hi(load_x_hi),.wy_hi(load_y_hi),.wnode_hi(load_node_hi),
   .re(advance&&s_valid&&(s_cam_a==c||s_cam_b==c)),
   .rbuffer(s_buffer),.rx({1'b0,s_x}>>step),.ry({1'b0,s_y}>>step),
   .nodes(all_nodes[c*168+:168]));
 end endgenerate
 assign m_nodes_a=all_nodes[qa*168+:168];
 assign m_nodes_b=all_nodes[qb*168+:168];
 always @(posedge clk)begin
  if(!rst_n)begin m_valid<=0;qa<=0;qb<=0;m_tag<=0;end
  else if(advance)begin
   m_valid<=s_valid;
   if(s_valid)begin
    qa<=s_cam_a;qb<=s_cam_b;m_step_a<=s_step_a;m_step_b<=s_step_b;m_tag<=s_tag;
    m_xfrac_a<=s_x&((1<<s_step_a)-1);m_yfrac_a<=s_y&((1<<s_step_a)-1);
    m_xfrac_b<=s_x&((1<<s_step_b)-1);m_yfrac_b<=s_y&((1<<s_step_b)-1);
   end
  end
 end
endmodule
