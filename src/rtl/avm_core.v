// Portable AVM core. Frame buffers and LUTs are external, byte addressed.
// Physical DDR/PCIe adapters deliberately sit outside this simulation release.
module avm_core #(parameter WIDTH=1280, HEIGHT=720, CACHE_PREFETCH_TILES=2)(
 input wire clk,rst_n,start,input wire[31:0]frame_id,
 input wire[127:0]camera_bases,input wire[11:0]src_width,src_height,
 input wire[15:0]src_stride,input wire[31:0]desc_base,route_base,
 input wire cfg_valid,output wire cfg_ready,input wire[1:0]cfg_kind,
 input wire[8:0]cfg_index,input wire[31:0]cfg_data,
 output wire busy,output wire done,output reg[31:0]output_frame_id,config_version,
 output wire m_valid,input wire m_ready,output wire[15:0]m_pixel,
 output wire m_sof,m_eol,
 output wire req_valid,input wire req_ready,output wire[31:0]req_addr,
 output wire[4:0]req_len,input wire rsp_valid,output wire rsp_ready,
 input wire[127:0]rsp_data,input wire rsp_last,
 output reg[767:0]stat_sums,output reg[127:0]stat_counts,
 output wire[31:0]cache_misses_a,cache_misses_b,
 output reg[31:0]cycle_count,output reg[31:0]error_flags
);
 localparam TW=128, MX=(WIDTH+15)/16, MY=(HEIGHT+15)/16;
 localparam IDLE=0,DREQ=1,DRESP=2,CAMSET=3,NREQ=4,NRESP=5,
            RREQ=8,RRESP=9,RUN=10,DRAIN=11,FINISH=12,WAIT_PATCH=13;
 reg[3:0]state,loader_state;
 assign busy=state!=IDLE;
 wire launch=start&&!busy;
 reg[191:0]gains,shadow_gains;
 reg[8:0]alpha[0:256],shadow_alpha[0:256];
 reg cfg_pending;reg[31:0]shadow_version;
 assign cfg_ready=!cfg_pending&&!start;
 integer ci;
 always @(posedge clk)begin
  if(!rst_n)begin
   gains<={12{16'd4096}};shadow_gains<={12{16'd4096}};
   cfg_pending<=0;config_version<=0;shadow_version<=0;
   for(ci=0;ci<257;ci=ci+1)begin alpha[ci]<=ci;shadow_alpha[ci]<=ci;end
  end else begin
   if(cfg_valid&&cfg_ready)case(cfg_kind)
    0:if(cfg_index<12)shadow_gains[cfg_index*16+:16]<=cfg_data[15:0];
    1:if(cfg_index<=256&&cfg_data<=256)shadow_alpha[cfg_index]<=cfg_data[8:0];
    2:begin cfg_pending<=1;shadow_version<=cfg_data;end
   endcase
   if(launch&&cfg_pending)begin
    gains<=shadow_gains;config_version<=shadow_version;cfg_pending<=0;
    for(ci=0;ci<257;ci=ci+1)alpha[ci]<=shadow_alpha[ci];
   end
  end
 end

 reg[127:0]bases;reg[11:0]sw,sh;reg[15:0]stride;
 reg[31:0]dbase,rbase;reg[15:0]macro_index,mx,my;
 // A slot is reserved when loading starts and is released only after every
 // output from its macro has entered the stripe reorder. The loader can work
 // on the other slot throughout RUN/DRAIN, without overwriting unread nodes.
 reg buffer_sel,load_buffer;reg[15:0]load_macro_index;
 reg[1:0]bank_busy,patch_ready_slots;
 reg[31:0]patch_addr[0:3];reg[2:0]step[0:7];reg[7:0]active;
 reg[2:0]camera;reg[2:0]desc_i;
 reg[8:0]nodes_left;reg[7:0]beats_left;reg[4:0]burst_left;
 reg[31:0]load_addr;reg[4:0]nx,ny,side;
 reg[5:0]route_beat;
 // Eight independent shallow banks: one write/bank per 128-bit load beat.
 // Two macro slots add 512 bytes over the former single route slot; the node
 // RAM already held two patches. Steps/validity add 16 bits across four cameras.
 reg[15:0]routes0[0:63],routes1[0:63],routes2[0:63],routes3[0:63];
 reg[15:0]routes4[0:63],routes5[0:63],routes6[0:63],routes7[0:63];
 reg[8:0]issue_i,written;
 wire[11:0]out_x=(mx<<4)+issue_i[3:0],out_y=(my<<4)+issue_i[7:4];
 wire inside_frame=out_x<WIDTH&&out_y<HEIGHT;
 wire[4:0]real_w=(WIDTH-(mx<<4)>=16)?16:WIDTH-(mx<<4);
 wire[4:0]real_h=(HEIGHT-(my<<4)>=16)?16:HEIGHT-(my<<4);
 wire[8:0]macro_pixels=real_w*real_h;
 reg[15:0]route;
 always @*case(issue_i[2:0])
  0:route=routes0[{buffer_sel,issue_i[7:3]}];1:route=routes1[{buffer_sel,issue_i[7:3]}];
  2:route=routes2[{buffer_sel,issue_i[7:3]}];3:route=routes3[{buffer_sel,issue_i[7:3]}];
  4:route=routes4[{buffer_sel,issue_i[7:3]}];5:route=routes5[{buffer_sel,issue_i[7:3]}];
  6:route=routes6[{buffer_sel,issue_i[7:3]}];7:route=routes7[{buffer_sel,issue_i[7:3]}];
 endcase
 // Invalid routes still traverse the arithmetic path as explicit black pixels.
 wire va=route[4]&&active[{buffer_sel,route[1:0]}],vb=route[5]&&active[{buffer_sel,route[3:2]}];
 wire[1:0]cam_a=va?route[1:0]:(vb&&route[3:2]==0?2'd1:2'd0);
 wire[1:0]cam_b=vb?route[3:2]:(cam_a==0?2'd1:2'd0);
 wire[8:0]alpha_index=route[14:6]<=256?route[14:6]:9'd0;
 wire[TW-1:0]input_tag={{(TW-39){1'b0}},alpha_index,vb,va,cam_b,cam_a,out_y,out_x};
 wire patch_s_ready,patch_valid,patch_ready;
 wire[TW-1:0]patch_tag;wire[167:0]nodes_a,nodes_b;
 wire[2:0]step_a,step_b;wire[3:0]xfrac_a,yfrac_a,xfrac_b,yfrac_b;
 wire[2:0]crq,crdy,crv,crr,crlast;wire[95:0]cra;wire[14:0]crlen;wire[383:0]crdata;
 wire node_we=loader_state==NRESP&&crv[0]&&crr[0]&&nodes_left!=0;
 wire[4:0]nx_hi=nx==side-1?5'd0:nx+1'b1;
 wire[4:0]ny_hi=nx==side-1?ny+1'b1:ny;
 wire[4:0]nx_next=nx_hi==side-1?5'd0:nx_hi+1'b1;
 wire[4:0]ny_next=nx_hi==side-1?ny_hi+1'b1:ny_hi;
 avm_patch_store #(.TAG_W(TW)) patch(
  .clk(clk),.rst_n(rst_n),.load_we(node_we),.load_buffer(load_buffer),
  .load_cam(camera[1:0]),.load_x(nx),.load_y(ny),
  .load_node(crdata[41:0]),.load_we_hi(node_we&&nodes_left>1),
  .load_x_hi(nx_hi),.load_y_hi(ny_hi),.load_node_hi(crdata[105:64]),
  .s_valid(state==RUN&&inside_frame),.s_ready(patch_s_ready),.s_buffer(buffer_sel),
  .s_cam_a(cam_a),.s_cam_b(cam_b),.s_x(issue_i[3:0]),.s_y(issue_i[7:4]),
  .s_step_a(va?step[{buffer_sel,cam_a}]:3'd4),.s_step_b(vb?step[{buffer_sel,cam_b}]:3'd4),.s_tag(input_tag),
  .m_valid(patch_valid),.m_ready(patch_ready),.m_nodes_a(nodes_a),.m_nodes_b(nodes_b),
  .m_step_a(step_a),.m_step_b(step_b),.m_xfrac_a(xfrac_a),.m_yfrac_a(yfrac_a),
  .m_xfrac_b(xfrac_b),.m_yfrac_b(yfrac_b),.m_tag(patch_tag));
 wire[1:0]grid_s_ready,grid_valid,grid_ready;
 wire[83:0]grid_uv;wire[255:0]grid_tags;
 assign patch_ready=&grid_s_ready;
 genvar g;
 generate for(g=0;g<2;g=g+1)begin:GRID
  wire[167:0]ns=g==0?nodes_a:nodes_b;
  // Clear unused camera nodes rather than propagating uninitialized RAM Xs.
  wire enabled=g==0?patch_tag[28]:patch_tag[29];
  avm_grid_interp #(.TAG_W(TW)) interp(
   .clk(clk),.rst_n(rst_n),.s_valid(patch_valid&&grid_s_ready[1-g]),.s_ready(grid_s_ready[g]),
   .s_u00(enabled?ns[20:0]:21'd0),.s_v00(enabled?ns[41:21]:21'd0),
   .s_u10(enabled?ns[62:42]:21'd0),.s_v10(enabled?ns[83:63]:21'd0),
   .s_u01(enabled?ns[104:84]:21'd0),.s_v01(enabled?ns[125:105]:21'd0),
   .s_u11(enabled?ns[146:126]:21'd0),.s_v11(enabled?ns[167:147]:21'd0),
   .s_xfrac(g==0?xfrac_a:xfrac_b),.s_yfrac(g==0?yfrac_a:yfrac_b),
   .s_step_log2(g==0?step_a:step_b),.s_tag(patch_tag),
   .m_valid(grid_valid[g]),.m_ready(grid_ready[g]),.m_u(grid_uv[g*42+:21]),
   .m_v(grid_uv[g*42+21+:21]),.m_tag(grid_tags[g*TW+:TW]));
 end endgenerate

 wire coord_valid=&grid_valid;wire coord_ready;
 wire[20:0]ua=grid_uv[20:0],vv_a=grid_uv[41:21],ub=grid_uv[62:42],vv_b=grid_uv[83:63];
 wire[TW-1:0]coord_tag={57'd0,vv_b[7:0],ub[7:0],vv_a[7:0],ua[7:0],grid_tags[38:0]};
 reg[1:0]sent;
 wire[1:0]cache_s_ready,cache_valid,cache_ready,cache_sample_valid;
 wire[127:0]cache_pixels;wire[255:0]cache_tags;
 assign coord_ready=(sent[0]||cache_s_ready[0])&&(sent[1]||cache_s_ready[1]);
 assign grid_ready[0]=coord_ready&&grid_valid[1];
 assign grid_ready[1]=coord_ready&&grid_valid[0];
 always @(posedge clk)begin
  if(!rst_n)sent<=0;
  else if(coord_valid&&coord_ready)sent<=0;
  else if(coord_valid)sent<=sent|cache_s_ready;
 end
 wire[63:0]misses;
 generate for(g=0;g<2;g=g+1)begin:CACHE
  wire[20:0]u=g==0?ua:ub,v=g==0?vv_a:vv_b;
  wire enabled=(g==0?coord_tag[28]:coord_tag[29])&&!u[20]&&!v[20];
  avm_tile_cache #(.TAG_W(TW),.PREFETCH_TILES(CACHE_PREFETCH_TILES)) cache(
   .clk(clk),.rst_n(rst_n),.s_valid(coord_valid&&!sent[g]),.s_ready(cache_s_ready[g]),
   .s_enable(enabled),.s_cam(g==0?coord_tag[25:24]:coord_tag[27:26]),
   .s_x(u[19:8]),.s_y(v[19:8]),.s_tag(coord_tag),
   .camera_bases(bases),.src_width(sw),.src_height(sh),.src_stride(stride),.invalidate(launch),
   .m_valid(cache_valid[g]),.m_ready(cache_ready[g]),.m_pixels(cache_pixels[g*64+:64]),
   .m_tag(cache_tags[g*TW+:TW]),.m_sample_valid(cache_sample_valid[g]),
   .req_valid(crq[g+1]),.req_ready(crdy[g+1]),.req_addr(cra[(g+1)*32+:32]),
   .req_len(crlen[(g+1)*5+:5]),.rsp_valid(crv[g+1]),.rsp_ready(crr[g+1]),
   .rsp_data(crdata[(g+1)*128+:128]),.rsp_last(crlast[g+1]),
   .hit_count(),.miss_count(misses[g*32+:32]),.read_beats());
 end endgenerate
 assign cache_misses_a=misses[31:0];assign cache_misses_b=misses[63:32];
 wire[1:0]pixel_valid,pixel_ready;wire[47:0]rgb;wire[255:0]pixel_tags;
 generate for(g=0;g<2;g=g+1)begin:PIXEL
  wire[TW-1:0]ct=cache_tags[g*TW+:TW];
  wire[TW-1:0]ct_valid={ct[TW-1:72],cache_sample_valid[g],ct[70:0]};
  avm_bilinear #(.TAG_W(TW)) bilinear(
   .clk(clk),.rst_n(rst_n),.s_valid(cache_valid[g]),.s_ready(cache_ready[g]),
   .s_pixels(cache_pixels[g*64+:64]),.s_fx(g==0?ct[46:39]:ct[62:55]),
   .s_fy(g==0?ct[54:47]:ct[70:63]),.s_tag(ct_valid),
   .m_valid(pixel_valid[g]),.m_ready(pixel_ready[g]),.m_rgb(rgb[g*24+:24]),
   .m_tag(pixel_tags[g*TW+:TW]));
 end endgenerate
 wire[TW-1:0]ta=pixel_tags[TW-1:0],tb=pixel_tags[2*TW-1:TW];
 wire blend_s_ready,blend_valid,blend_ready;wire[15:0]blend_pixel;wire[TW-1:0]blend_tag;
 assign pixel_ready[0]=blend_s_ready&&pixel_valid[1];
 assign pixel_ready[1]=blend_s_ready&&pixel_valid[0];
 avm_gain_blend #(.TAG_W(TW)) fusion(
  .clk(clk),.rst_n(rst_n),.s_valid(&pixel_valid),.s_ready(blend_s_ready),
  .s_rgb_a(rgb[23:0]),.s_rgb_b(rgb[47:24]),
  .s_gain_a(gains[ta[25:24]*48+:48]),.s_gain_b(gains[ta[27:26]*48+:48]),
  .s_alpha(alpha[ta[38:30]]),.s_valid_a(ta[71]),.s_valid_b(tb[71]),.s_tag(ta),
  .m_valid(blend_valid),.m_ready(blend_ready),.m_pixel(blend_pixel),.m_tag(blend_tag));
 wire reorder_done;
 reg frame_output_done;
 always @(posedge clk)begin
  if(!rst_n||launch)frame_output_done<=0;
  else if(reorder_done)frame_output_done<=1;
 end
 avm_stripe_reorder #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) reorder(
  .clk(clk),.rst_n(rst_n),.start(launch),.s_valid(blend_valid),.s_ready(blend_ready),
  .s_pixel(blend_pixel),.s_x(blend_tag[11:0]),.s_y(blend_tag[23:12]),
  .m_valid(m_valid),.m_ready(m_ready),.m_pixel(m_pixel),.m_sof(m_sof),.m_eol(m_eol),.done(reorder_done));
 assign done=state==FINISH&&frame_output_done;

 // Uncompensated RGB statistics. Conservative common RGB validity mask.
 wire stats_fire=(&pixel_valid)&&blend_s_ready&&ta[71]&&tb[71];
 wire[1:0]ca=ta[25:24],cb=ta[27:26];
 wire[1:0]low_cam=ca<cb?ca:cb,high_cam=ca<cb?cb:ca;
 wire adjacent=low_cam<2&&high_cam>=2;
 wire[1:0]edge_id={low_cam[0],high_cam[0]};
 wire[23:0]low_rgb=ca<cb?rgb[23:0]:rgb[47:24],high_rgb=ca<cb?rgb[47:24]:rgb[23:0];
 wire stats_ok=low_rgb[23:16]>=16&&low_rgb[23:16]<=235&&low_rgb[15:8]>=16&&low_rgb[15:8]<=235&&low_rgb[7:0]>=16&&low_rgb[7:0]<=235&&
               high_rgb[23:16]>=16&&high_rgb[23:16]<=235&&high_rgb[15:8]>=16&&high_rgb[15:8]<=235&&high_rgb[7:0]>=16&&high_rgb[7:0]<=235;
 integer sc;
 always @(posedge clk)begin
  if(!rst_n||launch)begin stat_sums<=0;stat_counts<=0;end
  else if(stats_fire&&adjacent&&stats_ok)begin
   stat_counts[edge_id*32+:32]<=stat_counts[edge_id*32+:32]+1'b1;
   for(sc=0;sc<3;sc=sc+1)begin
    stat_sums[(edge_id*6+sc)*32+:32]<=stat_sums[(edge_id*6+sc)*32+:32]+low_rgb[(2-sc)*8+:8];
    stat_sums[(edge_id*6+3+sc)*32+:32]<=stat_sums[(edge_id*6+3+sc)*32+:32]+high_rgb[(2-sc)*8+:8];
   end
  end
 end

 // Descriptor/node/route loader. Never reads the pixel framebuffer itself.
 reg[4:0]lut_len;reg[31:0]lut_addr;
 reg[8:0]boundary_beats;
 always @*begin
  lut_addr=0;lut_len=1;
  if(loader_state==DREQ)begin lut_addr=dbase+load_macro_index*64+desc_i*16;lut_len=4-desc_i;end
  if(loader_state==NREQ)begin
   lut_addr=load_addr;lut_len=beats_left>16?16:beats_left;
  end
  if(loader_state==RREQ)begin
   lut_addr=rbase+load_macro_index*512+route_beat*16;lut_len=32-route_beat>16?16:32-route_beat;
  end
  boundary_beats=(13'd4096-{1'b0,lut_addr[11:0]})>>4;
  if(boundary_beats<lut_len)lut_len=boundary_beats;
 end
 assign crq[0]=(loader_state==DREQ||loader_state==NREQ||loader_state==RREQ);
 assign cra[31:0]=lut_addr;assign crlen[4:0]=lut_len;
 assign crr[0]=(loader_state==DRESP||loader_state==NRESP||loader_state==RRESP);
 avm_mem_arbiter arbiter(.clk(clk),.rst_n(rst_n),.c_req_valid(crq),.c_req_ready(crdy),
  .c_req_addr(cra),.c_req_len(crlen),.c_rsp_valid(crv),.c_rsp_ready(crr),
  .c_rsp_data(crdata),.c_rsp_last(crlast),.req_valid(req_valid),.req_ready(req_ready),
  .req_addr(req_addr),.req_len(req_len),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
  .rsp_data(rsp_data),.rsp_last(rsp_last));
 integer si;
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;loader_state<=IDLE;macro_index<=0;mx<=0;my<=0;buffer_sel<=0;
   load_buffer<=0;load_macro_index<=0;bank_busy<=0;patch_ready_slots<=0;
   camera<=0;desc_i<=0;route_beat<=0;issue_i<=0;written<=0;
   bases<=0;sw<=0;sh<=0;stride<=0;dbase<=0;rbase<=0;
   active<=0;nodes_left<=0;beats_left<=0;burst_left<=0;nx<=0;ny<=0;side<=0;
   load_addr<=0;output_frame_id<=0;cycle_count<=0;error_flags<=0;
   for(si=0;si<4;si=si+1)patch_addr[si]<=0;
   for(si=0;si<8;si=si+1)step[si]<=4;
  end else begin
   if(busy)cycle_count<=cycle_count+1'b1;
   if(blend_valid&&blend_ready)written<=written+1'b1;
   if((&pixel_valid)&&blend_s_ready&&ta[70:0]!=tb[70:0])error_flags[0]<=1;
   if(launch)begin
     bases<=camera_bases;sw<=src_width;sh<=src_height;stride<=src_stride;dbase<=desc_base;rbase<=route_base;
     output_frame_id<=frame_id;cycle_count<=0;error_flags<=0;
     macro_index<=0;mx<=0;my<=0;buffer_sel<=0;desc_i<=0;written<=0;state<=WAIT_PATCH;
     load_buffer<=0;load_macro_index<=0;bank_busy<=0;patch_ready_slots<=0;loader_state<=IDLE;
   end else begin
   case(loader_state)
    IDLE:if(busy&&load_macro_index<MX*MY&&!bank_busy[load_buffer])begin
     bank_busy[load_buffer]<=1;desc_i<=0;loader_state<=DREQ;
    end
    DREQ:if(crq[0]&&crdy[0])loader_state<=DRESP;
    DRESP:if(crv[0]&&crr[0])begin
     patch_addr[desc_i]<=crdata[31:0];step[{load_buffer,desc_i[1:0]}]<=crdata[34:32];
     active[{load_buffer,desc_i[1:0]}]<=crdata[35]&&crdata[34:32]<=4;
     if(crdata[34:32]>4)error_flags[1]<=1;
     desc_i<=desc_i+1'b1;
     if(crlast[0])begin
      if(desc_i==3)begin camera<=0;loader_state<=CAMSET;end
      else loader_state<=DREQ;
     end
    end
    CAMSET:begin
     if(camera==4)begin route_beat<=0;loader_state<=RREQ;end
     else if(!active[{load_buffer,camera[1:0]}])camera<=camera+1'b1;
     else begin
      side<=(16>>step[{load_buffer,camera[1:0]}])+1;
      nodes_left<=((16>>step[{load_buffer,camera[1:0]}])+1)*((16>>step[{load_buffer,camera[1:0]}])+1);
      beats_left<=((((16>>step[{load_buffer,camera[1:0]}])+1)*((16>>step[{load_buffer,camera[1:0]}])+1))+1)>>1;
      nx<=0;ny<=0;load_addr<=patch_addr[camera];loader_state<=NREQ;
     end
    end
    NREQ:if(crq[0]&&crdy[0])begin burst_left<=lut_len;loader_state<=NRESP;end
    NRESP:if(crv[0]&&crr[0])begin
     load_addr<=load_addr+16;beats_left<=beats_left-1'b1;
     burst_left<=burst_left-1'b1;
     nodes_left<=nodes_left>1?nodes_left-2:0;
     nx<=nx_next;ny<=ny_next;
     if(beats_left==1)begin camera<=camera+1'b1;loader_state<=CAMSET;end
     else if(burst_left==1)loader_state<=NREQ;
     else loader_state<=NRESP;
    end
    RREQ:if(crq[0]&&crdy[0])loader_state<=RRESP;
    RRESP:if(crv[0]&&crr[0])begin
     routes0[{load_buffer,route_beat[4:0]}]<=crdata[15:0];routes1[{load_buffer,route_beat[4:0]}]<=crdata[31:16];
     routes2[{load_buffer,route_beat[4:0]}]<=crdata[47:32];routes3[{load_buffer,route_beat[4:0]}]<=crdata[63:48];
     routes4[{load_buffer,route_beat[4:0]}]<=crdata[79:64];routes5[{load_buffer,route_beat[4:0]}]<=crdata[95:80];
     routes6[{load_buffer,route_beat[4:0]}]<=crdata[111:96];routes7[{load_buffer,route_beat[4:0]}]<=crdata[127:112];
     route_beat<=route_beat+1'b1;
     if(route_beat==31)begin
      patch_ready_slots[load_buffer]<=1;load_buffer<=!load_buffer;
      load_macro_index<=load_macro_index+1'b1;loader_state<=IDLE;
     end else if(crlast[0])loader_state<=RREQ;
    end
    default:loader_state<=IDLE;
   endcase
   case(state)
    IDLE:state<=IDLE;
    WAIT_PATCH:if(patch_ready_slots[buffer_sel])begin
     patch_ready_slots[buffer_sel]<=0;issue_i<=0;written<=0;state<=RUN;
    end
    RUN:if(!inside_frame||patch_s_ready)begin
     if(issue_i==255)state<=DRAIN;else issue_i<=issue_i+1'b1;
     if(inside_frame&&route[4]&&route[5]&&route[1:0]==route[3:2])error_flags[2]<=1;
    end
    DRAIN:if(written==macro_pixels)begin
     bank_busy[buffer_sel]<=0;
     if(macro_index==MX*MY-1)state<=FINISH;
     else begin
      macro_index<=macro_index+1'b1;buffer_sel<=!buffer_sel;state<=WAIT_PATCH;
      if(mx==MX-1)begin mx<=0;my<=my+1'b1;end else mx<=mx+1'b1;
     end
    end
    FINISH:if(frame_output_done)state<=IDLE;
    default:state<=IDLE;
   endcase
   end
  end
 end
endmodule
