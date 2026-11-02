// Four independent 640x360 frame buffers. Each channel has a private DDR
// address range; the shared AXI command/data ports are arbitrated here.
module quad_ddr3_framebuffer #(
    parameter ADDR_WIDTH=28, parameter MEM_DQ_WIDTH=16,
    // Four slots per camera, each slot is 2^18 DDR address units.
    parameter CHANNEL_STRIDE=1048576,
    parameter [3:0] CAMERA_ENABLE=4'b0001
)(
    input wire ddr_clk, input wire ddr_rstn,
    input wire pix_clk, input wire [3:0] in_clk, input wire [3:0] in_vs, input wire [3:0] in_de,
    input wire [63:0] in_data,
    input wire out_vs, input wire out_active, input wire [11:0] out_x, input wire [11:0] out_y,
    output reg [15:0] out_data, output reg out_de,
    output wire [ADDR_WIDTH-1:0] axi_awaddr, output wire [3:0] axi_awid,
    output wire [3:0] axi_awlen, input wire axi_awready, output wire axi_awvalid,
    output wire [127:0] axi_wdata, output wire [15:0] axi_wstrb,
    input wire axi_wlast, output wire axi_wvalid, input wire axi_wready,
    output wire [ADDR_WIDTH-1:0] axi_araddr, output wire [3:0] axi_arid,
    output wire [3:0] axi_arlen, input wire axi_arready, output wire axi_arvalid,
    output wire axi_rready, input wire [127:0] axi_rdata, input wire axi_rvalid,
    input wire axi_rlast, input wire [3:0] axi_rid
);
    wire [3:0] awv, awr, wv, ww, arv, arr, rr;
    wire [ADDR_WIDTH-1:0] awa[0:3], ara[0:3];
    wire [3:0] awl[0:3], arl[0:3], awi[0:3], ari[0:3];
    wire [127:0] wd[0:3]; wire [15:0] ws[0:3];
    wire [15:0] pix[0:3]; wire [3:0] de, ready;
    wire ddr_init_done = ddr_rstn;
    reg [1:0] wsel, rsel, wp, rp;
    reg wbusy, rbusy, wgrant, rgrant;
    wire [3:0] rd_en;
    assign rd_en[0] = out_active && out_y < 360 && out_x < 640;
    assign rd_en[1] = out_active && out_y < 360 && out_x >= 640;
    assign rd_en[2] = out_active && out_y >= 360 && out_x < 640;
    assign rd_en[3] = out_active && out_y >= 360 && out_x >= 640;
    genvar g;
    generate for (g=0; g<4; g=g+1) begin: FB
      if (CAMERA_ENABLE[g]) begin: ENABLED
        fram_buf #(.CTRL_ADDR_WIDTH(ADDR_WIDTH),.H_NUM(640),.V_NUM(360),
                   .PIX_WIDTH(16),.ADDR_OFFSET(g*CHANNEL_STRIDE),
                   .LINE_ADDR_WIDTH(18),.QUAD_MODE(1)) u_fb (
          .ddr_clk(ddr_clk),.ddr_rstn(ddr_init_done),.vin_clk(in_clk[g]),
          .wr_fsync(in_vs[g]),.wr_en(in_de[g] && CAMERA_ENABLE[g]),.wr_data(in_data[g*16 +: 16]),
          .vout_clk(pix_clk),.rd_fsync(out_vs && CAMERA_ENABLE[g]),.rd_en(rd_en[g] && CAMERA_ENABLE[g]),
          .vout_de(de[g]),.vout_data(pix[g]),.init_done(ready[g]),
          .axi_awaddr(awa[g]),.axi_awid(awi[g]),.axi_awlen(awl[g]),.axi_awsize(),.axi_awburst(),.axi_awready(awr[g]),.axi_awvalid(awv[g]),
          // WLAST belongs only to the channel whose burst owns the shared
          // write-data bus. Broadcasting it advances every channel's line
          // counters when any one camera finishes a burst.
          .axi_wdata(wd[g]),.axi_wstrb(ws[g]),
          .axi_wlast(axi_wlast && axi_wready && wbusy && (wsel==g)),
          .axi_wvalid(wv[g]),.axi_wready(ww[g]),.axi_bid(4'd0),
          .axi_araddr(ara[g]),.axi_arid(ari[g]),.axi_arlen(arl[g]),.axi_arsize(),.axi_arburst(),.axi_arvalid(arv[g]),.axi_arready(arr[g]),
          .axi_rready(rr[g]),.axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid && rbusy && rsel==g),.axi_rlast(axi_rlast),.axi_rid(axi_rid));
      end else begin: DISABLED
        // Do not instantiate RAM IP for an unconnected camera.
        assign awa[g]=0; assign awi[g]=0; assign awl[g]=0; assign awv[g]=0;
        assign wd[g]=0; assign ws[g]=0; assign wv[g]=0;
        assign ara[g]=0; assign ari[g]=0; assign arl[g]=0; assign arv[g]=0;
        assign rr[g]=0; assign pix[g]=0; assign de[g]=0; assign ready[g]=0;
      end
    end endgenerate
    integer k; reg [1:0] wn, rn; reg wf, rf;
    always @* begin
        wn=wp; rn=rp; wf=0; rf=0;
        for(k=0;k<4;k=k+1) begin
            if(awv[(wp+k)%4] && !wf) begin wn=(wp+k)%4; wf=1; end
            if(arv[(rp+k)%4] && !rf) begin rn=(rp+k)%4; rf=1; end
        end
    end
    assign axi_awaddr=awa[wsel]; assign axi_awid=awi[wsel]; assign axi_awlen=awl[wsel]; assign axi_awvalid=awv[wsel]&&wgrant;
    assign axi_wdata=wd[wsel]; assign axi_wstrb=ws[wsel]; assign axi_wvalid=wv[wsel]&&wbusy;
    assign axi_araddr=ara[rsel]; assign axi_arid=ari[rsel]; assign axi_arlen=arl[rsel]; assign axi_arvalid=arv[rsel]&&rgrant;
    assign axi_rready=rr[rsel]&&rbusy;
    generate for(g=0;g<4;g=g+1) begin: ARB
      assign awr[g]=(wgrant && wsel==g) ? axi_awready:0; assign ww[g]=(wbusy&&wsel==g)?axi_wready:0;
      assign arr[g]=(rgrant && rsel==g)?axi_arready:0;
    end endgenerate
    always @(posedge ddr_clk or negedge ddr_rstn) begin
      if(!ddr_rstn) begin
        wbusy<=0;rbusy<=0;wsel<=0;rsel<=0;wp<=0;rp<=0;wgrant<=0;rgrant<=0;
      end else begin
        if(!wbusy && !wgrant && wf) begin wgrant<=1;wsel<=wn; end
        if(wgrant && axi_awvalid && axi_awready) begin wbusy<=1;wgrant<=0; end
        else if(wbusy && axi_wvalid && axi_wready && axi_wlast) begin wbusy<=0;wp<=wsel+1'b1; end
        if(!rbusy && !rgrant && rf) begin rgrant<=1;rsel<=rn; end
        if(rgrant && axi_arvalid && axi_arready) begin rbusy<=1;rgrant<=0; end
        else if(rbusy && axi_rvalid && axi_rlast) begin rbusy<=0;rp<=rsel+1'b1; end
      end
    end
    reg [1:0] quadrant_d1, quadrant_d2;
    reg active_d1, active_d2;
    reg [3:0] ready_meta, ready_sync, ready_frame;
    reg vs_d;
    always @(posedge pix_clk or negedge ddr_rstn) begin
      if (!ddr_rstn) begin
        quadrant_d1<=0;quadrant_d2<=0;active_d1<=0;active_d2<=0;
        ready_meta<=0;ready_sync<=0;ready_frame<=0;vs_d<=0;out_de<=0;out_data<=0;
      end else begin
        quadrant_d1 <= {out_y>=360,out_x>=640}; quadrant_d2<=quadrant_d1;
        active_d1<=out_active;active_d2<=active_d1;
        ready_meta<=ready;ready_sync<=ready_meta;vs_d<=out_vs;
        if(out_vs && !vs_d) ready_frame<=ready_sync & CAMERA_ENABLE;
        out_de<=active_d2;
        out_data <= ready_frame[quadrant_d2] ? pix[quadrant_d2] : 16'd0;
      end
    end
endmodule
