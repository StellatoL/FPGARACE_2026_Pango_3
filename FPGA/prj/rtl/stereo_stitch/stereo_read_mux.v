// Ownership lasts through logical completion, after all physical DDR reads retire.
module stereo_read_mux (
    input wire clk, input wire rst_n,
    input wire seq_valid, seq_ready, input wire [2:0] display_mode,
    input wire random_mode, req_valid, req_ready,
    input wire single_done, quad_done, rsp_valid, rsp_ready,
    output wire [2:0] route, output wire idle
);
    // Routes: 0 idle, 1 single, 3 coordinate read, 4 quad.
    reg [2:0] owner;
    assign idle=owner==0;
    assign route=owner!=0 ? owner : seq_valid ?
                 (display_mode==0 ? 3'd4 : 3'd1) :
                 random_mode && req_valid ? 3'd3 : 3'd0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) owner<=0;
        else if(owner==0) begin
            if(seq_valid && seq_ready) owner<=route;
            else if(req_valid && req_ready) owner<=3;
        end else if((owner==1 && single_done) ||
                    (owner==3 && rsp_valid && rsp_ready) || (owner==4 && quad_done)) owner<=0;
    end
endmodule
