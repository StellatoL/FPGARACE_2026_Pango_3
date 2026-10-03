// One outstanding burst, round-robin among LUT/cache A/cache B.
// The selected request is held through backpressure; no response interleaving.
module avm_mem_arbiter (
    input wire clk, input wire rst_n,
    input wire [2:0] c_req_valid, output reg [2:0] c_req_ready,
    input wire [95:0] c_req_addr, input wire [14:0] c_req_len,
    output reg [2:0] c_rsp_valid, input wire [2:0] c_rsp_ready,
    output wire [383:0] c_rsp_data, output reg [2:0] c_rsp_last,
    output reg req_valid, input wire req_ready,
    output reg [31:0] req_addr, output reg [4:0] req_len,
    input wire rsp_valid, output reg rsp_ready,
    input wire [127:0] rsp_data, input wire rsp_last
);
    localparam IDLE=2'd0, ISSUE=2'd1, RESPONSE=2'd2;
    reg [1:0] state, owner, next_owner;
    integer i, candidate;
    reg found;
    reg [1:0] selected;
    assign c_rsp_data={rsp_data,rsp_data,rsp_data};
    always @* begin
        selected=0; found=0; candidate=0;
        for(i=0;i<3;i=i+1) begin
            candidate=next_owner+i;
            if(candidate>=3) candidate=candidate-3;
            if(!found && c_req_valid[candidate]) begin
                selected=candidate; found=1;
            end
        end
        c_req_ready=0; c_rsp_valid=0; c_rsp_last=0;
        req_valid=0; req_addr=0; req_len=0; rsp_ready=0;
        if(state==IDLE && found) begin
            req_valid=c_req_valid[selected];
            req_addr=c_req_addr[selected*32+:32];
            req_len=c_req_len[selected*5+:5];
            c_req_ready[selected]=req_ready;
        end else if(state==ISSUE) begin
            req_valid=c_req_valid[owner];
            req_addr=c_req_addr[owner*32+:32];
            req_len=c_req_len[owner*5+:5];
            c_req_ready[owner]=req_ready;
        end
        if(state==RESPONSE) begin
            c_rsp_valid[owner]=rsp_valid;
            c_rsp_last[owner]=rsp_last;
            rsp_ready=c_rsp_ready[owner];
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin state<=IDLE; owner<=0; next_owner<=0; end
        else case(state)
            IDLE: if(found) begin
                owner<=selected;
                // Fall through on an immediately accepted grant; lock the
                // selected owner if the memory request is backpressured.
                state<=req_ready?RESPONSE:ISSUE;
            end
            ISSUE: if(req_valid && req_ready) state<=RESPONSE;
            RESPONSE: if(rsp_valid && rsp_ready && rsp_last) begin
                next_owner<=(owner==2)?0:owner+1'b1;
                state<=IDLE;
            end
            default: state<=IDLE;
        endcase
    end
endmodule
