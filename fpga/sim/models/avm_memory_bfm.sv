`timescale 1ns/1ps
// Behavioral service model only. All handshakes are core-clock synchronous;
// DDR clocks are accounted as equivalent time (131.25 MHz versus 100 MHz).
// Each capture job writes 16 virtual beats, queued at CAPTURE_MBPS MB/s and
// round-robin scheduled with complete read bursts. It consumes service time.
module avm_memory_bfm #(
    parameter integer MEMORY_WORDS=1048576,
    parameter MEMHEX="",
    parameter real CAPTURE_MBPS=221.184,
    parameter integer STRESS=0,
    parameter integer SEED=32'h51ac35a7
) (
    input wire clk, input wire rst_n,
    input wire req_valid, output wire req_ready,
    input wire [31:0] req_addr, input wire [4:0] req_len,
    output reg rsp_valid, input wire rsp_ready,
    output reg [127:0] rsp_data, output reg rsp_last
);
    reg [127:0] memory[0:MEMORY_WORDS-1];
    reg [63:0] read_beats,virtual_write_beats,request_count,service_cycles,latency_cycles;
    reg [63:0] capture_jobs_queued;
    integer state,delay_count,word_index,beats_left,capture_pending;
    integer cycle_count,request_cycle;
    reg [31:0] random_state;
    real capture_credit;
    reg last_was_read;
    string memory_filename;
    localparam IDLE=0, WAIT_READ=1, READ=2, CAPTURE=3;
    wire capture_turn=(capture_pending>0) && (!req_valid || last_was_read);
    wire random_accept=(STRESS==0 || random_state[1:0]!=0);
    assign req_ready=rst_n && state==IDLE && !capture_turn && random_accept;
    function automatic integer ddr_to_core(input integer clocks);
        ddr_to_core=(clocks*16+20)/21;
    endfunction
    initial begin
        memory_filename=MEMHEX;
        if($value$plusargs("MEMHEX=%s",memory_filename)) begin end
        if(memory_filename!="") begin
            $display("BFM loading %s (%0d 128-bit words)",memory_filename,MEMORY_WORDS);
            $readmemh(memory_filename,memory);
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;delay_count<=0;word_index<=0;beats_left<=0;
            capture_pending=0;capture_credit=0.0;last_was_read<=0;
            random_state<=SEED;rsp_valid<=0;rsp_data<=0;rsp_last<=0;
            read_beats<=0;virtual_write_beats<=0;request_count<=0;
            service_cycles<=0;latency_cycles<=0;capture_jobs_queued<=0;
            cycle_count<=0;request_cycle<=0;
        end else begin
            cycle_count<=cycle_count+1;
            random_state<={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
            capture_credit=capture_credit+CAPTURE_MBPS/100.0;
            if(capture_credit>=256.0) begin
                capture_credit=capture_credit-256.0;
                capture_pending=capture_pending+1;
                capture_jobs_queued<=capture_jobs_queued+1;
            end
            if(state!=IDLE) service_cycles<=service_cycles+1;
            case(state)
                IDLE: begin
                    rsp_valid<=0;
                    // req_ready was calculated before this edge; never take a
                    // freshly enqueued capture job over an accepted read.
                    if(req_valid && req_ready) begin
                        if(req_len<1 || req_len>16 || req_addr[3:0]!=0 || ({1'b0,req_addr[11:0]}+req_len*16)>4096)
                            $fatal(1,"Illegal burst addr=%h len=%0d",req_addr,req_len);
                        if((req_addr>>4)+req_len>MEMORY_WORDS) $fatal(1,"DDR read outside fixture");
                        word_index<=req_addr>>4;beats_left<=req_len;
                        delay_count<=ddr_to_core(8+((STRESS!=0)?random_state[14:8]%65:0))+2-1;
                        request_count<=request_count+1;request_cycle<=cycle_count;
                        state<=WAIT_READ;last_was_read<=1;
                    end else if(capture_pending>0 && (!req_valid || last_was_read)) begin
                        capture_pending=capture_pending-1;
                        delay_count<=ddr_to_core(8+16)-1;
                        state<=CAPTURE;last_was_read<=0;
                    end
                end
                CAPTURE: if(delay_count==0) begin
                    virtual_write_beats<=virtual_write_beats+16;state<=IDLE;
                end else delay_count<=delay_count-1;
                WAIT_READ: if(delay_count==0) begin
                    rsp_data<=memory[word_index];rsp_last<=(beats_left==1);rsp_valid<=1;
                    latency_cycles<=latency_cycles+(cycle_count-request_cycle);state<=READ;
                end else delay_count<=delay_count-1;
                READ: if(!rsp_valid || rsp_ready) begin
                    if(rsp_valid) begin
                        read_beats<=read_beats+1;
                        word_index<=word_index+1;beats_left<=beats_left-1;
                        if(beats_left==1) begin rsp_valid<=0;rsp_last<=0;state<=IDLE;end
                        else if(STRESS!=0 && random_state[4:2]==0) rsp_valid<=0;
                        else begin
                            rsp_data<=memory[word_index+1];rsp_last<=(beats_left==2);rsp_valid<=1;
                        end
                    end else if(STRESS==0 || random_state[6:5]!=0) begin
                        rsp_data<=memory[word_index];rsp_last<=(beats_left==1);rsp_valid<=1;
                    end
                end
            endcase
        end
    end
endmodule
