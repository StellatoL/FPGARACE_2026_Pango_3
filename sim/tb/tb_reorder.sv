`timescale 1ns/1ps
module reorder_case #(parameter W=37,H=35,CASE_ID=0)(output reg complete=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,start=0,sv=0,mr=0;
    reg [15:0] sp;reg [11:0] sx,sy;
    wire sr,mv,sof,eol,done;wire[15:0]mp;
    avm_stripe_reorder #(.WIDTH(W),.HEIGHT(H)) dut(clk,rst_n,start,sv,sr,sp,sx,sy,mv,mr,mp,sof,eol,done);
    integer received=0,sent=0,frame_id=0,cycles=0;
    integer stalled_writes=0,stalled_outputs=0;
    bit held=0,done_seen=0;reg[17:0]hold_value;
    function automatic [15:0] value(input integer x,y,frame);
        value=(y*W+x)*37+frame*193+CASE_ID;
    endfunction
    always @(posedge clk) begin
        if(!rst_n||start) begin received=0;sent=0;held=0;done_seen=0; end
        else begin
            if(held&&(!mv||{mp,sof,eol}!==hold_value)) $fatal(1,"reorder stall instability case %0d",CASE_ID);
            held=mv&&!mr;hold_value={mp,sof,eol};
            if(sv&&sr) sent++;
            if(sv&&!sr) stalled_writes++;
            if(mv&&!mr) stalled_outputs++;
            if(mv&&mr) begin
                if(received>=W*H || mp!==value(received%W,received/W,frame_id))
                    $fatal(1,"reorder data case %0d index %0d expected %h got %h",CASE_ID,received,value(received%W,received/W,frame_id),mp);
                if(sof!==(received==0)||eol!==(received%W==W-1)) $fatal(1,"reorder markers case %0d",CASE_ID);
                received++;
            end
            if(done) begin
                if(done_seen||received!=W*H) $fatal(1,"invalid completion case %0d",CASE_ID);
                done_seen=1;
            end
        end
    end
    task automatic begin_frame(input integer id);
        begin
            @(negedge clk);start=1;sv=0;mr=0;frame_id=id;
            @(negedge clk);start=0;
        end
    endtask
    task automatic send_frame;
        integer ty,tx,ly,lx;
        begin
            for(ty=0;ty<H;ty+=16)
                for(tx=0;tx<W;tx+=16)
                    for(ly=0;ly<16&&ty+ly<H;ly++)
                        for(lx=0;lx<16&&tx+lx<W;lx++) begin
                            @(negedge clk);sv=1;sx=tx+lx;sy=ty+ly;sp=value(tx+lx,ty+ly,frame_id);
                            @(posedge clk);while(!sr) @(posedge clk);
                        end
            @(negedge clk);sv=0;
        end
    endtask
    task automatic consume_frame(input bit long_stall);
        integer ticks;
        begin
            ticks=0;
            while(!done_seen) begin
                @(negedge clk);
                // Both buffers fill before release; exercise ownership while
                // the writer presents the first pixel of the third stripe.
                mr=!(long_stall&&ticks<W*40) && ($urandom_range(0,4)!=0);
                ticks++;
                if(ticks>W*H*10+5000) $fatal(1,"reorder deadlock %0d",CASE_ID);
            end
            @(negedge clk);mr=0;
        end
    endtask
    task automatic fill_first_stripe;
        integer tx,ly,lx;
        begin
            for(tx=0;tx<W;tx+=16)
                for(ly=0;ly<16&&ly<H;ly++)
                    for(lx=0;lx<16&&tx+lx<W;lx++) begin
                        @(negedge clk);sv=1;sx=tx+lx;sy=ly;sp=value(tx+lx,ly,frame_id);
                        @(posedge clk);while(!sr) @(posedge clk);
                    end
            @(negedge clk);sv=0;
        end
    endtask
    initial begin
        repeat(3) @(negedge clk);rst_n=1;
        // Reset with a partly written stripe must discard it entirely.
        begin_frame(0);
        @(negedge clk);sv=1;sx=0;sy=0;sp=value(0,0,0);
        @(negedge clk);sv=0;rst_n=0;
        repeat(2) @(negedge clk);rst_n=1;
        // Also flush a completed stripe while its output is stalled.
        begin_frame(0);
        fill_first_stripe();
        wait(mv);
        repeat(5) @(negedge clk);
        rst_n=0;
        repeat(2) @(negedge clk);
        rst_n=1;
        begin_frame(1);
        fork send_frame();consume_frame(1);join
        if(received!=W*H||sent!=W*H) $fatal(1,"count mismatch case %0d",CASE_ID);
        begin_frame(2);
        fork send_frame();consume_frame(0);join
        if(received!=W*H||sent!=W*H) $fatal(1,"repeat count mismatch case %0d",CASE_ID);
        if(H>32&&stalled_writes==0) $fatal(1,"ownership backpressure not covered");
        if(stalled_outputs==0) $fatal(1,"output backpressure not covered");
        $display("PASS reorder case=%0d %0dx%0d frames=2 reset flush write_stalls=%0d output_stalls=%0d",CASE_ID,W,H,stalled_writes,stalled_outputs);
        complete=1;
    end
endmodule
module tb_reorder;
    wire a,b,c,d,e;
    reorder_case #(.W(37),.H(35),.CASE_ID(0)) partial(a);
    reorder_case #(.W(32),.H(48),.CASE_ID(1)) aligned_case(b);
    reorder_case #(.W(1),.H(1),.CASE_ID(2)) tiny(c);
    reorder_case #(.W(1280),.H(35),.CASE_ID(3)) all_chunks(d);
    reorder_case #(.W(257),.H(35),.CASE_ID(4)) partial_chunk(e);
    initial begin wait(a&&b&&c&&d&&e);$display("PASS all reorder cases");$finish;end
    initial begin #5000000;$fatal(1,"reorder suite timeout");end
endmodule
