`timescale 1ns/1ps
module tb_arithmetic;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg bsv=0,bmr=0,gsv=0,gmr=0,csv=0,cmr=0;
    wire bsr,bmv,gsr,gmv,csr,cmv;
    reg [63:0] bp; reg [7:0] bx,by; reg [31:0] bt;
    wire [23:0] brgb; wire [31:0] bmt;
    reg signed [20:0] gu[0:3],gv[0:3];
    reg [3:0] gx,gy; reg [2:0] gl; reg [31:0] gt;
    wire signed [20:0] gou,gov; wire [31:0] gmt;
    reg [23:0] ca,cb; reg [47:0] cga,cgb; reg [8:0] alpha;
    reg va,vb; reg [31:0] ct;
    wire [15:0] cp; wire [31:0] cmt;
    avm_bilinear #(.TAG_W(32)) bilinear(clk,rst_n,bsv,bsr,bp,bx,by,bt,bmv,bmr,brgb,bmt);
    avm_grid_interp #(.TAG_W(32)) grid(clk,rst_n,gsv,gsr,gu[0],gu[1],gu[2],gu[3],
        gv[0],gv[1],gv[2],gv[3],gx,gy,gl,gt,gmv,gmr,gou,gov,gmt);
    avm_gain_blend #(.TAG_W(32)) color(clk,rst_n,csv,csr,ca,cb,cga,cgb,alpha,va,vb,ct,cmv,cmr,cp,cmt);

    function automatic integer channel(input reg[15:0] p,input integer c);
        integer value;
        begin
            if(c==0) begin value=(p>>11)&31; channel=value*8+value/4; end
            else if(c==1) begin value=(p>>5)&63; channel=value*4+value/16; end
            else begin value=p&31; channel=value*8+value/4; end
        end
    endfunction
    function automatic [23:0] sample_ref(input reg[63:0] p,input integer fx,fy);
        longint unsigned n; integer c,q; reg[23:0] out;
        begin
            out=0;
            for(c=0;c<3;c=c+1) begin
                n=channel(p[15:0],c)*(256-fx)*(256-fy)
                 +channel(p[31:16],c)*fx*(256-fy)
                 +channel(p[47:32],c)*(256-fx)*fy
                 +channel(p[63:48],c)*fx*fy;
                q=(n+32768)/65536;
                out=(out<<8)|q;
            end
            sample_ref=out;
        end
    endfunction
    function automatic longint signed divide_round(input longint signed n,input integer denominator);
        if(n<0) divide_round=-((-n+denominator/2)/denominator);
        else divide_round=(n+denominator/2)/denominator;
    endfunction
    function automatic signed [20:0] grid_ref(input integer a,b,c,d,fx,fy,step);
        longint signed h0,h1,n;
        begin
            h0=divide_round(longint'(a)*(step-fx)+longint'(b)*fx,step);
            h1=divide_round(longint'(c)*(step-fx)+longint'(d)*fx,step);
            n=divide_round(h0*(step-fy)+h1*fy,step);
            grid_ref=n;
        end
    endfunction
    function automatic [15:0] color_ref(input reg[23:0] a,b,input reg[47:0] ga,gb,
                                       input integer al,input bit a_valid,b_valid);
        integer c,ac,bc,pc,result;
        begin
            if(!b_valid) al=256; else if(!a_valid) al=0;
            result=0;
            for(c=0;c<3;c=c+1) begin
                ac=(((a>>(16-8*c))&255)*((ga>>(16*c))&65535)+2048)/4096;
                bc=(((b>>(16-8*c))&255)*((gb>>(16*c))&65535)+2048)/4096;
                if(ac>255) ac=255; if(bc>255) bc=255;
                if(!a_valid) ac=0; if(!b_valid) bc=0;
                pc=(ac*al+bc*(256-al)+128)/256;
                if(c==1) result=(result<<6)|(pc/4);
                else result=(result<<5)|(pc/8);
            end
            color_ref=result;
        end
    endfunction
    reg[23:0] bexp[0:16000]; reg[31:0] betag[0:16000];
    reg signed[20:0] gexpu[0:16000],gexpv[0:16000]; reg[31:0] getag[0:16000];
    reg[15:0] cexp[0:16000]; reg[31:0] cetag[0:16000];
    time btime[0:16000],gtime[0:16000],ctime[0:16000];
    integer bs=0,bo=0,gs=0,go=0,cs=0,co=0;
    integer total_b=0,total_g=0,total_c=0;
    integer bfull=0,gfull=0,cfull=0;
    bit baccepted=0,gaccepted=0,caccepted=0;
    bit bstall=0,gstall=0,cstall=0;
    reg[55:0] bhold; reg[73:0] ghold; reg[47:0] chold;
    integer cycle=0,j;
    always @(posedge clk) begin
        baccepted=bsv&&bsr; gaccepted=gsv&&gsr; caccepted=csv&&csr;
        if(!rst_n) begin
            bs=0;bo=0;gs=0;go=0;cs=0;co=0;
            bstall=0;gstall=0;cstall=0;
        end else begin
            if(bstall && (!bmv || {brgb,bmt}!==bhold)) $fatal(1,"bilinear changed under stall");
            if(gstall && (!gmv || {gou,gov,gmt}!==ghold)) $fatal(1,"grid changed under stall");
            if(cstall && (!cmv || {cp,cmt}!==chold)) $fatal(1,"color changed under stall");
            bstall=bmv&&!bmr; bhold={brgb,bmt};
            gstall=gmv&&!gmr; ghold={gou,gov,gmt};
            cstall=cmv&&!cmr; chold={cp,cmt};
            if(baccepted) begin bexp[bs]=sample_ref(bp,bx,by);betag[bs]=bt;btime[bs]=$time;bs++; end
            if(gaccepted) begin
                gexpu[gs]=grid_ref(gu[0],gu[1],gu[2],gu[3],gx,gy,1<<gl);
                gexpv[gs]=grid_ref(gv[0],gv[1],gv[2],gv[3],gx,gy,1<<gl);
                getag[gs]=gt;gtime[gs]=$time;gs++;
            end
            if(caccepted) begin cexp[cs]=color_ref(ca,cb,cga,cgb,alpha,va,vb);cetag[cs]=ct;ctime[cs]=$time;cs++; end
            if(bmv&&bmr) begin
                if(bo>=bs || brgb!==bexp[bo] || bmt!==betag[bo])
                    $fatal(1,"bilinear mismatch index=%0d expected=%h got=%h tag=%d/%d",bo,bexp[bo],brgb,betag[bo],bmt);
                if(cycle<500) begin if($time-btime[bo]!=30) $fatal(1,"bilinear latency/II"); bfull++; end
                bo++;total_b++;
            end
            if(gmv&&gmr) begin
                if(go>=gs || gou!==gexpu[go] || gov!==gexpv[go] || gmt!==getag[go])
                    $fatal(1,"grid mismatch index=%0d exp=(%0d,%0d) actual=(%0d,%0d)",go,gexpu[go],gexpv[go],gou,gov);
                if(cycle<500) begin if($time-gtime[go]!=40) $fatal(1,"grid latency/II"); gfull++; end
                go++;total_g++;
            end
            if(cmv&&cmr) begin
                if(co>=cs || cp!==cexp[co] || cmt!==cetag[co])
                    $fatal(1,"color mismatch index=%0d exp=%h actual=%h",co,cexp[co],cp);
                if(cycle<500) begin if($time-ctime[co]!=20) $fatal(1,"color latency/II"); cfull++; end
                co++;total_c++;
            end
        end
    end
    initial begin
        repeat(3) @(negedge clk);
        rst_n=1;
        for(cycle=0;cycle<10000;cycle=cycle+1) begin
            @(negedge clk);
            // Flush occupied pipelines, including stalled outputs.
            if(cycle==777) rst_n=0;
            if(cycle==780) rst_n=1;
            if(!rst_n) begin bsv=0;gsv=0;csv=0;bmr=0;gmr=0;cmr=0; end
            else begin
                bmr=(cycle<500||cycle>8500)?1:$urandom_range(0,3)!=0;
                gmr=(cycle<500||cycle>8500)?1:$urandom_range(0,2)!=0;
                cmr=(cycle<500||cycle>8500)?1:$urandom_range(0,3)!=0;
                // A long common output stall verifies all elastic stages hold.
                if(cycle>=3000&&cycle<3250) begin bmr=0;gmr=0;cmr=0; end
                if(!bsv||baccepted) begin
                    bsv=cycle<8000 && (cycle<500 || $urandom_range(0,4)!=0);
                    bp={$urandom,$urandom};bx=cycle%256;by=$urandom;
                    if(cycle%7==0) bp=64'hffffffffffffffff;
                    if(cycle%7==1) bp=64'h001ff80007e00000;
                    if(cycle%7==2) begin bx=0;by=0; end
                    bt=cycle;
                end
                if(!gsv||gaccepted) begin
                    gsv=cycle<8000 && (cycle<500 || $urandom_range(0,4)!=0);
                    gl=cycle%5;gx=$urandom_range(0,(1<<gl)-1);gy=$urandom_range(0,(1<<gl)-1);
                    for(j=0;j<4;j++) begin gu[j]=$urandom;gv[j]=$urandom; end
                    if(cycle%7==0) begin
                        gl=1;gx=1;gy=1;
                        gu[0]=-1;gu[1]=0;gu[2]=0;gu[3]=0;
                        gv[0]=1;gv[1]=0;gv[2]=0;gv[3]=0;
                    end
                    if(cycle%7==1) begin
                        gu[0]=-1048576;gu[1]=1048575;gu[2]=-1048576;gu[3]=1048575;
                        gv[0]=-1048576;gv[1]=-1048576;gv[2]=-1048576;gv[3]=-1048576;
                    end
                    gt=cycle;
                end
                if(!csv||caccepted) begin
                    csv=cycle<8000 && (cycle<500 || $urandom_range(0,4)!=0);
                    ca=$urandom;cb=$urandom;va=cycle%2;vb=(cycle/2)%2;
                    for(j=0;j<3;j++) begin cga[j*16+:16]=$urandom_range(0,8192);cgb[j*16+:16]=$urandom_range(0,8192); end
                    if(cycle%17==0) cga=48'hffffffffffff;
                    if(cycle%17==1) begin ca=24'hff0000;cb=24'h0000ff;cga=48'h100010001000;cgb=cga; end
                    alpha=cycle%257;ct=cycle;
                end
            end
        end
        repeat(10) @(negedge clk);
        if(bs!=bo||gs!=go||cs!=co) $fatal(1,"undrained pipelines");
        if(total_b<4000||total_g<4000||total_c<4000) $fatal(1,"insufficient randomized coverage");
        if(bfull<490||gfull<490||cfull<490) $fatal(1,"II=1 coverage failed");
        $display("PASS arithmetic bilinear=%0d grid=%0d gain_blend=%0d II1=%0d/%0d/%0d stalls, signed ties, saturation, reset flush",total_b,total_g,total_c,bfull,gfull,cfull);
        $finish;
    end
    initial begin #200000; $fatal(1,"arithmetic timeout"); end
endmodule
