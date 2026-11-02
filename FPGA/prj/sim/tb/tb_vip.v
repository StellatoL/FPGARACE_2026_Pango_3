`timescale 1ns/1ps

`define IMG_HDISP 713
`define IMG_VDISP 652

module tb_top_bmp();

    reg         clk;
    reg         rst_n;
    reg  [1:0]  vip_crtl;

    reg         pre_vsync;
    reg         pre_href;
    reg         pre_de;
    reg  [15:0] pre_rgb;

    wire        post_vsync;
    wire        post_href;
    wire        post_de;
    wire [23:0] post_rgb;

    vip #(
        .IMG_HDISP(`IMG_HDISP),
        .IMG_VDISP(`IMG_VDISP)
    ) u_vip (
        .clk              (clk),
        .rst_n            (rst_n),
        .vip_crtl         (vip_crtl),
        .pre_frame_vsync  (pre_vsync),
        .pre_frame_href   (pre_href),
        .pre_frame_de     (pre_de),
        .pre_rgb          (pre_rgb),
        .post_frame_vsync (post_vsync),
        .post_frame_href  (post_href),
        .post_frame_de    (post_de),
        .post_rgb         (post_rgb)
    );

    wire        white_balance_vsync;
    wire        white_balance_href;
    wire        white_balance_de;
    wire [23:0] white_balance_rgb;

    wire        dehaze_vsync;
    wire        dehaze_href;
    wire        dehaze_de;
    wire [23:0] dehaze_rgb;

    wire        dark_vsync;
    wire        dark_href;
    wire        dark_de;
    wire [7:0]  dark_val;

    wire        trans_vsync;
    wire        trans_href;
    wire        trans_de;
    wire [7:0]  trans_val;

    wire        light_vsync;
    wire        light_href;
    wire        light_de;
    wire [7:0]  light_y;
    wire [7:0]  light_cb;
    wire [7:0]  light_cr;

    wire        light_rgb_vsync;
    wire        light_rgb_href;
    wire        light_rgb_de;
    wire [7:0]  light_rgb_r;
    wire [7:0]  light_rgb_g;
    wire [7:0]  light_rgb_b;

    assign white_balance_vsync = u_vip.white_balance_frame_vsync;
    assign white_balance_href  = u_vip.white_balance_frame_href;
    assign white_balance_de    = u_vip.white_balance_frame_clken;
    assign white_balance_rgb   = u_vip.white_balance_rgb;

    assign dehaze_vsync = u_vip.pos_frame_vsync;
    assign dehaze_href  = u_vip.pos_frame_href;
    assign dehaze_de    = u_vip.pos_frame_clken;
    assign dehaze_rgb   = u_vip.pos_rgb;

    assign dark_vsync = u_vip.u_vip_dark_channel_top.dark_frame_vsync;
    assign dark_href  = u_vip.u_vip_dark_channel_top.dark_frame_href;
    assign dark_de    = u_vip.u_vip_dark_channel_top.dark_frame_clken;
    assign dark_val   = u_vip.u_vip_dark_channel_top.dark_img;

    assign trans_vsync = u_vip.u_vip_dark_channel_top.transmission_frame_vsync;
    assign trans_href  = u_vip.u_vip_dark_channel_top.transmission_frame_href;
    assign trans_de    = u_vip.u_vip_dark_channel_top.transmission_frame_clken;
    assign trans_val   = u_vip.u_vip_dark_channel_top.transmission;

    assign light_vsync = u_vip.light_frame_vsync;
    assign light_href  = u_vip.light_frame_href;
    assign light_de    = u_vip.light_frame_clken;
    assign light_y     = u_vip.img_y_light;
    assign light_cb    = u_vip.img_cb_light;
    assign light_cr    = u_vip.img_cr_light;

    ycbcr2rgb u_ycbcr2rgb_probe_light (
        .clk             (clk),
        .rst_n           (rst_n),
        .pre_frame_vsync (light_vsync),
        .pre_frame_href  (light_href),
        .pre_frame_de    (light_de),
        .img_y           (light_y),
        .img_cb          (light_cb),
        .img_cr          (light_cr),
        .post_frame_vsync(light_rgb_vsync),
        .post_frame_href (light_rgb_href),
        .post_frame_de   (light_rgb_de),
        .img_red         (light_rgb_r),
        .img_green       (light_rgb_g),
        .img_blue        (light_rgb_b)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    integer file_in;
    integer file_white_balance;
    integer file_dark;
    integer file_trans;
    integer file_dehaze;
    integer file_light;
    integer file_final;
    reg     file_open_success;

    reg [7:0] bmp_header [0:53];
    integer data_offset, bpp, bytes_per_pixel, bmp_width;
    integer bmp_height_signed, bmp_height, row_padding_bytes;

    integer frame, row, col, i;
    reg [7:0] read_b, read_g, read_r, read_a, dummy_pad;
    integer seek_status;

    integer white_balance_frame_cnt, dark_frame_cnt, trans_frame_cnt, dehaze_frame_cnt, light_frame_cnt, final_frame_cnt;
    integer white_balance_row_cnt, dark_row_cnt, trans_row_cnt, dehaze_row_cnt, light_row_cnt, final_row_cnt;
    integer pad_idx_white_balance, pad_idx_dark, pad_idx_trans, pad_idx_dehaze, pad_idx_light, pad_idx_final;

    reg white_balance_vsync_d1, white_balance_href_d1;
    reg dark_vsync_d1, dark_href_d1;
    reg trans_vsync_d1, trans_href_d1;
    reg dehaze_vsync_d1, dehaze_href_d1;
    reg light_rgb_vsync_d1, light_rgb_href_d1;
    reg post_vsync_d1, post_href_d1;

    initial begin
        rst_n = 1'b0;
        vip_crtl = 2'b11;
        pre_vsync = 1'b0;
        pre_href  = 1'b0;
        pre_de    = 1'b0;
        pre_rgb   = 16'd0;
        file_open_success = 1'b0;

        white_balance_frame_cnt = 0;
        dark_frame_cnt = 0;
        trans_frame_cnt = 0;
        dehaze_frame_cnt = 0;
        light_frame_cnt = 0;
        final_frame_cnt = 0;

        white_balance_row_cnt = 0;
        dark_row_cnt = 0;
        trans_row_cnt = 0;
        dehaze_row_cnt = 0;
        light_row_cnt = 0;
        final_row_cnt = 0;

        white_balance_vsync_d1 = 1'b0;
        white_balance_href_d1 = 1'b0;
        dark_vsync_d1 = 1'b0;
        dark_href_d1 = 1'b0;
        trans_vsync_d1 = 1'b0;
        trans_href_d1 = 1'b0;
        dehaze_vsync_d1 = 1'b0;
        dehaze_href_d1 = 1'b0;
        light_rgb_vsync_d1 = 1'b0;
        light_rgb_href_d1 = 1'b0;
        post_vsync_d1 = 1'b0;
        post_href_d1 = 1'b0;

        file_in     = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_in/hazed.bmp", "rb");
        file_white_balance = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/0_white_balance.bmp", "wb");
        file_dark   = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/1_dark_channel.bmp", "wb");
        file_trans  = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/2_transmission.bmp", "wb");
        file_dehaze = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/3_dehazed.bmp", "wb");
        file_light  = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/4_light_enhanced.bmp", "wb");
        file_final  = $fopen("C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/5_final_output.bmp", "wb");

        if (file_in == 0 || file_white_balance == 0 || file_dark == 0 || file_trans == 0 || file_dehaze == 0 || file_light == 0 || file_final == 0) begin
            $display("[FATAL ERROR] File open failed.");
            $stop;
        end else begin
            file_open_success = 1'b1;

            for (i = 0; i < 54; i = i + 1) begin
                bmp_header[i] = $fgetc(file_in);
                $fwrite(file_white_balance, "%c", bmp_header[i]);
                $fwrite(file_dark,   "%c", bmp_header[i]);
                $fwrite(file_trans,  "%c", bmp_header[i]);
                $fwrite(file_dehaze, "%c", bmp_header[i]);
                $fwrite(file_light,  "%c", bmp_header[i]);
                $fwrite(file_final,  "%c", bmp_header[i]);
            end

            bmp_width         = {bmp_header[21], bmp_header[20], bmp_header[19], bmp_header[18]};
            bmp_height_signed = {bmp_header[25], bmp_header[24], bmp_header[23], bmp_header[22]};
            data_offset       = {bmp_header[13], bmp_header[12], bmp_header[11], bmp_header[10]};
            bpp               = {bmp_header[29], bmp_header[28]};
            bmp_height        = (bmp_height_signed < 0) ? -bmp_height_signed : bmp_height_signed;

            if (bmp_width != `IMG_HDISP || bmp_height != `IMG_VDISP) begin
                $display("[FATAL ERROR] BMP size mismatch: %0d x %0d, expected %0d x %0d.", bmp_width, bmp_height, `IMG_HDISP, `IMG_VDISP);
                $stop;
            end

            if (bpp != 24 && bpp != 32) begin
                $display("[FATAL ERROR] Format %0d-bit not supported.", bpp);
                $stop;
            end

            bytes_per_pixel = bpp / 8;
            row_padding_bytes = (4 - ((bmp_width * bytes_per_pixel) % 4)) % 4;

            for (i = 54; i < data_offset; i = i + 1) begin
                dummy_pad = $fgetc(file_in);
                $fwrite(file_white_balance, "%c", dummy_pad);
                $fwrite(file_dark,   "%c", dummy_pad);
                $fwrite(file_trans,  "%c", dummy_pad);
                $fwrite(file_dehaze, "%c", dummy_pad);
                $fwrite(file_light,  "%c", dummy_pad);
                $fwrite(file_final,  "%c", dummy_pad);
            end

            $display("=================================================");
            $display("===  SIMULATION STARTED (vip pipeline, 3 frames) ===");
            $display("=================================================");
            #100 rst_n = 1'b1;
            #100;

            for (frame = 1; frame <= 3; frame = frame + 1) begin
                $display("[INFO] Time %0t : Sending frame %0d ...", $time, frame);

                if (frame > 1)
                    seek_status = $fseek(file_in, data_offset, 0);

                @(posedge clk);
                pre_vsync <= 1'b1;
                #200;

                for (row = 0; row < bmp_height; row = row + 1) begin
                    @(posedge clk);
                    pre_href <= 1'b1;

                    for (col = 0; col < bmp_width; col = col + 1) begin
                        pre_de = 1'b1;
                        read_b = $fgetc(file_in);
                        read_g = $fgetc(file_in);
                        read_r = $fgetc(file_in);
                        if (bpp == 32)
                            read_a = $fgetc(file_in);

                        pre_rgb <= {read_r[7:3], read_g[7:2], read_b[7:3]};
                        @(posedge clk);
                    end

                    pre_de   <= 1'b0;
                    pre_href <= 1'b0;
                    for (i = 0; i < row_padding_bytes; i = i + 1)
                        dummy_pad = $fgetc(file_in);
                    #2000;
                end

                pre_vsync <= 1'b0;
                #50000;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            white_balance_frame_cnt <= 0;
            white_balance_row_cnt <= 0;
            white_balance_vsync_d1 <= 1'b0;
            white_balance_href_d1 <= 1'b0;
        end else if (file_open_success) begin
            white_balance_vsync_d1 <= white_balance_vsync;
            white_balance_href_d1  <= white_balance_href;

            if (white_balance_vsync && !white_balance_vsync_d1)
                white_balance_frame_cnt <= white_balance_frame_cnt + 1;

            if (white_balance_frame_cnt == 3 && white_balance_href && white_balance_de) begin
                if (bpp == 24)
                    $fwrite(file_white_balance, "%c%c%c", white_balance_rgb[7:0], white_balance_rgb[15:8], white_balance_rgb[23:16]);
                else
                    $fwrite(file_white_balance, "%c%c%c%c", white_balance_rgb[7:0], white_balance_rgb[15:8], white_balance_rgb[23:16], 8'h00);
            end

            if (white_balance_frame_cnt == 3 && !white_balance_href && white_balance_href_d1) begin
                for (pad_idx_white_balance = 0; pad_idx_white_balance < row_padding_bytes; pad_idx_white_balance = pad_idx_white_balance + 1)
                    $fwrite(file_white_balance, "%c", 8'h00);
                white_balance_row_cnt <= white_balance_row_cnt + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            dark_frame_cnt <= 0;
            dark_row_cnt <= 0;
            dark_vsync_d1 <= 1'b0;
            dark_href_d1 <= 1'b0;
        end else if (file_open_success) begin
            dark_vsync_d1 <= dark_vsync;
            dark_href_d1  <= dark_href;

            if (dark_vsync && !dark_vsync_d1)
                dark_frame_cnt <= dark_frame_cnt + 1;

            if (dark_frame_cnt == 3 && dark_href && dark_de) begin
                if (bpp == 24)
                    $fwrite(file_dark, "%c%c%c", dark_val, dark_val, dark_val);
                else
                    $fwrite(file_dark, "%c%c%c%c", dark_val, dark_val, dark_val, 8'h00);
            end

            if (dark_frame_cnt == 3 && !dark_href && dark_href_d1) begin
                for (pad_idx_dark = 0; pad_idx_dark < row_padding_bytes; pad_idx_dark = pad_idx_dark + 1)
                    $fwrite(file_dark, "%c", 8'h00);
                dark_row_cnt <= dark_row_cnt + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            trans_frame_cnt <= 0;
            trans_row_cnt <= 0;
            trans_vsync_d1 <= 1'b0;
            trans_href_d1 <= 1'b0;
        end else if (file_open_success) begin
            trans_vsync_d1 <= trans_vsync;
            trans_href_d1  <= trans_href;

            if (trans_vsync && !trans_vsync_d1)
                trans_frame_cnt <= trans_frame_cnt + 1;

            if (trans_frame_cnt == 3 && trans_href && trans_de) begin
                if (bpp == 24)
                    $fwrite(file_trans, "%c%c%c", trans_val, trans_val, trans_val);
                else
                    $fwrite(file_trans, "%c%c%c%c", trans_val, trans_val, trans_val, 8'h00);
            end

            if (trans_frame_cnt == 3 && !trans_href && trans_href_d1) begin
                for (pad_idx_trans = 0; pad_idx_trans < row_padding_bytes; pad_idx_trans = pad_idx_trans + 1)
                    $fwrite(file_trans, "%c", 8'h00);
                trans_row_cnt <= trans_row_cnt + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            dehaze_frame_cnt <= 0;
            dehaze_row_cnt <= 0;
            dehaze_vsync_d1 <= 1'b0;
            dehaze_href_d1 <= 1'b0;
        end else if (file_open_success) begin
            dehaze_vsync_d1 <= dehaze_vsync;
            dehaze_href_d1  <= dehaze_href;

            if (dehaze_vsync && !dehaze_vsync_d1)
                dehaze_frame_cnt <= dehaze_frame_cnt + 1;

            if (dehaze_frame_cnt == 3 && dehaze_href && dehaze_de) begin
                if (bpp == 24)
                    $fwrite(file_dehaze, "%c%c%c", dehaze_rgb[7:0], dehaze_rgb[15:8], dehaze_rgb[23:16]);
                else
                    $fwrite(file_dehaze, "%c%c%c%c", dehaze_rgb[7:0], dehaze_rgb[15:8], dehaze_rgb[23:16], 8'h00);
            end

            if (dehaze_frame_cnt == 3 && !dehaze_href && dehaze_href_d1) begin
                for (pad_idx_dehaze = 0; pad_idx_dehaze < row_padding_bytes; pad_idx_dehaze = pad_idx_dehaze + 1)
                    $fwrite(file_dehaze, "%c", 8'h00);
                dehaze_row_cnt <= dehaze_row_cnt + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            light_frame_cnt <= 0;
            light_row_cnt <= 0;
            light_rgb_vsync_d1 <= 1'b0;
            light_rgb_href_d1 <= 1'b0;
        end else if (file_open_success) begin
            light_rgb_vsync_d1 <= light_rgb_vsync;
            light_rgb_href_d1  <= light_rgb_href;

            if (light_rgb_vsync && !light_rgb_vsync_d1)
                light_frame_cnt <= light_frame_cnt + 1;

            if (light_frame_cnt == 3 && light_rgb_href && light_rgb_de) begin
                if (bpp == 24)
                    $fwrite(file_light, "%c%c%c", light_rgb_b, light_rgb_g, light_rgb_r);
                else
                    $fwrite(file_light, "%c%c%c%c", light_rgb_b, light_rgb_g, light_rgb_r, 8'h00);
            end

            if (light_frame_cnt == 3 && !light_rgb_href && light_rgb_href_d1) begin
                for (pad_idx_light = 0; pad_idx_light < row_padding_bytes; pad_idx_light = pad_idx_light + 1)
                    $fwrite(file_light, "%c", 8'h00);
                light_row_cnt <= light_row_cnt + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            final_frame_cnt <= 0;
            final_row_cnt <= 0;
            post_vsync_d1 <= 1'b0;
            post_href_d1  <= 1'b0;
        end else if (file_open_success) begin
            post_vsync_d1 <= post_vsync;
            post_href_d1  <= post_href;

            if (post_vsync && !post_vsync_d1)
                final_frame_cnt <= final_frame_cnt + 1;

            if (final_frame_cnt == 3 && post_href && post_de) begin
                if (bpp == 24)
                    $fwrite(file_final, "%c%c%c", post_rgb[7:0], post_rgb[15:8], post_rgb[23:16]);
                else
                    $fwrite(file_final, "%c%c%c%c", post_rgb[7:0], post_rgb[15:8], post_rgb[23:16], 8'h00);
            end

            if (final_frame_cnt == 3 && !post_href && post_href_d1) begin
                for (pad_idx_final = 0; pad_idx_final < row_padding_bytes; pad_idx_final = pad_idx_final + 1)
                    $fwrite(file_final, "%c", 8'h00);

                final_row_cnt <= final_row_cnt + 1;
                if (final_row_cnt == bmp_height - 1) begin
                    $display("\n=================================================");
                    $display("[SUCCESS] vip pipeline images captured.");
                    $fflush(file_white_balance);
                    $fflush(file_dark);
                    $fflush(file_trans);
                    $fflush(file_dehaze);
                    $fflush(file_light);
                    $fflush(file_final);
                    $fclose(file_in);
                    $fclose(file_white_balance);
                    $fclose(file_dark);
                    $fclose(file_trans);
                    $fclose(file_dehaze);
                    $fclose(file_light);
                    $fclose(file_final);
                    $display("Outputs:");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/0_white_balance.bmp");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/1_dark_channel.bmp");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/2_transmission.bmp");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/3_dehazed.bmp");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/4_light_enhanced.bmp");
                    $display("  C:/Users/32875/Desktop/FPGA/1_Verilog/25G/25G/57_ov5640_lcd_yuv/sim/img_out/5_final_output.bmp");
                    $display("=================================================\n");
                    $finish;
                end
            end
        end
    end

endmodule
