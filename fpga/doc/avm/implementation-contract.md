# AVM implementation contract

Target: PG2L50H-6IFBG484. First delivery is portable RTL and simulation, not a board bitstream.

## Arithmetic (normative)

Camera IDs front=0, back=1, left=2, right=3. Coordinates are zero-based pixel centers. Signed coordinates: 21-bit two's complement, 8 fractional bits. round_shift_signed(n,s) = sign(n)*floor((abs(n)+2^(s-1))/2^s), s=0 is identity. Grid interpolation: horizontal weighted sums rounded by log2(step), then vertical weighted sum rounded by log2(step). Steps 16/8/4/2/1. Nodes packed uint64 little endian: u bits20:0, v bits41:21, other bits zero. No intermediate image resampling.

RGB565 expands by bit replication: R=(r5<<3)|(r5>>2), G=(g6<<2)|(g6>>4), B=(b5<<3)|(b5>>2). Bilinear: h0=p00*(256-fx)+p10*fx; h1=p01*(256-fx)+p11*fx; pixel=(h0*(256-fy)+h1*fy+32768)>>16, no horizontal rounding. Gain: unsigned Q4.12, (pixel*gain+2048)>>12 saturated to 255. Blend: (a*alpha+b*(256-alpha)+128)>>8. RGB888 to RGB565 truncates low bits. If only A valid alpha=256; if only B valid alpha=0; neither valid yields black. alpha LUT has 257 unsigned 9-bit entries, initialized to identity. Signed nodes outside representable range saturate; invalid geometry masked before sampling.

## DDR file ABI v1

All addresses are bytes, aligned to 16; all binary words little endian. Source images are raster RGB565, each camera base 16-byte aligned, row stride a multiple of16 (image width padded, dimensions separate). 16x16 macros raster order, edge macros padded/masked.

Each macro has 4 descriptors of16 bytes, camera order. Descriptor bits31:0 node patch byte address; bits34:32 log2(step); bit35 active; remaining zero. Each active patch is (16/step+1)^2 nodes in row-major order, uint64/node, padding to16 bytes. Each macro route block is256 uint16 in local raster order. Route bits1:0 camA, 3:2 camB, bit4 validA, bit5 validB, bits14:6 alpha_index(0..256), bit15 reserved zero. Two valid cameras must differ. At most two cameras per pixel. Route block address=route_base+macro_index*512; descriptor block=desc_base+macro_index*64.

Fixtures: tools/avm emits sim/data/<case>/memory.hex (128-bit little-endian words, hex text), expected.hex (RGB565 raster), frame.json (dimensions, camera bases, stride, desc_base, route_base, memory_words, gains[4][3], fixture provenance), golden images and metrics. A fixture may include camera calibration JSON and matlab_input.mat. Full default synthetic dimensions1280x720; public source960x640/output1280x720 with proportional padding. Small regression width64 height48, source128x96, same geometry scaled. Gains flatten camera-major RGB; packed vector gain[16*(cam*3+channel)+:16].

## Shared RTL interfaces

Verilog-2001 synthesizable modules; SystemVerilog testbenches. All portable core signals clk100MHz, active-low synchronous reset rst_n. Elastic streams valid/ready, stable while valid&&!ready. memory request: req_valid, req_ready, req_addr[31:0], req_len[4:0] number of128-bit beats (1..16); response rsp_valid,rsp_ready,rsp_data[127:0],rsp_last. Requests aligned16, cannot cross4KiB. Each client one outstanding. Shared arbiter serializes clients. BFM servesDDR131.25MHz equivalent scheduling, first data after8DDR clocks+2core clocks; no direct cross-clock handshake. Synthetic capture load consumes actual service slots equivalent221.184MB/s at720p30.

avm_bilinear #(TAG_W=128): input s_valid/output s_ready; s_pixels[63:0] four565 packed p00 at15:0,p10 at31:16,p01 at47:32,p11 at63:48; s_fx/s_fy[7:0],s_tag[TAG_W-1:0]; output m_valid/input m_ready,m_rgb[23:0] R high,m_tag. Exactly3 elastic arithmetic stages, II1.

avm_grid_interp #(TAG_W=128): input s_valid/output s_ready; signed s_u00/s_u10/s_u01/s_u11 and s_v00/s_v10/s_v01/s_v11[20:0],s_xfrac/s_yfrac[3:0],s_step_log2[2:0],s_tag; output m_valid/input m_ready,signed m_u/m_v[20:0],m_tag. Four elastic stages after PDS optimization: horizontal total27, horizontal round21, vertical total27, vertical round21. II=1, latency4 without stalls; step1 frac0. Each axis still rounds separately. Exact arithmetic uses a*step+(b-a)*fraction, with 22-bit signed difference and 27-bit total, then arithmetic right shift with bias half-sign(total).

avm_gain_blend #(TAG_W=128): input s_valid/output s_ready,s_rgb_a/s_rgb_b[23:0],s_gain_a/s_gain_b[47:0] channel R in low16,G next,B high16,s_alpha[8:0],s_valid_a/s_valid_b,s_tag; output m_valid/input m_ready,m_pixel[15:0],m_tag. Two stages gain then blend.

avm_tile_cache #(TAG_W=128): input s_valid/output s_ready,s_enable,s_cam[1:0],s_x/s_y[11:0],s_tag; camera_bases[127:0],src_width/src_height[11:0],src_stride[15:0],invalidate (only idle), output m_valid/input m_ready,m_pixels[63:0],m_tag,m_sample_valid; memory req/rsp interface above; hit_count/miss_count/read_beats[31:0]. Each lane16 slots32x32 pixels, merged four parity banks4096x16. s_enable=0/out-of-bounds returns zero with sample_valid0. Miss starts from presented valid request even if s_ready0 (source keeps it stable); accepted hits II1. Keep output stable under stall; fill and replacement must not corrupt outstanding samples. Cross-tile 2x2 can require4resident tiles; protect needed tags during fills. Source storage raster, tile row64bytes, clip last image tile (valid four-neighbor requests only). Data outside source need not be read. All misses serviced through actual bursts, no behavioral framebuffer shortcut in RTL.

Core output raster RGB565 valid/ready with sof/eol, done only after final handshake. Macro-major compute reordered by two16-line buffers with explicit ownership. Config atomic at start/frame boundary, LUT changes only idle. Statistics use pre-gain bilinear RGB in valid overlap, camera-pair sums/counts; host model5Hz bounded log solve +0.1log EMA, rank deficiency hold old channel gains, quantize Q4.12.

PDS implementation: cache port A uses one shared read/write address expression so ADS infers true dual-port DRM. Replacement rotates a protected-slot mask and selects its lowest set bits in round-robin order. Miss setup adds two clocks to register selected tile, source address, then page-limited burst length; accepted hits retain II=1. Each 1280x16 stripe is split into five 4096-word chunks per buffer, preserving one-cycle reads and logical 80KiB capacity. The 16x128 response FIFO explicitly uses `syn_ramstyle="lut_ram"` to preserve its asynchronous fall-through read, including empty-to-enqueue read/write coincidence; no collision checking is disabled. See pds-validation.md for actual resources and synthesis timing; physical I/O/CDC/post-route constraints are separate.

## Acceptance

Full frame RTL exactly equals Python. Grid error <=.25 source pixels after normative rounding. Independent MATLAB and ray-plane reference. Random backpressure/reset/cache crossing/4KiB/config tests. 30fps target is measured from frame start with source data ready to final output under specified BFM; never label as Fmax or board latency. Stress worst-case latency/full refinement may reduce throughput but cannot corrupt pixels or deadlock. No PDS place-route or board claims.
