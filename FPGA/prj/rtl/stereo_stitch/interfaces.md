# 双目 RTL 接口契约

时钟：按键、UART 和光栅为 clk_50m；其余运算、缓存、读仲裁为 core_clk。低有效复位，两个域同步释放，共同异步复位源。RAM 数据不复位，valid 复位。

模式 0=融合，1..4=摄像头1..4。D 默认256，限制0..1278。全部普通通道 valid/ready，负载在阻塞时稳定；DDR req/addr/len 保持至 done，done/error 仅属于所选会话。

配置通过 stereo_mailbox 保持 bundled-data 至应答。geometry 仅一个32位、8边界流水 divider；完成五次参数除法及1280列权重准备后才可提交。双bank：active 属于活动/后续帧，work 只写非active；prepared 等待空闲边界提交期间不接受下一配置。坐标Q12，权重Q8（融合9位）。像素包携带 fxL/fxR/fy/wR、覆盖掩码、SOF/EOL/EOF，和邻域同步背压。

行缓存每相机4个bank，用源行号低2位索引，保存完整行标签。当前行对优先，预取下一输出行对；重复行复用。奇偶列分RAM，每bank同步读两个相邻像素。读结果进入独立寄存器后不再引用RAM；填充禁止覆盖当前行对，响应阻塞时读结果保持。新帧清有效标记，关闭先等行读结束。

frame_reader 同周期 acquire 两路，快照slot及sequence；缺任一路立即失败。每行10个16拍突发，DDR低16位最早；2048位暂存后逐像素填充。line_done 在末像素接受后产生，下一周期不接新行；关闭等在途事务和填充完毕后release，仅随后报告frame_done。

仲裁锁定显示/坐标会话所有权，至done/响应接受后释放。桥请求模式固定至EOF/abort排空，且等待core完成应答后才允许下一VS请求。缺帧/错误发送abort；已输出图像不能撤回。计数复位0、32位回绕。

运算顺序：四邻域分量乘加不提前舍入，(S+32768)>>16；两路插值后再((256-wR)*L+wR*R+128)>>8。黑边不请求源数据。覆盖区间[-2048,5240832)，夹取x至1279<<12、y至719<<12。

UART：请求/应答快照跨域；固定宽十六进制，字段见README。整组不超过10Hz，周期1Hz，包最长32字节，保持长度至Tx_Done。

以下是实际实现契约；未进行功能验证。

## 复位值、时钟和参数

| 模块 | 域及复位 | 复位状态/参数 |
| --- | --- | --- |
| stereo_mode_select | clk_50m / rst_n | mode_pending=0，输入同步寄存器=11，消抖稳定态=1；CLK_HZ=50000000 |
| stereo_key_control | clk_50m / rst_n | D_DEFAULT=256（夹取0..1278），direction_inc=1，按键初值同上；CLK_HZ=50000000 |
| stereo_mailbox | src_clk、dst_clk，各自rst_n | toggle/数据/目标valid=0；第一次主动发送；WIDTH=11，顶层覆盖为15 |
| stereo_geometry | core_clk / rst_n | active_valid=0，active_bank=0，活动数值=0，内部H=720、W=1280；RAM不复位，weight_data仅weight_en后有意义 |
| stereo_coord_gen | core_clk / rst_n | valid/坐标/余数/tag=0；start装入活动几何，cancel清valid |
| stereo_frame_reader | core_clk / rst_n | IDLE、无锁、done/error/序号/填充索引/地址=0；突发数据不复位；mem_len固定15 |
| stereo_line_cache | core_clk / rst_n | 全行invalid、标签=0、无填充、rsp_valid/cache_error=0；RAM及读输出寄存器不复位 |
| stereo_resample_blend | core_clk / rst_n | 六级valid及最终像素/标记复位为0；中间数据与元数据不复位，仅在valid时有意义；clear清全部valid |
| stereo_read_mux | core_clk / rst_n | owner=0；route为组合结果，不接受复位期间的请求 |
| stereo_stitch | core_clk / rst_n | IDLE、所有done/error/计数/地址valid=0；第一次cfg_valid主动准备 |
| stereo_uart_status | clk_50m、core_clk / 各域rst_n | 快照/计数/toggle/valid=0，period_due=1、tx_size=1；CLK_HZ=50000000，UART_BAUD=115200 |

组合ready及负载不是复位寄存器；valid=0时无传输含义。共同系统复位同时覆盖FIFO两端和所有mailbox，两级复位管在各自时钟域释放。divider的rst接!rst_n（高有效），其公共源码保持原样。

## 配置生命周期

mailbox的15位快照为 `{mode_pending[2:0],direction_inc,d_pending[10:0]}`。mode单独随桥的帧请求保持传递，mailbox低11位驱动几何准备；方向只影响按键调节。mode变更与D准备时间独立，不要求新mode等新D准备。

geometry输入cfg_valid/ready只接受一份配置。prepared_valid之后等待prepared_ready；该模块把prepared参数保存为内部work寄存器，在提交后通过active_*和商余数输出暴露完整配置，不另行输出可被提前使用的work参数。prepared_ready只在**新的融合请求到达且旧会话空闲**时给出，seq_ready在此拍为0，下一拍才能锁图，避免半套配置。单摄期间可准备一份配置并保留最新mailbox值，返回融合后继续追上最新D。active_bank改变之前，任何读权重请求必须已经排空。

qx0/qxs/qy0/qys为24位无符号商；rx0/rxs/ry0/rys为11位余数。32位除法器输入先零扩展再移位；den最小4096、最大5238784，分子最大1343748096。像素坐标由25位有符号U/V及UR进行覆盖/夹取，再提取11位x、10位y及8位小数。横余数和/纵余数和为12位，单次比较减法。

## 像素、缓存与延迟

| meta[57:0] | 内容 |
| --- | --- |
| [57:38] | 20位tag=输出线性序号0..921599 |
| [37:35] | EOF、EOL、SOF |
| [34:33] | 左路/右路使能，黑边均0 |
| [32:25]、[24:17]、[16:9] | fxL、fxR、fy，8位0..255 |
| [8:0] | wR，9位0..256 |

coord_gen的49位meta是上表去掉wR，权重同步RAM读出与坐标寄存器同拍装载后拼成58位。任一弹性级阻塞即保持其数据与标记。缓存每路邻域64位：低到高P00/P10/P01/P11。缓存同步RAM读一级，运算改为六级：输入邻域寄存、水平加权和、垂直乘积、插值舍入、融合乘积、最终舍入；加权重/坐标寄存一级，共八个寄存级。无缺行和背压时，坐标在边沿N被接受，融合结果在N+7边沿后可见，最早在N+8接受；流水线可连续每拍接受一个坐标。缺行/背压增加等待周期，不能以固定周期猜元数据位置。

bank索引=source_y[1:0]，相机ID=0(R)/1(L)。invalid=valid_rows清零；filling由request_pending/filling及line_camera/line_bank表示；ready=valid_rows且tag匹配；in_use由当前待接受请求的y0/y1行对派生，不覆盖其bank。两幅图的垂直步长不超过2，因此当前/下一行对总共最多四个源行。当前请求所需行优先，之后预取下一行对；disabled路不阻塞当前像素，即使后台提前填它的行。黑边不发新行读。

每bank有独立even/odd 640×16 RAM及同步输出寄存器。奇数列的右邻居从下一偶列读出；1279列复制自己。响应生成时数据已经进入输出寄存器，阻塞期间禁止更新这些寄存器，因此RAM bank后续可释放。line_done成功后才置整行有效；新帧仅清标志。

插值乘数和加法均扩为23位，非负凸组合最大63×65536，舍入仍在23位内；分量饱和再打包。融合乘加16位足够，最大63×256+128。活动路径无变量除法/取模。

## 错误与会话归属

begin_valid/ready同拍检查latest_valid[1:0]，同时发acquire=0011并保存两slot；begin_done/error下一拍反馈。源序号仅成功锁定时更新。line_req负载一直保存到line_done，DDR错误仅在mem_done采样。line_error可早于1280像素；缓存清有效并置cache_error。

stitch在错误时停止新坐标、清弹性像素valid，仍接收已发起的行填充。待cache_idle（无排队请求及填充）后发close_valid，reader释放锁、close_done，随后stitch单次seq_done/error。成功需末像素握手并走同一排空/释放路径。无配置或无图直接失败，未锁任何槽。该错误清空是普通valid保持规则的显式异常终止；桥随后插入abort，不会把残余像素用于下一帧。

read_mux的owner=0空闲、1单摄、2融合、3坐标。空闲显示优先；坐标响应被接受才释放所有权。未获权模块的acquire/release/mem_req都被屏蔽。桥在EOF/abort消费完成**且core完成toggle同步返回**后才重新请求。

新串口快照core_live为241位，全部在core请求到达时锁存，ack跨回video后采样；下一次请求作为上一份已接收的确认，不提前覆盖held_core。视频域本地计数同时采样，但与core快照不构成同一时刻的全系统原子快照。包内长度、数据和整组快照直到发送完保持不变。UART字段和异常归属见README。
## RAM接口适配更新

stereo_ram_read_hold使用core_clk/rst_n。DATA_WIDTH默认16、DEPTH默认640；写地址/读地址为$clog2(DEPTH)位。wr_en时写入；rd_en时接受读地址，同拍沿后出现同步RAM结果。内部read_previous与held_data复位为0，直接选择最新有效RAM结果或保持结果，无额外正常读延迟；阻塞后即使原RAM地址被覆盖，输出也保持上次接受的数据。

geometry以DATA_WIDTH=9、DEPTH=4096复用此模块，wr_addr={!active_bank,column}、rd_addr={active_bank,weight_x}，边缘写入与除法结果合并为一个写口。cache每个bank的even/odd均以DATA_WIDTH=16、DEPTH=640实例化，替代直接声明的数组。正常读延迟及meta对齐保持原契约。权重输出现在是wire，复位通过RAM adapter输出0。公共dualport_ram源码未修改。

<!-- GENERATED PORTS -->

## 实际端口表

### stereo_coord_gen

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `start` |
| input | `1 bit` | `cancel` |
| input | `[10:0]` | `d` |
| input | `[9:0]` | `h, top` |
| input | `[23:0]` | `qx0, qxs, qy0, qys` |
| input | `[10:0]` | `rx0, rxs, ry0, rys` |
| output | `1 bit` | `valid` |
| input | `1 bit` | `ready` |
| output | `[10:0]` | `out_x` |
| output | `[10:0]` | `left_x, right_x` |
| output | `[9:0]` | `source_y, next_y` |
| output | `[48:0]` | `meta` |

### stereo_frame_reader

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `begin_valid` |
| output | `1 bit` | `begin_ready` |
| output | `1 bit` | `begin_done` |
| output | `1 bit` | `begin_error` |
| input | `1 bit` | `close_valid` |
| output | `1 bit` | `close_done` |
| input | `[3:0]` | `latest_valid, latest_slot` |
| input | `[127:0]` | `frame_sequences` |
| output | `[3:0]` | `acquire, release_slot` |
| output | `[31:0]` | `sequence_l, sequence_r` |
| input | `1 bit` | `line_req_valid` |
| output | `1 bit` | `line_req_ready` |
| input | `[1:0]` | `line_camera, line_bank` |
| input | `[9:0]` | `line_y` |
| output | `1 bit` | `fill_valid` |
| input | `1 bit` | `fill_ready` |
| output | `[1:0]` | `fill_camera, fill_bank` |
| output | `[10:0]` | `fill_x` |
| output | `[15:0]` | `fill_pixel` |
| output | `1 bit` | `line_done, line_error` |
| output | `1 bit` | `mem_req` |
| output | `[27:0]` | `mem_addr` |
| output | `[3:0]` | `mem_len` |
| input | `1 bit` | `mem_done, mem_error` |
| input | `[2047:0]` | `mem_data` |

### stereo_geometry

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `cfg_valid` |
| output | `1 bit` | `cfg_ready` |
| input | `[10:0]` | `cfg_d` |
| output | `1 bit` | `prepared_valid` |
| input | `1 bit` | `prepared_ready` |
| output | `1 bit` | `active_valid` |
| output | `1 bit` | `active_bank` |
| output | `[10:0]` | `active_d` |
| output | `[9:0]` | `active_h, active_top` |
| output | `[23:0]` | `qx0, qxs, qy0, qys` |
| output | `[10:0]` | `rx0, rxs, ry0, rys` |
| input | `1 bit` | `weight_en` |
| input | `[10:0]` | `weight_x` |
| output | `[8:0]` | `weight_data` |

### stereo_key_control

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `key_n` |
| output | `[10:0]` | `d_pending` |
| output | `1 bit` | `direction_inc` |

### stereo_line_cache

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `clear` |
| input | `1 bit` | `stop` |
| input | `1 bit` | `req_valid` |
| output | `1 bit` | `req_ready` |
| input | `[10:0]` | `left_x, right_x` |
| input | `[9:0]` | `source_y, next_y` |
| input | `[57:0]` | `req_meta` |
| output | `1 bit` | `rsp_valid` |
| input | `1 bit` | `rsp_ready` |
| output | `[63:0]` | `left_pixels, right_pixels` |
| output | `[57:0]` | `rsp_meta` |
| output | `1 bit` | `line_req_valid` |
| input | `1 bit` | `line_req_ready` |
| output | `[1:0]` | `line_camera, line_bank` |
| output | `[9:0]` | `line_y` |
| input | `1 bit` | `fill_valid` |
| output | `1 bit` | `fill_ready` |
| input | `[1:0]` | `fill_camera, fill_bank` |
| input | `[10:0]` | `fill_x` |
| input | `[15:0]` | `fill_pixel` |
| input | `1 bit` | `line_done, line_error` |
| output | `1 bit` | `cache_error` |
| output | `1 bit` | `idle` |

### stereo_mailbox

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `src_clk` |
| input | `1 bit` | `src_rst_n` |
| input | `[WIDTH-1:0]` | `src_data` |
| input | `1 bit` | `dst_clk` |
| input | `1 bit` | `dst_rst_n` |
| output | `[WIDTH-1:0]` | `dst_data` |
| output | `1 bit` | `dst_valid` |

### stereo_mode_select

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `key_n` |
| output | `[2:0]` | `mode_pending` |

### stereo_ram_read_hold

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk, rst_n` |
| input | `1 bit` | `wr_en` |
| input | `[$clog2(DEPTH)-1:0]` | `wr_addr` |
| input | `[DATA_WIDTH-1:0]` | `wr_data` |
| input | `1 bit` | `rd_en` |
| input | `[$clog2(DEPTH)-1:0]` | `rd_addr` |
| output | `[DATA_WIDTH-1:0]` | `rd_data` |

### stereo_read_mux

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `seq_valid, seq_ready` |
| input | `[2:0]` | `display_mode` |
| input | `1 bit` | `random_mode, req_valid, req_ready` |
| input | `1 bit` | `single_done, stitch_done, rsp_valid, rsp_ready` |
| output | `[1:0]` | `route` |
| output | `1 bit` | `idle` |

### stereo_resample_blend

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `clear` |
| input | `1 bit` | `in_valid` |
| output | `1 bit` | `in_ready` |
| input | `[63:0]` | `left_pixels, right_pixels` |
| input | `[57:0]` | `in_meta` |
| output | `1 bit` | `pixel_valid` |
| input | `1 bit` | `pixel_ready` |
| output | `[15:0]` | `pixel` |
| output | `1 bit` | `pixel_sof, pixel_eol, pixel_eof` |

### stereo_stitch

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `1 bit` | `cfg_valid` |
| input | `[10:0]` | `cfg_d` |
| input | `1 bit` | `seq_valid` |
| output | `1 bit` | `seq_ready` |
| output | `1 bit` | `seq_done, seq_error` |
| output | `1 bit` | `pixel_valid` |
| input | `1 bit` | `pixel_ready` |
| output | `[15:0]` | `pixel` |
| output | `1 bit` | `pixel_sof, pixel_eol, pixel_eof` |
| input | `[3:0]` | `latest_valid, latest_slot` |
| input | `[127:0]` | `frame_sequences` |
| output | `[3:0]` | `acquire, release_slot` |
| output | `1 bit` | `mem_req` |
| output | `[27:0]` | `mem_addr` |
| output | `[3:0]` | `mem_len` |
| input | `1 bit` | `mem_done, mem_error` |
| input | `[2047:0]` | `mem_data` |
| output | `[10:0]` | `active_d` |
| output | `[9:0]` | `active_h, active_top` |
| output | `[31:0]` | `sequence_l, sequence_r` |
| output | `[31:0]` | `read_error_count, coverage_error_count` |
| output | `1 bit` | `config_pending, geometry_valid` |

### stereo_uart_status

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk, rst_n` |
| input | `1 bit` | `core_clk, core_rst_n` |
| input | `[2:0]` | `core_mode` |
| input | `[10:0]` | `d_pending, active_d` |
| input | `1 bit` | `direction_inc` |
| input | `[2:0]` | `mode_pending` |
| input | `[9:0]` | `active_h, active_top` |
| input | `[31:0]` | `sequence_l, sequence_r, read_beats, write_beats` |
| input | `[31:0]` | `read_errors, coverage_errors` |
| input | `[3:0]` | `frame_valid, overflow, write_drop` |
| input | `1 bit` | `protocol_error, config_pending, geometry_valid` |
| input | `[31:0]` | `underflows, aborts` |
| output | `1 bit` | `uart_tx` |

### camera_ddr3_store

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `core_clk` |
| input | `1 bit` | `reset_n` |
| input | `[3:0]` | `cam_clk` |
| input | `[3:0]` | `cam_vsync` |
| input | `[3:0]` | `cam_de` |
| input | `[63:0]` | `cam_pixel` |
| input | `[2:0]` | `display_mode` |
| input | `1 bit` | `cfg_valid` |
| input | `[10:0]` | `cfg_d` |
| output | `[10:0]` | `stitch_active_d` |
| output | `[9:0]` | `stitch_h, stitch_top` |
| output | `[31:0]` | `stitch_sequence_l, stitch_sequence_r` |
| output | `[31:0]` | `stitch_read_errors, stitch_coverage_errors` |
| output | `1 bit` | `stitch_config_pending, stitch_geometry_valid` |
| input | `1 bit` | `read_mode` |
| input | `1 bit` | `seq_valid` |
| output | `1 bit` | `seq_ready` |
| input | `[1:0]` | `source_sel` |
| output | `1 bit` | `seq_done` |
| output | `1 bit` | `seq_error` |
| output | `1 bit` | `pixel_valid` |
| input | `1 bit` | `pixel_ready` |
| output | `[15:0]` | `pixel` |
| output | `1 bit` | `pixel_sof` |
| output | `1 bit` | `pixel_eol` |
| output | `1 bit` | `pixel_eof` |
| input | `1 bit` | `req_valid` |
| output | `1 bit` | `req_ready` |
| input | `[1:0]` | `camera_id` |
| input | `[10:0]` | `x` |
| input | `[9:0]` | `y` |
| input | `[TAG_WIDTH-1:0]` | `req_tag` |
| output | `1 bit` | `rsp_valid` |
| input | `1 bit` | `rsp_ready` |
| output | `[15:0]` | `rsp_pixel` |
| output | `[TAG_WIDTH-1:0]` | `rsp_tag` |
| output | `1 bit` | `rsp_error` |
| output | `[31:0]` | `selected_sequence` |
| output | `[3:0]` | `frame_valid` |
| output | `[127:0]` | `frame_sequences` |
| output | `[3:0]` | `capture_overflow` |
| output | `[3:0]` | `write_drop` |
| output | `1 bit` | `protocol_error` |
| output | `[31:0]` | `write_beats` |
| output | `[31:0]` | `read_beats` |
| output | `[31:0]` | `busy_cycles` |
| output | `[159:0]` | `request_wait_cycles` |
| output | `[27:0]` | `axi_awaddr` |
| output | `[3:0]` | `axi_awlen` |
| output | `1 bit` | `axi_awvalid` |
| input | `1 bit` | `axi_awready` |
| output | `[127:0]` | `axi_wdata` |
| output | `[15:0]` | `axi_wstrb` |
| input | `1 bit` | `axi_wready` |
| input | `1 bit` | `axi_wlast` |
| output | `[27:0]` | `axi_araddr` |
| output | `[3:0]` | `axi_arlen` |
| output | `1 bit` | `axi_arvalid` |
| input | `1 bit` | `axi_arready` |
| input | `[127:0]` | `axi_rdata` |
| input | `1 bit` | `axi_rvalid` |
| input | `1 bit` | `axi_rlast` |

### cds_video_bridge

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `core_clk` |
| input | `1 bit` | `video_clk` |
| input | `1 bit` | `reset_n` |
| input | `[2:0]` | `mode_pending` |
| output | `[2:0]` | `frame_mode` |
| output | `[2:0]` | `active_mode` |
| input | `1 bit` | `raster_vs` |
| input | `1 bit` | `raster_de` |
| input | `[11:0]` | `raster_x` |
| input | `[11:0]` | `raster_y` |
| output | `1 bit` | `video_vs` |
| output | `1 bit` | `video_de` |
| output | `[15:0]` | `video_pixel` |
| output | `1 bit` | `underflow_sticky` |
| output | `1 bit` | `seq_valid` |
| input | `1 bit` | `seq_ready` |
| output | `[1:0]` | `source_sel` |
| input | `1 bit` | `seq_done` |
| input | `1 bit` | `seq_error` |
| input | `1 bit` | `pixel_valid` |
| output | `1 bit` | `pixel_ready` |
| input | `[15:0]` | `pixel` |
| input | `1 bit` | `pixel_sof` |
| input | `1 bit` | `pixel_eol` |
| input | `1 bit` | `pixel_eof` |
| output | `[31:0]` | `abort_count` |
| output | `[31:0]` | `fifo_empty_count` |
| output | `[31:0]` | `marker_error_count` |
| output | `[31:0]` | `drain_frame_count` |
| output | `[31:0]` | `first_fault` |

### uart_data_tx

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `[MAX_DATA_WIDTH - 1 : 0]` | `data` |
| input | `1 bit` | `send_en` |
| input | `[7:0]` | `tx_data_size` |
| output | `1 bit` | `uart_tx` |
| output | `1 bit` | `Tx_Done` |
| output | `1 bit` | `uart_state` |

### uart_byte_tx

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst_n` |
| input | `[7:0]` | `data_byte` |
| input | `1 bit` | `send_en` |
| output | `1 bit` | `uart_tx` |
| output | `1 bit` | `Tx_Done` |
| output | `1 bit` | `uart_state` |

### divider

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `clk` |
| input | `1 bit` | `rst` |
| input | `[XLEN-1:0]` | `a` |
| input | `[XLEN-1:0]` | `b` |
| input | `1 bit` | `vld` |
| output | `[XLEN-1:0]` | `quo` |
| output | `[XLEN-1:0]` | `rem` |
| output | `1 bit` | `ack` |

### dualport_ram

| 方向 | 位宽 | 端口 |
| --- | --- | --- |
| input | `1 bit` | `wr_clk` |
| input | `1 bit` | `wr_rst` |
| input | `[$clog2(DEPTH)-1:0]` | `wr_addr` |
| input | `[DATA_WIDTH-1:0]` | `wr_data` |
| input | `1 bit` | `wr_en` |
| input | `1 bit` | `rd_clk` |
| input | `1 bit` | `rd_rst` |
| input | `[$clog2(DEPTH)-1:0]` | `rd_addr` |
| output | `[DATA_WIDTH-1:0]` | `rd_data` |

