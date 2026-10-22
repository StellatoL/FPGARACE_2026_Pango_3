# FPGA 参考工程与 ARM ADAS 接入核查

核查日期：2026-10-16。范围为只读检查 RTL、约束、FPGA 文档及 ARM 接入代码；未执行烧录、I2C、PIO、DMA 或 PDS 操作。以下结论描述工作区源码，不能推定当前板上 bitstream 与其一致。

## 已确认兼容的协议

| 项目 | FPGA 源码依据 | ARM 接入依据 | 结论 |
|---|---|---|---|
| 图像格式 | `FPGA/rtl/pcie_video.v:132–138`、`:996–1003` | `src/hardware/pango_fun.cpp:284`、`src/ui/adas_controller.cpp:187–188` | 均为 1280×720、RGB565、每行 2560 字节、720 行，一帧 1843200 字节 |
| 四宫格位置 | `FPGA/rtl/video_control/quad_ddr3_framebuffer.v:32–35` | `src/core/adas_types.h:10`、`src/common/frame_packet.h:27–30` | 左上通道 0、右上 1、左下 2、右下 3；ARM 赋予前、右、后、左语义 |
| 主机 DMA 四槽 | `FPGA/rtl/pcie/pcie_tx_fun.v:282–296`、`:307–312`、`:337` | `src/hardware/pango_fun.h:199–202` | 四个完整拼接帧轮转，与 FPGA 内部每个相机四槽 DDR 缓存是两层独立缓冲 |
| 完成状态 | `FPGA/rtl/pcie/pio_crtl.v:159–173` | `src/hardware/pcie_frame_status.h:4–9` | bit0 完成，bit[2:1] 已完成槽索引；读状态清完成位。RTL 的第 153 行注释位序不准确，应以第 173 行逻辑为准 |
| 启停命令 | `FPGA/rtl/pcie/pio_crtl.v:67–73` | `src/ui/mainwindow.cpp:270`、`:281`；`src/ui/adas_controller.cpp:215–216` | BAR0 地址 0，启动 0xffffffe5，停止 0xffffff00，保持原协议 |
| DMA 地址与使能 | `FPGA/rtl/pcie/pio_crtl.v:40–48` | 原 `getDevice()` / `dma_set_config()` / ioctl 实现保持不变 | 0x050/054、0x020/024、0x040/044、0x030/034 为四槽地址；0x060 写 1 锁存配置 |
| I2C | `FPGA/rtl/iic_slave/i2cSlave_define.v:24`、`registerInterface.v:84–124` | `src/hardware/pango_fun.cpp:308–311`、`:334–373` | 7 位地址 0x66；0x01 选择源、0x02 实际源；0x03–06 保留原预处理定义 |

ADAS 在 `src/ui/adas_controller.cpp:104–105` 使用 DirectConnection，在采集线程下一次 `read()` 前复制所选主机槽，随后只向显示和工作线程传递拥有内存的帧。`src/common/frame_packet.h:55–69` 不复用仍被持有的帧池槽，池满直接丢帧。它改善 ARM 消费者的生命周期，不修改 DMA 或 FPGA 地址管理。

停止与关闭路径先在 `src/ui/adas_controller.cpp:211–236` 请求原停止命令并等待采集线程；2 秒内未退出则返回失败，保留缓冲和映射。`MainWindow::on_Key1_clicked()` 关闭分支之前已有 `stop_source()` 保护，关闭事件也检查 `shutdown()`。阻塞 `read()` 能否被原停止命令唤醒，仍须设备在线时验证。

## 影响实际 ADAS 验收的差异与限制

### 1. 当前默认三路为测试色条，唯一物理相机位于左下

`FPGA/rtl/pcie_video.v:16–17` 同时设定 `CAMERA_ENABLE=4'b1111` 和 `CAMERA_TEST_PATTERN=4'b1011`。后者实际决定通道 0、1、3 采用 `camera_test_pattern`，而非独立摄像头。测试图复用通道 2 的时钟和 VS/DE（`:247–258`、`:366–417`）；因此既不能作为四路真实环境识别证据，也不代表三个独立传感器时序。

现有物理连接器绑定 `cmos3_*`，对应通道 2 / 左下。有效 LOC 和相机时钟约束可见 `FPGA/prj/pcie_video.fdc:126–134`、`:223–257`。`FOUR_CAMERA_DDR3.md:18–22` 描述未来其他连接器接入，却遗漏了目前 `CAMERA_TEST_PATTERN` 的覆盖行为；按该文档“保持 CAMERA_ENABLE 即得到四路真实采集”不足以复现实际源选择。

对 ADAS 的影响：ARM 默认只对左上“前视”执行前向灯控、车道与估速。若直接编译此默认 RTL，左上的色条不是前路图像，左下真实相机按约定被解释为后视。不能擅自改变用户确认的前右后左映射；接线和 bitstream 应满足约定，或使用符合布局的 HDMI 素材/单路文件回放验证应用。四路真实相机验收须另行确认引脚、测试参数和实际烧录版本。本次不修改这些硬件文件。

### 2. 源码文档明确保留了“新四路路径未经构建/上板”的状态

`FPGA/FOUR_CAMERA_DDR3.md:59` 明确说明该次四路修改未运行仿真、PDS 综合、布局布线和板上验证，且原 bitstream 不是本次修改的结果。工作区存在 PDS 日志及产物，不足以证明其对应当前 RTL 和参数。用户已经验证的硬件链路应继续保留；本轮 ARM 软件不能把该段新增四路 RTL 的上板状态自动记为通过。

### 3. HDMI 是整幅视频，应用须选择匹配的单路/四宫格布局

`FPGA/rtl/pcie_video.v:465–495` 对 HDMI 做 1080p→720p 缩放及隔帧，`:964–968` 强制直接进入 PCIe；不存在把单路 HDMI 自动转换为四宫格的逻辑。四宫格 HDMI 素材本身应当已包含约定的四个视角；普通整幅道路视频应在 ARM 选单路布局。HDMI 模式也会强制绕过 FPGA 预处理，与 `FOUR_CAMERA_DDR3.md:7` 和 `:49` 一致。

### 4. 已增加实际源只读同步，保留约一秒轮询检测窗

`FPGA/rtl/video_control/video_channel_selector.v:104–108` 保留实体 key1 切源，0x02 可读出真实源。ARM 已在 `src/ui/adas_controller.cpp:287–303` 增加 `sync_live_source()`，复用受保护实现中原有 `i2c_query_source()`，读取实际源而不写入选择寄存器。`prepare_live()` 调用一次（`:245`），实时采集中的 `refresh_stats()` 约每秒调用一次（`:471`、`:596`）。`QSignalBlocker` 同步两个 radio 状态，避免读取状态又触发选择寄存器写入（`:296–297`）。

来源变化、从未知恢复、或者查询失败从已知变为未知，均通过现有 `load_profile()`、`update_settings()`、`advance_generation(false)` 失效旧标定关联及感知/运动历史，并丢弃 250ms 过渡帧（`:299–301`）。未知源采用独立档案身份并强制清除标定（`:305–316`），标定操作也会拒绝未知源（`:400`）。GUI 成功切源原回调仍保留。默认启动和文件回放不执行该硬件查询；方法调用位于 GUI 线程，采集线程仍只访问原子代次/接入状态与自有帧池，未增加 DMA 指针共享或新的硬件协议。

这是软件实现核查，尚未做实体按键/I2C 在线试验。实体切源到软件察觉存在名义约 1s 的检测窗，Qt 事件循环繁忙或 I2C 调用等待可能进一步延长，因此不是硬性 ≤1s 保证；此窗内不能承诺已清除旧来源标定。初次启动采集时 `prepare_live()` 早于 `getDevice()`，设备尚未打开会跳过该次查询（`:288`），并保持未知标定状态，待实时刷新确认。板上验收需测量实际检测时间、失败恢复以及历史清除，不能把代码核查当作硬件切源已通过。

### 5. 主机新帧不等于每路相机有新曝光

`FPGA/rtl/fram_buf/fram_buf.v:148–163` 的 init_done 首帧后保持；`rd_buf.v:112–120` 每个拼接帧选择最新完成槽。四个相机曝光不强同步，且读端约为 40.4fps（`pcie_video.v:114–125`、`:168–171`）。因此同一相机帧可被重复读入不同的 PCIe 拼接帧。某相机停帧后，其最后 DDR 图像也可能一直重复输出；主机收到的 bit0/两位槽号没有每路曝光计数、时间戳或摄像头心跳。

对 ADAS 的影响：ARM 的接收时间与结果年龄只证明拼接帧已接收/模型已处理，不能独立证明每路传感器活跃；重复曝光也会影响视觉估速的时间准确性，且不能用于统计真实相机曝光率。这属于现有协议的信息限制，不应通过改写接收时间来掩盖。硬件验证应增加逐路遮挡/断开测试，区分重复画面和新曝光，当前版本不得宣称具备完整摄像头失效诊断。

## 保护检查和待实测项

本轮执行 `sha256sum -c .codex/fpga-baseline.sha256`，四项均为 OK：pango_fun.cpp、pango_fun.h、pcie_frame_status.h、pango_pci_driver_mul.ko。核查未修改 FPGA 文件或上述受保护文件。

核查时 `/dev/pango_pci_driver` 不存在；实际 PCIe 枚举、DMA 四槽读取色序、四路曝光、启动/停止、硬件切源、原预处理开关和外部输入到显示延迟均未实测。DDR 突发轮询、帧槽锁定和源切换清 FIFO 的 RTL 意图可从源码确认，时序正确性及资源余量必须由 PDS、仿真和板上证据验证。
