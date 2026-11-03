# FPGA · 视频采集、处理与 PCIe 传输

本目录包含选题三辅助驾驶系统的 FPGA 工程。顶层为 [pcie_video](prj/rtl/pcie_video.v)，负责四路 OV5640 采集、独立 DDR3 双帧槽、四宫格/单摄输出、HDMI 输入、可选图像增强和 PCIe DMA。ARM 端负责目标识别、驾驶规则及 Qt 显示。

[返回仓库说明](../README.md) · [ARM 使用说明](../arm/README.md)

## 工程配置与目录

| 项目 | 当前文件中的配置 |
| --- | --- |
| 工程入口 | [prj/pcie_video.pds](prj/pcie_video.pds) |
| 顶层模块 | `pcie_video` |
| 工具版本记录 | PDS / Fabric Compiler `2022.2-SP6.4` |
| FPGA | Logos2 `PG2L50H`，`FBG484`，速度等级 `-6` |
| DDR3 接口 | DQ 16 位、DQS 2 位、行 15 位、列 10 位、Bank 3 位 |
| PCIe | Endpoint，Gen2，x2；用户数据通路 128 位 |
| 视频输出到 ARM | 1280×720 RGB565，1 像素/拍 |

```text
prj/
├── pcie_video.pds
├── pcie_video.fdc
├── rtl/
│   ├── pcie_video.v             # 系统顶层
│   ├── ov5640/                  # SCCB 配置、DVP RGB565 采集
│   ├── hdmi/                    # MS72xx I2C 配置
│   ├── camera_ddr3_store/       # 四路采集跨域、帧槽、DDR 读写、视频桥
│   ├── video_control/           # 四宫格读出、视频源与处理路径选择
│   ├── stereo_stitch/           # 五档模式选择、读复用、MATLAB 辅助素材
│   ├── video/vip/               # 白平衡、去雾、亮度增强、滤波与色彩转换
│   ├── pcie/                    # PIO、行 DMA、TLP/DMA 控制
│   ├── iic_slave/               # ARM 控制寄存器
│   ├── fram_buf/                # 帧缓存辅助模块与光栅时序
│   └── debug/uart/              # 保留的 UART 模块
├── ipcore/                      # PLL、PCIe、DDR3、FIFO 等 Pango IP
├── constraints/{camera,hdmi}/   # 两套约束副本
├── generate_bitstream/
│   ├── camera/pcie_video.sbit
│   └── hdmi/pcie_video.sbit
└── sim/                         # ModelSim 工程、VIP TB、图像与调试截图
```

目录中还保留其他缓存、图像处理与拼接辅助文件。是否进入当前数据通路，应以顶层的实际实例连接为准。

## 数据通路

### 四路摄像头

```text
四路 OV5640：各自 DVP / RGB565 / 1280×720
  → cds_capture / 异步 FIFO
  → cds_writer + cds_slots：每路两个完整帧槽
  → cds_ddr_port → DDR3 IP
  → 四宫格 cds_quad_reader 或单摄顺序读
  → cds_video_bridge / 1280×720 光栅
  → 源选择 → 可选 VIP → video_crtl → pcie_tx_fun → PCIe
```

四路摄像头写入独立运行，显示模式只影响读出。帧槽控制避免读会话占用的槽被写入覆盖；四宫格在会话开始时锁定各路可用帧，缺失的源以黑色填充。各路是独立采集流，不能将四宫格理解为外部同步曝光。

四宫格对各路完整帧保留偶数行、偶数列，得到四个 640×360 视区：

| 区域 | 摄像头 / 内部 ID | ARM 对应视角 |
| --- | --- | --- |
| 左上 | 摄像头 1 / ID 0 | 前 |
| 右上 | 摄像头 2 / ID 1 | 右 |
| 左下 | 摄像头 3 / ID 2 | 后 |
| 右下 | 摄像头 4 / ID 3 | 左 |

`KEY1` 低有效，经过消抖，在 **四宫格 → 摄像头 1 → 2 → 3 → 4 → 四宫格** 间循环。模式在帧边界交接，单摄保持全幅 1280×720。

当前 ARM GUI 只读取摄像头/HDMI 源状态，不能同步这个五档模式。四路 ADAS 演示应保持四宫格；ARM 的“放大前视”可在不改变 FPGA 数据布局的情况下放大画面。

### HDMI

HDMI 接收端通过 MS72xx 配置模块初始化。顶层在 `pixclk_in` 域寄存 RGB、VS、HS、DE，并将 RGB888 截取为 RGB565；有效视频使用 DE，未将 HS 当作像素有效信号。

当前使用目标为 **1280×720 逐行 30 Hz**。HDMI 路径不进入摄像头 DDR3 缓存，而是经过源选择、处理选择和 DMA 发送到 ARM。顶层未提供已连接的 FPGA HDMI 发射像素接口；最终显示由 ARM Qt 窗口完成。

源码注释写有“HDMI 强制旁路预处理”，但实际 `processing_path_req = preprocess_sync` 没有按源屏蔽。因此要使用原始 HDMI 通路，需通过 ARM 显式关闭 `0x03[0]`；不能依赖该注释推断预处理一定关闭。

## DDR3 与帧槽

| 内容 | 约定 |
| --- | --- |
| 单帧有效负载 | 1280×720×2 = 1,843,200 字节 |
| 单路存储 | 两个完整 RGB565 帧槽 |
| 地址单位 | Pango 控制器使用 16 位 RGB565 字地址，不能直接按字节地址解释 |
| 通道基址 | `camera_id × 0x0200000`，单位为 16 位字 |
| 槽偏移 | `slot × 0x0100000`，单位为 16 位字，即每槽 2 MiB 空间 |
| 数据端口 | 128 位；四宫格读模块按 16 拍突发组织数据 |

四路各两个槽共划分 16 MiB 空间，有效图像负载小于所划分槽容量。这套 FPGA DDR3 帧槽与主机的四帧 DMA 环形缓冲用途不同。

[camera_ddr3_store.v](prj/rtl/camera_ddr3_store/camera_ddr3_store.v) 还提供带 tag 的逻辑坐标请求/响应口，当前顶层将 `req_valid` 固定为 0；显示使用顺序读会话，不应把坐标口当作已经接入 ARM 的图像访问接口。

## 图像增强与控制位

当前 VIP 路径包含：RGB565 扩展 → 可选白平衡 → 可选暗通道去雾 → RGB/YCbCr 转换 → 可选亮度增强 → 双边滤波 → RGB 输出。预处理总开关关闭时走原始旁路。

顶层向 `vip_crtl[3:0]` 传入：

```text
{ preprocess_active, box_draw_active, dehaze_active, white_balance_active }
         总开关          实际为亮度增强      去雾              白平衡
```

`vip.v` 实际将 bit 3 解释为总开关、bit 2 解释为亮度增强。顶层的 `vdp`、`vdp_box_draw` 实例及相关输出选择均已注释，当前没有接入车牌定位和画框功能。ARM UI 仍保留“车牌定位+画框”的名称，其对应寄存器在这份 RTL 中实际控制亮度增强。

处理特性控制在像素域同步并在帧边界更新；`video_process_selector` 在原图/处理图交接时等待帧边界并屏蔽交接区间。

## ARM 控制接口

### I2C 寄存器

ARM 使用 `/dev/i2c-0`，FPGA 从机地址为 **7 位 `0x66`**。写入为 `[寄存器地址, 值]`，读取为先写地址、再读一字节。复位时读写控制寄存器均为 0。

| 地址 | 方向 | 当前语义 |
| --- | --- | --- |
| `0x00` | R/W | 回环寄存器 |
| `0x01` | R/W | bit 0：0=摄像头，1=HDMI；每次写入翻转更新 toggle |
| `0x02` | R | 实际源 bit 0；经跨域同步读回 |
| `0x03` | R/W | bit 0：VIP 总开关 |
| `0x04` | R/W | 保留“画框”旧名称，实际进入 VIP bit 2 亮度增强 |
| `0x05` | R/W | bit 0：白平衡 |
| `0x06` | R/W | bit 0：暗通道去雾 |

`0x05/0x06` 及亮度增强需在总开关开启时生效。总线控制 SDA/SCL 在当前主约束中分别为 **R19 / P19、LVCMOS33**；实际硬件仍需核对板卡连线与电压。

顶层将源选择器的本地 `key_in` 固定为 1，摄像头/HDMI 切换由 ARM I2C 命令进行。`KEY1` 的用途是摄像头显示模式切换，不能将其误认为 HDMI 切换键。

### PCIe BAR0 / PIO

下表是 RTL 解码使用的**字节偏移**：

| 偏移 | 用途 |
| --- | --- |
| `0x000` | 写 `0xffffffe5` 启动视频发送；写 `0xffffff00` 停止 |
| `0x050` / `0x054` | 主机帧缓冲 0 地址低 / 高 32 位 |
| `0x020` / `0x024` | 主机帧缓冲 1 地址低 / 高 32 位 |
| `0x040` / `0x044` | 主机帧缓冲 2 地址低 / 高 32 位 |
| `0x030` / `0x034` | 主机帧缓冲 3 地址低 / 高 32 位 |
| `0x060` | 写 1 产生 DMA 配置锁存使能脉冲 |
| `0x140` | 帧状态：bit 0=完成，bits 2:1=槽号；读取清除完成标志 |

源代码中有“高位完成、低位索引”的旧注释，但实际返回拼接是 `{29'd0, i_wr_index, r_wr_frame_done}`，应以上表为准。

DMA 以一行 **2560 字节 = 640 DW** 为单位，一帧 720 次行请求，使用 64 位主机目标地址和四槽循环。主机地址由驱动配置，不是四路摄像头 DDR3 的地址。

## 在 PDS 中导入与构建

当前文件集保留了工程整理前的路径，重新构建前需要处理以下问题；本 README 只记录操作依据，没有修改 RTL 或 `.pds`。

1. 使用 PDS 2022.2-SP6.4 打开 `prj/pcie_video.pds`，核对器件为 PG2L50H/-6/FBG484，顶层为 `pcie_video`。
2. `.pds` 中自定义源码大多记录为 `../rtl/...`，而本目录实际为 `prj/rtl/...`。按当前布局将这些引用重新绑定到工程旁的 `rtl/...`。
3. 工程中还列出缺失的 `../rtl/video/vdp/**` 源文件。当前顶层相关实例已注释；若使用当前不含车牌处理的配置，应从工程源文件列表移除这些失效项。仅改路径不能恢复缺失的功能。
4. 保留 `.pds` 声明的 IP 描述符：`PLL_50mHZ`、`pcie_test`、`video_buffer`、`ddr3`、`fram_buf/rd_fram_buf`、`fram_buf/wr_fram_buf`，检查 IP 生成文件与当前 PDS 安装匹配。I2C 的 include 目录应能找到 `rtl/iic_slave/i2cSlave_define.v`。
5. 使用匹配板卡的约束。工程当前引用 `prj/pcie_video.fdc`，它与 `constraints/hdmi/pcie_video.fdc` 内容一致；`constraints/camera/` 为另一份约束副本。不要只切换文件名而跳过引脚、电压、时钟检查。
6. 运行解析、综合、映射、布局布线和时序检查，再生成位流；确认缺失文件、未驱动网络和关键时序问题已处理。不能以目录中已有 `.sbit` 推断当前源码已经通过这些步骤。

### 时钟核对

| 信号 / 配置 | 当前记录与注意点 |
| --- | --- |
| `sys_clk` | 顶层注释为 27 MHz，但 `PLL_50mHZ.v` 的 `CLKIN_FREQ` 为 25.0 MHz；应先核实板载晶振，再统一配置与约束 |
| `clk_50m` | 摄像头输出光栅和控制逻辑按名义 50 MHz 使用；是否符合该频率取决于 PLL 输入与配置 |
| `core_clk` | 顶层 DDR 计数说明按 25 MHz×42/8=131.25 MHz 使用；修改 DDR IP 后需同步核对计数值 |
| 摄像头 DVP 时钟 | 当前 FDC 对四路 `cmos*_pclk` 使用 20.830 ns 周期 |
| HDMI 像素时钟 | 顶层 720p30 注释为 74.25 MHz，而 FDC 使用 6.670 ns 约束；需按实测输入模式核对 |
| PCIe | FDC 记录参考时钟 10 ns、`pclk` 4 ns、`pclk_div2` 8 ns |

摄像头输出光栅参数为 H_TOTAL=1650、V_TOTAL=750；在准确 50 MHz 下理论扫描约 40.4 Hz。这是按配置推算的输出节奏，不是摄像头输入帧率、整机有效帧率或无丢帧保证。

### 已有位流

仓库保留 [camera 位流](prj/generate_bitstream/camera/pcie_video.sbit) 和 [hdmi 位流](prj/generate_bitstream/hdmi/pcie_video.sbit)。两者位于不同配置目录，但未附可核验的源码版本、时序验收和整机性能清单。使用前应确认硬件适配；需要与当前源码建立对应关系时应重新构建验证。

## 仿真与辅助素材

- [sim/pcie_video.mpf](prj/sim/pcie_video.mpf) 和 [tb_vip.v](prj/sim/tb/tb_vip.v) 提供 VIP 图像仿真入口；输入与输出素材位于 `sim/img_in`、`sim/img_out`。
- 当前 TB 顶层名为 `tb_top_bmp`，图像尺寸宏为 713×652，文件路径包含原作者的绝对 Windows 路径。运行前需改为本地输入/输出路径，并与 BMP 实际尺寸一致。
- TB 的 `vip_crtl` 仍为 2 位，而当前 VIP 端口为 4 位。若要验证启用后的增强通路，需在仿真副本中对齐位宽与控制值；当前 `2'b11` 扩展后不会置位全局 bit 3。
- 重建仿真库和依赖后再运行，不能将附带的 `sim/work` 编译缓存或已有 BMP 当作当前源码的回归测试结果。该 TB 也不覆盖整条摄像头/DDR3/PCIe 链路。
- [camera_ddr3_store/files.f](prj/rtl/camera_ddr3_store/files.f) 是存储子系统相关文件清单，不是完整顶层工程构建脚本。
- `stereo_stitch/matlab/run_stereo_params.m` 提供双图水平平移参数估计，自动匹配需要 Computer Vision Toolbox，也支持人工对应点；结果不会自动写入当前顶层。现行五档模式没有双目无缝拼接模式。

## 上板联调建议

1. 先核对时钟、复位和 DDR3 初始化状态，再确认 ARM 侧 PCIe 枚举与匹配驱动。
2. 关闭 VIP，以单摄确认完整 1280×720 像素；再切回四宫格核对摄像头 1…4 的位置和画面稳定性。
3. 在 ARM 打开设备并采集，检查整帧内容和连续槽号；确认每行 2560 字节、单帧 1,843,200 字节。
4. 由 ARM I2C 选择 HDMI，读取 `0x02` 确认状态，使用匹配时序的道路素材；先显式关闭 VIP 验证原始图。
5. 按需逐项开启白平衡、去雾、亮度增强，观察切换边界与输出稳定性，再进入 ARM ADAS。
6. 记录当前实现的资源、时序、各路丢帧与有效传输指标，与 ARM 接收/显示/推理报告分开统计。

可在各自时钟域关注顶层的 `camera_frame_valid`、`camera_capture_overflow`、`camera_write_drop`、`camera_ddr_protocol_error`、`camera_output_underflow` 和帧桥诊断计数；PCIe TX 还保留 FIFO 溢出/欠载调试信号。标记能否进入实际调试网络需在实现后确认。

## 当前边界

- `key2` 和 `stitch_uart_tx` 保留在顶层端口中，但当前没有相应有效功能连接，不能按旧注释宣称可调拼接参数或输出双目状态串口。
- 板端 LED/链路指示与 ARM ADAS 决策不是同一通路；目前没有从 ARM 决策到 FPGA 驾驶指示灯或车辆执行器的完整接口。
- 本目录未提供当前顶层经复核的时序、资源、整机端到端延迟和四路稳定运行结论。`ipcore/ddr3/pnr` 内的文件属于 IP 级工程产物，不能作为顶层系统报告。
- FPGA 的神经网络目标识别不在当前 RTL 内；目标模型部署在 RK3568 NPU，CPU 完成车道线与规则处理。系统功能需与 [ARM 实现](../arm/README.md) 配合验证。

源文件与 Pango IP 保留各自版权/许可声明；请同时保留这些声明和模型随附许可文本。
