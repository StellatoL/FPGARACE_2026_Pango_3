# ARM · RK3568 ADAS 应用

本目录是 FPGA 辅助驾驶系统的 ARM 端，运行于 **RK3568 AArch64 Linux**。应用通过现有 Pango PCIe 驱动接收 1280×720 RGB565 视频，通过 I2C 控制 FPGA 输入源和预处理；使用 RGA、RKNN/NPU、OpenCV 和 Qt 完成感知、规则决策与显示。

[返回仓库说明](../README.md) · [FPGA 与接口说明](../FPGA/README.md)

## 主要功能与处理流程

```text
FPGA PCIe/DMA 四帧接收缓冲
  → 帧池复制 / 最新帧投递
  ├→ RGA 裁剪、缩放、RGB 转换 → vehicle / traffic 双 RKNN 模型
  ├→ CPU / OpenCV 前视车道线分析
  └→ Qt 视频显示
感知结果 → 目标跟踪 / 超时与代次检查 → ADAS 规则 / 局部路径 → HUD
```

- `vehicle.rknn`：使用 COCO 类别清单，应用取其中的行人、自行车、汽车、摩托车、公交车与卡车。
- `traffic.rknn`：斑马线、行人、红/绿/黑/黄灯；其中行人与斑马线的重叠关系用于斑马线危险判断。
- CPU 车道线：颜色与局部对比度提取、Hough 候选、边界拟合及虚实线证据判断；独立线程以最高约 15 Hz 的节奏处理前视最新帧。这是调度上限，实际频率以报告为准。
- 规则与显示：灯控停车/放行、红灯强行驶报警、虚线变道条件检查、实线强制变道报警、行人急刹、碰撞风险、车道偏离、仿真超速及局部路径叠加。
- 速度与导航：当前 GUI 使用手动仿真速度和手动导航意图；没有从视频估计真实车速或自动读取导航图标的启用流程。

当前输入只来自 FPGA PCIe。电脑播放视频或图片时，需经 HDMI 输入 FPGA；程序未提供直接打开视频文件、RTSP 或 USB 摄像头的命令行选项。

## 目录与入口

| 路径 | 用途 |
| --- | --- |
| [ZYNQ_video_show.pro](ZYNQ_video_show.pro) | Qt 5 qmake 工程；`TARGET = img_recv_demo`，C++17 |
| [build.sh](build.sh) / [run_adas.sh](run_adas.sh) | 构建、依赖检查与默认启动入口 |
| [src/main.cpp](src/main.cpp) | QApplication 与命令行参数 |
| [src/hardware](src/hardware/) | PCIe ioctl、BAR0 映射、DMA 接收和 I2C 控制 |
| [src/common](src/common/) | 帧池、帧包、最新帧邮箱和结果结构 |
| [src/inference](src/inference/) | 模型清单校验、RGA、RKNN 推理和任务调度 |
| [src/core](src/core/) | 车道几何、地面标定、跟踪、规则与局部规划 |
| [src/ui](src/ui/) | Qt 表单、驾驶控制、采集生命周期、HUD、诊断 |
| [models/adas](models/adas/) | 模型文件、清单、来源与许可文本 |
| [third_party/rknn](third_party/rknn/) | RKNN API 头文件和 AArch64 `librknnrt.so` |
| [drivers](drivers/) | `pango_pci_driver_mul.ko` 内核模块二进制 |
| [config/adas.ini](config/adas.ini) | 超时设置；UI 保存的 ROI 配置 |
| [resources](resources/) | Qt 样式与图标 |

工程名中的 `ZYNQ` 为保留名称；当前依赖和构建脚本要求 AArch64 Linux/RK3568 环境。

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 平台 | RK3568，64 位 AArch64 Linux；构建脚本要求在目标架构上原生构建 |
| 编译工具 | 支持 C++17/`std::filesystem` 的 g++、make、pkg-config、binutils |
| GUI | Qt 5 Core/Gui/Widgets/Network 与可用的图形会话 |
| 图像库 | OpenCV 4，`pkg-config opencv4` 可用 |
| RGA | BSP 提供的 RGA 驱动、用户态库和 `/usr/include/rga/{rga.h,RgaApi.h,drmrga.h}` |
| RKNN | 附带 AArch64 Runtime、兼容的板端 NPU 驱动；模型要求 API 前缀 `2.3.2 (` |
| 设备 | `/dev/pango_pci_driver`、`/dev/mem`、`/dev/i2c-0` |

RGA 用户态库在运行时加载 `librga.so` 或 `librga.so.2`，并查找 `c_RkRgaInit`、`c_RkRgaBlit`、`c_RkRgaDeInit`。仅安装头文件不足以保证运行，神经网络启用前还会进行硬件预处理自检。

## 构建

以下命令均在 `arm/` 目录执行。Windows/x86 主机可阅读源码，但附带的 RKNN 库不能直接用于该主机运行。

```bash
# 只检查平台、工具链、模型/驱动等资产和输出目录
bash build.sh --check

# Ubuntu / Debian：按需安装基础构建包并构建
bash build.sh --install-deps

# 已具备依赖时构建
bash build.sh
```

脚本可安装的基础包为 `build-essential qtbase5-dev qt5-qmake pkg-config libopencv-dev binutils`。板端 RGA/NPU BSP、PCIe 驱动和 FPGA 配置需另行准备。

### 构建选项

| 参数 / 环境变量 | 行为 |
| --- | --- |
| `--check` | 完成检查后退出，不生成、清理或编译构建目录 |
| `--install-deps` | 显式安装缺少的 Ubuntu/Debian 基础构建包 |
| `--clean` | 仅清理被本脚本标记且属于当前源码树的构建目录 |
| `--no-rga` | 诊断构建；神经网络 ADAS 将报告 RGA 不可用 |
| `BUILD_DIR` | 输出目录，默认 `arm/build_adas` |
| `BUILD_JOBS` | 编译并行数，默认 2；兼容 `ADAS_BUILD_JOBS` 后备值 |
| `QMAKE_BIN` | 指定 Qt 5 qmake 路径或命令 |

```bash
BUILD_JOBS=2 bash build.sh
# 使用独立输出目录：
BUILD_DIR="$PWD/build_board" bash build.sh
```

缺少适配的 RGA 头文件时，脚本会自动选择 `CONFIG+=no_rga`。这种构建可用于纯视频/界面诊断，**不具备神经网络感知能力**；网络图像预处理失败时不会转为 CPU 推理或 CPU 图像预处理。CPU 车道线线程是单独的处理分支。

默认构建产物：

```text
build_adas/
├── img_recv_demo
├── librknnrt.so
├── run_img_recv_demo.sh
├── models    -> 源目录/models
├── resources -> 源目录/resources
└── config    -> 源目录/config
```

启动脚本设置 `LD_LIBRARY_PATH` 并检查运行资源。上述资源是链接，部署时需保留源目录或将资源一并整理；只复制可执行文件不能构成完整运行包。自定义 `BUILD_DIR` 后，应使用该输出目录中的 `run_img_recv_demo.sh`，因为顶层 `run_adas.sh` 固定寻找 `build_adas/`。

## 驱动与设备准备

```bash
uname -r
modinfo drivers/pango_pci_driver_mul.ko
# 仅在内核匹配且模块尚未加载时执行：
sudo insmod drivers/pango_pci_driver_mul.ko
ls -l /dev/pango_pci_driver /dev/mem /dev/i2c-0
```

附带 `.ko` 的 `vermagic` 为 `6.1.99 SMP mod_unload aarch64`。当前目录只提供模块二进制，未附驱动源码和编译入口；内核不匹配时需要获取对应驱动，不能靠重编 Qt 应用解决。

PCIe 访问包括驱动 ioctl 和 `/dev/mem` BAR0 映射；I2C 固定打开 `/dev/i2c-0`。启动账号需要具备这些设备的访问权限。若板端必须使用管理员权限，请在可用的图形会话中启动，并保留相应显示环境。构建脚本不会加载内核模块，也不会修改 FPGA、模型或系统 RKNN 安装。

## 启动与界面操作

```bash
# 首次联调建议确认纯视频链路
sh ./run_adas.sh --mode raw

# 默认模式是 adas
sh ./run_adas.sh --mode adas

# 自定义输出目录的启动入口示例
sh ./build_board/run_img_recv_demo.sh --mode adas
```

1. 配置 FPGA 并连接摄像头，或让电脑通过 HDMI 输出 1280×720 逐行 30 Hz 道路素材。
2. 点击“打开”，确认 PCIe 链路信息有效。窗口启动时不会自动打开 FPGA 或自动采集。
3. 选择“摄像头”或“HDMI”，通过 I2C 切换并确认源状态。
4. 点击“采集视频”。需要结束时点击“关闭采集”。
5. 确认画面后切换“ADAS 辅助驾驶”，设置当前车道、各车道信号灯 ROI、仿真速度与限速；需要米制距离时执行路面标定。
6. 通过“保存”保存显示区域 PNG，或通过“导出诊断报告…”保存 JSON。

### 输入布局

摄像头源自动按四宫格解释；HDMI 源按全幅单前视解释。四宫格对应：

| 位置 | 图像区域 | ARM 视角 |
| --- | --- | --- |
| 左上 | x=0…639，y=0…359 | 前视 / 摄像头 1 |
| 右上 | x=640…1279，y=0…359 | 右视 / 摄像头 2 |
| 左下 | x=0…639，y=360…719 | 后视 / 摄像头 3 |
| 右下 | x=640…1279，y=360…719 | 左视 / 摄像头 4 |

FPGA `KEY1` 的单摄模式没有相应 I2C 状态上报给此 GUI；进行四路 ADAS 时应保持 FPGA 四宫格。界面“放大前视”只改变显示，仍保留四路安全检查。`--single-view` 只设置初始解释，实际源确认后仍会自动同步。

源或布局改变时，程序清理旧代次的感知、跟踪和决策，并切换对应标定配置；源状态未知时停用来源标定。

### 模拟驾驶快捷键

| 按键 | 操作 |
| --- | --- |
| `1` / `2` / `3` | 选择左 / 直行 / 右车道 |
| `↑` / `↓` | 普通行驶 / 人工停车 |
| `空格` | 切换人工停车 |
| `←` / `W` / `→` | 输入左 / 直行 / 右手动导航意图 |
| `Esc` / `N` | 清除 / 循环切换导航意图 |
| `A` / `D` | 切换左 / 右转向灯 |
| `Ctrl+↑` | 强制行驶，用于红灯场景测试 |
| `Ctrl+←` / `Ctrl+→` | 强制左 / 右变道请求 |
| `J` / `L` | 切换强制左 / 右变道请求 |
| `V` | 放大前视 |

焦点处于文本框、数值框或下拉框时，驾驶快捷键不会接管输入。强制操作用于演示规则与报警，并不保证能越过所有停车/感知条件。所有驾驶动作均为模拟，当前没有真实车辆控制输出。

## 标定与信号灯 ROI

点击“前视路面四点标定”，在同一地面已知矩形上依次选择 **左上 → 右上 → 右下 → 左下**，填写实测宽、长；右键可清除点。要输出米制目标距离，还需填写车头到矩形近边的实测距离；保持“未测量”时不输出该距离。

标定保存到 `config/calibrations/<profile>.yml`，按 FPGA 源与布局区分，四宫格前视为 640×360，HDMI 前视为 1280×720。改变相机安装、素材几何或画面裁剪后应重新标定。

“各车道信号灯 ROI”分别设置左、直行、右车道的灯区域。X/Y/W/H 是相对前视的百分比，区域不得越界；默认均为前视上半幅，需要按实际灯位置缩小。配置保存到 `config/adas.ini` 的 `profiles/<profile>` 分组。

基础超时设置为：

```ini
[timing]
front_timeout_s=0.5
side_timeout_s=1.5
```

## 模型与调度

[manifest.json](models/adas/manifest.json) 使用 schema 3，目标 `rk3568`，双模型均为 INT8、640×640 RGB8 输入。`lane` 为 `null`，当前没有车道线 RKNN 模型。

| 模型 | 类别 / 输出约定 |
| --- | --- |
| `traffic.rknn` | `crosswalk, person, light_red, light_green, light_black, light_yellow`；`yolov5_raw_nchw_v1` |
| `vehicle.rknn` | COCO 80 类清单，应用过滤为行人与车辆相关类；`yolov5_rk_sigmoid_nchw_v1` |

加载时会校验清单约定、文件 SHA-256、运行库版本、输入尺寸、输出数量/布局及量化信息。若更换模型，必须同时匹配类别顺序、锚框、输出解释、输入规格、Runtime 前缀和文件校验值。可通过环境变量 `ADAS_MODEL_MANIFEST` 指定其他兼容清单。

双模型轮流调度，每个模型的前视最短间隔为 0.125 s；无转向意图时其余视区为 0.5 s，有转向意图时目标侧为 0.25 s、其他侧后视为 1 s。这些是源码中的调度间隔，实际频率还取决于 RGA/NPU 耗时。线程保留最新帧，处理不过来时覆盖旧待处理帧，并在诊断中统计。

当前双模型没有专用 `roadblock` 类别。类型和决策引擎预留的路障接口不等于已有路障感知模型；导航图标识别也没有接入当前工作线程。

## PCIe 与 I2C 契约

### 帧传输

固定分辨率为 1280×720，RGB565，行跨度 2560 字节，单帧 1,843,200 字节。应用分配四帧缓冲；收到完成槽号后，从 `img_buf + slot × 1843200` 复制到帧池，再交给异步处理。

驱动 `read()` 的正返回值按状态字解释：bit 0 表示帧就绪，bits 2:1 表示槽号 0…3。它不是普通视频设备“返回读取字节数”的约定。FPGA、驱动和用户态必须保持一致。

头文件中保留的 `W_IMAGE=960`、`H_IMAGE=540` 等旧宏不代表当前运行尺寸；实际接收、帧池、界面与顶层 DMA 使用 1280×720。

### I2C

总线为 `/dev/i2c-0`，7 位从机地址 `0x66`。写操作为寄存器地址与值两个字节；读操作先写寄存器地址，再读一个字节。

| 寄存器 | 含义 |
| --- | --- |
| `0x00` | FPGA 读写回环 |
| `0x01` | 源命令：0=摄像头，1=HDMI |
| `0x02` | 只读实际源：0=摄像头，1=HDMI |
| `0x03` | bit 0：预处理总开关 |
| `0x04` | UI 名称为“车牌定位+画框”；当前 RTL 实际连接 VIP 亮度增强位 |
| `0x05` | bit 0：白平衡，在总开关开启时生效 |
| `0x06` | bit 0：暗通道去雾，在总开关开启时生效 |

当前 FPGA 的车牌定位/画框实例已注释，不能用该 UI 开关验证车牌功能。HDMI 的预处理请求也没有被 RTL 强制屏蔽；若需要原始 HDMI 通路，请显式关闭预处理总开关。

## 命令行与诊断

| 参数 | 含义 |
| --- | --- |
| `--mode adas` / `--mode raw` | ADAS / 纯视频，默认 `adas` |
| `--single-view` | 初始布局设为单前视，实际源确认后会同步 |
| `--quit-after <seconds>` | 指定正数秒数后正常关闭，最大 86400 |
| `--report <path>` | 退出时写性能/能力 JSON |
| `--screenshot <path>` | 定时关闭时保存窗口截图，通常与 `--quit-after` 配合 |
| `--window-size <WxH>` | 设置窗口大小，最小 900×540 |
| `--help` | 查看帮助 |

```bash
mkdir -p results
sh ./run_adas.sh --mode adas --window-size 1280x720 \
  --quit-after 60 \
  --report "$PWD/results/adas-report.json" \
  --screenshot "$PWD/results/adas-window.png"
```

上述命令不会自动点击“打开/采集视频”；需在测试窗口内完成采集操作。建议使用绝对输出路径，并预先创建父目录。

报告包含输入/显示帧率、各视区模型更新率、CPU 车道频率、最新推理耗时、P95 接收到 HUD 延迟、帧池丢弃/待处理覆盖、源状态、标定状态、各组件就绪情况和决策状态。`-1` 等无效统计值表示尚无样本，不能按零延迟理解。

**延迟范围是 ARM 接收回调到 Qt pixmap 更新**，排除了相机、FPGA、显示扫描和事件确认；平均频率的分母包含程序启动以来的等待时间。比较结果时应说明采集时长、输入布局、模型状态和操作过程。

## 常见问题

| 现象 | 核对项 |
| --- | --- |
| 构建提示不是 AArch64 | 在 RK3568 64 位 Linux 原生构建；附带 RKNN 库是 ELF64 AArch64 |
| Qt/OpenCV 找不到 | 执行基础依赖安装；检查 Qt 5 qmake 和 `pkg-config opencv4` |
| 驱动加载失败 | 比较 `uname -r` 与模块 `vermagic`，查看板端内核日志；当前仓库没有驱动源码 |
| 打开设备失败/无 BAR 映射 | 检查 PCIe 枚举、驱动节点及 `/dev/mem` 权限 |
| 源切换失败或来源未知 | 检查 `/dev/i2c-0`、地址 `0x66`、连线、FPGA 版本和权限 |
| 纯视频正常，模型不可用 | 检查构建是否禁用 RGA、RGA 自检与动态库、NPU 驱动、RKNN 版本及模型清单 |
| 摄像头图像被错误分区 | 将 FPGA `KEY1` 切回四宫格，按前/右/后/左对应接线 |
| 距离不可用 | 执行来源/布局匹配的实测标定，并填写车头到近边距离 |
| 路口不放行/变道一直等待 | 检查灯 ROI、前视模型有效性、观测超时、线型和目标侧新观测 |

本目录未附自动化测试套件，也没有当前目标板的实测性能结论。`build.sh --check` 是平台和资产检查，不能替代驱动、RGA、NPU 与 FPGA 的整机联调。
