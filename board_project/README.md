# 紫光同创 FPGA + RK3568 ADAS

本工程保留原有上板视频和控制链路，在 ARM 应用层实现轻量感知、模拟驾驶决策及 Qt 叠加显示。2026-10-25已将用户提供的airockchip/yolov5八类权重转换为RK3568 INT8，并接入默认检测模型包。**车道模型仍待提供，当前明确显示未配置，不授权变道；没有CPU车道回退。模型转换及运行接口已验证，赛题准确率仍需独立验收。**

## 现场输入与硬件保护

现场素材链路固定为：

```text
电脑打开U盘图片/视频 → HDMI → FPGA → 现有PCIe/DMA → ARM → Qt显示
四路摄像头          → FPGA拼接 → 同一PCIe/DMA → ARM
```

ARM不读取现场U盘，不负责文件解码，不增加VPU、MPP或GStreamer依赖。原有摄像头/HDMI切源、预处理、车牌框叠加、白平衡和去雾按钮及其FPGA调用保持不变。FPGA工程、驱动、`src/hardware`、原有MainWindow源/头/UI和`build_release`保持冻结。没有新增FPGA写入、通信协议或LED寄存器操作。

传输保持1280×720 RGB565，四路局部640×360：左上前视、右上右视、左下后视、右下左视。HDMI为整幅前视。应用复用既有完成通知、自有帧池和最新帧消费；不会长期持有DMA裸指针，也不要求驱动导出DMA-BUF。

硬件输入源与显示缩放分开：实际源确认后摄像头对应四路、HDMI对应单路；前视放大只改变绘制，不能取消四路侧向安全判断。通信未知时停用来源绑定的标定和自动授权。

只读RTL显示HDMI在DMA前走原始视频分支。现有FPGA处理应在其实际生效路径演示，不能将ARM识别或直接文件诊断记作FPGA预处理验收。

## GitHub目录与驱动

仓库为`StellatoL/FPGARACE_2026_Pango_3`，本工程完整入口位于`board_project/`；仓库原有`arm/`与`fpga/`子工程保留供独立开发，不与这里的上板链路混用。克隆后执行：

```bash
git clone git@github.com:StellatoL/FPGARACE_2026_Pango_3.git
cd FPGARACE_2026_Pango_3/board_project
./build.sh --check
./build.sh
```

驱动二进制明确纳入版本控制，路径为`board_project/drivers/pango_pci_driver_mul.ko`；其SHA256见[驱动文件清单](drivers/SHA256SUMS)。本次发布保留ARM源码、FPGA项目/约束/RTL/IP及当前上板bitstream、驱动、现有临时模型、RKNN依赖、测试视频、测试与文档，并保留原`build_release`程序及运行库。编译对象、PDS中间数据库、日志、机器标定、迁移压缩包和本机工作备份由`.gitignore`排除，文件不从本机删除。2026-10-25新增八类候选模型见下文；本次更新包含八类原始PT、ONNX、RKNN候选产物和默认检测模型包。车道不加入占位权重。

历史验证记录中的`.codex/verification/`日志和回滚归档保留在原开发机；仓库携带测试需要的`.codex/fpga-baseline.sha256`，可在克隆后核验原通信文件和驱动。完整717项保护清单含未发布的编译缓存，仅用于原开发机完整性审计。

## 编译与在其他RK3568迁移

依赖保持C++17、qmake/Qt5、OpenCV4和工程内RKNN运行库；RGA通过系统头文件与动态库接入。

```bash
./build.sh --check
./build.sh --install-deps       # Ubuntu/Debian缺依赖时显式使用
./build.sh
./build_adas/run_img_recv_demo.sh
```

默认最多2个编译任务，可设置`BUILD_JOBS`。增量构建输出到`build_adas`；原`build_release`保留。支持换目录及外置输出：

```bash
BUILD_DIR=/path/to/output BUILD_JOBS=2 ./build.sh
/path/to/output/run_img_recv_demo.sh
```

不用修改源码中的`/home/cat`路径。launcher按自身位置定位资源和RKNN库。`--clean`只清理脚本持有标记的输出目录。`--no-rga`可构建诊断/旧功能兼容程序，但新版ADAS会报告硬件适配不可用，绝不回退到CPU图像处理。RGA、RKNN和内核接口兼容性仍需目标BSP验证，不自动替换系统驱动或库。

当前驱动文件是`drivers/pango_pci_driver_mul.ko`。本机2026-10-22核查：运行内核4.19.232，模块vermagic为6.1.99，PCIe未枚举到设备且无`/dev/pango*`节点。不能强制加载这个不匹配模块；应在与该上板工程匹配的系统验证现有驱动。不把此情况记成通信程序回归失败或上板通过。

## ARM职责与模型

正式ADAS不运行CPU灰度、滤波、阈值、形态学、HSV导航、Hough车道或LK估速。原有FPGA处理按钮继续工作；RGA仅做模型输入裁剪、缩放、RGB565→RGB8和常量padding。源数据仍是Qt高位红的RGB565，RGA API对应`BGR_565`；已通过硬件颜色/stride/裁剪/padding校验。CPU只解码模型张量、数值拟合、跟踪、规则、规划和Qt绘制。

| 模型 | 固定输入 | 能力 |
|---|---|---|
| YOLOv5n INT8 | NHWC RGB8 640×640 | person、vehicle、crosswalk、light_red、light_green、cone、barrier、debris |
| MobileNetV2-0.35车道模型INT8 | NHWC RGB8 512×256道路ROI | 四候选边界×96行、可见性、漆段/间隙、实虚线 |

旧的四个RKNN文件仍留作旧模式兼容，不能代替新版ADAS能力。默认新模型包路径`models/adas/manifest.json`，或用`ADAS_MODEL_MANIFEST`指定完整manifest路径。manifest验证SHA256、RK3568目标、类别能力、归一化、SDK前缀及严格输出契约；RGA自检或张量校验失败时感知失败。训练交付依据见[两个模型的完整接入规范](docs/model-replacement-roadmap.md)：含8类、手动导航、ONNX/RKNN输入区别、严格输出与锚点、车道标注/安全门槛、量化、打包和回滚。最小类别配置可用[8类训练YAML](docs/model-contract/scene-8-classes.yaml)。本机实测RKNN API 2.3.2、RKNPU驱动0.8.2；转换版本以目标板查询和实际验收为准。

当前默认模型包为`models/adas/manifest.json`，版本`adas8-candidate-2026-10-25`，`lane: null`。原始PT保留在`models/incoming/adas8_best.pt`，转换产物/metadata保留在`models/adas8-candidate-2026-10-25/`。采用上游`export.py --rknpu`，三个输出头均为已sigmoid的39通道；新格式`yolov5_rk_sigmoid_nchw_v1`避免重复sigmoid。

单个NPU线程共享检测模型，车道模型配置后才加载执行。常态四路目标为前视8Hz、其他各2Hz；转向时目标侧4Hz，其他侧后视各1Hz。前视灯色ROI目标4Hz，单路全帧目标12Hz；视觉导航任务已关闭。车道配置后目标四路10Hz、单路12Hz，缺失时不占NPU频次，也不产生虚假延迟统计。以上是调度目标，实际帧率和完整链路延迟尚待实流测量。

转换使用Toolkit2/runtime 2.3.2，真实NPU输出验证通过；96帧校准与12帧数值冒烟均来自白天道路视频，覆盖不足，不能作为红绿灯/行人/斑马线/路障的准确率证据。用户提供的训练结果中部分关键类别较弱，debris未提供AP。[八类接入及验证记录](docs/adas8-verification.md)列出文件哈希、转换命令、限制及后续车道接入方式。

## 决策与模拟驾驶

- 当前模型识别红/绿灯；红灯确认后停等，信号丢失维持停等；确认绿灯不能解除人工停车或紧急停车。强制行驶可展示闯红灯报警。
- 虚线且目标路径安全时模拟变道；实线保持直行，强制实线变道报警；未知不授权。
- 四路必须具有目标侧人员、车辆和路障有效观测，并取得转向请求之后的新观测；检测为空与检测不可用不同。
- 单路检查前视可见的目标路径障碍，输出模拟行为，不伪造侧向已验证结论。
- 导航通过N键或手动方向控件输入，关闭时不回退到视觉导航；区分路口转向和路段变道；后者遵守相同安全约束。数值局部规划给出保持/左变道/右变道/停车及最多16个路径采样点。
- 行人、车辆或路障占据危险路径时停车或否决对应候选。危险解除必须由新有效观测确认。
- 前视有效期500ms，侧后视1.5s；处理超时会失效，不靠延长有效期提高授权率。光线、遮挡或漆段证据不足时保持未知。
- 默认仿真速度0km/h、限速60km/h；显示速度来源。外部速度接口保留，选用后缺数据即无效，不自动改用仿真值。视觉估速延期。
- 有来源匹配四点标定时显示米制距离；未测车头偏移不显示米数。未标定仍保留框尺度TTC与近距报警。

快捷键：`1/2/3`本车车道，`A/D`转向灯，`J/L`强制变道，空格人工停车，`N`手动导航，`V`前视放大；编辑输入框时不响应这些快捷键。

电脑暂停、跳转、换图片或换视频不会通过现有视频接口向ARM提供媒体事件。ARM采用收帧单调时间，只作为接收时间估计；**每次更换素材后使用“复位决策”清除历史，路面/视角变化后重做标定。** 不宣称同步得到电脑PTS或自动识别静态图片。

## 四点标定及现场操作

1. 接通电脑HDMI到FPGA，电脑播放素材；在原有上位机按钮选择HDMI并开始采集。四路摄像头演示使用原摄像头入口。
2. 有匹配模型时查看能力状态、各视角检测频率和结果年龄；模型缺失时停车提示是预期失效保护。
3. 需要测距时，在前视同一平面上选择已知矩形，依次左上、右上、右下、左下，填写实测宽长及车头到近边距离。配置绑定实际来源和分辨率。
4. 路口视频用1/2/3选择车道，观察红停绿行；强制行驶验证红灯报警。
5. 道路图片/视频用A/D请求变道，J/L验证实线强制变道报警；四路用目标侧障碍和感知缺失测试否决。
6. 带导航视频由操作者按N输入对应方向（关闭→左→直行→右→关闭）；路口提示与路段变道分别处理，仍受安全规则约束。当前缺车道模型时不能完成虚实线变道闭环。
7. 使用行人横穿、近距车辆和路障素材验证停车。每种素材切换后复位，记录误报、漏报、未知和确认延迟。

原有录像功能没有实现，保持不可用；1080P、3分钟以内提交演示视频可从外部录制，不代表输入/推理必须1080P。

## 验证与报告

```bash
./tests/run_tests.sh
./tests/build_script_test.sh
python3 tests/model_bundle_test.py
ADAS_TEST_RGA=1 ./tests/run_tests.sh  # 可选：需要真实可用RGA
```

测试包括独立观测时钟、来源隔离、确认去重、灯色ROI与物体隔离、全类别有效性、四路侧向否决、单路目标路径、速度来源、局部规划、严格模型契约、有界检测解码及线程所有权。旧CPU车道/文件源测试只保留为历史诊断，不链接到正式ADAS。

实际硬件在线后可用`tools/adas_benchmark.py --duration 60 --output /path/to/new/report`打开应用，在窗口点击原有采集按钮；报告仅统计FPGA实流。无采集帧时脚本失败，不将空跑记成性能通过。程序另支持`--quit-after`、`--report`、`--screenshot`和`--window-size`；旧`--adas 文件`入口已移除。

工程目标为30fps输入预览≥25fps、ARM收帧到OSD P95≤250ms、危险出现到报警P95≤500ms。目标均待最终权重和实际链路测量；尤其4Hz灯色三次确认本身约需500ms，再加推理/显示，不能把确认等待排除后宣称完整报警达标。完整输入到显示延迟应外测。最终版本需补30分钟稳定性、四路实流、FPGA按钮和切源回归。

历史ARM改造验证见[改造记录](docs/arm-neural-verification.md)，当前八类模型接入见[2026-10-25验证记录](docs/adas8-verification.md)。赛题逐项验收、模型准确率和上板表现必须分别举证，逻辑测试不等于感知准确率。

## 回滚

修改前源码和程序归档：`.codex/verification/arm-npu-2026-10-20/before-source-program.tar.gz`；717项保护文件校验：同目录`protected.sha256`。回滚时关闭当前程序，先将归档解压到另一个目录检查，不覆盖受保护硬件文件。可继续使用保留的`build_release`或归档程序及其对应源码。改造前的本机回滚依据为归档和SHA256；发布后的版本还可通过Git提交记录恢复。
