# RK3568 ARM 显示、推理与 ADAS 软件

这是 Qt 5 / OpenCV / RKNN 的 Linux AArch64 应用，qmake 工程为 [adas_arm.pro](adas_arm.pro)，可执行文件名仍为 `img_recv_demo`。原工程名 `ZYNQ_video_show.pro` 属于历史命名，本软件当前面向 RK3568。

**文件回放、旧 PCIe 接收和推理代码已经存在；当前 FPGA AVM 核心与本程序尚未完成板级联调。** 随工程导入的 2026-10-03 板端历史记录（保留本地） 描述 RK3568 LubanCat 上的测试，本次只在 x86_64 WSL 重跑可移植 CPU 测试和工程入口检查，详见 [本次验证报告](../doc/validation/README.md)。

## 1. 功能与入口

| 功能 | 当前实现 |
|---|---|
| 视频显示 | Qt 主窗口，文件/视频回放与 PCIe 输入切换，RGB565 转显示图像 |
| 车牌识别 | 检测、候选过滤、字符推理、CTC 文本解码、结果叠加 |
| 交通检测 | 6 类交通主模型、可选车辆模型、SORT 跟踪、场景规则 |
| 车道线 | CPU 分析实/虚线与可见行程，保留 Unknown 状态 |
| 导航箭头 | 从固定图像区域提取导航面板箭头，判别左/直/右 |
| ADAS | 灯色与障碍时序确认、变道约束、行人提醒、模拟速度、TTC 提醒 |
| 演示输出 | HUD、文字、方向箭头与屏幕 LED；没有实体 LED/执行器闭环 |

推理线程将帧指针放入三个任务槽，忙时可丢弃输入；底层输入使用四个帧缓冲。源缓冲和推理线程之间没有已经完成联调验证的完整所有权协议，不能仅凭指针传递宣称安全零拷贝。ADAS 模式下，车道线和导航处理受交通模型 `model_ready` 条件影响；单独 CPU 测试通过不代表模型缺失时整个 GUI 感知流程仍运行。

`save_video` 按钮处理仍是占位逻辑。`src/core/plate_recog_task.*` 保留原件，但不在当前 qmake 的 SOURCES/HEADERS 构建清单中。

## 2. 目录与模块

| 路径 | 职责 |
|---|---|
| `src/main.cpp` | Qt 入口、QSS、`--adas <文件>` |
| `src/ui/` | 窗口、UI 表单、输入切换、图像和 HUD |
| `src/core/` | 车牌、交通、ADAS 规则、车道线与导航算法 |
| `src/inference/` | 推理线程、模式、SORT、YOLO 后处理、可选 RGA |
| `src/hardware/` | 旧 Pango PCIe/I2C 访问、帧状态解析、文件输入 |
| `src/common/` | 公共图像类型、结果与车牌字符表 |
| `tests/` | 现有四个 C++ 测试及 `run_host_tests.sh` |
| `resources/` | `style.qss` 和项目实际引用的 `video.ico` |
| `models/` | 四个导入 RKNN 模型，见 [模型说明](models/README.md) |
| `third_party/` | RKNN 头文件/运行库、ztu 封装，见 [第三方说明](third_party/README.md) |
| `drivers/prebuilt/` | 旧 PCIe 内核模块，仅二进制 |
| `tools/` | 视频生成、模型转换、RKNN 布局探针 |
| `doc/history/` | 历史设计与板端记录，保留当时路径 |
| `build/` | 当前生成结果：`release/`、`host-tests/`、`test-videos/` 等 |

原 `build_release/` 完整保存在 `../archive/imported-arm/build_release/`，其中 Makefile、对象和可执行程序绑定导入时的环境，不作为当前构建目录。

## 3. 板端环境与构建

完整程序要求 **64 位 ARM Linux**。带入的 `librknnrt.so` 和 `.ko` 都是 AArch64 ELF，不能用于 Windows、x86_64 Linux 或 32 位 ARM。运行库还必须匹配板端 RKNPU 驱动，内核模块必须匹配内核和设备。

源码仓库附带 [预编译 PCIe 驱动](drivers/prebuilt/pango_pci_driver_mul.ko)，不附带模型、RKNN 运行库和受限 SDK 头文件。先按 [第三方说明](third_party/README.md) 补齐头文件/运行库和三个必需模型，核对 [资产清单](models/assets.json)。

在 Ubuntu/Debian 板端，从 `arm/` 执行：

```bash
sudo apt-get update
sudo apt-get install build-essential qtbase5-dev qt5-qmake pkg-config \
  libopencv-dev libopencv-contrib-dev libfreetype6-dev
bash build.sh --check
bash build.sh
```

脚本不自动安装系统包，也不递归删除输出目录。默认增量构建到 `build/release/`，成功后复制 `librknnrt.so` 及所有 `models/*.rknn` 到输出目录中的 `models/`。额外车辆模型只搜索可执行文件目录及其 `models/`，因此不能省略该部署步骤。

```text
build/release/
  img_recv_demo
  librknnrt.so
  models/*.rknn
```

可用环境变量：

| 变量 | 用途 |
|---|---|
| `BUILD_DIR` | 输出目录；相对路径基于 `arm/`；构建与运行须一致 |
| `QMAKE_BIN` | 选择 Qt 5 的 qmake |
| `JOBS` | make 并行数，默认 nproc |
| `NO_RGA=1` | 传递 `CONFIG+=no_rga`，禁用可选 RGA |

默认在发现 `/usr/include/rga/rga.h` 时编入 RGA 预处理，实际运行仍需要兼容的板端 RGA 库。交叉编译工具链、sysroot 和 CMake 工程目前没有提供；本构建脚本只做板端原生编译。

`bash build.sh --check` 也可在 x86_64 Linux 检查工具与资产，会明确显示主机限制。`bash build.sh --stage-assets` 仅复制运行库和模型，不构建、不运行应用，适用于资源部署检查。

## 4. 运行

```bash
bash run.sh
bash run.sh --adas /absolute/path/to/road.mp4
```

启动器设置运行库搜索路径并切换至 `arm/`，保证 `resources/style.qss` 可读。命令行相对视频路径在切换目录前按调用者位置转换；包含空格时正常引用即可：

```bash
BUILD_DIR=build/release bash run.sh --adas "../videos/road sample.mp4"
```

GUI 中可选择视频、车牌、交通检测或 ADAS 模式，文件与 PCIe 输入互斥。文件源按比例填充到 1280×720、转换 RGB565，视频最高 30 fps、静态图约 5 fps；实际显示与推理帧率由处理负载决定。

在板端可使用 `QT_QPA_PLATFORM=offscreen` 做无屏启动检查，但成功启动不能证明识别质量或 PCIe 输入正确。本次没有在 x86 主机上把导入 AArch64 可执行程序当作当前应用运行。

## 5. 旧硬件接口

程序当前访问 `/dev/pango_pci_driver`、`/dev/mem` 与 `/dev/i2c-0`。驱动涉及自定义 ioctl、BAR 映射和 read 返回状态；这不是标准视频设备接口，也不是当前 AVM 核心接口。I2C 地址和寄存器来自旧工程。

[驱动说明](drivers/README.md) 列出了所带二进制与限制；[整体接口状态](../doc/integration-status.md) 对照旧协议和新 AVM 通路。导入记录当时没有 `/dev/pango_pci_driver`，文件回放测试不能证明 PCIe 可用。必须取得驱动源码、匹配设备与协议后才能完成可靠板级集成。

## 6. CPU 测试与视频 E2E

在 Linux（包括有 OpenCV 开发包的 x86_64 WSL）执行：

```bash
bash tests/run_host_tests.sh
```

脚本用 g++17 编译生产源码和现有测试，结果写 `build/host-tests/run.log`：

- `adas_engine_test.cpp`：交通规则、滞回、变道、导航、行人、超速、TTC、复位。
- `plate_text_codec_test.cpp`：5 项字符去重、空白和车牌文本比较。
- `lane_nav_logic_test.cpp`：15 项线型与箭头判定。
- `adas_pipeline_e2e_test.cpp`：提供视频目录时执行现有场景视频测试。

生成视频并执行 E2E（需要 NumPy 和 ffmpeg）：

```bash
python3 -m pip install numpy
python3 tools/gen_adas_test_videos.py --outdir build/test-videos
bash tests/run_host_tests.sh build/test-videos
```

没有 libx264 时可加 `--codec mpeg4`。本机整理验证没有 ffmpeg 命令，使用根 `tools/maintenance/generate_arm_test_videos.py` 将相同场景绘图通过 OpenCV mp4v 编码；此替代方式明确记在验证报告中。合成视频只检查传统视觉和规则链，不能测出真实 RKNN 模型的准确率。

## 7. 模型、转换与诊断

目前加载的文件名及角色见 [模型说明](models/README.md)。`PANGO_PLATE_DET_MODEL` 和 `PANGO_PLATE_REC_MODEL` 可覆盖车牌模型路径；覆盖前应核对 RKNN 输入维度、布局、归一化、输出排列和字符表。文件名 `plate_lprnet.rknn` 不能单独证明其内部就是某一版 LPRNet。

以下工具需要另外准备 ONNX 或外部源码/权重、PyTorch 与 RKNN Toolkit2；这些开发依赖不在应用正常运行依赖中：

```bash
python3 tools/convert_plate_color.py --onnx /path/to/plate_recognition_color.onnx
python3 tools/convert_plate_models_to_rknn.py --lpr-repo /path/to/YOLOv5-LPRNet-source
```

默认转换结果进入 `build/model-conversion/`，不会覆盖已导入模型；覆盖生成结果需 `--overwrite`。两个转换工具来自不同模型路线，输出维度和字符数可能不同，应使用 `tools/rknn_layout_probe.cpp` 在兼容 RKNN 环境核对，再明确部署。本次只验证参数/语法与路径，未重新转换或替换模型。

## 8. 历史与许可

本地历史索引（不随 Git 提交）中的计划、板卡名称、临时路径和计数是当时记录，不作为本次实测结论。生产 C++、UI、模型保持原样；驱动提交副本仅替换非加载调试区中的个人目录，代码、数据和模块元信息均未变。导入的带水印图标保留本地备份，提交使用自绘几何图标。

根 Apache-2.0 LICENSE 是原 FPGA 仓库许可；第三方不能由该文件推定授权。预编译驱动模块标注 GPL v2，当前缺少对应源码；其他依赖见各自说明。本次提交排除了受限 SDK 头文件、RKNN 模型、运行库及原始历史记录，并按用户要求包含经调试路径脱敏的驱动。
