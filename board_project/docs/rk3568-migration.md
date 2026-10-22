# 在其他RK3568设备上编译与运行（2026-10-22）

本工程使用64位AArch64 RKNN运行库，在目标RK3568 Linux本地编译，沿用qmake、Qt5、C++17、OpenCV4；无需GStreamer、VPU或Python训练环境。相同SoC不代表内核、RGA/RKNN BSP、glibc等必然兼容，构建成功与硬件运行分别验证。

## 迁移源码

携带`src/`、`third_party/`、`models/`、`resources/`、默认`config/adas.ini`、qmake工程、三个构建/启动脚本、README和docs。需要测试时加tests/tools；不要复制旧Makefile、对象文件、旧标定和`.codex`备份。

```bash
tar -czf /tmp/adas-source.tar.gz \
  src third_party models resources config/adas.ini drivers .codex/fpga-baseline.sha256 \
  ZYNQ_video_show.pro build.sh build_adas.sh run_adas.sh tests tools README.md docs
```

源码包附带drivers与小型原始校验清单供测试核对，构建不会加载它们。FPGA和驱动保持原有部署，可独立备份；ARM编译不会安装模块、烧录或更改协议。目标设备重新确认相机、路面与ROI，不复用另一台设备的标定。

## 构建

解压到任意专用工程目录，进入后运行：

```bash
./build.sh --check
./build.sh --install-deps          # 缺Ubuntu/Debian开发包时显式安装并构建
./build.sh                        # 默认2任务，增量构建
./build_adas/run_img_recv_demo.sh
```

支持外置输出：

```bash
BUILD_DIR=/tmp/adas-build BUILD_JOBS=2 ./build.sh
cd /tmp
/tmp/adas-build/run_img_recv_demo.sh --quit-after 8 \
  --report /tmp/adas-report.json --screenshot /tmp/adas-display.png
```

`BUILD_DIR`不能覆盖源码、保留程序、系统目录或符号链接路径；`--clean`仅清理具有匹配所有权标记的目录。`--check`不创建输出目录，不要求设备节点存在；默认不会运行sudo/apt。`--no-rga`用于无RGA的诊断/旧模式构建，新ADAS感知明确不可用，不使用CPU图像处理回退。

## 模型与资源

将训练验证完成的新包置于`models/adas/manifest.json`对应目录，或设置`ADAS_MODEL_MANIFEST`绝对路径。包包含两份权重和manifest，检查SDK、SHA256、输入/输出契约和类别；实际权重目前待提供，程序缺模型保持停车。旧四模型保留不代表新ADAS已就绪。步骤见[模型替换](model-replacement-roadmap.md)。

生成的launcher相对定位自己的RKNN库及资源，并返回源码根运行。可从任意当前目录启动；搬动源码后需重新构建资源链接。只复制一个二进制/launcher不能保证运行依赖完整。

## 硬件与现场输入

U盘在电脑打开，通过HDMI输入FPGA，再经原PCIe/DMA进入ARM。ARM不接受`--adas 文件`或文件播放，不引入解码库。开始采集使用原有按钮，视频源、预处理、叠加、白平衡和去雾调用原样保留。

2026-10-22当前环境运行内核4.19.232，已有`drivers/pango_pci_driver_mul.ko`的vermagic为6.1.99，无PCIe设备枚举和pango节点。不要用强制加载规避内核不匹配。使用对应原上板系统/驱动组合验证，原驱动源码与通信接口不在本次修改范围。

## 验证口径

1. `--check`、干净构建和`ldd`只证明构建/用户态依赖。
2. 离屏启动、报表和截图证明程序生命周期/UI，零采集帧不算视频或性能通过。
3. 实际采集需模型能力有效、帧计数增长、坐标/色彩正确、各路数据新鲜。
4. 切源、原有处理按钮、DMA稳定和采集启停须在设备在线后回归。采集线程未退出不能解除映射或复用资源。
5. 完整系统延迟外部测量；ARM内部耗时与确认等待单列，最终版本补30分钟连续运行。

外置输出可使用相同回归入口：

```bash
ADAS_APP_BUILD_DIR=/tmp/adas-build ./tests/run_tests.sh
```

当前仅能用本机换目录/外置输出验证可迁移构建，不将其描述为第二台实体板验证。
