# RK3568迁移与构建

运行需要64位AArch64 Linux、C++17、Qt5/qmake、OpenCV4、系统RGA和兼容的RKNN/NPU BSP。当前工程仅ADAS和纯视频模式；不需要Torch、ONNX、Toolkit2、文件解码栈或车牌模型。

## 获取与构建

```bash
git clone git@github.com:StellatoL/FPGARACE_2026_Pango_3.git
cd FPGARACE_2026_Pango_3/board_project
./build.sh --check
./build.sh --install-deps       # 只在Ubuntu/Debian缺依赖时显式使用
./build.sh
./build_adas/run_img_recv_demo.sh
```

默认2个编译任务，可设置`BUILD_JOBS`。`--check`核对平台、工具链、运行库和必需资产，不创建输出或要求PCIe设备在线。驱动包含在`drivers/pango_pci_driver_mul.ko`，但不会被构建脚本加载。

外置构建示例：

```bash
BUILD_DIR=/tmp/adas-build BUILD_JOBS=2 ./build.sh
cd /tmp
/tmp/adas-build/run_img_recv_demo.sh --quit-after 5 --report /tmp/adas-report.json
```

源码可放到新目录，包括带空格的目录；无需修改开发机路径。launcher按自身位置解析运行库与资源链接。移动源码后重新构建，不仅复制二进制；不要带旧对象、Makefile、用户配置或机器标定。复用有本脚本所有权标记的输出可用`--clean`；脚本拒绝清理源码、系统路径、符号链接或别的源码树的输出。

RGA缺失时仍可构建，但ADAS目标检测报告不可用，不回退CPU图像适配。`--no-rga`仅供诊断。系统RGA须支持当前Qt RGB565字节对应的`BGR_565`及自检；相同SoC不保证所有BSP自动兼容。RKNN运行库随工程提供，当前模型要求SDK前缀`2.3.2 (`；不要盲目覆盖系统库/内核驱动。

## 模型与配置

完整携带`models/adas/`，两个权重均在包内，以相对路径、SHA、类序、输出形状和SDK校验。仅支持schema3双640，旧八类/960/车道网络已移除。具体契约见[模型接入说明](model-replacement-roadmap.md)。

`config/adas.ini`为默认配置，标定需在目标设备按实际来源和路面重新执行。来源变化会使旧代次感知失效。正式输入固定PC播放→HDMI→FPGA→PCIe→ARM，或FPGA四摄像头拼接；应用无文件播放源。

## 上板检查

```bash
sha256sum -c .codex/fpga-baseline.sha256
./tests/build_script_test.sh
./tests/run_tests.sh
```

另一块板上先确认内核与模块vermagic、PCIe枚举及设备节点，再验证采集启停、摄像头/HDMI切源、FPGA原按钮、画面颜色、四路位置和退出。不要强制加载版本不匹配的模块。本机无PCIe节点，离屏启动和模型冒烟不能算实流通过。

按真实输入测各role/view有效Hz、结果年龄、丢帧、CPU/内存、ARM收帧到OSD延迟及30分钟稳定性。预览≥25fps、OSD P95≤250ms、危险到报警P95≤500ms仍是待实测目标，必须计入确认窗口。输入到显示总延迟另做外部测量。

本轮在同一RK3568的发布目录重新干净编译并运行；没有第二台设备地址，因此不声称完成第二台实体板验证。结果见[清理与验证](project-cleanup-2026-10-28.md)。
