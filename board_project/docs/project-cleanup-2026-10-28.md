# 工程精简与验证（2026-10-28）

最终只提供ADAS和纯视频。按用户明确要求，删除ARM车牌识别、旧交通模式及对应UI/线程/模型、旧8类/960/神经网络车道兼容、视觉导航与估速、未用文件源、旧训练转换工具和ztu依赖。双640模型、传统车道、短时跟踪、手动导航、四路侧向安全判断及必要回归工具保留。

FPGA原车牌定位框按钮属于硬件处理，继续保留，与删除的ARM文字识别不同。FPGA源/IP/约束/主bitstream、驱动和通信源未改。当前两个RKNN也未改。清理前后467项保留的硬件/模型关键文件SHA一致，清单存于工程外归档minimal-protected-files.json。四项通信/驱动基线可运行：

```bash
sha256sum -c .codex/fpga-baseline.sha256
```

## 删除范围与恢复

工程外归档为`/home/cat/ADAS_archive/cleanup-2026-10-28`，不参与构建运行，也不提交GitHub。

第一轮`project/`移走3,850个文件、1,528,075,696字节：旧PT/960/重复候选、400图校准、日志、回滚包、编译及PDS缓存，逐项SHA见archive-manifest.json。后续按最新要求，`minimal-project/`另移走165个文件、146,994,995字节：车牌/旧交通程序、四个根目录旧模型、旧build_release、官方n的ONNX/转换/校准素材、测试媒体、历史工具/测试/文档；逐项SHA见minimal-removal-manifest.json。原源码备份在minimal-*和dual-runtime-diagnostic，旧文档在before-docs.tar.gz。

用户测试视频project_video.mp4和新增四段视频均移出运行工程；离线诊断时可显式引用归档或工程外媒体。生产仍为电脑播放→HDMI→FPGA→PCIe→ARM，无ARM文件播放器。

原目录约1.95GB（包含当时的Git发布副本），精简后约75MB（构建对象另计）。这是工程目录减重，归档仍占磁盘。Git发布副本移至`/home/cat/ADAS_publish/FPGARACE_2026_Pango_3`。

恢复时关闭应用，将需要的文件从归档按原相对路径复制到独立目录，并核对相应SHA清单；不要用旧工程覆盖当前源码。当前程序不接受旧schema1/2，历史模型应配合历史程序检查。驱动`drivers/pango_pci_driver_mul.ko`明确纳入版本控制。迁移见[RK3568指南](rk3568-migration.md)。

## 精简后验证

在同一RK3568的独立发布目录，以当前源码执行以下验证，均exit0：

- `BUILD_JOBS=2 ./build.sh --clean`干净编译；仅原硬件源的未使用参数告警，硬件源未修改。
- 全部CPU/线程/规则/跟踪/模型契约/解码/调度/真实SDK边界替身回归通过；Qt界面29项通过、0失败；build.sh构建安全与迁移脚本11/11通过。
- 真实RGA颜色、非紧密stride、裁剪、四视角、640黑边及信号ROI通过。RGA1.9.2在原BSP兼容模式运行，未升级系统库/驱动。
- 默认包实际RGA/NPU冒烟：公开bus.jpg按HDMI整幅前视输入，Vehicle及Traffic各运行一次，0失败/0溢出；主模型输出3个人、1辆bus，交通辅助人员单独记录。静态图片仅ingest一次，4个手动命令仅重新求值；不是mAP或实流性能测试。
- 从`/tmp`启动ADAS/raw正常退出；旧plate/traffic命令返回2。二进制无PlateRecognizer、PlateInferThread、TrafficSceneDetector、FileVideoSource、decode_neural_lane符号。
- 当前`/home/cat/ADAS/build_adas`程序已替换为新编译版本，`./run_adas.sh`启动通过。源码/resource定位不依赖工程外归档。
- 四项通信/驱动基线、467项保留硬件/模型SHA、3,850+165项归档SHA全部核对通过；独立静态审查无重要问题。

完整日志在工程外归档的final-build.log、final-tests.log、final-gui.log、final-build-script.log、final-rga.log、final-real-model/和final-launch/。发布到GitHub的源码入口为board_project，驱动与当前双模型均纳入版本控制。历史四视频结果见[双640记录](dual640-verification-2026-10-28.md)，不能代替本次回归或准确率证据。

本机无PCIe设备节点，真实HDMI/四路输入、FPGA按钮上板行为、完整延迟和30分钟稳定性未重新验收。没有放宽线型、观测时效或侧向安全条件。
