# ADAS 验证与交付记录

日期：2026-10-16。验证平台为本机 RK3568/aarch64 Linux 4.19.232、Qt 5.12.8、OpenCV 4.2.0、项目自带 RKNN。软件处理和文件回放在本机执行；没有连接可用的 `/dev/pango_pci_driver`。

## 基线保护

- 原源码、测试及 `build_release` 保存在 `.codex/baseline-2026-10-15.tar.gz`。
- `build_release/img_recv_demo` 与备份中的程序 SHA256 相同：`d6af56c75ca85c56ede7f9abe475edc5a21946dd5a8abd168414a8b61ebc151d`。
- `sha256sum -c .codex/fpga-baseline.sha256` 的四项均为 OK：`pango_fun.cpp`、`pango_fun.h`、`pcie_frame_status.h`、驱动 `.ko`。
- 新输出在 `build_adas`。未修改 FPGA 源码、引脚、PDS 工程、寄存器定义或驱动。复用原 I2C 0x02 状态查询进行应用层源身份同步；文件回放不访问 FPGA。
- 原有阻塞 DMA 读取取消、停止标志及槽更新协议保持原样；自有帧复制不能替代硬件退出验证。

## 软件验证

命令：`./build_adas.sh`、`./tests/run_tests.sh`。最终构建与完整回归均退出0，几何测试0失败，GUI 7项通过/0失败。证据为 `.codex/verification/build.log`、`tests.log`；61项源码/脚本校验在 `source-sha256.txt`。

| 测试 | 覆盖范围 |
|---|---|
| `frame_pipeline_test` | 自有帧生命周期、DMA 数据随后改写不影响快照、池满非阻塞丢帧、RGB565 色值、四路偏移/步长、实际任务调度 |
| `adas_engine_test` | 红黄停/绿行、强行驶报警、人工停车保持、实虚线/未知/外侧车道、侧视不可用/失败/过期、视角隔离、危险清除、TTC、速度失效和手动模式 |
| 同一引擎测试 | 同帧不同组件、重复/倒序/旧代次拒收，交通/车辆/路障独立时间戳和风险清除；导航只按新帧确认；离开绿灯路口恢复路段约束 |
| `road_geometry_test` | 四点凸性与单位、保存/加载/来源尺寸、斜线实虚分类/横纹干扰、导航形状、已知米制运动/深度/旋转/静止/动态遮罩/不规则时间、失效原因 |
| `adas_pipeline_test` | 使用 NPU 替身，实际线程与调度；四路/类别轨迹隔离、慢处理与待处理覆盖、切代次、先发交通后慢车辆、全景车辆框裁回局部坐标、两组件动态遮罩独立过期 |
| `file_video_source_test` | 实际本地解码/自有缓冲、静态只发一次、60fps 素材按媒体时间采样、暂停/循环/定位/恢复代次、错误与停止退出 |
| `adas_ui_test` | 1024×600、快捷键编辑保护、静态帧模式切换/只计一次、错误/EOF 代次、暂停后换文件恢复、标定期间换布局拒绝保存 |
| `plate_text_codec_test` | 保留原车牌字符编解码回归 |

工作线程替身测试证明并发/坐标/接口行为，不能证明 RKNN 准确率。路障测试只向纯软件观测接口注入 `components=Roadblock`、`class_id=11`，生产路径没有测试目标或真实路障模型。

最终优化版定量地平面序列的末帧平滑估计（工作图保持原生坐标映射）：

| 理论速度 | 估计速度 |
|---|---|
| 5.4 km/h | 5.2446 km/h |
| 10.8 km/h | 10.8747 km/h |
| 18 km/h | 18.0182 km/h |

缺车头偏移但直线运动：10.8→10.8177 km/h。已测偏移纯旋转：0.0958935 km/h；未测偏移而明显旋转时返回无效。新增 1280×720 非等比例原生输入同样通过米制坐标回归。此为已知透视、平面和纹理的定量测试，不代表真实相机标定误差或道路速度精度。

修复前后证据保存在 `.codex/verification/`：`roadblock-red/green.log`、`navigation-red/green.log`、`junction-red/green.log`、`expired-red/green.log`、`ui-review-red.log` 与最终 `tests.log`。异步 ingest 使用当前单调时间拒绝过期样本的确认作用；过期绿灯不能解除红灯、过期空目标不能清除危险，当前边界未知时旧虚线票数不能授权变道。构建可能包含 Qt 5.12/第三方库的弃用警告，按退出状态判定。

## 实际模型与性能

模型文件校验记录：`.codex/verification/model-hashes.sha256`。最终程序SHA256为 `ed1f20a7374cab4c19ec52c9decd163d1e540989591187351709c41a54c7fa9d`。

真实 RKNN 冒烟：`performance-accepted-60s/report.json` 的 traffic_ready、vehicle_ready 均为true；`plate-accepted-smoke/report.json` 的 legacy_model_ready 为true且legacy_error为空。均正常退出0，证明现有模型加载及推理/后处理可执行，合成输入的空检测不构成准确率证据。运行时有既有 RGA 驱动兼容模式提示，本轮未升级驱动。新模型、训练与转换延期。

性能调整证据：`geometry-profile-before.log` 对同一前视素材测得未标定 38.74ms、含 LK 91.50ms；`geometry-profile-after.log` 为 33.39/60.46ms。最终保留 480 像素车道/导航与 384 像素光流工作宽度，保持原生坐标及上表精度，尚不宣称达到全部频率目标。

第一次长测在 142.8 秒时为补充过期确认保护而主动停止，记录位于 `stability-interrupted-pre-expiry-fix`，不能计作30分钟通过。最终程序已另行完成下述正式30分钟测试。

60秒无并行编译/其他模型干扰的离屏文件测量（开启测试标定和完整 LK）：

| 指标 | 目标 | 本轮测量 |
|---|---|---|
| 30fps 输入预览 | ≥25fps | 输入29.87 / 预览28.30fps，达到本次文件测量目标 |
| 前视 CPU 感知 | 15Hz | 13.67Hz，未达到 |
| 交通 NPU 前/右/后/左 | 8/2/2/2Hz | 5.45/1.32/1.32/1.32Hz，未达到 |
| 车辆更新率，每路 | 记录 | 0.66Hz（一次全景任务分四路） |
| ARM帧就绪至HUD P95 | ≤250ms | 交通前视142.59 / 几何147.48 / 车辆540.64ms；车辆未达到，整体目标未全部达到 |
| 危险出现到报警 P95 | ≤500ms | 缺真实标注视频，未验收 |
| 完整视频输入至显示延迟 | 外部测量 | 无 PCIe 实流/外部设备，未验收 |
| 连续运行 | 30分钟 | 最终程序文件回放1800.31秒、正常退出0，见下文；PCIe实流仍待验 |

详细原始结果位于 `performance-accepted-60s/`。该次捕获1809帧、呈现1714个不同源帧，帧池丢弃0，NPU/CPU待处理覆盖1237/963；采样RSS峰值293844kB，进程退出0。预览计数不包含对同一帧的重复重绘。`*_ready`表示最近模型能力状态；`*_age_ms=-1`表示当前代次尚无该组件，循环边界或退出时可出现，不能将ready直接当作当前可用。

正式30分钟测试于北京时间2026-10-16 19:34:49至20:04:50执行，同一最终程序、离屏四宫格文件循环、测试标定与完整LK开启。应用统计1800.31秒，监控器包含启动退出共1801.65秒，正常退出0、没有触发超时。原始证据：[应用报告](../.codex/verification/stability-30min/report.json)、[监控记录](../.codex/verification/stability-30min/monitor.json)、[每5秒采样](../.codex/verification/stability-30min/samples.jsonl)、[采样汇总](../.codex/verification/stability-30min/sample-summary.json)、[退出界面](../.codex/verification/stability-30min/ui.png)。

| 30分钟指标 | 实测 |
|---|---|
| 收帧 / 呈现不同源帧 | 54069 / 51056 |
| 平均输入 / 预览 | 30.03 / 28.36fps |
| CPU / 交通NPU前右后左 | 14.01Hz / 5.51、1.33、1.33、1.33Hz |
| 车辆更新率，各路 | 约0.81Hz |
| ARM帧就绪至HUD P95 | 交通前视149.82 / 几何146.78 / 车辆532.63ms |
| 帧池丢弃 / NPU、CPU待处理覆盖 | 0 / 36516、28415 |
| 360次采样RSS及观察到的VmHWM峰值 | 299068kB，约292.06MiB |
| 第5至10分钟RSS范围 / 最后5分钟RSS范围 | 298540–299068kB / 299068–299068kB |
| 30秒后采样线程数 / 最高采样温度 | 始终4 / 53.75℃ |

该结果通过本次文件回放稳定性检查；CPU/NPU频率和车辆延迟仍有未达项。测试标定有效、两类RKNN模型可执行；素材不满足真实地面运动假设，因此速度保持无效，未将其当作速度或识别准确率验收。采样没有覆盖退出截图瞬间的所有内存变化，也不证明NPU/驱动全部内存没有泄漏。仅为该临时素材创建的标定已从活动配置中移除，证据副本保留在测试目录中。

车辆 FP16 模型比六类交通模型耗时长，因此采用独立低频全景任务，每 14 个交通任务执行一次（首任务也执行），先发布交通结果；全景车辆推理的有效视角分辨率低于逐路输入。界面及 JSON 分别记录两类模型的更新率、原始采样年龄和延迟。过期车辆结果会被隐藏并阻止侧向授权，不能用缓存刷新时间伪造实时性。

当前交通任务约71ms、车辆任务约393ms；仅14次交通任务每秒已需要约994ms的串行预算，再追加车辆任务无法达到原频率目标。前视车辆结果也可能越过默认500ms有效期，其确认作用会被拒绝，已知危险保持到新的有效清除观测。模型量化/替换已延期，后续需在模型预算和真实道路精度约束下再优化，不能把目前车辆视频碰撞时延记为通过。

复现性能/稳定性：

```bash
g++ -std=c++17 -O2 tools/generate_adas_fixture.cpp \
  $(pkg-config --cflags --libs opencv4) -o /tmp/adas-fixture-gen
/tmp/adas-fixture-gen /tmp/adas-fixtures
# 只为上述临时合成文件恢复本次测试标定；已有档案不覆盖
mkdir -p config/calibrations
cp -n .codex/verification/stability-30min/fixture-calibration.yml \
  config/calibrations/c38690580e0a227ef24ee5c9.yml
python3 tools/adas_benchmark.py /tmp/adas-fixtures/mosaic.avi \
  --seconds 1800 --out .codex/verification/stability-30min
```

生成器的 30fps 四宫格是调度、内存和绘制负载素材，不是模型识别准确率素材。需通过 UI 对此素材的路面区域设置测试标定才会启用 LK；测试标定只绑定此临时文件，不能用于真实道路。监控器保存程序 SHA256、退出码、每 5 秒 RSS/峰值 RSS/线程数/CPU 累计时间、系统可用内存/CMA 及温度。RSS 不等于 NPU/驱动全部内存。

应用 JSON 的延迟从 ARM 帧就绪开始，到 GUI 更新 Qt pixmap 为止；本次文件路径在RGB565转换后记就绪时间，PCIe路径在原采集完成回调记时间。不含曝光、FPGA/PCIe前段、文件解码前段、屏幕扫描和多次观测确认。`npu_overwritten/cpu_overwritten` 是最新帧调度中主动覆盖的待处理帧，不能解释为 FPGA 丢包。

## FPGA 参考源码核查与上板待验

详见 [FPGA 核查笔记](fpga-reference-review.md)。实际 bitstream 版本尚未确认：默认 `CAMERA_TEST_PATTERN=4'b1011`，三路色条，唯一物理 CAM3 在左下；四路 RTL 附带文档声明尚未完成其 PDS/上板验证。HDMI 是整幅输入，普通道路素材需选单路，四宫格素材需自行预拼接。

拼接帧可重复读取某一路 DDR 最新旧帧；现有协议没有各路曝光编号/心跳。ARM 的结果年龄只能证明收到并处理了拼接帧，不能证明各摄像头仍更新。视觉速度也受接收时间与实际曝光节奏差异影响。

设备在线后必须完成：

- 核对实际烧录版本、测试图参数、四路独立物理接线及前/右/后/左位置。
- 采集启停至少 20 次、四槽 RGB565 色序/步长/无闪烁，确认慢处理时仍为完整自有帧。
- GUI 与实体按键双向切源，确认 0x02 查询、来源标定/ROI 切换、过渡帧丢弃和无旧代次覆盖；查询失败时标定停用。
- 原有 FPGA 预处理开关，以及 HDMI 强制绕过预处理的行为。
- 阻塞读取退出后才解除映射；若不能按时退出，确认应用拒绝关闭/切源并保留资源。
- 分别遮挡/停帧/断开各路，记录重复曝光与协议诊断限制。
- 给定四类真实场景、夜间/逆光/遮挡测试，按实际标注统计准确率、漏报/误报与危险确认延迟。
- 用外部计时方法测输入至显示延迟，与 ARM 内部 JSON 分别报告。

以上均为待验项，不因软件回放通过而自动通过。程序、配置操作、演示步骤及回滚命令见 [README](../README.md)。
