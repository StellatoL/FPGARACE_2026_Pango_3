# 单路规则与四路隔离验证

后续车道几何精修与构建迁移见 [车道优化验证记录](lane-refinement-verification.md)。本页保留首轮单路规则实现及其程序版本的历史证据，最新程序、识别对比和性能以新记录为准。

日期：2026-10-18。范围为单路道路规则、道路几何及模式隔离；当前所有RKNN权重均为临时权重。本轮不训练或替换模型、不升级SDK，不修改FPGA、驱动或原通信控制。

## 功能变化

- `EgoState.input_layout` 默认四路；纯C++引擎只在单路输入下免除侧视观测约束。Qt传递输入布局，未新增模式控件或“赛题演示模式”文案。
- A/D或视觉导航提出方向请求，确认虚线才能变道；实线保持直行；J/L强制实线变道报警。未知不授权。
- 单路保留前视Traffic、Geometry有效性、红灯、人工停车、暂停、危险及最外侧车道限制；不制造假侧视数据。前视Vehicle继续沿用原低频观测及已知风险保持，并未新增500ms内必须有效的门控；其缺失不能当作前方无车证明。
- 四路保留目标侧Traffic/Vehicle及已配置Roadblock的有效性、时间和障碍门控。仅“放大前视”不改变该规则。
- 几何观测中断超过前视有效期后，视频线型/导航重新确认；停车或前视不可用时，不再提示“允许模拟变道”。

## 保护与回退

本轮修改前源码及程序快照：[before-source-and-program.tar.gz](../.codex/verification/single-view-2026-10-18/before-source-and-program.tar.gz)。保存的程序SHA256为 `ed1f20a7374cab4c19ec52c9decd163d1e540989591187351709c41a54c7fa9d`。另有246项受保护清单，覆盖FPGA相关源码/约束、硬件文件、驱动、4个临时模型与原build_release。

原始上板基线、硬件hash及2026-10-16性能记录继续保留，见 [原验证记录](verification.md)。本轮新的程序不能直接沿用旧程序的30分钟性能声明；PCIe实流仍需设备在线验证。

最终246项受保护文件全部校验一致，见 [校验输出](../.codex/verification/single-view-2026-10-18/protected-after.log)；`mainwindow.cpp/.h`也与本轮快照一致。当前程序SHA256为 `f8b2e75cb10e60874ecd1ad47375540831a887523c3d7fd5aeb56db22463dfad`，源码、测试、诊断工具、程序和原视频的 [验收清单](../.codex/verification/single-view-2026-10-18/accepted-program-source.sha256) 已保存。

## 测试证据

- [GUI RED](../.codex/verification/single-view-2026-10-18/ui-red.log)：旧程序7项通过/1项失败，单路前视/虚线有效仍未得到变道授权；使用原有API复现，非新字段编译错误。
- [引擎RED](../.codex/verification/single-view-2026-10-18/engine-red.log)：旧确认票跨观测中断，恢复首帧即变道。
- [核心GREEN](../.codex/verification/single-view-2026-10-18/engine-green.log)：新单路、导航、安全优先、确认/过期和原四路规则通过。
- [GUI GREEN](../.codex/verification/single-view-2026-10-18/ui-green.log)：8项通过/0失败，覆盖单路→四路→单路、切四路后重新提供当前代次前视/虚线、旧代次拒收、放大前视不免侧视、视觉导航和无新增模式按钮。
- 道路几何的 [窄线RED](../.codex/verification/single-view-2026-10-18/geometry-red.log)、[连通身份RED](../.codex/verification/single-view-2026-10-18/geometry-red-connected.log) 与 [几何GREEN](../.codex/verification/single-view-2026-10-18/geometry-green.log) 已保留；新增窄透视/候选污染/弯曲连通实线/亮背景回归，原导航及米制估速仍为0失败。
- [最终构建](../.codex/verification/single-view-2026-10-18/final-build.log) 与独立的 [完整回归](../.codex/verification/single-view-2026-10-18/final-tests.log) 均正常退出0；帧池、核心规则、几何/米制估速、调度/跟踪、文件回放和车牌回归通过，GUI为8项通过/0失败。
- 一次组合构建/测试会话收到SIGTERM(143)，原因未确定；[中断日志](../.codex/verification/single-view-2026-10-18/final-tests-interrupted.log) 保留，不计作完整通过。上述完整回归为之后单独重跑的正常结束结果，未忽略失败。各项退出状态见 [final-checks.json](../.codex/verification/single-view-2026-10-18/final-checks.json)。

## 用户提供的视频

原文件 `video_example/project_video.mp4` 为1280×720、25fps、50.4秒H.264视频。抽查画面是高速道路，包含黄实线、白虚线及车辆，没有导航叠层；不能将其记为真实导航视频测试。

直接BGR的固定5Hz采样252帧：修改前左实/虚/未知为166/7/79，右0/1/251；修改后左221/0/31，右1/77/174。固定人工样本中5/10/20秒右虚线判定相符，0秒仍漏为Unknown，24秒真实右虚线误判为Solid；亮路面、阴影、遮挡和远处短段仍有覆盖限制。见 [人工核查图](../.codex/verification/single-view-2026-10-18/geometry-manual-samples.png) 及 [几何诊断汇总](../.codex/verification/single-view-2026-10-18/geometry-summary.json)。这不是逐帧标注的准确率统计。

另使用相同诊断工具、相同RGB565往返精度对比旧/新几何（规则均为本轮单路规则，Traffic为测试替身）：

| 输出分布，252个采样 | 旧几何 | 新几何 |
|---|---|---|
| 左实/虚/未知 | 160/5/87 | 222/0/30 |
| 右实/虚/未知 | 1/0/251 | 3/67/182 |
| 仅几何＋规则右变道允许次数 | 0 | 33 |

原始数据位于 [diagnostic-before](../.codex/verification/single-view-2026-10-18/diagnostic-before/summary.json) 与 [diagnostic-after](../.codex/verification/single-view-2026-10-18/diagnostic-after/summary.json)。RGB565下23.2/23.8/24秒有右实线输出，应作为已知疑似误判保留；不能把33次允许当作实际模型的变道次数。原视频没有导航，诊断却出现5次分散的背景形状导航检测，这些不是正确导航识别证据；实际导航需要正确ROI、代表性素材和后续模型/算法验证。

ffprobe报告1260视频帧，独立计数解码同样1260；本机OpenCV诊断两次均读到1257帧即结束，末尾3帧差异尚未修复。本轮没有改解码器，不将读到EOF称为逐帧完整解码验收。

### 实际应用交互回放

使用本轮构建对象链接的 [交互驱动](../.codex/verification/single-view-2026-10-18/real-video-driver.cpp)，操作原有转向/强制变道控件，真实加载并运行临时Traffic及Vehicle模型；没有注入检测观测。原视频未改写，单路、循环回放约54.91秒后正常退出0，覆盖一次原片及循环恢复。每秒记录JSON，四个时刻截取界面。该次是offscreen功能验证，不是独立性能基准或实车安全证明。

| 请求阶段，按应用时间留出切换余量 | 1Hz采样结果 |
|---|---|
| 3–14秒右请求 | 11个采样均保持直行，车道证据不足 |
| 15–25秒左请求 | 10个采样均保持直行；7次实线、3次未知，不授权 |
| 26–35秒强制左请求 | 9个采样均判左实线、保持直行并报警 |
| 36–51秒右请求 | 15个采样中1次确认虚线并授权右变道，其余14次证据不足 |

证据：[阶段汇总](../.codex/verification/single-view-2026-10-18/real-video/phase-summary.json)、[右变道授权采样38](../.codex/verification/single-view-2026-10-18/real-video/sample-38.json)、[强制实线报警画面](../.codex/verification/single-view-2026-10-18/real-video/ui-31.png)、[最终应用报告](../.codex/verification/single-view-2026-10-18/real-video/report.json)。这证明原视频可以触发完整规则链路，但授权零散，不能称为稳定的实虚线识别。线段拟合还可能选到邻道或不合理的左右组合，当前未知和误分类比例仍需后续改进。

本次短回放平均输入24.73fps、呈现23.05fps、CPU约10.00Hz、前视Traffic约8.98Hz；ARM接收到HUD的前视P95约161.54ms。Vehicle低频约0.66Hz，其接收到HUD的P95约821.05ms，不能将Traffic频率当全部类别实时频率。该次呈现、CPU及车辆刷新仍有性能缺口；它也不能替代四路性能或物理输入到显示输出的延迟测量。

### 导航与静态输入

使用保留的 [导航样例生成代码](../.codex/verification/single-view-2026-10-18/navigation-fixtures.cpp) 生成左/直/右蓝绿箭头和双侧虚线图像，再分别通过实际 `run_adas.sh` 单路入口运行。每个文件只采集1帧，真实临时RKNN参与，没有手动导航或观测注入。三次均正常退出0：视觉导航方向和最终箭头分别为Left/Left、Straight/Straight、Right/Right；左右请求允许变道，直行不发出变道请求。见 [导航汇总](../.codex/verification/single-view-2026-10-18/navigation-summary.json)、[左](../.codex/verification/single-view-2026-10-18/navigation-left/ui.png)、[直](../.codex/verification/single-view-2026-10-18/navigation-straight/ui.png)、[右](../.codex/verification/single-view-2026-10-18/navigation-right/ui.png)。这是可控合成素材的链路验证，真实导航UI准确率仍需独立素材；视频三票、超时重确认和导航实线冲突由核心回归验证。

可重复运行右导航样例：

```bash
./run_adas.sh --adas .codex/verification/single-view-2026-10-18/navigation-fixtures/nav-3.png --single-view
```

单路运行：

```bash
cd /home/cat/ADAS
./run_adas.sh --adas video_example/project_video.mp4 --single-view
```

也可使用既有“四路画布”输入选项选择布局；无需新的模式按钮。先选直行车道，A/D提出方向意图，J/L强制实线变道，空格停车/释放。导航图像应把蓝绿箭头纳入现有可配置ROI；没有导航叠层的道路视频不作为导航正确性验证素材。

## 离线几何诊断

```bash
g++ -std=c++17 -O2 -Isrc tools/evaluate_road_video.cpp \
  src/core/adas_engine.cpp src/core/road_geometry.cpp src/core/ground_calibration.cpp \
  $(pkg-config --cflags --libs opencv4) -o /tmp/evaluate-road-video
/tmp/evaluate-road-video video_example/project_video.mp4 /tmp/adas-road-diagnostic 5
```

该工具输出逐采样CSV与JSON，并执行RGB565往返转换；**不运行RKNN**，用显式标注的有效空Traffic替身隔离几何/规则。输出的变道次数不是实际模型结果或道路安全证明。真实模型回放和该诊断必须分开记录。

模型统一替换、类别映射、INT8/SDK验证、前视全类别优先与侧后抽样方案见 [后续模型路线](model-replacement-roadmap.md)；详细实施步骤见 [实施计划](superpowers/plans/2026-10-18-single-view-adas.md)。
