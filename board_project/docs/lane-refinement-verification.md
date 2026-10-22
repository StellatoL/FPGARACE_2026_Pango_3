# 车道几何优化与RK3568源码迁移验证

开始日期：2026-10-18；最终收尾验证：2026-10-20。输入为原始`video_example/project_video.mp4`；本轮只调整道路几何、诊断和构建脚本，现有RKNN模型仍是临时模型。详细步骤见 [实施计划](superpowers/plans/2026-10-18-lane-refinement.md)，目标板构建步骤见 [迁移说明](rk3568-migration.md)。

## 保护边界

本轮修改前源码/程序保存在 [before-source-program.tar.gz](../.codex/verification/lane-refinement-2026-10-18/before-source-program.tar.gz)，原程序SHA256为`f8b2e75cb10e60874ecd1ad47375540831a887523c3d7fd5aeb56db22463dfad`。最初工程的build.sh另保留在原2026-10-15归档及本轮证据目录，旧脚本含默认清理行为，不建议覆盖当前迁移入口。

246项FPGA/硬件/驱动/模型/原build_release文件受保护，额外冻结ADAS引擎、types、四路调度、mainwindow源/UI和默认配置。单路/四路授权规则、确认票、500ms/1.5s有效期、停车优先和目标侧障碍否决不修改。没有用历史线型填未知，也没有向FPGA增加写入。

## 对照方法

旧算法来自本轮开始时的源码快照；诊断工具相同，分别链接旧/新几何，使用相同RGB565往返精度、stride=5、源帧时间和单线程OpenCV，串行运行。每次解码1257帧、取252帧。ffprobe标称1260帧，现有OpenCV末尾少3帧的既有差异仍保留；原文件不改写。

工具`tools/evaluate_road_video.cpp`输出线型、原始像素端点、置信及处理耗时。其Traffic观测是显式有效空替身，不运行RKNN；授权计数不作精度指标。实际模型回放另行记录。

固定人工样本为9个开发时刻（0/5/10/20/23.4/24/30/40/48秒）及3个首轮保留时刻（8/18/44秒），在查看修正输出前保存 [标记及口径](../.codex/verification/lane-refinement-2026-10-18/manual-annotations.json) 和hash。每帧标3个实际可见漆段点，容差32原始像素；位置全部匹配且类型为Dashed才正确。Unknown/无效留在分母；不是只计算有效输出的平均误差。部分亮路面帧仅能标远处可见点，不能将其称为近区全范围精度。

这些标记是同一段视频的稀疏人工核查，数量少、时序相关，不能代替独立数据集准确率；该片没有真实右实线，真实实线与假虚线风险还需额外道路素材。保留样本首轮为0/3正确；后续精修仍使用既定开发帧，保留集只作同片固定核查，不宣称跨场景泛化。

复查当前版本可在工程根目录编译独立诊断工具，不访问FPGA或NPU：

```bash
g++ -std=c++17 -O2 -Isrc tools/evaluate_road_video.cpp \
  src/core/road_geometry.cpp src/core/ground_calibration.cpp src/core/adas_engine.cpp \
  $(pkg-config --cflags --libs opencv4) -o /tmp/evaluate-road
/tmp/evaluate-road video_example/project_video.mp4 /tmp/road-after 5 0,125,600
```

旧版源码及本轮对照工具/CSV保存在证据目录。旧/新比较须用同一工具、RGB565精度和帧列表，不能拿不同采样的统计相减。

## 算法与回归

- 白色候选结合局部灰度对比，减少亮水泥与白线的整片连接；黄色独立保留。原始窄颜色连通身份用于阻止连续实线因对比变化被切成多个虚线段。
- Solid证据按完整近处可见道路统计，长孤立虚线不能用自己的Hough短区间作为分母；分离漆段不合并成连续实线。
- 候选按本车左右位置、最近边界和双边相容性选择；仅有更外侧线时不能拿它代替当前缺失边界。限制候选数量，绘制起点收在实际证据范围，减少向山坡/消失点以上外推。
- Dashed仍要求三个独立漆段与两个间隙；同一物理漆段的强像素碎片不能重复计数。完整安全决策保持原字节。
- 三段必须有一致明确的主色。中间版本在615/1050帧把白色景物和黄色实线混采为Dashed，此回归已记录并用颜色门控拒绝，不以当时增加的虚线输出作为完成证据。
- 原图核查确认0/125/250帧的第三条白漆有4–6个原始支持行、方向差6.6–11.6°、细长比3.66–4.17，缩至480后只剩一行。新增当前帧局部原图补证：仅对已有两个白色身份的已选Unknown边界，保持几何不变；三原始身份中至少两个分别关联粗图锚点，均需足够支持行、方向差≤15°、细长比≥3，并保持间隙、跨度、占有率。完整矩形ROI先建weak/raw连通身份再筛带，接触ROI边缘的身份拒绝，不能把裁断实线拼成虚线。每ROI≤16万像素、每帧最多两侧；组件>1024或超面积拒绝。

[初始RED](../.codex/verification/lane-refinement-2026-10-18/geometry-red.log)为5项失败，覆盖亮水泥、长单段假Solid、邻道抢占及缺失替代等；首轮几何回归为0失败。首轮串行真实视频右Unknown/Solid/Dashed由182/3/67变为177/0/75，人工共同正确由1/12变为4/12，位置错误2变0，但Unknown改善有限，因此继续检查采样。

诊断发现水平采样带在陡透视线处对应的法向容差更小。960宽诊断未解决问题，未提高全局分析尺寸。曾试验允许一行压缩短段，开发帧出现反光钉干扰，该方法撤回，不将其GREEN或输出增加记为最终改进。强弯实验仍允许保守Unknown，不以放宽条件强求Dashed。

原图补证正例有单独 [RED](../.codex/verification/lane-refinement-2026-10-18/geometry-native-red.log) 与 [GREEN](../.codex/verification/lane-refinement-2026-10-18/geometry-native-green.log)。该路径检查原图组件形状，拒绝测试的短横反光块；粗图路径仍存在“高亮4行原始短横矩形缩为两行后冒充第三漆段”的合成反例，已在 [初始记录](../.codex/verification/lane-refinement-2026-10-18/geometry-native-initial-red.log) 留痕。本轮未扩大到所有粗图组件的方向分类，因此不能概括为已排除全部反光钉假虚线。真实道路实线、复杂弯道与相机姿态变化仍需更多独立素材。

最终源码SHA256为`6a9a905accc46ce3bd924be851e46b526164c7bae768baf89796b3135dfa9e01`（road_geometry.cpp）。最终`./build_adas.sh`及`./tests/run_tests.sh`正常退出0：帧池、决策、几何/导航/米制估速、车牌、四路跟踪/调度/去重、文件源和GUI均通过，GUI为8通过/0失败/0跳过。日志：[构建](../.codex/verification/lane-refinement-2026-10-18/final-build.log)、[完整回归](../.codex/verification/lane-refinement-2026-10-18/final-tests.log)；测试产物`/tmp/adas-tests.arctr8`。Qt5旧头的deprecated-copy及既有RGA兼容提示仍出现，未升级SDK或驱动。

## 最终同帧对比

最终串行结果见 [旧版](../.codex/verification/lane-refinement-2026-10-18/accepted-before/summary.json)、[新版](../.codex/verification/lane-refinement-2026-10-18/accepted-after/summary.json) 和 [人工标记比较](../.codex/verification/lane-refinement-2026-10-18/accepted-comparison.json)。两次帧号、媒体时间和1257解码帧数完全对应。

| 指标 | 改前 | 改后 |
|---|---:|---:|
| 全片252采样：右Unknown | 182 | 155 |
| 全片252采样：右Solid输出 | 3 | 0 |
| 全片252采样：右Dashed输出 | 67 | 97 |
| 全片252采样：左Unknown / Solid / Dashed | 30 / 222 / 0 | 25 / 227 / 0 |
| 人工12样本：线型及全部标点共同正确 | 1 / 12 | 7 / 12 |
| 人工12样本：未知/无效 | 10 | 5 |
| 人工12样本：虚线误判Solid | 1 | 0 |
| 人工12样本：拟合位置超32px容差 | 2 | 0 |
| 开发9样本共同正确 | 1 / 9 | 5 / 9 |
| 固定保留3样本共同正确 | 0 / 3 | 2 / 3 |
| 几何耗时均值 | 60.61ms | 77.01ms |
| 几何耗时P95 | 67.64ms | 95.40ms |

全片分布不是准确率。右Unknown仍有155/252，人工样本仍漏掉5/12，问题只是改善，并未全部解决。已有有效拟合点的平均误差22.52→6.38px只是条件统计，不能移除未知后称为全片精度。615/1050帧的左侧黄实线回归均为Unknown，没有再被混色证据误授权为虚线。该拒绝也意味着相应时刻不能保证“实线强变道”报警所需的实线识别证据，不能把Unknown算成Solid识别成功。

2026-10-20最终叠加图审查修正了双边在消失点上方交叉绘制的问题：共同起点取两边证据起点和交点下方，沿原直线截取，不改变拟合或线型。[新增RED](../.codex/verification/lane-refinement-2026-10-18/geometry-extent-red.log)复现1失败，[GREEN](../.codex/verification/lane-refinement-2026-10-18/geometry-extent-green.log)为0失败。再次逐帧核对252个采样，线型、有效性和底部坐标与截取前一致，193个双边有效样本的顶部均有序；见 [核对结果](../.codex/verification/lane-refinement-2026-10-18/extent-invariant-check.json)。截取前证据保存在`pre-extent/`，未冒充最终程序结果。

以下左列改前、右列改后；上排5秒由Unknown恢复Dashed，下排24秒由错误Solid改为保守Unknown并纠正位置。绿色Dashed、黄色Solid、灰色Unknown。

![同一原视频帧的旧新几何叠加](../.codex/verification/lane-refinement-2026-10-18/comparison.png)

## 最终真实模型交互回放

最终程序SHA256为`4f0ff9a82a422ed00b635fc659ce8342ce6e026a80e02f5764ab0f56232ba30f`。使用现有临时Traffic和Vehicle RKNN实际推理，未注入空观测。程序约54.01秒正常退出0，采集1337帧/呈现1248帧，池丢弃0；约第51秒自动循环一次，来源代次3→4，所以采集数超过原视频单次1257帧。未标定时速度保持无效。

交互仍使用已有控件：2秒请求右变道、14秒请求左变道、25秒强制左变道、35秒取消强制并请求右变道。1Hz采样结果如下，只证明这次操作闭环，不是识别准确率或授权成功率：

| 采样秒数 | 实际观测 |
|---|---|
| 3–13，右请求 | 1次允许模拟变道，10次车道信息不足 |
| 15–24，左请求 | 6次实线保持、3次车道信息不足、1次前视不可用，均未授权 |
| 26–34，强制左 | 9次报警，均未授权；其中3次前视不可用 |
| 36–49，右请求 | 4次允许、9次车道信息不足、1次前视不可用 |

该片没有导航图层，不能用它证明导航识别准确率；既有导航图形回归保持通过，未修改导航算法。记录：[实际推理日志](../.codex/verification/lane-refinement-2026-10-18/real-video-runtime.log)、[最终报告](../.codex/verification/lane-refinement-2026-10-18/real-video/report.json)、[逐秒交互汇总](../.codex/verification/lane-refinement-2026-10-18/real-video/interaction-summary.json)、[报警截图](../.codex/verification/lane-refinement-2026-10-18/real-video/ui-31.png)。

这次单路25fps文件输入平均采集24.75fps、呈现23.11fps；CPU几何8.85Hz、前视Traffic 9.04Hz、Vehicle 0.65Hz。ARM收帧回调到Qt更新P95为Traffic 163.77ms、几何214.48ms、Vehicle 569.46ms，均不含相机/FPGA/显示扫描和事件确认。视觉证据更严格但处理成本上升；CPU15Hz目标仍未达到，车辆时效仍不足。没有将单路短回放等同四路性能验收，也未沿用旧版30分钟稳定性结论；本轮未重跑30分钟或外部端到端测试。

## 构建与迁移

`build.sh`统一输出到build_adas，默认增量/2任务；显式`--install-deps`才安装依赖，预检AArch64 64位用户态、Qt5、OpenCV4、C++17及RKNN ELF/用户态库。RGA可选，切换编译配置会重建旧对象；`--clean`只接受本脚本标记的安全目录，含尾斜杠的符号链接也拒绝。build_release保留。

构建脚本 [11项回归](../.codex/verification/lane-refinement-2026-10-18/build-script-final.log) 通过，覆盖预检无副作用、平台/依赖、资产/SDK、增量、清理安全、外置目录和显式安装。旧RGA开关残留及尾斜杠保护均有RED与修复记录；符号链接目标有合法marker，避免因其他条件拒绝造成假通过。测试夹具早期遗漏sudo边界曾误触发一次apt-get update，立即终止自有测试子进程；未执行apt install，系统包日志未变化，后续用sudo/apt边界替身隔离。

异路径源码目录为`/tmp/adas-rk3568-migration.atHAME/source`，输出为同层`output`。从原工程之外执行`build.sh --no-rga`，不携带旧对象/Makefile/相机标定；初次干净构建正常退出0。外置launcher从`/tmp`启动，清除原模型/动态库覆盖变量，使用复制到新目录的视频。

首轮干净构建和实际临时RKNN回放正常退出0，采集325帧/呈现195帧，证明不依赖旧对象的路径与能力；当时并发编译，不作性能基准。初始日志：[构建](../.codex/verification/lane-refinement-2026-10-18/migration-build.log)、[运行](../.codex/verification/lane-refinement-2026-10-18/migration-runtime.log)。

2026-10-20收尾时此前临时目录已被系统清理，重新在`/tmp/adas-rk3568-migration.PIrCkG/source`复制源码、模型与默认配置，输出为同层`output`；未携带旧对象/Makefile/标定。最终冻结版本通过`build.sh --no-rga`干净构建。外置launcher从`/tmp`启动，约13.23秒正常退出0，采集318帧/呈现293帧；`input`指向新源码内视频、`mode=3`、`input_layout=single`、Traffic/Vehicle均ready且分别实际处理110/8次。QSS/视频/控件截图正常，ldd无notfound，RKNN库解析到外置output，未回用原绝对路径。日志：[最终构建](../.codex/verification/lane-refinement-2026-10-18/migration-final-build.log)、[最终运行](../.codex/verification/lane-refinement-2026-10-18/migration-final-runtime.log)、[报告](../.codex/verification/lane-refinement-2026-10-18/migration-final-report.json)、[截图](../.codex/verification/lane-refinement-2026-10-18/migration-final-ui.png)、[ldd](../.codex/verification/lane-refinement-2026-10-18/migration-final-ldd.log)。无RGA版hash为`3b0cdaaee3df598ded0451d845ffade69956e4eec11dbbdd14c4d7e7daa1adf7`；与本机默认启用可选RGA的二进制不同，几何源码一致。

最终 [源码冻结清单](../.codex/verification/lane-refinement-2026-10-18/final-source-freeze.sha256)、246项保护清单、安全模块及人工标记hash全部一致，原视频hash也一致。源码快照比对确认本轮应用代码仅改road_geometry.cpp，未改Qt、决策、四路调度或硬件。构建脚本11/11及比较口径2项测试也通过。

用户明确只要求build.sh在另一设备可用，未提供目标板；因此本轮不声称另一块RK3568实机运行通过。RKNN与目标BSP驱动兼容、相机重标定、HDMI输出和FPGA实流仍需目标板核对。本机仍无`/dev/pango_pci_driver`，不安装或升级驱动/SDK。

## 回退

原`build_release`及最初工程归档保留。若需仅回退本轮车道算法，先将本页开头的`before-source-program.tar.gz`解压到独立目录核对，再恢复其中的`src/core/road_geometry.cpp`和对应`tests/road_geometry_test.cpp`，用当前`build.sh`重新编译；迁移入口可以保留。归档包含本轮之前的程序，但不包含完整模型/运行库资产，不能将归档中的可执行文件单独视为可迁移安装包。
