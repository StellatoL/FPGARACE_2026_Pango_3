> 本文是本次精简前的双640部署历史证据。后续已删除ARM车牌/旧交通模式、schema1/2兼容和开发素材；下文路径、兼容性及MainWindow冻结描述仅指当时版本。历史工件和日志已归档，当前树与重新验证结果以[清理说明](project-cleanup-2026-10-28.md)为准。当前两个RKNN文件及FPGA通信字节保持不变。

# 双640模型部署与验证（2026-10-28）

## 当前结果

默认`models/adas/manifest.json`已切换schema3双640，程序已重新编译，可直接运行：

```bash
./build.sh
./run_adas.sh --mode adas
```

官方Rockchip YOLOv5n负责主人员/车辆；原`models/traffic_yolov5.rknn`负责灯色、斑马线及辅助过街人员。传统车道和手动导航/强制操作保留。四段新增视频按HDMI单路整幅画布验证，共352个采样帧、704次真实模型执行，推理失败及候选溢出均为0。相同采样下，旧960八类方案有153个失败帧。

该结果是部署、输出解码和规则集成的验证，不是模型准确率、真实HDMI/PCIe或完整实时性能验收。旧交通模型仍有误检和漏检，以下单独记录。

## 模型来源和实际契约

官方来源固定到[Rockchip Model Zoo YOLOv5 v2.3.2](https://github.com/airockchip/rknn_model_zoo/tree/bad6c7334531becaf90a561988519b7bec34d0ab/examples/yolov5)，commit `bad6c7334531becaf90a561988519b7bec34d0ab`。明确下载README中的`yolov5n.onnx`，未采用下载脚本默认的s_relu。使用当前Toolkit2 2.3.2，按官方mean0/std255、RK3568、INT8、20张官方COCO subset校准转换；converter仅适配本机DATASET_PATH。

| 工件 | SHA256 |
|---|---|
| 官方n ONNX | `b3237d0f59a5d90ff6ecec3dca220e0ada8abd929f8a158b08b12e1a3bad7232` |
| vehicle.rknn | `566c70942faccfcd854c4eab164d4dd1ff92b1aecbcdcd356f56605ff6fca131` |
| traffic.rknn／原traffic_yolov5.rknn | `62296449866eefb1391015cce7592007bd0fb35a6062cc510c72953bd4dd7db4` |
| 默认manifest | `6620348c73c286335606955f2dd6bc959136c2fbec2e7d363a6b4e3c6cc29da5` |

| 角色 | 输入与真实输出 | 接入范围 |
|---|---|---|
| vehicle | 640×640 RGB/NHWC；C255 NCHW、80/40/20，已sigmoid | COCO IDs 0/1/2/3/5/7：person、bicycle、car、motorcycle、bus、truck；主人员/车辆跟踪 |
| traffic | 640×640 RGB/NHWC；C33 NCHW、80/40/20，raw logits | crosswalk/person/light_red/light_green/light_black/light_yellow，保留原自定义anchors |

两个输入均UINT8 NHWC、`pass_through=0`；runtime应用mean0/std255，输出`want_float=1`自动反量化。vehicle概率不能再做sigmoid，traffic raw头做一次sigmoid。两者使用黑色padding0；历史combined八类路径保持原padding114及schema1/2兼容。原交通实现的默认补边已核对为黑色，不能继续沿用此前padding114的假设。

COCO在完整80类中取最佳类后才筛选，未选类别不能降级映射为车辆。动物`zebra`不是斑马线，COCO `traffic light`不能代替灯色。class-aware NMS阈值0.45、score0.25，保留有限性、概率边界和300个NMS后输出的失败保护；没有提高安全阈值、伪造有效空检测或用top-K掩盖输出饱和。新共享NMS是0.45，不能称为旧交通解码器0.40的逐行复刻。

官方n的ONNX checker、真实NPU属性、六组相同生产RGA输入字节的ONNX/NPU对照已通过。六组18个NPU头均有限且在[0,1]；MAE约0.00194–0.00257、cosine约0.99791–0.99926。量化存在个别元素差异，没有将数值接近当成mAP。完整来源、校准、转换命令、许可证及失败记录见[模型目录](../models/yolov5n-coco-640-2026-10-27/README.md)和本机证据`official-model/REPORT.md`。

## 程序与安全规则

一个NPU线程常驻两个上下文，实际任务在所有视角间严格Vehicle→Traffic交替；未开始任务不推进轮次。每个角色维护独立的视角时钟，前视及请求侧优先。修复了启动时所有未处理视角deadline相同时，请求左侧仍排在右/后侧之后的问题，新增左右两种启动顺序测试。

vehicle是主人员和车辆的唯一跟踪来源。traffic人员使用独立辅助标识12，只用于过街判定和显示，不重复更新主轨迹、主确认或人员数据年龄。Crosswalk、Signal和主人员/车辆的队列、缓存、状态、序号与时间戳独立；COCO空观测不能清除交通过街急刹，交通空观测也不能清除主人员危险。紧急状态只能由相应来源的新有效观测解除。侧后视traffic结果不能控制前视灯色。

当前配置不含锥桶、路栏、掉落物识别。roadblock明确保持Missing；由经过校验的schema3选择人员+车辆能力范围，旧八类默认仍要求完整三类障碍来源。四路变道继续要求目标侧人员/车辆全部有效、新鲜、晚于请求且未见相关阻挡；失败、未知、过期均否决。没有放宽线型、确认次数、500ms/1.5s时效或请求后观测条件。单路不伪造侧视结果。

原摄像头/HDMI切源及FPGA预处理按钮、通信、DMA、驱动、MainWindow和上板FPGA工程未修改。现场仍为PC播放图片/视频→HDMI→FPGA→PCIe→ARM；未添加生产文件播放器。离线工具只复用自有RGB565缓冲进行诊断，不读写DMA。

## 四段新增视频前后对照

完整读取到EOF，3Hz源媒体采样，99/99/99/55帧。前后视频SHA、源帧号、媒体时钟相同；逐帧车道结果去除处理耗时后完全相同。全部输入为`HDMI_SINGLE`，前视crop为`[0,0,1280,720]`，两个模型都实际映射到640×640中的`[0,140,640,360]`，不是四路裁剪。人工命令时刻保持一致，没有用时间轴生成灯色、线型或目标。

| 视频 | 采样帧 | 旧960失败帧 | 双640失败帧 | vehicle候选峰值／NMS后峰值 | traffic候选峰值／NMS后峰值 |
|---|---:|---:|---:|---:|---:|
| 雨天路口 `1a591…` | 99 | 23 | 0 | 91／7 | 43／5 |
| 白天高速 `3b0a3…` | 99 | 14 | 0 | 37／3 | 9／1 |
| 收费站/廊道 `48c10…` | 99 | 61 | 0 | 132／10 | 24／2 |
| 夜桥 `f4931…` | 55 | 55 | 0 | 69／6 | 0／0 |

旧对照为已保留的`media-after-fast-nms`完整张量/精确NMS版本，不是早先1024候选截断版。两方案模型及能力范围不同，因此失败帧差异不能解释为准确率提升，也不直接比较旧八类与新COCO的原始框数。

人工查看各段第20采样帧：新n能框出夜桥车辆、高速前车、收费站车辆；雨天交通模型输出红灯和斑马线。仍观察到跨车辆子类重叠框，传统车道对复杂/片头场景会拒绝为未知。高速段交通模型还有6个斑马线框及1个绿灯框，需要逐框核查，不作为正确识别证据。没有为提高授权率而降低线型置信度、替换模型输出或关闭安全判断。

## 行人跟踪与静态手动操作

复用已下载并保存来源/哈希的公开CAVIAR行人视频，前120个10Hz采样，分别自然运行和每5帧人工删除当次person检测，两次均240次真实NPU执行、零失败。自然运行记录127个人员检测、26个预测观测、6次短时预测后同编号恢复；删除实验记录38个预测观测、20次同编号恢复。后者是人为丢检测，不是模型自然漏检。没有真值身份标注，不能称为MOTA或身份证明；这些画面为室内行人，仅验证检测/跟踪，不是道路避障准确率。

官方bus静态画布实际检测3个人、1辆bus并触发危险走廊急刹。只摄取一次静态观测，各模型执行一次，5条手动命令产生6次决策重评估，不重复推理或累加确认，也不从图片产生速度。

另用已下载的三张真实交通灯图片测试旧交通模型：`red_mingzhu_road.jpg`输出红灯；`red_nakamizo.jpg`与`green_huaxin.jpg`本次全画布640输入均未给出有效灯色，路口保持未知停等。不能据此宣称真实素材红停绿行已全面验收。红停、绿行、人工停车保持、用户强制红灯行驶报警、实虚线/强制变道、导航冲突等规则通过独立结构化及Qt测试验证；不能将规则注入算成模型识别成功。

## 构建、回归与迁移

| 检查 | 实际结果 |
|---|---|
| 本工程qmake增量全量依赖重建、调度修正后重建 | 退出0，默认`build_adas`更新，`build_release`保留 |
| `ADAS_TEST_GUI=0 tests/run_tests.sh` | 退出0；含旧回归、新manifest/运行时/调度/适配/决策、跟踪20项、Python 26项 |
| Qt界面回归 | 28通过、0失败；含6项双模型新增测试；旧控制器RED有5个预期失败 |
| `tests/build_script_test.sh` | 11/11通过 |
| 真实RGA | RGB565颜色、带步长、四区域、960旧兼容、双640黑补边通过 |
| 保护清单 | 717/717一致；含FPGA、通信、原模型、MainWindow、驱动、保留程序 |
| 本板换目录原生干净编译 | 新源码路径`relocated source`、独立输出`external output`，退出0；732.69s |
| 同步最终调度修正 | 仅header变化，触发pipeline重建，退出0；46.57s |
| 迁移后从`/tmp`启动launcher | 退出0，双模型能力范围3、独立Crosswalk启用；源码与默认模型与本工程一致 |
| 默认配置正常启动及真实NPU | 默认路径校验与离屏启动通过；真实NPU冒烟记录单独保存 |

迁移在当前RK3568进行，没有连接第二台设备。构建树未携带旧对象，输出目录含空格，launcher从自己的位置定位库和模型。`build.sh`无需训练Python环境，也不加载驱动或修改BSP；另一板的内核、glibc、RGA/NPU兼容性仍须现场核对。迁移步骤见[指南](rk3568-migration.md)。

## 性能及未验收项

四视频采样期间同时存在编译负载。包括RGA、取输出拷贝与后处理的vehicle平均约82.1–83.1ms，traffic约66.7–69.1ms；两者串行约149–152ms。离线整帧分析均值约239–261ms，另有文件解码开销。官方n单独SDK API计时更低，二者计时范围不同。

前视每角色8Hz、常态其余每角色2Hz是调度上限/预算，不是已达到的频率。按本次两角色耗时，单路8对/s约需1.2s计算预算，四路14对/s约需2.1s，不能认为实时目标已满足。需要实际输入负载下继续测量并优化模型或调度成本，保持原安全条件；不能用3Hz媒体采样或离线工具耗时替代预览FPS及收帧到HUD延迟。

当前无`/dev/pango*`。未验证真实HDMI/PCIe启停、切源、四路实流、FPGA按钮回归、端到端延迟或30分钟稳定性；这些都不记为通过。首次独立direct-RGA helper曾出现一次间歇性堆错误，随后生产RGA路径及本轮测试通过，仍保留失败记录，不宣称已确定或根治原因。没有实例GT，未计算mAP、召回率、误检率或MOTA。

## 激活、回退与证据

2026-10-28将两个已验证权重复制到`models/adas`，最后替换manifest；旧默认目录先逐文件验证完整备份，再移除默认目录中不再引用的960 `scene.rknn`。原始PT/960候选和`models/traffic_yolov5.rknn`均保留。

旧默认备份：`.codex/verification/dual640-2026-10-27/previous-default`。旧scene SHA为`138ea72a84dd39e16bec969894d47026b54dae9c1473dfc350afc29c743d8031`，旧manifest SHA为`1e118b168e213fccd90a69b71ec32fc621466a7917741be3d3b3bbb347ce9371`。如需检查历史兼容，关闭程序后显式指定该备份manifest；它仍有旧960误检问题。原验证程序`build_release`和硬件文件无需回滚。

本机完整证据位于`.codex/verification/dual640-2026-10-27/`，目录名沿用任务开始日期，实际执行日期见UTC日志：

- `official-model/`：官方来源、20图校准、转换、属性、相同字节ONNX/NPU、独立包校验。
- `tests.log`、`tests/`、`runtime-{red,green}.log`、`contract-audit/`、`core-audit/`、`ui-audit/`、`diagnostic-audit/`：软件、SDK边界替身、Qt及工具回归；替身不算真实NPU。
- `rga-test.log`、`media-dual640/`、`comparison.json`、`static-bus/`、`extra-scenes/`：真实RGA/NPU、原视频及静态结果。
- `migration*.json`、`migration*-build.log`、`migration-ui.*`：换目录源码哈希、构建与启动。
- `activation.json`、`active-ui.*`、`active-default-npu/`、`protected.log`：默认激活、启动、真实模型及保护文件。

未提交或推送本轮修改到GitHub；本次请求未要求发布。
