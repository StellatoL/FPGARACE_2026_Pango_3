# 双640检测模型接入与替换

本轮主线为 **schema3：官方YOLOv5n COCO80 + 原交通6类**，默认包位于[`models/adas`](../models/adas/README.md)。已于2026-10-28复制到默认`models/adas`并启用；原默认960包完整备份。构建、软件/Qt回归、真实RGA/NPU、四段单路视频及本板换目录编译结果见[双640验证记录](dual640-verification-2026-10-28.md)；没有把它们计作准确率或HDMI/PCIe实流验收。

仅接受schema3双640。旧schema1/2、八类/960、神经网络车道的兼容代码及开发素材已移除；历史工件保存在工程外，运行不依赖它们。

## 1. 模型角色与能力范围

| 角色 | 工件与契约 | 应用职责 |
|---|---|---|
| vehicle | 官方Rockchip YOLOv5n；COCO80原类序；3个C255 NCHW sigmoid头 | 只选person、bicycle、car、motorcycle、bus、truck，生成主人员/车辆观测与跟踪 |
| traffic | 原`models/traffic_yolov5.rknn`逐字节副本；6类；3个C33 NCHW raw头 | 斑马线、辅助过街人员、红绿黑黄灯；前视可灯控，侧后视只发布斑马线分量 |
| lane | `lane:null` | 传统CPU车道独立线程；不部署车道占位权重 |

traffic精确类序为`crosswalk, person, light_red, light_green, light_black, light_yellow`，对应原模型zebra/person/red/green/black/yellow的相同索引。交通person只参与斑马线风险和屏显，不写入主人员/车辆跟踪、不重复确认计票，也不刷新主观测的数据年龄。交通灯、斑马线和主障碍观测分别保留自己的时钟；COCO空帧不清除交通危险。

vehicle只接入COCO IDs `[0,1,2,3,5,7]`。先在完整80类中取每anchor最佳类，再筛选；不把其他获胜类重映射为车辆。COCO `zebra`是动物，不能当作斑马线；COCO `traffic light`也不能提供红绿灯色。

本配置没有道路障碍模型，roadblock保持Missing。`requiredCombined`仅指person/vehicle两分量有效新鲜，不要求或伪装有效空路障。四路变道仍要求目标侧person/vehicle均晚于转向请求，未知、失败、过期或阻挡均否决。不能声称能检测锥桶、路栏或掉落物。

## 2. 固定输入、输出和锚框

两角色均为RK3568 INT8、640×640 RGB8/NHWC、mean `[0,0,0]` / std `[255,255,255]`、黑色letterbox padding0，SDK前缀`2.3.2 (`。ARM提交UINT8 NHWC、`pass_through=0`，由runtime执行已配置的归一化；`want_float=1`读取反量化输出，不手工重复除255或施加错误激活。

ONNX参考输入为float32 RGB/NCHW `[1,3,640,640]` /255。两模型stride均为8/16/32、每头3个anchor，空间维度80/40/20：

| 角色 | 输出契约 | 实际head形状 | 激活 |
|---|---|---|---|
| vehicle | `yolov5_rk_sigmoid_nchw_v1` | `[1,255,80,80]`, `[1,255,40,40]`, `[1,255,20,20]` | 图中已sigmoid，decoder不能重复sigmoid |
| traffic | `yolov5_raw_nchw_v1` | `[1,33,80,80]`, `[1,33,40,40]`, `[1,33,20,20]` | raw logits，decoder sigmoid一次 |

通道均按anchor拼接`x,y,w,h,objectness,classes...`。score为objectness×最佳类概率，阈值0.25、按类别NMS IoU0.45；保留固定张量自然上界及300个NMS后输出保护，非有限、实质概率越界或真正输出饱和均失败，不能截断后宣称成功。

标准vehicle anchors：

```text
[10,13,16,30,33,23]
[30,61,62,45,59,119]
[116,90,156,198,373,326]
```

原traffic自定义anchors，必须保持精确顺序和值：

```text
[5.64062,12.39062,7.85547,18.5,10.71875,28.14062]
[63.5,9.07812,15.92188,41.3125,25.48438,69.9375]
[218.375,16.32812,49.90625,100.3125,612.5,90.0]
```

FPGA传输保持1280×720 RGB565，摄像头四视角各640×360；HDMI为整幅前视。RGA按视角裁剪并等比适配640：完整16:9内容为640×360、上下各140黑色填充。框按实际crop/目标矩形反映射并裁剪到源crop边界；不改变FPGA、DMA或按钮接口。

## 3. 官方vehicle来源与转换

固定[Rockchip Model Zoo v2.3.2 YOLOv5](https://github.com/airockchip/rknn_model_zoo/tree/bad6c7334531becaf90a561988519b7bec34d0ab/examples/yolov5)，commit `bad6c7334531becaf90a561988519b7bec34d0ab`。从该README明确下载`yolov5n.onnx`；下载脚本默认s_relu，不能直接用作n工件。

官方ONNX、转换脚本与校准素材已移出运行工程，来源与哈希保留在本包`provenance.json`和工程外归档。目标板无需这些开发资产。重新转换时在独立开发目录按上述固定官方版本下载`yolov5n.onnx`，使用Toolkit2 2.3.2、RK3568、INT8、mean0/std255和有效校准清单；官方脚本的DATASET_PATH须指向实际校准集。官方默认下载脚本可能选s_relu，应明确核对n的下载链接和ONNX SHA。不要以改名代替模型转换。

traffic沿用原6类RKNN，不重新量化。当前n转换使用20张官方COCO subset校准图；这不等同于独立精度评测。新的训练权重必须按下面的类序、锚框和激活契约导出，否则需要对应实现与测试，不能直接覆盖。

## 4. schema3完整包与替换

```text
adas/
├── manifest.json
├── vehicle.rknn
├── traffic.rknn
├── README.md
├── provenance.json
└── licenses/
```

manifest顶层为`schema:3, target:rk3568, toolkit_version:2.3.2, lane:null`，含version；road_roi为历史元数据，不控制模型裁剪。每role各自声明相对file、SHA256、INT8、runtime_api_prefix、input、output、完整训练classes及3×6 anchors。准确实例见[当前manifest](../models/adas/manifest.json)；字段规则以[neural_contract.cpp](../src/inference/neural_contract.cpp)和[dual_manifest_test.cpp](../tests/dual_manifest_test.cpp)为准。两模型必须是包内不同真实文件，禁止用目录外软链接或只换其中一个文件。

| 工件 | SHA256 |
|---|---|
| 原traffic / 包内traffic | `62296449866eefb1391015cce7592007bd0fb35a6062cc510c72953bd4dd7db4` |
| 官方n ONNX | `b3237d0f59a5d90ff6ecec3dca220e0ada8abd929f8a158b08b12e1a3bad7232` |
| 官方n RKNN / 包内vehicle | `566c70942faccfcd854c4eab164d4dd1ff92b1aecbcdcd356f56605ff6fca131` |

关闭旧程序后可显式验证独立待验证包：

```bash
ADAS_MODEL_MANIFEST="$PWD/models/adas/manifest.json" ./run_adas.sh --mode adas
```

程序默认入口为`models/adas/manifest.json`，本轮已按完整备份→复制两个权重→最后替换manifest的顺序激活，SHA及回退路径见本轮报告。以后替换同样须备份完整包并重启，避免运行时混用版本。不要只改manifest里的尺寸、类序或output字符串来“适配”不兼容权重。

## 5. 调度预算与验收

一个NPU线程常驻两个模型，实际开始的任务在所有视角间全局严格V/T交替；某role尚无到期新帧时保持轮次，不用另一role重复调用填满空档。每个role各自前视上限8Hz；四路常态另三视角各2Hz，有意图时目标侧4Hz、另两侧后视各1Hz。单路各role只处理整幅前视、上限8Hz。双模型不另叠加旧4Hz灯色ROI任务；CPU车道独立目标15Hz。

这些是目标预算，不是实测性能。令包含RGA、取输出和解码的完整V/T耗时分别为Tv/Tt，四路目标粗预算为`14*(Tv+Tt)` ms/s，单路为`8*(Tv+Tt)` ms/s，还需CPU/线程/显示余量。实测须记录实际role交替、每role每view有效Hz、失败、结果年龄、预览、P95和30分钟稳定性；不能把源视频3Hz抽样写成实时吞吐。

本轮验证分别记录：文件契约、真实RGA/NPU、同字节ONNX/NPU数值、原视频感知、结构化规则、干净迁移构建与真实HDMI/PCIe实流。缺少逐框GT不声称mAP/召回率；0采集帧启动不算实流通过。实际通过项及模型漏检/误检限制见[本轮报告](dual640-verification-2026-10-28.md)；迁移步骤见[RK3568指南](rk3568-migration.md)。
