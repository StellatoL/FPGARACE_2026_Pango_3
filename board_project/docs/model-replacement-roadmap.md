# 两个ADAS模型的训练、转换与部署接入规范

版本：2026-10-25。当前检测模型采用airockchip/yolov5的8类`yolov5_rk_sigmoid_nchw_v1`；车道沿用`lane_rows_4x96_v1`。检测模型已实际转换为RK3568 INT8并打包到`models/adas`，车道权重仍待提供。导航改为手动输入，不要求黄灯或导航检测类别。转换成功不表示赛题识别精度通过。

**按本文完成导出、INT8转换、打包及板端验收后，可通过模型包路径接入，无需修改FPGA通信或ARM后处理。直接接入的前提是输出语义也符合约定；仅文件名、形状或转换成功不能证明兼容，更不能证明识别精度。**

## 1. 交付总表

| 项目 | 模型A：统一目标检测 | 模型B：轻量车道 |
|---|---|---|
| 建议结构 | airockchip/yolov5 YOLOv5n、ReLU、`--rknpu`检测头 | 本工程MobileNetV2-0.35及四个任务头 |
| 包内名称 | `scene.rknn` | `lane.rknn` |
| 固定batch | 1 | 1 |
| ONNX输入 | `images`，float32，NCHW `[1,3,640,640]`，0..1 | `rgb`，float32，NCHW `[1,3,256,512]`，0..1 |
| ARM提交给RKNN的输入 | uint8，NHWC `[1,640,640,3]`，RGB，0..255 | uint8，NHWC `[1,256,512,3]`，RGB，0..255 |
| RKNN量化 | RK3568 INT8 | RK3568 INT8 |
| ARM输入适配 | RGA裁剪、等比缩放、114填充 | RGA道路ROI裁剪，直接缩放到512×256 |
| 输出 | 3个已sigmoid的NCHW检测头 | 4个固定名称、固定顺序的结构化张量 |
| 运行范围 | 前/右/后/左分别推理；前视灯色ROI复用同一模型 | 只处理前视 |
| 需要随权重交付 | `scene.json`，含真实类别顺序和真实像素锚点 | `lane.json`，含训练时的道路ROI |

框架可以采用满足相同语义和性能要求的其他轻量主干，但现有车道导出脚本只接受本工程`LaneNetwork`检查点；其他实现需由训练方导出同样的ONNX接口。普通YOLOv8/YOLO11、分割掩码、UFLD原版列分类输出均不能直接替换现有输出契约。

完整双模型包包含下列三件文件；当前仅检测模型时，省略lane.rknn，manifest使用schema2且显式`lane: null`。不允许创建空文件或使用旧模型占位：

```text
adas-v1/
├── manifest.json
├── scene.rknn
└── lane.rknn
```

训练交付归档另需保留：原始检查点、两份ONNX、两份JSON metadata、训练类别表/标注规范、转换环境版本与日志、校准集清单和独立测试报告。所有文件应对应同一训练版本。

## 2. 模型A的八类和检测头

### 2.1 与实际权重一致的类别

使用[8类数据配置](model-contract/scene-8-classes.yaml)。本次检查点的真实顺序如下，训练标签、检查点、scene.json和manifest必须一致，不能仅修改JSON来重排模型输出。

| class_id | 精确名称 | 对象 |
|---:|---|---|
| 0 | `person` | 行人，需覆盖横穿、远处和遮挡 |
| 1 | `vehicle` | 参与避障的车辆，细分归并规则由训练集统一 |
| 2 | `crosswalk` | 整组斑马线区域 |
| 3 | `light_red` | 有效红灯面/信号单元 |
| 4 | `light_green` | 有效绿灯面/信号单元 |
| 5 | `cone` | 交通锥 |
| 6 | `barrier` | 路栏、水马、施工隔离 |
| 7 | `debris` | 路面掉落物；需真实正负例和独立指标 |

加载器要求至少这8个精确名称且无重复，总数8..64；本次官方RK导出器严格验收8类。内部障碍编号为person→1、vehicle→10、cone/barrier/debris→11，不能与训练class_id混淆。黄灯及nav_left/nav_straight/nav_right不再是必需项；保留的旧raw张量兼容路径不会启用视觉导航。导航只接收手动方向，N键依次关闭→左→直行→右→关闭；路段变道仍受同一车道与障碍安全判断约束。

### 2.2 标注和数据要求

- YOLO标签每行为`class_id cx cy width height`，相对原图归一化；斑马线与行人分别标框，保留脚部关系。
- 四宫格先分离四视角，混合单路HDMI、侧后视、空场景和灯色ROI样本。不要将拼接全图当成一幅前视训练图。
- 灯色框中心需要稳定落入对应车道灯ROI；提供广告灯、路灯、刹车灯等负例。三组灯ROI默认相同，仅适用共用灯；独立车道灯必须按素材设置。
- 覆盖夜间、强光、遮挡、远行人、小灯、RGB565与实际FPGA处理状态；按完整路线划分训练、验证、独立测试，避免相邻帧泄漏。
- 导航UI、道路箭头和黄灯不添加到本次8类标签。手动导航与感知准确率分别验收。

### 2.3 RK官方输出契约

标识`yolov5_rk_sigmoid_nchw_v1`，batch固定为1，stride依次8/16/32，每头3个锚点。当前C=8：

| 索引 | 本次ONNX/RKNN输出名 | 输出形状 |
|---:|---|---|
| 0 | `output0` | `[1,39,80,80]` |
| 1 | `286` | `[1,39,40,40]` |
| 2 | `288` | `[1,39,20,20]` |

输出名可能随上游图编号变化；检测输出按形状与顺序强校验。通道按3个锚点依次拼接，各锚点为`x,y,w,h,objectness,class_0,...,class_7`。**所有通道已经sigmoid，ARM不重复sigmoid**。不接受NHWC头、5D置换、`[1,25200,13]`、已解码框或内置NMS输出。

```text
cx = (x*2 - 0.5 + grid_x) * stride
cy = (y*2 - 0.5 + grid_y) * stride
w  = (w*2)^2 * anchor_w
h  = (h*2)^2 * anchor_h
score = objectness * max(class_probability)
```

所有概率须有限且位于[0,1]。乘积阈值0.25，按类别NMS IoU0.45，候选上限1024、输出上限300，超限判失败。低于阈值的空检测与模型不可用有不同状态；不会因类别精度偏弱降低阈值或放松变道条件。

锚点来自实际检查点，单位为640输入图像素，按头与锚点顺序排列。当前为`[[10,13,16,30,33,23],[30,61,62,45,59,119],[116,90,156,198,373,326]]`。换权重后重新提取，不把网格单位或旧autoanchor结果写入metadata。

原`yolov5_raw_nchw_v1`只用于显式声明的sigmoid前logits兼容模型，ARM按其契约做一次sigmoid；严禁仅改output名称来混用两种输出。`export_models.py scene`保留该高级兼容入口，本次权重必须使用下一节的`export_rk_scene.py`官方路径。

## 3. 模型B的候选、标注与输出

### 3.1 四个候选的身份固定

| 候选索引 | 身份 |
|---:|---|
| 0 | 本车左边界外侧的下一条边界 |
| 1 | **本车所在车道的左边界** |
| 2 | **本车所在车道的右边界** |
| 3 | 本车右边界外侧的下一条边界 |

缺失本车边界时保持该槽位缺失，不能将外侧邻道线补入1或2，也不能将剩余线从左到右重新紧凑编号。实际决策取1和2，0和3不补位。标注的本车车道是图像内自车所在道路走廊，不随UI的左/直/右路口车道模拟按键重新编号。

本版本不预测线色，也不输出道路分割图。双线/复合线须按本车实际需跨越的边界标注：包含禁止跨越实线的边界不得标为虚线；无法确定所属边界或线型时为未知。

### 3.2 道路ROI、行锚点与坐标

默认`road_roi=[0,0.42,1,0.58]`，含义为前视原始图内归一化的`[x,y,width,height]`，不是左上右下坐标。ROI必须全部位于0..1。

图像先取前视，再按ROI裁剪，最后**直接拉伸**到宽512、高256；不做车道输入letterbox。选择ROI应使两种前视尺寸下的偶数裁剪宽高均至少2像素，并覆盖足够真实道路支持行。训练、量化和部署必须使用同一个ROI。更换ROI必须重新生成训练/量化输入和`lane.json`，不能仅改manifest后继续使用原权重。

部署裁剪坐标按偶数对齐，令前视尺寸为W×H：

```text
crop_x = floor(roi_x * W) 向下对齐到偶数
crop_y = floor(roi_y * H) 向下对齐到偶数
crop_w = min(W-crop_x, max(2, floor(roi_w * W)向下对齐到偶数))
crop_h = min(H-crop_y, max(2, floor(roi_h * H)向下对齐到偶数))
```

默认ROI在640×360前视为`(0,150,640,208)`；在1280×720前视为`(0,302,1280,416)`。不能简单假定裁剪到画面最后一个像素；标签应相对于这个实际裁剪矩形。

每条候选有96行，r从0到95，按ROI顶部到底部等距排列，连续坐标约定：

```text
Y_original(r) = crop_y + crop_h * r / 95
X_original(c,r) = crop_x + crop_w * x[c,r]
```

`x=0/1`表示ROI的左右边界，`r=0/95`表示上下边界。不要改为整幅原图的归一化x，也不要将行顺序倒置或改用UFLD数据集的非均匀行锚点。

### 3.3 训练JSONL格式

每行是一张原始前视图的记录；图像路径相对于JSONL所在目录。字段如下：

| 字段 | 形状/类型 | 含义 |
|---|---|---|
| `image` | 字符串 | 原始前视图；不是已经裁剪/缩放的网络输入 |
| `x` | `[4][96]`浮点数 | 相对于实际ROI的边界横坐标0..1；未知/无法确定几何位置为-1 |
| `paint` | `[4][96]`整数 | 0未知/遮挡/不确定，1真实可见漆段，2有证据的无漆间隙 |
| `type` | `[4]`整数 | 0未知，1实线，2虚线 |

`lane_present`没有单独的训练标签字段；当前训练脚本以`x>=0`作为几何有效标签。**已知虚线间隙**可以利用其相邻漆段确定同一边界的几何位置，此时x应为有效值、paint为2；遮挡或证据不足的位置不能这样推断，应x=-1、paint=0。若把全部虚线间隙都写成x=-1，运行时就没有足够的有效间隙证据，仍会判未知。

不能将整条插值曲线都标paint=1；不能将阴影、车辆遮挡、磨损不确定区域、斑马线横纹标成虚线间隙。公开数据集通常只有曲线点，必须补齐漆段/间隙和实虚线标注。端点、转弯、分叉或邻道身份不明时保留未知。

若做水平翻转，需同时逆转候选槽位`0↔3、1↔2`并令有效x变为`1-x`；paint/type跟随候选移动，行顺序不变。改变视角或几何增广后必须重算ROI内标签。

训练脚本先裁剪原始前视，模拟RGB565颜色精度，再双线性缩放512×256和除255。使用`uint16`中间值，当前模拟为R/B五位、G六位再展开至0..255；实际FPGA/RGA颜色和采样误差仍需实流样本校准。

检查点必须包含`model`状态字典和`road_roi`；本工程脚本还保存`contract`及`validation_loss`。只用validation loss选出的检查点仍须做独立线型/位置测试。

### 3.4 四个固定输出

标识：`lane_rows_4x96_v1`，输出索引、名称及秩均必须一致：

| 索引 | 精确名称 | 形状 | 元素数 | 数值含义 |
|---:|---|---|---:|---|
| 0 | `lane_x` | `[1,4,96]` | 384 | **已sigmoid**的ROI横坐标，所有元素均在[0,1] |
| 1 | `lane_present` | `[1,4,96]` | 384 | **已sigmoid**的几何有效概率，所有元素均在[0,1] |
| 2 | `lane_paint` | `[1,4,96,3]` | 1152 | 三分类原始logits，最后一维为未知/漆段/间隙 |
| 3 | `lane_type` | `[1,4,3]` | 12 | 三分类原始logits，最后一维为未知/实线/虚线 |

数值必须有限。无效几何仍输出合法范围x并用present抑制；**-1只用于训练标签，不得出现在网络的lane_x输出中**。最后两项不在导出图里做softmax，ARM统一做softmax及取最大类。前两项不能省略sigmoid，不能因为元素数相同就交换它们。

内存按列出的维度连续展开：`x[c*96+r]`，`paint[(c*96+r)*3+k]`，`type[c*3+k]`。不接受转置后的`[1,96,4]`或`[1,4,3,96]`。转换后只允许输出名带可选末尾`:0`；其他重命名、顺序变化、维度压缩会被拒绝。

### 3.5 当前后处理门槛：训练时必须考虑

这些是现有安全条件的记录，本轮不降低它们来提高授权率。模型准确输出也可能因证据不足得到Unknown；不能训练模型虚构可见性来绕过条件。

| 检查 | 现有条件 |
|---|---|
| 单行几何有效 | present≥0.8，x在[0,1] |
| 单行漆段/间隙可信 | 几何有效，且paint最大类softmax概率≥0.8 |
| 单条边界几何成立 | ≥64行几何有效、≥12行可信漆段、最末有效行≥90；数值拟合非退化 |
| 拟合残差 | 所有有效点相对线性拟合最大横向偏差≤ROI宽的0.05 |
| 近端身份 | 左边界x在整幅前视[0.10,0.49)；右边界在(0.51,0.90] |
| 双边界共同支持 | 共同跨度≥32行；底部车道宽占前视宽[0.18,0.85]，上部左右顺序正确 |
| 虚线 | type的虚线概率≥0.85；≥3漆段、≥2间隙，每个计数段连续≥3行；总漆段≥24行、间隙≥6行 |
| 实线 | type的实线概率≥0.85；连续可信漆段最长≥58行、总漆段≥82行、间隙≤6行 |

未知行会切断虚线证据链，虚线段数需在最后一次未知行之后满足条件；未知不会计为无漆间隙。当前是线性数值拟合，复杂弯道残差过大时拒绝。ROI包含过多天空/车头或支持行过少，会因上述条件未知；应在训练前选择并验证真实道路ROI，不能抬高假present或将遮挡改标间隙。外侧候选不补本车候选，过期结果也不补当前证据。

## 4. 统一输入适配与归一化

现场链路仍为`PC播放素材/摄像头 → FPGA → 原PCIe/DMA → ARM`。ARM不播放训练视频或U盘文件；下面的训练、图片变换和量化操作只在开发PC进行。正式ARM基础图像处理仍由原FPGA功能承担，RGA只适配模型输入。

传输画布1280×720 RGB565，四路布局为左上前、右上右、左下后、右下左，每路640×360；HDMI为整幅1280×720前视。检测模型逐路batch1复用，并不接收4通道、4帧batch或将四路画布缩成一张640图。显示前视放大不影响实际四路调度与侧向安全。

检测输入对当前视角/ROI做等比缩放后居中填充到640×640，RGB三个通道填充值均为114。实际适配令源裁剪宽高为cw/ch：

```text
s = min(640/cw,640/ch)
dw = max(2,floor(cw*s)向下对齐到偶数)
dh = max(2,floor(ch*s)向下对齐到偶数)
dst_x = (640-dw)/2 整数除法
dst_y = (640-dh)/2 整数除法
```

640×360与1280×720完整前视最终内容区均为640×360，上下各140像素填充。各车道灯默认ROI`[0,0,1,0.5]`，实际可配置。灯色任务裁剪三组灯ROI的外包矩形，再用同一检测模型推理；视觉导航任务已关闭。训练/量化不能只准备全帧尺度。

ONNX只接受已除255的float32 RGB。RKNN转换设置`mean_values=[[0,0,0]]`、`std_values=[[255,255,255]]`，板端提交RGB uint8，并保持`pass_through=0`，由RKNN执行输入适配。**不要在ARM额外除255；不要在ONNX图里再嵌入一次除255；不要改用BGR或ImageNet mean/std。**

manifest的NHWC描述的是ARM提交接口，不要求RKNN的查询属性也一定NHWC：当前加载器允许同尺寸的NCHW或NHWC输入属性。网络内部INT8、应用输入uint8以及`want_float=1`输出float32是不同层面，不应混为一谈。输出由RKNN反量化，ARM不再次按scale/zp转换。

## 5. 开始导出前固定板端版本

2026-10-22用本工程自带`third_party/rknn/lib/librknnrt.so`及旧模型仅做`rknn_init/query`，实测：

| 项目 | 当前环境 |
|---|---|
| `api_version` | `2.3.2 (429f97ae6b@2025-04-09T09:09:27)` |
| RKNPU `drv_version` | `0.8.2` |
| Linux内核 | `4.19.232` |
| 模型转换目标 | `rk3568` |

这不是新模型的推理/精度通过。查询日志在`.codex/verification/model-contract-2026-10-22/sdk-query.log`。该RKNPU驱动版本与`pango_pci_driver_mul.ko`的PCIe驱动vermagic是两回事，不要混用。

PC端优先建立与上述运行库配套的RKNN-Toolkit2 2.3.2转换环境，使用对应wheel支持的Python版本；记录精确版本、依赖和模型编译日志。相同版本号仍需在板端验证算子/输出；不以“RK3568相同”或“转换器更新”代替兼容性确认。不要自动升级系统NPU驱动、原PCIe驱动或工程运行库来迁就新权重。

`runtime_api_prefix`由**板端实际api_version**取前缀；本环境可填`2.3.2 (`，包含版本后的分隔符。不能填Toolkit版本代替查询值，也不能缩成`2`绕过校验。换RK3568板/BSP时重新查询；用工程launcher实际加载的运行库，不用另一份Python Lite包的版本代替。

不依赖FPGA的查询程序见[附录A](#附录a板端sdk与张量只读查询)。它会加载模型和读取属性，不执行推理；模型输出的语义和真实推理仍需后续验收。

## 6. 在训练PC导出ONNX

在PC或已有转换环境准备airockchip/yolov5、与检查点兼容的PyTorch/torchvision、ONNX环境，固定版本并归档依赖。车道使用本工程`tools/adas_models/lane_model.py`；板卡编译/部署不需要训练环境。

以下命令均从本工程源码根运行，示例绝对路径需换成PC实际路径。

```bash
# YOLOv5训练：在其仓库目录运行；超参数只是起点。
python train.py --img 640 --batch 16 --epochs 150 \
  --data /path/to/ADAS/docs/model-contract/scene-8-classes.yaml \
  --cfg models/yolov5n.yaml --weights yolov5n.pt --name adas_scene
```

训练完成后返回本工程源码根：

```bash
python tools/adas_models/export_rk_scene.py \
  --yolov5-repo /pc/airockchip-yolov5 --weights /pc/training/best.pt \
  --output /pc/export/scene.onnx

python tools/adas_models/train_lane.py \
  --train /dataset/lane/train.jsonl --validation /dataset/lane/validation.jsonl \
  --road-roi 0 0.42 1 0.58 --output /pc/export/lane.pt

python tools/adas_models/export_models.py lane \
  --weights /pc/export/lane.pt --output /pc/export/lane.onnx
```

导出器使用opset12、batch1和固定尺寸，不设置动态轴。RK检测导出器调用未修改的上游`export.py --rknpu`，生成已sigmoid的三头，并验证8类、真实anchors、输出形状及末端Sigmoid算子；车道导出器保存固定输出名字及检查点ROI。生成文件应成对交付：

```text
scene.onnx + scene.json
lane.onnx  + lane.json
```

ONNX先检查结构、输出顺序及相同输入下PyTorch/ONNX数值一致性；图中不得增加CPU图像处理或NMS。若由其他训练工程导出，必须达到同样的契约并提供等价metadata，不能把官方通用export的decoded输出直接交付。

## 7. 校准数据与RKNN-Toolkit2转换

### 7.1 两套独立校准清单

使用实际/近似实际FPGA RGB565成像分布，优先采样真实链路帧。校准图在PC预先处理为**网络最终输入大小**：

- scene：640×640 RGB，含114 padding；混合四视角全帧和灯色ROI外包裁剪，覆盖必需类别与夜间/强光/空场景。
- lane：从原始前视取与训练metadata一致的实际ROI，模拟RGB565分布后直接缩放512×256；不要使用全帧、检测letterbox或已经归一化保存的图。
- 建议从数百张有代表性的样本起步，数量依精度/覆盖评估，不将张数当硬性通过标准。图像文件为正常RGB色彩编码，每个清单一行一个绝对文件路径；清单中不是.mp4、目录或标签文件。
- 校准保存为0..255图像，由转换配置做除255；不要先存0..1图再除255。使用OpenCV制作时需确认其BGR内存与保存/转换的RGB语义一致。

例如`scene-calibration.txt`内容：

```text
/dataset/calib/scene/front_0001.png
/dataset/calib/scene/right_0015.png
/dataset/calib/scene/signal_roi_0023.png
/dataset/calib/scene/navigation_roi_0030.png
```

不要依赖Toolkit对不匹配尺寸图片的隐式拉伸来代替本工程的裁剪与padding。RGB565精度模拟只是近似，最终需对比实际RGA输入的颜色/空间采样结果。

### 7.2 转换命令

使用RKNN-Toolkit2，不能使用面向其他旧平台的RKNN-Toolkit：

```bash
python tools/adas_models/convert_rknn.py \
  --onnx /pc/export/scene.onnx --calibration /dataset/scene-calibration.txt \
  --output /pc/export/scene.rknn

python tools/adas_models/convert_rknn.py \
  --onnx /pc/export/lane.onnx --calibration /dataset/lane-calibration.txt \
  --output /pc/export/lane.rknn
```

脚本固定配置`target_platform='rk3568'`、mean0/std255及`do_quantization=True`。不导出RK3588/RV系列目标，不交付FP16-only、加密/绑定设备或依赖自定义算子插件的模型。当前加载器要求至少一个输入/输出属性显示量化，纯浮点模型会被拒绝；这项检查**不能证明全图INT8或所有算子均在NPU**，训练方仍须审核转换和性能日志，避免大计算回落CPU。

RKNN输出通过普通`RKNN_QUERY_OUTPUT_ATTR`查询及`rknn_outputs_get(want_float=1)`获取，不使用native NC1HWC2内存直接解码。转换器可能改输出名/顺序，必须在板端核查：scene按三层stride，lane按四个精确名字/维度。修正PC导出/转换流程，不能仅修改manifest使不匹配的张量蒙混通过。

对固定独立样本比较PyTorch→ONNX→RKNN：检测框/类别/score、lane_x/present、paint/type概率及最终实虚线结果。特别检查INT8后x/present仍在[0,1]、0.8/0.85附近的置信度变化、红绿灯误检、虚线间隙消失和邻道补位。量化误差允许范围由独立精度报告确认，不能只看余弦相似度或转换返回0。

## 8. Metadata与manifest打包

`scene.json`至少包括以下字段；由导出器生成实际值，不手工复制示例anchors：

| 字段 | 要求 |
|---|---|
| `output` | 当前`yolov5_rk_sigmoid_nchw_v1`；高级兼容路径可显式声明raw格式 |
| `width` / `height` | 640 / 640 |
| `classes` | 与权重完全一致的名称数组，含必需8类，总数≤64 |
| `anchors` | 3×6真实像素锚点 |
| `normalization` | 导出器记录mean `[0,0,0]`、std `[255,255,255]` |

`lane.json`至少包含`output='lane_rows_4x96_v1'`、`width=512`、`height=256`及训练时的`road_roi`；导出器同样记录normalization。打包工具会生成固定mean/std，不能依靠修改metadata使其他归一化模型兼容。

检测权重和metadata必须齐全；车道权重与metadata可成对省略，此时打包器产生schema2的`lane: null`，车道保持Missing、不执行车道NPU、不授权变道。schema1仍要求真实车道模型。不能以空文件、旧模型或伪metadata占位。

在空目录生成新包，不覆盖已有版本：

```bash
python tools/prepare_adas_bundle.py \
  --scene /pc/export/scene.rknn --scene-metadata /pc/export/scene.json \
  --lane /pc/export/lane.rknn --lane-metadata /pc/export/lane.json \
  --version adas-v1 --toolkit-version 2.3.2 \
  --runtime-api-prefix '2.3.2 (' --output /pc/export/adas-v1
```

上述版本参数仅适用于本节实测基线及实际使用该Toolkit的情况；其他设备按查询结果和真实转换版本填写。打包器验证metadata和文件边界，复制实际提供的模型、计算SHA256并生成manifest；**它不解析RKNN内部图，也不证明SDK、张量或精度已通过**。

manifest字段要求：

| 范围 | 字段 | 约束 |
|---|---|---|
| 顶层 | `schema` / `target` | 新包2 / `rk3568`（兼容旧schema1） |
| 顶层 | `version` / `toolkit_version` | 实际非空版本信息，用于归档 |
| 顶层 | `road_roi` | 与lane训练相同的有效`[x,y,w,h]` |
| scene/lane | `file` | 包内相对路径，不含`..`，不经符号链接逃出包目录 |
| scene/lane | `sha256` | 实际模型的64位十六进制SHA256；换权重必须重新打包 |
| scene/lane | `quantization` | `int8` |
| scene/lane | `runtime_api_prefix` | 板端api_version真实前缀 |
| scene/lane | `input` | 固定width/height、`layout='NHWC'`、`format='RGB8'`、mean0/std255 |
| scene/lane | `output` | 各自的固定契约标识 |
| scene | `classes` / `anchors` | 与检测权重一致的顺序和真实锚点 |

每份模型必须非空且≤512MiB，manifest≤64KiB。这些是文件安全上限，不是RK3568推荐模型大小；应以轻量结构、峰值内存和真实耗时评估。

建议对整个包生成外部校验清单并随包传输：

```bash
cd /pc/export/adas-v1
sha256sum manifest.json *.rknn > SHA256SUMS
```

当前仅检测模型的打包示例（不传lane两项）：

```bash
python3 tools/prepare_adas_bundle.py \
  --scene models/adas8-candidate-2026-10-25/scene.rknn \
  --scene-metadata models/adas8-candidate-2026-10-25/scene.json \
  --version adas8-candidate-2026-10-25 --toolkit-version 2.3.2 \
  --runtime-api-prefix '2.3.2 (' --output /path/to/empty/adas8-bundle
```

后续车道训练完成，将真实lane.rknn/lane.json一起重新打成完整版本包。当前采用的校准样本只有project_video.mp4中96帧白天道路；12帧独立抽样仅验证数值接口，不证明八类准确率。必须用覆盖全部类别、视角、灯色ROI和困难光照的代表集重新量化验收。详细哈希和实际测试见[八类验证记录](adas8-verification.md)。

## 9. RK3568部署、替换与回滚

先将完整新包传到目标RK3568的独立版本目录，例如`/opt/adas-models/adas-v1`，该目录需由当前用户可读。不要覆盖旧四模型或模型正在使用的目录；保留旧版本用于回滚。

```bash
# 在目标板新包目录核对传输完整性
cd /opt/adas-models/adas-v1
sha256sum -c SHA256SUMS

# 先正常关闭当前应用；返回目标板工程根目录
cd /path/to/ADAS
export ADAS_MODEL_MANIFEST=/opt/adas-models/adas-v1/manifest.json
./build_adas/run_img_recv_demo.sh --mode adas
```

权重替换不需要重新编译ARM程序。也可将完整包放到当前工程的`models/adas/`，默认读取`models/adas/manifest.json`；推荐外置版本目录与环境变量，减少误覆盖。`ADAS_MODEL_MANIFEST`是**manifest文件路径**，不是模型目录。shell中设置的环境变量只对从该shell启动的进程生效；桌面/服务启动需在对应入口设置。

没有模型热重载。新增包、更改路径、SDK前缀或权重后必须退出并重启应用，仅切模式/复位不能保证重新加载。切到旧版本manifest并重启即可回滚，不改变FPGA寄存器、驱动或协议。

默认构建需包含RGA，实际系统RGA自检须成功，且未设置`PANGO_DISABLE_RGA=1`。`--no-rga`诊断构建即使权重正确也不启用新版ADAS，不存在CPU图像处理回退。不要将模型不兼容误判成切源指令或传输协议问题。

开始采集仍使用原有按钮；电脑播放U盘素材，通过HDMI送入FPGA，或选择原四摄像头。实际源确认决定单路/四路，显示前视放大不取消侧向安全。

## 10. 接入后的分层验收

### A. 接口验收

1. 包和已配置模型hash一致；查询到目标板SDK、输入/输出数目、形状、名字、格式和量化属性符合本文。
2. 实际RGA自检成功，模型初始化、输入设置、run、输出反量化均无错误。只做query成功还没有执行推理。
3. 使用已知RGB色块/空间标记样本经正式HDMI链路检查通道、裁剪、padding和坐标回映射，再测真实道路样本；测试素材由PC播放。
4. manifest缺失显示未配置；hash、SDK、张量、RGA或推理失败显示失败；成功推理无目标才是有效空。任何能力缺失不得当作侧向无障碍。

### B. 精度与决策验收

- `project_video.mp4`作为固定对比片段，由电脑播放经HDMI输入；另取未参与训练的完整道路片段测试。
- 检测报告逐类precision/recall、mAP及红绿灯混淆，不仅报总mAP；包含远目标、夜间、侧后视、路障和空场景。
- 车道报告本车边界身份、位置误差、实虚线联合正确率和未知率；Unknown计入分母，尤其覆盖右虚线、邻道、斑马线、遮挡和弯道。
- 验证红停绿行/强制闯红报警、虚线安全变道、实线保持/强制变道报警、导航跟随和行人/路障停车。人工停车、灯控、紧急停车按原优先级约束。
- 四路目标侧的人员、车辆和路障观测都必须有效、新鲜且在转向请求之后；前视放大仍应否决缺失侧向证据。不能靠延长TTL或削弱安全条件提高授权率。
- 每次在电脑上暂停、跳转或换素材后手动复位决策；当前传输没有PC媒体事件/PTS，ARM不能自动获得这些信息。视角/路面变化重新标定。

### C. 性能验收

| 模式/任务 | 当前调度目标 |
|---|---|
| 四路常态scene | 前8Hz，右/后/左各2Hz |
| 四路转向scene | 目标侧4Hz，另两个非前视各1Hz；前视8Hz |
| 四路lane | 前视10Hz |
| 单路scene/lane | 各12Hz |
| 前视灯色ROI | 4Hz，复用scene模型 |

令完整一次scene任务含RGA/推理/取输出/解码的耗时为Ts，lane为Tl，粗预算四路`18*Ts+10*Tl`、单路`16*Ts+12*Tl`，单位ms/s。Ts=35ms、Tl=15ms时分别780/740ms/s，仅是估算，线程、拷贝和显示仍需余量。所有任务串行共享NPU，不能拿单模型纯rknn_run的峰值FPS宣称四路同时达标。只调度最新帧，不靠排队掩盖延迟。

同时记录预览fps、每路有效推理频率、失败数、结果年龄、丢帧、内存和30分钟稳定性。前视有效期≤500ms、侧后视≤1.5s。25fps预览、ARM收帧至OSD P95≤250ms、危险至报警P95≤500ms均须实测；4Hz三次确认本身约500ms，不能从总延迟中扣除确认等待。完整输入到显示延迟另用外部测量。

现场采集报告可运行：

```bash
cd /path/to/ADAS
export ADAS_MODEL_MANIFEST=/opt/adas-models/adas-v1/manifest.json
python3 tools/adas_benchmark.py --duration 60 --output /tmp/adas-v1-run-01
```

窗口中仍需点击原采集按钮。脚本零采集帧会失败，但有帧也不自动意味着模型通过。检查`captured_frames>0`、`live_source_known=true`、`traffic_ready/vehicle_ready/roadblock_ready=true`，以及`lane_status/navigation_status/signal_status=1`并核对新鲜度/语义。状态0=未配置、1=有效、2=失败；有效lane张量不代表线型一定已知。报告的`npu_hz_average`等是接收结果次数，可能含失败结果，应结合状态和失败日志评估有效频率，不独立当作性能达标证据。

测试记录表至少包含包版本/hash、runtime/NPU驱动、素材与FPGA开关状态、实际源/ROI、每类指标、未知率、频率、P95及30分钟结论。当前最终权重仍训练中，以上验收保持待办，逻辑测试不能替代真实感知结果。

## 11. 不能直接替换的常见情况

| 现象 | 常见原因/处理 |
|---|---|
| manifest缺失/仍未配置 | 路径不对、环境变量未传给启动进程，或尚未开始采集；模型加载在工作线程首次处理帧时触发 |
| hash错误 | 传输损坏、只换了.rknn未重新打包；完整传包并重新校验 |
| SDK/target不符 | RK3588目标、错误Toolkit或运行库；按固定BSP重新转换，不能改版本字符串绕过 |
| scene头维度错 | 错误C、stride顺序、decoded/5D/NHWC或附带NMS；使用本工程原始头导出流程 |
| lane名字/顺序错 | 转换器重命名、同形张量交换、自动压缩维度；修正PC导出/转换并重测 |
| 框/线位置偏移 | letterbox当拉伸、ROI坐标或偶数裁剪不一致、x使用整图坐标、行顺序倒置 |
| 颜色/置信度异常 | BGR/RGB混淆、重复除255、ImageNet归一化、量化集缺少实际FPGA样本 |
| lane有效但始终未知 | 不足64/82支持行、虚线漆段/间隙标注不足、遮挡、拟合残差或槽位身份错误；先核对标签和独立输出，不放宽规则 |
| 包正确仍失败 | `--no-rga`、RGA不可用或禁用、NPU/算子不兼容、量化输出非有限或超界、候选过密超限 |
| 单模型快但四路过期 | 未计ROI/车道任务、反量化和CPU解码；按组合预算实测优化模型/调度，保留时效及安全条件 |

## 12. 对应代码与外部依据

工程契约以以下实现为准；修改训练接口前先比对，不改FPGA链路：

- [manifest、输出校验及解码](../src/inference/neural_contract.cpp)
- [RKNN运行、RGA适配和坐标回映射](../src/inference/neural_models.cpp)
- [共享NPU任务与ROI](../src/inference/adas_pipeline.cpp)、[调度频率](../src/inference/neural_schedule.h)
- [PC车道训练](../tools/adas_models/train_lane.py)、[车道网络](../tools/adas_models/lane_model.py)
- [固定契约ONNX导出](../tools/adas_models/export_models.py)、[RKNN转换](../tools/adas_models/convert_rknn.py)、[模型打包](../tools/prepare_adas_bundle.py)

2026-10-22核对的官方依据：[Rockchip RKNN-Toolkit2](https://github.com/airockchip/rknn-toolkit2)说明PC转换、板端Runtime和RK3568支持；[官方YOLOv5转换示例](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov5/python/convert.py)支持上述target、mean/std和量化流程；[YOLOv5 v7.0检测头](https://github.com/ultralytics/yolov5/blob/v7.0/models/yolo.py)用于核对锚点及原始卷积头语义；[RKNN C API](https://github.com/airockchip/rknn-toolkit2/blob/master/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h)用于核对query与输入输出接口。

open-adas/openpilot只提供标定、危险区和状态隔离的设计参考，不定义本工程的模型ABI。外部仓库更新不自动改变本文固定契约。

## 附录A：板端SDK与张量只读查询

以下程序使用工程头文件/运行库。保存为`/tmp/adas-model-sdk-probe.cpp`，在目标工程根目录编译；加载模型并打印普通逻辑张量属性，不做推理，不访问FPGA。旧模型可用于查询SDK，旧模型输出不需要、也不应被视为满足新的8类契约。

```cpp
#include <rknn_api.h>
#include <cstdio>
#include <fstream>
#include <vector>
int main(int argc,char **argv) {
  if(argc!=2) {std::fprintf(stderr,"usage: sdk_probe model.rknn\n");return 2;}
  std::ifstream f(argv[1],std::ios::binary|std::ios::ate);
  const auto size=f.tellg();
  if(!f||size<=0||size>512LL*1024*1024)return 2;
  std::vector<char> data(static_cast<size_t>(size));f.seekg(0);
  if(!f.read(data.data(),data.size()))return 2;
  rknn_context ctx=0;
  int ret=rknn_init(&ctx,data.data(),static_cast<uint32_t>(data.size()),0,nullptr);
  if(ret<0) {std::fprintf(stderr,"rknn_init=%d\n",ret);return 1;}
  struct Guard {rknn_context c;~Guard(){rknn_destroy(c);}} guard{ctx};
  auto query=[&](rknn_query_cmd cmd,void *out,uint32_t bytes) {
    int code=rknn_query(ctx,cmd,out,bytes);
    if(code<0)std::fprintf(stderr,"rknn_query(%d)=%d\n",int(cmd),code);
    return code>=0;
  };
  rknn_sdk_version version{};
  if(!query(RKNN_QUERY_SDK_VERSION,&version,sizeof(version)))return 1;
  std::printf("api_version=%s\ndrv_version=%s\n",version.api_version,version.drv_version);
  rknn_input_output_num count{};
  if(!query(RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)))return 1;
  for(int out=0;out<2;++out)for(uint32_t i=0;i<(out?count.n_output:count.n_input);++i) {
    rknn_tensor_attr a{};a.index=i;
    if(!query(out?RKNN_QUERY_OUTPUT_ATTR:RKNN_QUERY_INPUT_ATTR,&a,sizeof(a)))return 1;
    std::printf("%s[%u] name=%s shape=[",out?"OUT":"IN",i,a.name);
    for(uint32_t d=0;d<a.n_dims;++d)std::printf("%s%u",d?",":"",a.dims[d]);
    std::printf("] fmt=%s type=%s qnt=%s zp=%d scale=%g\n",get_format_string(a.fmt),get_type_string(a.type),get_qnt_type_string(a.qnt_type),a.zp,a.scale);
  }
  return 0;
}
```

```bash
g++ -std=c++17 -Wall -Wextra -Ithird_party/rknn/include \
  /tmp/adas-model-sdk-probe.cpp -Lthird_party/rknn/lib -lrknnrt \
  -o /tmp/adas-model-sdk-probe

# 新包到位后分别查询；这两条只检查加载/属性，不代表推理/精度通过。
LD_LIBRARY_PATH="$PWD/third_party/rknn/lib" /tmp/adas-model-sdk-probe \
  /opt/adas-models/adas-v1/scene.rknn
LD_LIBRARY_PATH="$PWD/third_party/rknn/lib" /tmp/adas-model-sdk-probe \
  /opt/adas-models/adas-v1/lane.rknn
```
