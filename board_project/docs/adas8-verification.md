# 八类模型接入与转换记录（2026-10-25）

用户最终确认使用实际8类检查点，取消9类要求。原FPGA、驱动、硬件接口、原MainWindow、旧模型及build_release均保持保护；本次按用户要求提交至既有GitHub仓库的board_project/目录，包含原始PT、ONNX、RKNN及相关程序和文档；驱动二进制继续纳入版本控制。

## 实际模型与部署

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| models/incoming/adas8_best.pt | 3,998,511 | `72efb036f9d3d0363cb83541f9fea747c3e9e899565c3ba26e972cbba579acf7` |
| models/adas8-candidate-2026-10-25/scene.onnx | 7,111,181 | `0861b39e36c6b31bf196fdcbc4bb6ddfa36ab86bf87d6cfe6f7acdb811e0a933` |
| models/adas/scene.rknn（候选目录同字节） | 2,321,589 | `e1071502f99b8157799af65175b6de1aa861b9cbfbeb8bc715e48687b64720ee` |

迁移用检测模型包为`dist/adas8-candidate-2026-10-25.tar.gz`（本机生成、附SHA256，不随Git提交），包含`adas/manifest.json`和`adas/scene.rknn`；解压到独立目录后用`ADAS_MODEL_MANIFEST`指向其中manifest，再启动当前编译程序。默认manifest版本`adas8-candidate-2026-10-25`，schema2，lane=null。输入uint8 NHWC RGB640×640、mean0/std255；3个已sigmoid的NCHW输出为39×80×80、39×40×40、39×20×20。实际顺序`person, vehicle, crosswalk, light_red, light_green, cone, barrier, debris`，不补黄灯或导航类。手动导航按N循环；关闭时忽略视觉导航结果。四路侧向人员/车辆/路障完整性、请求后新观测和新鲜度条件保持原样。

车道未提供时，每来源代次报告一次Missing，不运行车道任务、不计入NPU频次或延迟、不授权变道。检测模型仍独立工作。后续按[模型规范](model-replacement-roadmap.md)交付真实车道模型，并使用两份metadata重新打包完整包。

## 转换环境和复现

实测Python3.8、torch2.4.1、ONNX1.16.1、RKNN-Toolkit2 2.3.2；工程librknnrt API2.3.2、NPU驱动0.8.2。本机现成环境为`/home/cat/code/high_speed_pcie/.codex/rknn-venv/bin/python`，上游代码在`/home/cat/code/yolov5-master`。不要求迁移板具有转换环境。

```bash
# 从工程根执行；输出目录必须新建，不覆盖已有产物。
/path/to/rknn-venv/bin/python tools/adas_models/export_rk_scene.py \
  --yolov5-repo /path/to/airockchip-yolov5 \
  --weights models/incoming/adas8_best.pt --output /path/to/new-export/scene.onnx
/path/to/rknn-venv/bin/python tools/adas_models/convert_rknn.py \
  --onnx /path/to/new-export/scene.onnx \
  --calibration /path/to/calibration.txt --output /path/to/new-export/scene.rknn
python3 tools/prepare_adas_bundle.py \
  --scene /path/to/new-export/scene.rknn --scene-metadata /path/to/new-export/scene.json \
  --version your-version --toolkit-version 2.3.2 \
  --runtime-api-prefix '2.3.2 (' --output /path/to/new-bundle
```

导出器在临时目录调用未修改的[RK官方export.py --rknpu](https://github.com/airockchip/yolov5/blob/master/README_rkopt.md)，保留PT原文件，检查真实类别、末端Sigmoid、形状和anchors。INT8转换按[Rockchip YOLOv5示例](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov5/python/convert.py)设定目标RK3568与归一化。

停止当前程序后可用`ADAS_MODEL_MANIFEST=/absolute/new-bundle/manifest.json ./build_adas/run_img_recv_demo.sh`试新包；不改原FPGA输入源。回滚可以指向已验证旧包；回退本轮代码需从`.codex/verification/adas8-2026-10-25/before-adas8-source.tar.gz`解压到独立目录检查并重编译，不覆盖保护文件。

## 数值验证与限制

- 官方导出ONNX对原检查点3帧allclose通过，最大差异`2.563e-5`。
- INT8模型12帧真实NPU调用成功，全部输出有限且在[0,1]，形状符合契约；不放宽概率范围。
- ONNX与INT8逐头MAE约0.003886–0.005791，最大单点差异0.315769，最小余弦0.999251。高余弦不表示检测精度无损。
- NPU API输入提交/运行/取输出的中位耗时21.881ms、P95 22.244ms；排除RGA、复制、解码、Qt和FPGA，不作为整机帧率或端到端延迟。
- 编译器提示`onnx::Conv_290`存在权重离群值-23.542；可能影响INT8精度，保留原始警告，不据此宣称无损转换。
- 校准为project_video.mp4中96帧白天道路，另抽12帧数值冒烟。无八类覆盖证明，缺夜间、不同侧视和灯色ROI分布。后续应补齐代表性校准集并重新量化、逐类比较PT/ONNX/RKNN指标。
- 用户给出的测试AP50：vehicle48.5%、barrier36.7%、cone31.1%、person23.9%、crosswalk31.9%、light_red18.8%、light_green21.9%；debris未提供。仅引用用户结果，未在板上重测，不视作满足赛题。

本机完整转换日志在`.codex/verification/adas8-2026-10-25/`，包括conversion-report.json、校准清单、ONNX数值比较及实际NPU输出比较。该目录不随源码Git发布。

## 工程回归

| 验证 | 结果与证据 |
|---|---|
| `./build.sh`及修正后增量构建 | 通过，`build.log`、`build-rga-fix.log` |
| 完整规则/线程/契约/历史诊断回归 | 通过，`tests.log`；打包3项、导出器5项、GUI15项 |
| RGA颜色、补齐stride、裁剪、四区域、padding | 真实硬件通过，`rga-input-green.log`；修复前RED保留在`rga-input-red.log` |
| 实际RGA→新RKNN→解码 | 单路及四区域通过，`rga-model-smoke-green.log`、`rga-road-smoke.log`；真实道路第630帧有有效检测，未做人工标注准确率统计 |
| 构建脚本迁移/安全回归 | 11/11通过，`build-script-final.log`；并未测试第二台实体板 |
| 从`/tmp`启动launcher、无输入时正常关闭 | 通过，`startup.json`明确0采集帧；仅生命周期和资源定位验证 |
| 硬件、驱动、原MainWindow、旧模型及原程序保护 | 717/717 SHA256一致，`protected-after.log`；incoming PT也保持原hash |
| 只读代码复核 | 两次复核未发现P1/P2问题；确认未更改决策安全门槛 |

RGA完整链路初测触发了已有输入格式错误：项目RGB565字节为高位红、低位蓝，而[Rockchip枚举定义](https://raw.githubusercontent.com/airockchip/librga/main/include/rga.h)从低位命名，该布局对应`BGR_565`。修正ARM RGA输入格式及2字节stride分支后，真实色块与原CPU参考匹配（保留原8级量化容差），自检与严格输入适配通过。未交换源数据字节、未跳过自检、未修改系统RGA或任何FPGA接口。旧模式对应输入格式同步修正；正式ADAS仍无CPU图像回退。build.sh会检查该枚举是否可用，旧SDK不支持时采用明确不可用的诊断构建。

四区域接口冒烟是在离线道路图上裁剪，并非四台真实摄像头输入，也不是现场文件播放入口。离线诊断源码`tools/adas_models/smoke_scene.cpp`不链接到正式应用，可在已构建的工程根复现：

```bash
g++ -std=c++17 -O0 -fPIC -Isrc -Ithird_party/rknn/include \
  tools/adas_models/smoke_scene.cpp build_adas/neural_models.o \
  build_adas/neural_contract.o build_adas/rga_preprocessor.o \
  $(pkg-config --cflags --libs Qt5Core opencv4) \
  -Lthird_party/rknn/lib -lrknnrt -ldl -o /tmp/adas-scene-smoke
LD_LIBRARY_PATH="$PWD/third_party/rknn/lib" /tmp/adas-scene-smoke "$PWD" /path/to/image.png
# 可选真实RGA回归；默认测试不要求其他设备具备可用RGA。
ADAS_TEST_RGA=1 ./tests/run_tests.sh
```

逻辑注入只验证规则，不作为感知准确率。PCIe当前无在线设备且模块vermagic与内核不匹配，未加载或修改驱动；FPGA实流、源切换、图像处理按钮、现场延迟与30分钟稳定性仍须在匹配BSP上验收。
