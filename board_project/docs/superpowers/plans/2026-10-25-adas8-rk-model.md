# RK八类模型接入与转换计划

依据用户2026-10-25明确要求执行：使用incoming/adas8_best.pt，删除黄灯和视觉导航的模型必需项，导航由键盘/手动输入。既有FPGA交互与处理按钮冻结。

## 设计与边界

权重SHA256：72efb036f9d3d0363cb83541f9fea747c3e9e899565c3ba26e972cbba579acf7。真实names顺序：person, vehicle, crosswalk, light_red, light_green, cone, barrier, debris。ReLU激活、YOLOv5n宽度0.25、三头三锚点。遵循本机airockchip/yolov5及其README_rkopt.md的`export.py --rknpu`流程，输出为已sigmoid的3个NCHW头，与旧sigmoid前raw格式区别对待。

- manifest保留原schema1和raw输出；增加schema2允许lane=null，必需类别改为上述8类，拒绝缺少人员/车辆/任一路障能力。
- 新输出契约`yolov5_rk_sigmoid_nchw_v1`明确不重复sigmoid，输入仍640×640 RGB8 NHWC、mean0/std255、RK3568 INT8。保持有界解码、置信度乘积/NMS及新鲜度门槛。
- 检测模型可独立加载，lane不存在明确Missing，不以旧车道或CPU像素算法补齐，不授权缺证据变道。
- 导航使用现有N键和手动方向控件，关闭视觉导航任务；手动导航仍受路口灯控和路段变道安全约束。保留四路目标侧新观测与全类别安全判断。
- 原incoming权重、旧四模型、FPGA、drivers、src/hardware及原MainWindow不修改。生成物存独立版本目录，先验证再启用。

## 执行与验收

- [x] 归档、保护清单、模型元数据和转换环境核查。
- [x] 8类/schema2/可选lane/RK sigmoid与raw互斥解码回归，先验证RED再实现。
- [x] RK官方ONNX导出、元数据/锚点、代表样本量化、RKNN构建及真实NPU冒烟。校准集如仅来自project_video，应明确覆盖不足，不能宣称最终精度通过。
- [x] 手动导航、调度及能力状态接入，缺车道仍拒绝变道，四路侧向否决回归。
- [x] 默认构建、逻辑/线程/GUI回归、新模型真实加载与推理、输入权重和717项保护校验。
- [x] 更新README、模型规范、转换/部署与限制记录，保留可回滚入口。不将模型转换成功作为弱类别精度通过。

用户给出的AP50仅作为训练结果记录，不补填debris指标，不凭测试图片注入证明精度。现场链路仍PC素材→HDMI→FPGA→PCIe/DMA→ARM；离线模型转换或采样不增加正式文件输入途径。

实施补充：真实RGA冒烟定位到输入格式RGB/BGR命名不一致，经官方定义与两格式色块probe确认后修正ARM RGA格式及stride分支；自检未跳过、CPU回退未增加。测试与限制见docs/adas8-verification.md。
