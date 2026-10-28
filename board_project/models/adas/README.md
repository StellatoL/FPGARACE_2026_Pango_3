# 当前双640模型包

运行仅使用本目录`manifest.json`、`vehicle.rknn`和`traffic.rknn`。说明、来源与许可证随包保留；训练、ONNX、转换脚本、校准集不随运行工程携带。

| 角色 | 输入 | 输出及作用 |
|---|---|---|
| vehicle | 640×640 RGB8 NHWC，INT8，mean0/std255，黑色padding | 官方Rockchip YOLOv5n，3个C255 NCHW sigmoid头，COCO80中只取person/bicycle/car/motorcycle/bus/truck |
| traffic | 同上 | 原6类模型，3个C33 NCHW raw头，保留自定义anchors，用于斑马线、灯色和辅助过街人员 |

traffic类序为`crosswalk,person,light_red,light_green,light_black,light_yellow`。vehicle头已经sigmoid，traffic头仅在decoder中sigmoid一次。完整类序和anchors以manifest为准，SDK前缀固定`2.3.2 (`。不接受旧schema1/2、8类/960或车道网络；车道为传统CPU算法。

SHA256：

```text
566c70942faccfcd854c4eab164d4dd1ff92b1aecbcdcd356f56605ff6fca131  vehicle.rknn
62296449866eefb1391015cce7592007bd0fb35a6062cc510c72953bd4dd7db4  traffic.rknn
```

vehicle来源固定[Rockchip Model Zoo v2.3.2 YOLOv5](https://github.com/airockchip/rknn_model_zoo/tree/bad6c7334531becaf90a561988519b7bec34d0ab/examples/yolov5)，commit`bad6c7334531becaf90a561988519b7bec34d0ab`；原始n ONNX SHA为`b3237d0f59a5d90ff6ecec3dca220e0ada8abd929f8a158b08b12e1a3bad7232`，使用Toolkit2 2.3.2、20张官方COCO subset作INT8转换。traffic从原工程逐字节复制，未转换。

`provenance.json`中的开发机路径仅记录历史来源，不是运行依赖。`licenses/`保留官方来源许可证；traffic沿用原工件许可条件。旧源工件已移出工程归档，详见[清理说明](../../docs/project-cleanup-2026-10-28.md)。

替换时关闭程序，准备包含两个不同文件的完整包，更新与实际权重匹配的SHA、类序、anchors、激活契约，重启并做真实RGA/NPU冒烟；不通过改尺寸字符串适配权重。详细要求见[接入指南](../../docs/model-replacement-roadmap.md)。
