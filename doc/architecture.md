# 当前系统架构

本文件记录现有代码。整体目标是 RK3568_50H 上的四路环视及 ADAS 演示，当前实现由两个独立子工程组成。

## FPGA：AVM 算法核心

`fpga/src/rtl/avm_core.v` 是生产顶层，协调另外七个 RTL 模块：宏块数据装载、网格插值、瓦片缓存、双线性插值、增益融合、内存仲裁、光栅重排。输入是外部帧存中已就绪的四路 RGB565 图像及描述符/LUT；输出为带 SOF/EOL 的 16 位 ready/valid 像素流。

离线 Python 生成标定映射、网格、内存镜像和位精确预期；MATLAB 作独立双精度参考。ModelSim 通过 `sim/models/avm_memory_bfm.sv` 的事务模型提供帧存。PDS 模板针对 `PG2L50H-6IFBG484` 进行核心综合，不包含摄像头、DDR PHY、PCIe、板级管脚与异步时钟适配。

曝光统计来自 RTL，Python 计算增益后在后续帧回放验证；没有实现持续运行的 ARM 曝光控制服务。详细数据格式以 [FPGA 接口契约](../fpga/doc/avm/implementation-contract.md) 为准。

## ARM：显示、感知与演示决策

Qt 主窗口选择 `Pango_fun` 的旧 PCIe 路径或 `FileVideoSource` 的文件路径，两者提供 RGB565 图像给显示与 `PlateInferThread`。文件源按比例填充到 1280×720。推理线程根据模式运行车牌或交通检测；ADAS 模式在交通模型就绪后追加车道线、导航箭头和规则决策。

`core/` 保存感知与规则；`inference/` 保存线程、跟踪、后处理和可选 RGA；`common/` 定义公共数据和字符编码；`hardware/` 为输入和设备访问；`ui/` 完成叠加、HUD 和模拟输入。RKNN 的模型文件与 SDK 运行库属于外部二进制资产。

ADAS 规则提供滞回确认、灯色/行人/变道/TTC 等提醒，结果驱动窗口内的方向、文本与屏幕 LED。未实现车辆控制或 FPGA 实体 LED 命令。旧 `plate_recog_task` 源文件没有编入当前应用，保存作来源追溯。

## 构建与证据

两个子工程分别拥有构建根。FPGA 工具用脚本路径定位 `fpga/`；ARM qmake 用工程位置定位 `arm/`，构建输出和资源部署集中到 `arm/build/release/`。根目录不提供虚假的“整机一键编译”。

生产源码只有 `arm/src/` 与 `fpga/src/rtl/` 两处维护入口。`archive/`、FPGA 验证快照、旧发布 checkout 中的同名源码是历史副本。本次验证把源码哈希、仿真/综合结果、主机测试和导入记录分开，详见 [验证报告](validation/README.md)。
