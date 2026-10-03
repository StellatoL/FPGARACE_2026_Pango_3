# PDS 构建

从根目录运行 `python tools/avm/run_pds.py --label verified` 创建隔离工程。
[build.tcl](pds/build.tcl) 使用生成目录中的 ../rtl，[FDC](../constraints/avm_core.fdc) 约束 100MHz 核心。
目标为 PG2L50H-6IFBG484，默认 1280×720；完整命令见 [根目录 README](../README.md)。

已完成 ADS 综合和完整网表功能仿真；没有板级管脚、布局布线或上板验证。资源和边界见 [验证摘要](../doc/validation/README.md)。
