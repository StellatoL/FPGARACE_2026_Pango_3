# 公开验证摘要

本目录来自 2026-10-03 实际 ModelSim/PDS 验证与目录整理后的回归。
这里是经过审查的摘要和 PASS 行摘录，未发布含本机路径的原始完整日志。

- [results.json](results.json)：13 项 Python、9 次 RTL 单元、8 组/10 帧/5,541,888 像素整帧回归；完整 921,600 像素综合网表；资源与综合估计时序。
- [source-sha256.json](source-sha256.json)：当时通过审计的设计、平台、约束和运行器字节哈希；Git 禁用这些源码的换行转换。
- [pass-markers.txt](pass-markers.txt)：单元与网表日志中的真实 PASS 行。

results.json 保留两份本地原始结果 JSON 的 SHA256 作为来源标识；这些原始 JSON 不包含于公开仓库。源码哈希可独立核对，完整运行日志可通过 README 的命令重新生成。

此前四组 MATLAB 全帧参考通过；目录整理回归复用其结果，没有重新调用 MATLAB。
有限接缝标称 28.31660ms；宽交叠 36.34548ms、随机压力 54.58745ms，后两者未达到 30fps。
PDS 103.8529MHz 是综合估计；网表零延迟、无 SDF；尚未布局布线或上板，没有整机 bitstream。

复现步骤及边界见 [根目录 README](../../README.md)。
