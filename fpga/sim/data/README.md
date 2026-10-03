# 仿真数据

完整 fixture 不进入 Git。使用 tools/avm/generate.py 生成 small、synthetic720、synthetic720-wide 或 public。
生成的 memory/expected HEX、BIN、JSON、NPZ/MAT 和图像写入对应子目录。
small 与合成样例可离线生成；public 第一次需要从固定上游提交下载，经 SHA256 校验后缓存到 public_source。

完整命令、尺寸与许可见 [根目录 README](../../README.md) 和 [算法工具说明](../../tools/avm/README.md)。
