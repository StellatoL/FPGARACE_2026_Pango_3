# 仿真入口

生产 RTL 在 src/rtl；tb 内为自检平台，models 内为 DDR 事务 BFM。
全新克隆需要先执行 `python tools/avm/generate.py --case small`，再执行
`python sim/run.py --case small --profile stress --wave`（均从仓库根目录）。

`run.py` 支持从任意工作目录调用；输出在 build/avm/simulation。
完整样例、MATLAB、覆盖率、压力和反馈步骤见 [根目录 README](../README.md)。
[wave.do](wave.do) 用于打开转换后的 WLF。
