# 旧 PCIe 预编译驱动

[prebuilt/pango_pci_driver_mul.ko](prebuilt/pango_pci_driver_mul.ko) 随本次仓库提交，是 Linux AArch64 内核模块。模块声明 `GPL v2`，vermagic 为 `6.1.99 SMP mod_unload aarch64`；当前没有提供对应驱动源码或可重建工程。

原文件的非加载 `.debug_line_str` 区有 4 处个人编译目录。提交副本仅等长替换这些目录前缀，所有其他字节不变，含代码、数据、ELF 表、符号、重定位及 `.modinfo`。原件保持本地原样；发布 SHA256 为 `173904a078cd8ff166c350a24c950044a3dbab73800ee6460524c33f18db5277`，原 SHA256 及逐区校验见 [处理记录](prebuilt/publication-metadata.json)。[资产清单](../models/assets.json) 区分原文件与提交副本。

应用旧接口使用 `/dev/pango_pci_driver`、ioctl `'S'` 0～11、四槽 RGB565 和自定义 read 状态码，还涉及 `/dev/mem`。该模块能否在目标板运行取决于内核版本、模块配置/符号、PCIe 设备和 DMA 协议，不能仅按架构相同判断。

本次没有加载该模块，也没有验证它与当前 AVM RTL 兼容。应先取得对应源码、构建信息和寄存器/DMA 说明，再进行板端适配。详见 [联调状态](../../doc/integration-status.md)。
