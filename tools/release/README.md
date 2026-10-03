# 系统提交检查

从仓库根目录选择文件暂存后运行：

```powershell
python tools/release/check_publication.py --self-test
python tools/release/check_publication.py --report build/publication-check.json
```

依赖 Python 3.10+ 与 Pillow（图标结构检查）。检查器读取实际 Git 暂存 blob，校验明确的文件清单、敏感模式、SDK/本地产物排除规则、图片元数据、文档链接和源码 SHA256。检查结果不打印疑似秘密值。

`publication-files.json` 是本次批准的文件清单，`doc/validation/source-sha256.json` 绑定已核查源码。后续新增文件需审阅后更新清单；源码发生变化应重新验证，并根据实际结果更新哈希，不能仅为通过检查而重写验证基线。

检查器借用 `fpga/tools/release/check_publication.py` 的已有检测规则；后者是旧单 FPGA 工程检查器，整个系统请运行本目录入口。自动规则不覆盖所有未知秘密类型，提交差异、二进制来源和图像仍需逐项审阅。

唯一允许的内核模块是 `arm/drivers/prebuilt/pango_pci_driver_mul.ko`，并固定核对本次经调试路径脱敏的 SHA256。其他 `.ko` 不能直接加入；该例外来自用户明确要求，不放宽 SDK、模型或其他二进制的排除规则。
