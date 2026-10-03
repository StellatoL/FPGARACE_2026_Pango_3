# 发布检查

从仓库根目录暂存准备公开的文件后，运行：

```powershell
python tools/release/check_publication.py --self-test
python tools/release/check_publication.py --report build/publication-scan.json
```

检查器读取 Git 暂存 blob，而非仅检查工作树；检查敏感信息模式、文件范围、大小、Markdown 链接、PNG 附加块与已验证源码哈希。
扫描输出只含文件名、行号、规则，不回显可疑值。结果 JSON 保存在本地 build 中。
人工还需复核图片内容、提交身份和未知形式的秘密；自动规则不代表全面证明。
