# ARM + FPGA 系统仓库发布计划

> **For agentic workers:** 按 writing-plans 独立审阅计划，以 verification-before-completion 核验实际暂存内容；用户已明确授权向指定仓库提交，计划审阅通过后直接执行。

**Goal:** 将已整理的 ARM/FPGA 源码、详细 README 和准确验证摘要提交至 `StellatoL/FPGARACE_2026_Pango_3`，保留远端历史，不泄露本地敏感资料。

**Architecture:** 远端旧 FPGA 单工程迁至 `fpga/`，新增 `arm/` 和系统级 `doc/`、发布检查工具。使用隔离 Git checkout 准备提交，保留当前工作区和原始证据。公开文档只链接实际进入提交的文件。

**Tech Stack:** Git / GitHub、Python、ModelSim、PDS、Qt 5 / OpenCV / RKNN、Bash、PowerShell。

## 1. 当前事实与发布边界

- 当前工作区没有根 Git 仓库；旧发布 checkout 位于 FPGA build 内，已提交 FPGA 版本为 `87c0936`。
- 已通过认证 API 确认目标是私有仓库，保持原可见性；本计划所称公开/发布版本指经筛选的提交内容，不表示将仓库改为公开。
- 最近实际验证：13 项 Python，9 次 RTL 单元，8 组/10 帧/5,541,888 像素；PDS 1280×720 综合；921,600 像素综合网表仿真，无 SDF。ARM CPU：41 条 ADAS、15 条车道/导航、5 项文本、7 条视频 E2E。
- 本机未构建/运行 AArch64 完整应用，未完成当前 AVM—ARM 板级联调、布局布线及实体 LED 控制。MATLAB 最近一次目录回归复用此前四组参考。
- `arm/third_party/rknn/include/rknn_api.h` 文件头明确限制披露与分发，因此不进入公开提交；本地原件保留，公开目录写入 SDK 获取/放置说明。
- 四个 RKNN 模型、运行库和旧程序不上传；公开角色、放置路径、SHA256 和缺失条件。用户随后明确要求上传 `pango_pci_driver_mul.ko`，该驱动作为单独审查的二进制纳入。ztu 源封装保留现有内容及来源说明，不为第三方重新授予许可。
- 旧日志、历史计划、个人会话便笺、测试视频/图片原件、仿真库、备份、许可证材料不上传。公开验证摘要由真实记录提取，不冒充完整原始日志。

## 2. 目标提交结构

```text
README.md / LICENSE / .gitignore / .gitattributes
arm/
  README.md / adas_arm.pro / build.sh / run.sh
  src/ tests/ tools/ resources/
  models/README.md / assets.json
  third_party/README.md / ztu/ / rknn/include/README.md / rknn/lib/README.md
  drivers/README.md / prebuilt/README.md / pango_pci_driver_mul.ko / publication-metadata.json
fpga/
  README.md / .gitignore
  src/ sim/run.py / sim/tb/ / sim/models/ / sim/data/README.md / sim/wave.do
  constraints/ project/ tools/avm/
  doc/avm/（接口、精度与使用说明）
  doc/validation/（已公开 FPGA 历史摘要） / doc/figures/
doc/
  README.md / architecture.md / integration-status.md
  validation/README.md / results.json / source-sha256.json / pass-markers.txt
  plans/2026-10-03-system-publication.md
tools/release/check_publication.py / README.md
```

## Task 1：确认远端并准备隔离工作树

- [ ] 只读检查 GitHub HEAD/main、当前 checkout 和验证记录。网络失败时检查 Git/代理配置并重试，不回显凭据、不修改全局安全例外。
- [ ] 写完本计划并独立审阅。根工作区非 Git，采用独立 clone/checkout 放入 `build/publish-system/`；不污染旧 checkout。
- [ ] 读取并保留远端已有 LICENSE、提交历史和新增内容；若远端有用户未在本地的更新，先合并差异后发布，不 force push。

## Task 2：建立精确清单与公开文档

- [ ] 创建按文件白名单准备发布副本的本地工具，记录映射和每文件哈希。原生产 C++/UI、RTL/TB、测试及 qmake 内容不变。
- [ ] 重写根 README：功能/架构/目录、Windows FPGA 命令、Linux ARM 命令、克隆后的外部 SDK/模型补齐方法、CPU 测试、验证统计与未完成项。
- [ ] 更新 ARM/FPGA/文档索引，移除指向不上传日志、archive、历史便笺的公开链接；保留历史文件本地原件。
- [ ] 增加 SDK/模型/驱动说明；`rknn_api.h`、模型及运行库显式排除。驱动按用户要求纳入，扫描其内容，必要时只脱敏非加载调试区，并核对全部其他字节不变；检查器固定其 SHA256，不泛化放行二进制。
- [ ] 从本次已核验日志生成公开结果 JSON、PASS 摘录和源文件 SHA256；静态摘要保留验证日期、来源摘要哈希与局限。
- [ ] 新增系统级暂存内容检查器，支持 ARM 文件类型，核验全量暂存 blob、路径范围、秘密模式、二进制元数据、文档链接、源哈希、冲突与非普通文件。

## Task 3：验证实际发布树

- [ ] 重新读取本地 ModelSim/PDS/ARM 证据及 SHA256，确认历史实测绑定当前生产源码，不把新的文档提交日期写成新硬件测试。
- [ ] 在隔离发布树生成 small fixture，执行 ModelSim small 压力整帧和全部 9 次 RTL 单元；新位置下运行 ARM 现有 CPU 测试，视频 E2E 使用本地合成视频并标明输入不随 Git。
- [ ] 完整 ARM 构建的 SDK/模型缺失检查应明确失败；不在 x86 主机链接 AArch64 库，也不将 qmake 解析称为完整构建。
- [ ] 静态校验 Bash/Python、qmake 源清单、FPGA files.f、README 所列入口和相对链接。生成物进入 build/sim-data 并由 Git 忽略。
- [ ] 人工逐项核对 `git diff --stat`、文件列表、差异及输出图；对 `git ls-files --stage` 获取的真实 blob 做敏感信息扫描（不输出疑似秘密值）。
- [ ] 受控样本验证秘密探测规则；检查 Git 提交身份使用公开用户名与 GitHub noreply 邮箱，避免发布私人邮箱。

## Task 4：提交、推送及确认

- [ ] 提交前记录 tree ID、文件清单和扫描结果；提交内容与核验 tree 一致。
- [ ] 提交说明如实写明：整理 ARM/FPGA 工程；FPGA 仿真/综合及网表功能验证完成；ARM 主机 CPU 测试通过；ARM 完整应用和跨端板级链路本次未验证；尚未布局布线/上板联调。
- [ ] 使用普通 fast-forward push 更新远端 main。推送前重新读取 remote HEAD；遇到并发更新先 fetch/合并，再重做受影响检查，不覆盖远端提交。
- [ ] 若 Git HTTPS 传输持续不可用，使用已认证 GitHub Git Data API 上传同一批 blob/tree/commit，核对对象 SHA 后以 `force=false` 更新 main；同样先核对父提交，绝不改仓库可见性或跳过审查。
- [ ] 推送后核对远端 HEAD 与本地提交一致，确认工作树没有未提交源码。保留本地原始资产/日志/备份和隔离发布 checkout。

## 验收标准

远端仓库呈现完整双子工程源码和详细 README；所有公开链接指向提交内文件；所述验证有对应真实证据；缺失的外部 SDK/模型清楚说明；实际暂存内容没有敏感规则命中或未审查资产；远端提交 SHA 已确认。任何未完成硬件验证不得写成完成。

## 执行记录

独立审阅通过。已确认远端 main 为 `87c0936`，仓库保持私有。Git HTTPS 连接存在超时/重置，认证 GitHub API 可用；隔离 checkout 从同一提交的本地旧仓库复制，Git 对象与远端基线一致。

提交前已完成 144 个文件的白名单审查。复用并重新核对完整尺寸 ModelSim/PDS 历史证据及生产源码哈希；在实际发布树又通过新生成 small 的 3,072 像素压力仿真、9 次 RTL 单元和 ARM 41/15/5/7 项 CPU/视频检查。qmake 解析和外部 SDK 缺失提示符合文档。

用户明确要求上传驱动后，已纳入 `arm/drivers/prebuilt/pango_pci_driver_mul.ko`。原二进制有 4 个个人目录前缀，全部位于非加载 `.debug_line_str`；提交副本作等长替换，其他字节、ELF 表与重定位清单均核对不变。原文件保留本地；处理记录随驱动提交。旧图标含素材站水印，原件保留，提交采用独立生成的几何图标。

受限 SDK 头文件、模型/运行库、私人会话及原始日志未纳入。实际 Git 暂存 blob 的敏感模式、图像元数据、文档链接和源码哈希检查通过；最终提交 tree 与扫描 tree 必须一致，远端分支更新完成后另行核对提交 SHA。结果见 [系统验证摘要](../validation/README.md)。
