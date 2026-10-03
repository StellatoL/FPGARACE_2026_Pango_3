# RKNN SDK 与第三方封装

完整 ARM 应用使用 RKNN 和 ztu 封装。源码提交包含 `ztu/` 的四个原始封装头文件，未提供其完整来源版本；保留原内容，不由根 LICENSE 推定第三方授权。

SDK 头文件带明确的保密和限制分发声明，因此 `rknn_api.h` 与运行库均不随本次提交。请从设备供应商或 Rockchip 授权 SDK 获取与目标板 RKNPU 驱动、模型匹配的版本，再放置到：

```text
arm/third_party/rknn/include/rknn_api.h
arm/third_party/rknn/lib/librknnrt.so
```

Rockchip 的 [RKNN Toolkit2 官方仓库](https://github.com/airockchip/rknn-toolkit2) 可作为版本和 Runtime 文档入口；使用其中资产仍需遵守对应条款。此工程不自动下载或替换 SDK。

三个必需模型放到 `arm/models/`，额外车辆模型可选，详见 [模型说明](../models/README.md)。匹配本次本地验证环境的资产 SHA256 见 [assets.json](../models/assets.json)；资产未附带时，`bash build.sh --check` 会明确报缺失，不能将其称为应用构建成功。CPU 测试不依赖 SDK。

完整应用仅支持 Linux AArch64；本机 x86_64 WSL 没有验证 RKNN、驱动、GUI 或两端联调。Qt/OpenCV 和可选 RGA 使用板端系统依赖。模型转换需要另行准备 RKNN Toolkit2/PyTorch 及权重，转换开发环境不等同于运行时。
