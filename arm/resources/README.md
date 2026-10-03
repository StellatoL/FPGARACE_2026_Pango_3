# 应用资源

`style.qss` 为原应用样式。`video.ico` 是用简单几何图形生成的项目图标，表示四路图像与中央车辆；不包含文字、素材站标识或个人信息。

导入的旧图标含素材站水印，原件保留本地备份，本次提交改用自绘图标。重建命令为 `python tools/generate_app_icon.py`（从 arm/ 执行，需要 Pillow）；应用正常运行不依赖 Pillow。
