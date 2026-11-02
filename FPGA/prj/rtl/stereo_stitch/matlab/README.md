# 双摄水平拼接参数估计

用于当前 stereo_stitch RTL：估计右图在左图坐标系中的水平偏移 D，输出适用性诊断和 RGB565 拼接预览。没有自动修改 RTL、运行仿真或测试。

## 使用

1. 在 MATLAB 中打开本目录的 `run_stereo_params.m`。
2. 设置 `cfg.leftFile` / `cfg.rightFile`。默认使用已有 test_img 中 A/B 两图，但文件名不证明相机方向。当前 RTL 左图来自相机2(ID1)、右图来自相机1(ID0)，请按实际连接设置。
3. 运行入口脚本，查看 `output/report.txt`、匹配点分布及 `rtl_preview.png`。
4. 仅当 `rtlModelAcceptable=true` 且人工确认多组实拍效果后，将候选 D 填入顶层 `pcie_video.v` 的 `D_DEFAULT`。脚本不自动接入生成的 vh。

要求 MATLAB R2016b 或更高版本（隐式数组扩展、jsonencode）；自动特征匹配需要 Computer Vision Toolbox。较新版本使用 estgeotform2d，旧版本回退 estimateGeometricTransform2D。输入为严格1280×720的8位RGB或灰度PNG/JPEG，不隐式改变尺寸。

没有视觉工具箱时，可在 cfg.manualPoints 中输入人工对应点：每行 `[leftX leftY rightX rightY]`，坐标从1开始。至少8对，默认通过门限要求12对；建议20对以上，覆盖重叠区上下左右，不全部选在同一物体或直线上。人工点全部参与诊断，不自动剔除。可以用 MATLAB 图像数据提示读取点坐标，或加载无标题四列CSV。

## 参数含义

- `D = round(median(xLeft-xRight))`，右图坐标满足 `xRight=xLeft-D`。D必须在0..1278。负数不取绝对值、不自动交换图像，以免掩盖接线方向问题。
- 拼接宽度 `1280+D`，几何重叠宽度 `1280-D`。输出仍为1280×720，有效高度按RTL整数公式计算，上下补黑边。
- `dyDiagnosticOnly` 是 `yLeft-yRight` 的中值。现有RTL不支持垂直校正，这个值只用于诊断。
- 自动模式用投影模型剔除错误特征匹配，但不把投影变换用于预览。随后按纯水平整数D计算二维残差，避免软件校正掩盖硬件能力限制。
- 默认门限：至少12个内点、内点率35%、垂直中值偏移不超过2像素、二维残差P95不超过3像素、匹配点横向覆盖重叠区35%及纵向覆盖40%。这些是可配置的初步筛选门限，不是硬件验收标准。

## 输出

| 文件 | 内容 |
| --- | --- |
| report.txt | 候选D、几何、误差及拒绝原因 |
| stereo_params.mat / json | 参数、统计；MAT另含配置及对应点表 |
| matched_points.csv | 对应坐标、内点标记、dx/dy及整数D残差 |
| stereo_params.vh | VALID宏；仅模型通过时生成D宏，不自动接入工程 |
| rtl_preview.png | 当前RTL数值规则下的融合图；D合法时生成，模型不通过时仅供诊断 |

同一输出目录会覆盖上次结果。一次运行中若D非法，会删除上次预览，避免混淆；不同图对建议使用不同outputDir。输入/匹配失败会报错，此时不要把之前留存的结果当作本次结果。

`stereo_rtl_preview.m` 实现RGB565输入量化、Q12像素中心映射、边缘夹取、8位插值系数、水平阶段不舍入的双线性插值、0..256融合权重及最终舍入。PNG将5/6位通道按比例展开至8位显示。它是数值参考代码，不是RTL仿真，也不包含DDR调度、欠流、同步或曝光补偿。

## 采图与局限

固定相机位置、焦距和输入尺寸，采集尽可能同时曝光的图像；场景应有丰富纹理并覆盖实际工作距离。建议多组不同景深和位置的实拍图分别运行，比较D稳定性。自动内点可能偏向某一平面，较低残差不能证明其他景深没有重影。

如果垂直偏移或残差过大，应检查机械安装、镜头畸变、透视差异、运动和深度视差，不应仅放宽阈值。单对普通图像无法可靠求出完整相机内参、畸变和双目外参；需要棋盘格多姿态采集和独立标定流程。当前脚本刻意只导出已有RTL可以使用的D。

API参考：[matchFeatures](https://www.mathworks.com/help/vision/ref/matchfeatures.html)、[estgeotform2d](https://www.mathworks.com/help/vision/ref/estgeotform2d.html)。
