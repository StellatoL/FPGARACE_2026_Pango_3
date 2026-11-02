% 双摄水平拼接参数入口。只估计当前 RTL 支持的水平平移，不自动修改 RTL。
clear; clc;
root = fileparts(mfilename('fullpath'));
addpath(root);

% 文件名 A/B 不代表硬件通道，必须按实际画面确定左右关系。
% RTL 默认左图=摄像头2(ID1)，右图=摄像头1(ID0)。
cfg.leftFile  = fullfile(root, 'test_img', 'camera_A_1280x720.png');
cfg.rightFile = fullfile(root, 'test_img', 'camera_B_1280x720.png');
cfg.outputDir = fullfile(root, 'output');
cfg.showFigures = true;

% 自动模式需要 Computer Vision Toolbox。人工模式只需提供对应点：
% 每行 [左图x 左图y 右图x 右图y]，MATLAB从1开始，至少8对，分散覆盖重叠区。
% 例：cfg.manualPoints = readmatrix(fullfile(root,'matched_points.csv'));
cfg.manualPoints = [];
cfg.maxFeatures = 3000;
cfg.matchMaxRatio = 0.65;
cfg.ransacThreshold = 3;       % 投影模型仅用于剔除误匹配，不导出给RTL
cfg.minInliers = 12;
cfg.minInlierRatio = 0.35;
cfg.maxVerticalPixels = 2;     % 中值垂直偏移上限
cfg.maxResidualP95 = 3;        % 整数D模型的二维残差95百分位上限
cfg.minHorizontalCoverage = 0.35;
cfg.minVerticalCoverage = 0.40;
cfg.seed = 1;

result = estimate_stereo_params(cfg);
disp(result);
