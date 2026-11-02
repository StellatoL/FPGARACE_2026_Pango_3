function result = estimate_stereo_params(cfg)
%ESTIMATE_STEREO_PARAMS 估计右图到左图的平移 xLeft=xRight+D。
% cfg字段见run_stereo_params.m。输出候选参数、适用性判断及RTL数值预览。
% 单对普通图像不能完成相机内外参标定，也不能证明所有景深均适用。
left = read_rgb(cfg.leftFile);
right = read_rgb(cfg.rightFile);
assert(isequal(size(left),[720 1280 3]) && isequal(size(right),[720 1280 3]), ...
    '输入必须均为1280x720 RGB图像；不要自动缩放后直接用于原分辨率RTL。');
if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
oldRng = rng; cleanupRng = onCleanup(@() rng(oldRng)); %#ok<NASGU>
rng(cfg.seed);

if isempty(cfg.manualPoints)
    assert(exist('detectSURFFeatures','file')~=0, ...
        '自动匹配需要Computer Vision Toolbox，或设置cfg.manualPoints。');
    gl = uint8(round(0.299*double(left(:,:,1))+0.587*double(left(:,:,2))+0.114*double(left(:,:,3))));
    gr = uint8(round(0.299*double(right(:,:,1))+0.587*double(right(:,:,2))+0.114*double(right(:,:,3))));
    pl = detectSURFFeatures(gl); pr = detectSURFFeatures(gr);
    pl = selectStrongest(pl,cfg.maxFeatures); pr = selectStrongest(pr,cfg.maxFeatures);
    [fl,vl] = extractFeatures(gl,pl); [fr,vr] = extractFeatures(gr,pr);
    pairs = matchFeatures(fl,fr,'Unique',true,'MaxRatio',cfg.matchMaxRatio);
    a = vl.Location(pairs(:,1),:); b = vr.Location(pairs(:,2),:);
    assert(size(a,1)>=8,'匹配点少于8对：检查重叠、清晰度，或人工提供对应点。');
    % 单应模型仅帮助拒绝误匹配；最终残差始终按水平平移D计算。
    if exist('estgeotform2d','file')
        [~,inlier,status] = estgeotform2d(b,a,'projective', ...
            'MaxDistance',cfg.ransacThreshold,'Confidence',99.9,'MaxNumTrials',5000);
    else
        [~,inlier,status] = estimateGeometricTransform2D(b,a,'projective', ...
            'MaxDistance',cfg.ransacThreshold,'Confidence',99.9,'MaxNumTrials',5000);
    end
    assert(status==0,'几何一致性估计失败，请重新采图或使用人工对应点。');
    method = 'SURF + projective outlier rejection';
else
    p = double(cfg.manualPoints);
    assert(size(p,2)==4 && size(p,1)>=8 && all(isfinite(p(:))), ...
        'manualPoints必须为至少8行的有限数值Nx4矩阵。');
    a = p(:,1:2); b = p(:,3:4);
    assert(all(all(a>=1 & a<=[1280 720])) && all(all(b>=1 & b<=[1280 720])), ...
        '人工对应点越界。');
    inlier = true(size(a,1),1); % 人工点全部参与，不隐藏误选点
    method = 'manual correspondences';
end
inlier = logical(inlier(:));
assert(nnz(inlier)>=4,'有效匹配点不足。');
delta = double(a)-double(b);
dx = median(delta(inlier,1)); dy = median(delta(inlier,2));
d = round(dx); % 不能将负D取绝对值，否则会掩盖左右顺序错误
residual = hypot(delta(:,1)-d,delta(:,2));
e = sort(residual(inlier));
p95 = e(max(1,ceil(0.95*numel(e))));
overlap = 1280-d;
span = max(a(inlier,:),[],1)-min(a(inlier,:),[],1);
coverageX = span(1)/max(1,min(1280,overlap));
coverageY = span(2)/719;
reasons = {};
if d<0 || d>1278, reasons{end+1}='D超出0..1278：检查左右顺序或视野重叠'; end
if nnz(inlier)<cfg.minInliers, reasons{end+1}='有效对应点不足'; end
if mean(inlier)<cfg.minInlierRatio, reasons{end+1}='几何内点比例过低'; end
if abs(dy)>cfg.maxVerticalPixels, reasons{end+1}='垂直错位超过阈值'; end
if p95>cfg.maxResidualP95, reasons{end+1}='水平平移模型残差过大'; end
if coverageX<cfg.minHorizontalCoverage || coverageY<cfg.minVerticalCoverage
    reasons{end+1}='对应点覆盖范围不足，局部对齐不代表全图对齐';
end
result = struct('leftFile',cfg.leftFile,'rightFile',cfg.rightFile, ...
    'method',method,'D_candidate',d,'dxSubpixel',dx,'dyDiagnosticOnly',dy, ...
    'matchCount',size(a,1),'inlierCount',nnz(inlier),'inlierRatio',mean(inlier), ...
    'horizontalCoverage',coverageX,'verticalCoverage',coverageY, ...
    'residualMedian',median(e),'residualP95',p95, ...
    'rtlModelAcceptable',isempty(reasons),'reasons',{reasons});
result.geometry = [];
% 使用固定名称时防止上次预览被误认为此次结果。
previewFile = fullfile(cfg.outputDir,'rtl_preview.png');
if isfile(previewFile), delete(previewFile); end
if d>=0 && d<=1278
    [preview,geometry] = stereo_rtl_preview(left,right,d);
    result.geometry = geometry;
    imwrite(preview,previewFile);
end
points = table(a(:,1),a(:,2),b(:,1),b(:,2),inlier,delta(:,1),delta(:,2),residual, ...
    'VariableNames',{'leftX','leftY','rightX','rightY','inlier','dx','dy','rtlResidual'});
writetable(points,fullfile(cfg.outputDir,'matched_points.csv'));
save(fullfile(cfg.outputDir,'stereo_params.mat'),'result','cfg','points');
write_text(fullfile(cfg.outputDir,'stereo_params.json'),jsonencode(result));
report = sprintf(['D候选 = %d，亚像素水平偏移 = %.3f，垂直偏移 = %.3f 像素\n' ...
    '内点 = %d/%d，整数D二维残差P95 = %.3f 像素\n' ...
    '水平/垂直覆盖 = %.3f / %.3f\n当前RTL模型初步适用 = %d\n'], ...
    d,dx,dy,nnz(inlier),numel(inlier),p95,coverageX,coverageY,isempty(reasons));
for k=1:numel(reasons), report = [report sprintf('- %s\n',reasons{k})]; end %#ok<AGROW>
if ~isempty(result.geometry)
    report = [report sprintf('画布宽=%d，有效高度=%d，上黑边=%d，下黑边=%d\n', ...
        result.geometry.canvasWidth,result.geometry.activeHeight, ...
        result.geometry.top,result.geometry.bottom)];
end
report = [report sprintf(['仅D可用于当前RTL；dy不是当前RTL支持的补偿参数。\n' ...
    '适用性判断仅针对本图对及阈值，不代替多场景、运动同步及硬件验证。\n' ...
    '预览包含RGB565量化；模型不通过时预览仅供诊断。\n'])];
write_text(fullfile(cfg.outputDir,'report.txt'),report);
vh = sprintf('// Generated candidate. Does not modify the FPGA project.\n// D = %d; inspect report.txt.\n',d);
vh = [vh sprintf('`define STEREO_CALIBRATION_VALID %d\n',isempty(reasons))];
if isempty(reasons)
    vh = [vh sprintf('`define STEREO_D_DEFAULT %d\n',d)];
else
    vh = [vh '// No D macro exported: horizontal-only model was rejected.' newline];
end
write_text(fullfile(cfg.outputDir,'stereo_params.vh'),vh);
fprintf('%s\n输出目录：%s\n',report,cfg.outputDir);
if cfg.showFigures
    figure('Name','Correspondence diagnostics');
    subplot(1,2,1); image([left right]); axis image; hold on;
    plot([a(inlier,1) b(inlier,1)+1280]',[a(inlier,2) b(inlier,2)]','g-');
    title(sprintf('Inliers %d/%d',nnz(inlier),numel(inlier)));
    subplot(1,2,2); scatter(delta(:,1),delta(:,2),12,double(inlier),'filled');
    xlabel('xLeft-xRight'); ylabel('yLeft-yRight'); grid on;
    title(sprintf('D=%d, residual P95=%.2f px',d,p95));
    if ~isempty(result.geometry)
        figure('Name','RTL RGB565 preview'); image(preview); axis image;
        title(sprintf('D=%d, model acceptable=%d',d,isempty(reasons)));
    end
end
end

function rgb = read_rgb(path)
[rgb,map] = imread(path);
assert(isempty(map),'不支持索引图像，请先转换成8位RGB PNG。');
assert(isa(rgb,'uint8'),'请输入8位PNG/JPEG，不要隐式截断16位图像。');
if ismatrix(rgb), rgb = repmat(rgb,1,1,3); end
end

function write_text(path,content)
fid = fopen(path,'w','n','UTF-8');
assert(fid>=0,'无法写入文件：%s',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',content);
end
