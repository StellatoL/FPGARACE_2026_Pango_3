function [rgb,g] = stereo_rtl_preview(left,right,d)
%STEREO_RTL_PREVIEW 按当前RTL数值规则生成图像，不模拟握手/DDR/时钟。
% 输入8位RGB，先截断量化RGB565；Q12坐标、Q8插值系数、9位融合权重。
assert(isa(left,'uint8') && isa(right,'uint8') && ...
    isequal(size(left),[720 1280 3]) && isequal(size(right),size(left)), ...
    '输入必须为1280x720 uint8 RGB。');
validateattributes(d,{'numeric'},{'scalar','integer','>=',0,'<=',1278});
w = 1280+d;
h = floor((921600+floor(w/2))/w);
top = floor((720-h)/2);
g = struct('D',d,'canvasWidth',w,'overlapPixels',1280-d, ...
    'activeHeight',h,'top',top,'bottom',720-top-h);
% 精确商余数递推的闭式形式；像素中心在零基坐标整数位置。
u = floor(((0:1279)*2+1)*w*2048/1280)-2048;
ur = u-d*4096;
v = floor(((0:h-1)*2+1)*720*2048/h)-2048;
enabledL = u>=-2048 & u<5240832;
enabledR = ur>=-2048 & ur<5240832;
den = (1279-d)*4096;
weight = max(0,min(256,floor((ur*256+floor(den/2))/den)));
wl = (256-weight).*enabledL;
wr = weight.*enabledR;
wl(enabledL & ~enabledR)=256;
wr(enabledR & ~enabledL)=256;
rgb = zeros(720,1280,3,'uint8');
for c=1:3
    if c==2, step=4; maximum=63; else, step=8; maximum=31; end
    l = floor(double(left(:,:,c))/step);
    r = floor(double(right(:,:,c))/step);
    ls = sample_component(l,u,v);
    rs = sample_component(r,ur,v);
    value = floor((ls.*wl+rs.*wr+128)/256);
    rgb(top+(1:h),:,c) = uint8(round(value*255/maximum));
end
end

function value = sample_component(src,u,v)
u = max(0,min(1279*4096,u)); v = max(0,min(719*4096,v));
x = floor(u/4096); y = floor(v/4096);
fx = floor(mod(u,4096)/16); fy = floor(mod(v,4096)/16)';
x1 = min(1279,x+1); y1 = min(719,y+1);
% MATLAB矩阵下标从1开始；x为列，y为行。
p00 = src(y+1,x+1); p10 = src(y+1,x1+1);
p01 = src(y1+1,x+1); p11 = src(y1+1,x1+1);
horizontal0 = p00.*(256-fx)+p10.*fx;
horizontal1 = p01.*(256-fx)+p11.*fx;
value = floor((horizontal0.*(256-fy)+horizontal1.*fy+32768)/65536);
end
