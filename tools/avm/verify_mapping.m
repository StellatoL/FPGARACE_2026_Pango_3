function verify_mapping(directory)
% Independently evaluate the double-precision unified fisheye mapping.
% Input camera intrinsics use OpenCV's four-coefficient fisheye convention.
s = load(fullfile(directory, 'matlab_input.mat'));
xout = double(s.xx); yout = double(s.yy);
n = numel(xout); errors = zeros(4, 1);
for c = 1:4
    K = squeeze(s.K(c,:,:)); D = s.D(c,:);
    if s.public
        scale = min(double(s.width)/1200, double(s.height)/1600);
        x = (xout-(double(s.width)-1200*scale)/2)/scale;
        y = (yout-(double(s.height)-1600*scale)/2)/scale;
        switch c
            case 1, px=x; py=y;
            case 2, px=1199-x; py=1599-y;
            case 3, px=1599-y; py=x;
            case 4, px=y; py=1199-x;
        end
        H = squeeze(s.H(c,:,:)); P = squeeze(s.P(c,:,:));
        q = H \ [px(:)'; py(:)'; ones(1,n)];
        q = q ./ q(3,:);
        rays = P \ q;
    else
        x = (xout-(double(s.width)-1)/2)*16/double(s.width);
        y = ((double(s.height)-1)/2-yout)*16/double(s.width);
        R = squeeze(s.R(c,:,:)); t = s.t(c,:)';
        rays = R * [x(:)'; y(:)'; zeros(1,n)] + t;
    end
    rays(3,abs(rays(3,:))<1e-12)=1e-12;
    a = rays(1,:)./rays(3,:); b = rays(2,:)./rays(3,:);
    r = hypot(a,b); theta = atan(r);
    distorted = theta .* (1+D(1)*theta.^2+D(2)*theta.^4+D(3)*theta.^6+D(4)*theta.^8);
    factor = ones(1,n); nz = r > 1e-12; factor(nz) = distorted(nz)./r(nz);
    u = K(1,1)*a.*factor+K(1,3);
    v = K(2,2)*b.*factor+K(2,3);
    expected_u = squeeze(s.expected_uv(c,:,:,1));
    expected_v = squeeze(s.expected_uv(c,:,:,2));
    errors(c) = max(hypot(u(:)-expected_u(:),v(:)-expected_v(:)));
end
assert(max(errors)<1e-7, 'MATLAB unified mapping differs from Python');
result = struct('samples_per_camera',n,'cameras',4,'max_coordinate_difference_px',max(errors),...
    'per_camera_max_difference_px',errors','status','PASS','matlab_version',version);
if isfield(s,'frames')
    [xx,yy]=meshgrid(0:double(s.width)-1,0:double(s.height)-1);
    route=uint16(s.routes); ca=double(bitand(route,3))+1; cb=double(bitand(bitshift(route,-2),3))+1;
    va=bitand(route,16)~=0; vb=bitand(route,32)~=0;
    alpha=double(bitand(bitshift(route,-6),511))/256;
    output=zeros([size(xx),3]);
    for c=1:4
        [u,v]=map_full(s,c,xx,yy);
        raw=squeeze(s.frames(c,:,:)); [sh,sw]=size(raw);
        ix=floor(u); iy=floor(v); fx=u-ix; fy=v-iy;
        valid=isfinite(u)&isfinite(v)&ix>=0&iy>=0&ix+1<sw&iy+1<sh;
        ix=min(max(ix,0),sw-2); iy=min(max(iy,0),sh-2);
        ix(~isfinite(ix))=0; iy(~isfinite(iy))=0;
        idx00=sub2ind([sh,sw],iy+1,ix+1); idx10=sub2ind([sh,sw],iy+1,ix+2);
        idx01=sub2ind([sh,sw],iy+2,ix+1); idx11=sub2ind([sh,sw],iy+2,ix+2);
        for ch=1:3
            rgb=expand_channel(raw,ch);
            sampled=(rgb(idx00).*(1-fx)+rgb(idx10).*fx).*(1-fy)+(rgb(idx01).*(1-fx)+rgb(idx11).*fx).*fy;
            sampled(~valid)=0;
            corrected=min(sampled*double(s.gains(c,ch))/4096,255);
            weight=double((ca==c)&va).*alpha+double((cb==c)&vb).*(1-alpha);
            output(:,:,ch)=output(:,:,ch)+corrected.*weight;
        end
    end
    output=uint8(min(max(floor(output+.5),0),255));
    difference=abs(double(output)-double(s.expected_float));
    result.full_frame_pixels=numel(xx);
    result.float_reference_max_rgb_difference=max(difference,[],'all');
    result.float_reference_differing_channels=nnz(difference);
    assert(result.float_reference_max_rgb_difference<=1,'MATLAB float pixel reference differs');
    imwrite(output,fullfile(directory,'matlab_reference.png'));
end
fid = fopen(fullfile(directory,'matlab_verification.json'),'w');
assert(fid>=0); cleaner=onCleanup(@() fclose(fid));
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
fprintf('MATLAB PASS: %d points, maximum error %.3g px\n',4*n,max(errors));
if isfield(result,'full_frame_pixels')
    fprintf('MATLAB full frame: %d pixels, max RGB difference %g\n',result.full_frame_pixels,result.float_reference_max_rgb_difference);
end
end

function rgb=expand_channel(raw,ch)
switch ch
    case 1, q=bitshift(raw,-11); rgb=double(bitor(bitshift(q,3),bitshift(q,-2)));
    case 2, q=bitand(bitshift(raw,-5),63); rgb=double(bitor(bitshift(q,2),bitshift(q,-4)));
    case 3, q=bitand(raw,31); rgb=double(bitor(bitshift(q,3),bitshift(q,-2)));
end
end

function [u,v]=map_full(s,c,xout,yout)
n=numel(xout); K=squeeze(s.K(c,:,:)); D=s.D(c,:);
if s.public
    scale=min(double(s.width)/1200,double(s.height)/1600);
    x=(xout-(double(s.width)-1200*scale)/2)/scale;
    y=(yout-(double(s.height)-1600*scale)/2)/scale;
    switch c
        case 1, px=x; py=y;
        case 2, px=1199-x; py=1599-y;
        case 3, px=1599-y; py=x;
        case 4, px=y; py=1199-x;
    end
    H=squeeze(s.H(c,:,:)); P=squeeze(s.P(c,:,:));
    q=H\[px(:)';py(:)';ones(1,n)]; q=q./q(3,:); rays=P\q;
else
    x=(xout-(double(s.width)-1)/2)*16/double(s.width);
    y=((double(s.height)-1)/2-yout)*16/double(s.width);
    R=squeeze(s.R(c,:,:)); t=s.t(c,:)'; rays=R*[x(:)';y(:)';zeros(1,n)]+t;
end
rays(3,abs(rays(3,:))<1e-12)=1e-12;
a=rays(1,:)./rays(3,:); b=rays(2,:)./rays(3,:);
r=hypot(a,b); theta=atan(r);
distorted=theta.*(1+D(1)*theta.^2+D(2)*theta.^4+D(3)*theta.^6+D(4)*theta.^8);
factor=ones(1,n); nz=r>1e-12; factor(nz)=distorted(nz)./r(nz);
u=reshape(K(1,1)*a.*factor+K(1,3),size(xout));
v=reshape(K(2,2)*b.*factor+K(2,3),size(xout));
end
