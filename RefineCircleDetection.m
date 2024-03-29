function pntout = RefineCircleDetection(f,pntin,sz,flag)
% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
% refine circle detection using homography
ppi = 100;

[uc,vc] = meshgrid((1:sz(2))*ppi+1,(1:sz(1))*ppi+1);
pntideal = [uc(:),vc(:)];

h = homography_solve(pntideal',pntin');

[u,v] = meshgrid(1:size(f,2),1:size(f,1));
[up,vp] = meshgrid(1:(sz(2)+1)*ppi+1,1:(sz(1)+1)*ppi+1);

pnts = [up(:),vp(:)];
pnts = homography_transform(pnts',h);

g = interp2(u,v,single(f),pnts(1,:),pnts(2,:),"cubic");
g = reshape(g,size(up));

pntrefine = detectCircleGridPoints(g,sz,"PatternType","symmetric","CircleColor",flag);
if isempty(pntrefine)
    pntrefine = detectCircleGridPoints(g',sz,"PatternType","symmetric","CircleColor",flag);
    pntrefine = [pntrefine(:,2),pntrefine(:,1)];
end

pntout = homography_transform(pntrefine',h)';
end

function v = homography_solve(pin, pout)
if ~isequal(size(pin), size(pout))
    error('Points matrices different sizes');
end
if size(pin, 1) ~= 2
    error('Points matrices must have two rows');
end
n = size(pin, 2);
if n < 4
    error('Need at least 4 matching points');
end
% Solve equations using SVD
x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
rows0 = zeros(3, n);
rowsXY = -[X; Y; ones(1,n)];
hx = [rowsXY; rows0; x.*X; x.*Y; x];
hy = [rows0; rowsXY; y.*X; y.*Y; y];
h = [hx hy];
if n == 4
    [U, ~, ~] = svd(h);
else
    [U, ~, ~] = svd(h, 'econ');
end
v = (reshape(U(:,9), 3, 3)).';
end

function y = homography_transform(x, v)
q = v * [x; ones(1, size(x,2))];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
end