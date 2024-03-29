function res = ReprojectionError(params, wld_points, brd_points, ref_points)
% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
% joint optimization
xc = params(1);
yc = params(2);
zc = params(3);
A = params(4);
B = params(5);
D = params(6);
fx = params(7);
fy = params(8);
cx = params(9);
cy = params(10);
k1 = params(11);
k2 = params(12);
k3 = params(13);
p1 = params(14);
p2 = params(15);

pixel_points1 = zeros(size(brd_points));
pixel_points2 = zeros(size(brd_points));
for i = 1:size(brd_points,3)
    rotation_vector = params((i-1)*6+(16:18));
    translation_vector = params((i-1)*6+(19:21));

    rotation_matrix = rodrigues(rotation_vector);
    world_points = squeeze(wld_points(:,:,i));
    camera_points = world_points*rotation_matrix'+translation_vector';
    normalized_x = camera_points(:,1)./camera_points(:,3);
    normalized_y = camera_points(:,2)./camera_points(:,3);
    normalized_r2 = normalized_x.^2+normalized_y.^2;
    normalized_r4 = normalized_r2.*normalized_r2;
    normalized_r6 = normalized_r4.*normalized_r2;
    temp = 1+k1*normalized_r2+k2*normalized_r4+k3*normalized_r6;

    normalized_xd = normalized_x.*temp+2*p1*normalized_x.*normalized_y+p2*(normalized_r2+2*normalized_x.^2);
    normalized_yd = normalized_y.*temp+2*p2*normalized_x.*normalized_y+p1*(normalized_r2+2*normalized_y.^2);

    pixel_u = fx*normalized_xd+cx;
    pixel_v = fy*normalized_yd+cy;
    pixel_points1(:,:,i) = [pixel_u,pixel_v];
end

projector_center = [xc,yc,zc];
for i = 1:size(brd_points,3)
    rotation_vector = params((i-1)*6+(16:18));
    translation_vector = params((i-1)*6+(19:21));
    rotation_matrix = rodrigues(rotation_vector);

    world_points = squeeze(wld_points(:,:,i));
    camera_points = world_points*rotation_matrix'+translation_vector';

    t = (D-projector_center*[A;B;1])./((camera_points-projector_center)*[A;B;1]);
    camera_points = projector_center+t.*(camera_points-projector_center);

    normalized_x = camera_points(:,1)./camera_points(:,3);
    normalized_y = camera_points(:,2)./camera_points(:,3);
    normalized_r2 = normalized_x.^2+normalized_y.^2;
    normalized_r4 = normalized_r2.*normalized_r2;
    normalized_r6 = normalized_r4.*normalized_r2;
    temp = 1+k1*normalized_r2+k2*normalized_r4+k3*normalized_r6;

    normalized_xd = normalized_x.*temp+2*p1*normalized_x.*normalized_y+p2*(normalized_r2+2*normalized_x.^2);
    normalized_yd = normalized_y.*temp+2*p2*normalized_x.*normalized_y+p1*(normalized_r2+2*normalized_y.^2);

    pixel_u = fx*normalized_xd+cx;
    pixel_v = fy*normalized_yd+cy;
    pixel_points2(:,:,i) = [pixel_u,pixel_v];
end
res = [pixel_points1-brd_points,pixel_points2-ref_points];
end

function m = ssm(a)
m = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
end
function m = rodrigues(rvec)
theta = sqrt(rvec'*rvec);
n = rvec/theta;
m = cos(theta)*eye(3)+(1-cos(theta))*(n*n')+sin(theta)*ssm(n);
end