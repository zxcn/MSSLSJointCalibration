% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
clear
clc
close all

%% load detected points
load points.mat;

circlegrid_image_num = size(points,1);
circlegrid_point_num = size(points{1},1);

circlegrid_world_points = zeros(circlegrid_point_num,3,circlegrid_image_num);
circlegrid_image_points = zeros(circlegrid_point_num,2,circlegrid_image_num);
reference_image_points = zeros(circlegrid_point_num,2,circlegrid_image_num);

for i = 1:circlegrid_image_num
    pnts = points{i};
    circlegrid_world_points(:,:,i) = pnts(:,1:3);
    circlegrid_image_points(:,:,i) = pnts(:,4:5);
    reference_image_points(:,:,i) = pnts(:,6:7);
end

%% camera calibration
camera_params = estimateCameraParameters(circlegrid_image_points, squeeze(circlegrid_world_points(:,1:2,1)), ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters');
%% set init values
% optimization parameters: x
% x(1:3) xc, yc, zc
% x(4:6) A, B, D
% x(7:8) focal length
% x(9:10) principal point
% x(11:13) k1, k2, k3
% x(14:15) p1, p2
% x(6*N+(16:21)) axial angle, translation vector

init_params = zeros(15+circlegrid_image_num*6,1);
init_params(1) = 75;
init_params(6) = 900;
init_params(7:8) = camera_params.FocalLength;
init_params(9:10) = camera_params.PrincipalPoint;
init_params(11:13) = camera_params.RadialDistortion;
init_params(14:15) = camera_params.TangentialDistortion;
for i = 1:circlegrid_image_num
    aa = rotm2axang(camera_params.PatternExtrinsics(i).R);
    init_params((i-1)*6+(16:18)) = aa(1:3)'*aa(4);
    init_params((i-1)*6+(19:21)) = camera_params.PatternExtrinsics(i).Translation;
end

%% joint optimization
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','FunctionTolerance',1e-16,'Display','iter','StepTolerance',1e-16);
f = @(x)ReprojectionError(x,circlegrid_world_points,circlegrid_image_points,reference_image_points);
[params, resnorm, residual] = lsqnonlin(f,init_params,[],[],options);

total_reprojection_error = sqrt(residual(:,1:2:end).^2+residual(:,2:2:end).^2);
circlegird_reprojection_error = mean(total_reprojection_error(:,1:2:end),'all');
speckle_reprojection_error = mean(total_reprojection_error(:,2:2:end),'all');

save params params
%% visulization
circlegird_reprojection_errors = reshape([residual(1:4:end), residual(2:4:end)],circlegrid_point_num,2,circlegrid_image_num);
speckle_reprojection_errors = reshape([residual(3:4:end), residual(4:4:end)],circlegrid_point_num,2,circlegrid_image_num);

params_struct1 = toStruct(camera_params);
params_struct1.ReprojectionErrors = circlegird_reprojection_errors;
camera_params1 = cameraParameters(params_struct1);

params_struct2 = toStruct(camera_params);
params_struct2.ReprojectionErrors = speckle_reprojection_errors;
camera_params2 = cameraParameters(params_struct2);

rt = rigidtform3d;
stereo_params = stereoParameters(camera_params1,camera_params2,rt);

figure, showReprojectionErrors(stereo_params);
xlabel('Calibration board poses','FontName','Times New Roman')
ylabel('Mean error in pixels','FontName','Times New Roman')
title('Mean reprojection error per pose','FontName','Times New Roman')
% xticklabel('FontName','Times New Roman')
legend('\it{U_b}','\it{U_r}','Overall mean error: 0.195 pixels','FontName','Times New Roman')


%% print parameters
fprintf('----------Calibration Results----------\n')
fprintf('Projector center xc yc zc: %.6f %.6f %.6f\n',params(1),params(2),params(3));
fprintf('Reference plane A B D: %.6f %.6f %.6f\n',params(4),params(5),params(6));
fprintf('Focal length fx fy: %.6f %.6f\n',params(7),params(8));
fprintf('Principal point cx cy: %.6f %.6f\n',params(9),params(10));
fprintf('Distortion parameters k1 k2 k3: %.6f %.6f %.6f\n',params(11),params(12),params(13));
fprintf('Distortion parameters p1 p2: %.6f %.6f\n',params(14),params(15));
fprintf('---------------------------------------\n')