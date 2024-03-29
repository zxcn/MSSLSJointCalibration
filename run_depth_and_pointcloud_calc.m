% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
clear
clc
close all

%% load parameters
load params.mat;

zref = round(params(6));
center = [params(1),params(2),params(3)];
fx = params(7);
fy = params(8);
cx = params(9);
cy = params(10);
k1 = params(11);
k2 = params(12);
k3 = params(13);
p1 = params(14);
p2 = params(15);

%% read images
reference_image_path = ".\images\Real\ReferencePlane\900.png";
reference_image_data = double(imread(reference_image_path));
scene_image_path = '.\images\Real\PlasterModel\scene.png';
scene_image_data = double(imread(scene_image_path));
%% correct images
[h,w] = size(reference_image_data);
[reference_image_rectify,R,x,y] = CorrectReferenceImage(reference_image_data,params,zref);
scene_image_rectify = CorrectSceneImage(scene_image_data,params);

reference_image_rectify = reference_image_rectify/max(reference_image_rectify(:));
scene_image_rectify = scene_image_rectify/max(scene_image_rectify(:));
% imwrite(reference_image_rectify,'scene_image_rectify.png');
% imwrite(scene_image_rectify,'reference_image_rectify.png');
%% calculate disparity
cut = 25;
disparity = disparityBM(reference_image_rectify(:,1:end-cut),scene_image_rectify(:,cut+1:end),"DisparityRange",[0,128],"BlockSize",31,"ContrastThreshold",0.2)-cut;
disparity = double([disparity,nan(size(disparity,1),cut)]);
%% disparity to depth
depth = zref./(1-disparity*zref/fx/norm(center));
%% depth to point cloud
m = depth<2500 & depth>0;
m = bwareaopen(m,600,4);
disparity = disparity.*m;
depth = depth.*m;
pc = [x(m).*depth(m),y(m).*depth(m),depth(m)];
pc = pc*R';

figure,imshow(depth,[])
figure,pcshow(pc);
