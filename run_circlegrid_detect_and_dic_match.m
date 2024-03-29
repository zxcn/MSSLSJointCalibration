% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
clear
clc
close all

%% circle grid width, height, spacing
% real
circle_grid_cols = 14;
circle_grid_rows = 11;
circle_grid_spacing = 60;

% sim
% circle_grid_cols = 16;
% circle_grid_rows = 12;
% circle_grid_spacing = 60;

circle_grid_size = [circle_grid_cols,circle_grid_rows];

[circle_grid_x,circle_grid_y] = meshgrid(-(circle_grid_cols-1)/2:1:(circle_grid_cols-1)/2,-(circle_grid_rows-1)/2:1:(circle_grid_rows-1)/2);
circle_grid_x = circle_grid_spacing*circle_grid_x';
circle_grid_y = circle_grid_spacing*circle_grid_y';
circle_grid_z = zeros(size(circle_grid_x));

%% image path
% real
reference_speckle_image_path = ".\images\Real\ReferencePlane\900.png";
circlegrid_speckle_image_path = '.\images\Real\CircleGrid\Speckle\*.png';
circlegrid_flood_image_path = '.\images\Real\CircleGrid\Flood\*.png';

% sim
% reference_speckle_image_path = ".\images\Sim\ReferencePlane\Gray0001.png";
% circlegrid_speckle_image_path = '.\images\Sim\CircleGrid\Speckle\*.png';
% circlegrid_flood_image_path = '.\images\Sim\CircleGrid\Flood\*.png';

reference_speckle_image_data = double(imread(reference_speckle_image_path));
circlegrid_flood_image_list = dir(circlegrid_flood_image_path);
circlegrid_speckle_image_list = dir(circlegrid_speckle_image_path);

points = cell(length(circlegrid_flood_image_list),1);
%% detect circle and dic matching
for k = 1:length(circlegrid_flood_image_list)
    % read images
    circlegrid_flood_image_data = double(imread([circlegrid_flood_image_list(k).folder,'/',circlegrid_flood_image_list(k).name]));
    circlegrid_flood_image_data = imgaussfilt(circlegrid_flood_image_data,1);
    circlegrid_speckle_image_data = double(imread([circlegrid_speckle_image_list(k).folder,'/',circlegrid_speckle_image_list(k).name]));
    % detect circle array
    detected_circle_points = detectCircleGridPoints(circlegrid_flood_image_data,circle_grid_size,PatternType="symmetric",CircleColor="white");
    detected_circle_points = RefineCircleDetection(circlegrid_flood_image_data,detected_circle_points,circle_grid_size,"white");
    detected_circle_points = detected_circle_points-1;
    % DIC match
    dic_matched_points = DicMatch(reference_speckle_image_data,circlegrid_speckle_image_data,detected_circle_points);
    % plot data
    figure
    subplot(1,3,1), imshow(circlegrid_flood_image_data,[]), title('flood calibration board')
    hold on, scatter(detected_circle_points(:,1)+1,detected_circle_points(:,2)+1,'r+')
    subplot(1,3,2), imshow(circlegrid_speckle_image_data,[]), title('speckle calibration board')
    hold on, scatter(detected_circle_points(:,1)+1,detected_circle_points(:,2)+1,'r+')
    subplot(1,3,3), imshow(reference_speckle_image_data,[]), title('speckle reference plane')
    hold on, scatter(dic_matched_points(:,1)+1,dic_matched_points(:,2)+1,'r+')

    points{k} = [circle_grid_x(:),circle_grid_y(:),circle_grid_z(:),detected_circle_points,dic_matched_points];
	% save points to txt files
	% txt_file_name = ['.\points_',num2str(k),'.txt'];
    % writematrix(points{k},txt_file_name,'Delimiter', ' ');
end

save points points;