clear
clc

uu = 551:650;
vv = 751:850;

pth = 'D:\VSProjects\monocular_speckle\matlab_impl\plane\Depth\*.png';
list = dir(pth);
img = zeros(1200,1600,55);
for k = 1:length(list)
    img(:,:,k)= double(imread([list(k).folder,'/',list(k).name]));
end

img2 = reshape(img,[1200,1600,5,11]);
img3 = mean(img2,3);

z0 = 600:100:1600;
z = squeeze(mean(img3(uu,vv,:),[1,2]))';

figure, plot(z0,z-z0);