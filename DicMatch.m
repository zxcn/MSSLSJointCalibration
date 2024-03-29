function matched_points = DicMatch(reference_plane_image,board_speckle_image,circle_points)
% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
% dic match
InterpCoef=griddedInterpolant({1:1:size(reference_plane_image,1),1:1:size(reference_plane_image,2)}, reference_plane_image,'spline');
[dFdx,dFdy]=imgradientxy(board_speckle_image,'prewitt');

subset_size = 35;
half_subset_size = (subset_size-1)/2;

[dX,dY] = meshgrid(-half_subset_size:half_subset_size);

search_range_u = 5;
search_range_v = 100;

StopCritVal=1e-4;
matched_points = zeros(size(circle_points));
disp('DIC match begin.')
for index = 1:size(circle_points,1)
    v = circle_points(index,1)+1;
    u = circle_points(index,2)+1;
    [xx,yy] = meshgrid(v-half_subset_size:v+half_subset_size,u-half_subset_size:u+half_subset_size);
    f = interp2(board_speckle_image,xx,yy,'spline');
    dfdx = interp2(dFdx,xx,yy,'spline');
    dfdy = interp2(dFdy,xx,yy,'spline');
    c = zeros(size(reference_plane_image,1)-subset_size+1,size(reference_plane_image,2)-subset_size+1);
    % per pixel search using zncc
    for i = max([1,round(u)-search_range_u]):min([size(reference_plane_image,1),round(u)+search_range_u])
        for j = max([1,round(v)-search_range_v]):min([size(reference_plane_image,2),round(v)+search_range_v])
            if (i-half_subset_size>=1 && i+half_subset_size<=size(reference_plane_image,1) && j-half_subset_size>=1 && j+half_subset_size<=size(reference_plane_image,2))
                h = reference_plane_image(i-half_subset_size:i+half_subset_size,j-half_subset_size:j+half_subset_size);
                c(i,j) = Zncc(h,f);
            end
        end
    end
    % subpixel refine
    [~,B] = max(c(:));
    [max_u,max_v] = ind2sub(size(c),B);
    [P,C,Iter,StopVal]=SubCorr(InterpCoef,f,dfdx,dfdy,subset_size,2,[max_v;max_u],dX(:),dY(:),zeros(12,1),StopCritVal);
    % h = interp2(Itgt,vm+P(1)+dX,um+P(7)+dY,'spline');
    % figure(100),subplot(1,2,1),imshow(f,[]),subplot(1,2,2),imshow(h,[]);
    matched_points(index,1) = max_v+P(1)-1;
    matched_points(index,2) = max_u+P(7)-1;

    fprintf('%d/%d\n',index,size(circle_points,1));
end
disp('DIC match done.')
end