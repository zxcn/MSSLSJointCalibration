function [img,R,x1,y1] = CorrectReferenceImage(img,params,zref)
% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
% correct reference speckle image
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

center = [xc,yc,zc];

e1 = center'/sqrt(center*center');
e2 = [-center(2);center(1);0]/sqrt(center(1)*center(1)+center(2)*center(2));
e3 = cross(e1,e2);

R = [e1,e2,e3];
[h,w] = size(img);
[u1,v1] = meshgrid(0:w-1,0:h-1);
x1 = (u1-cx)/fx;
y1 = (v1-cy)/fy;

P1 = [x1(:),y1(:),ones(w*h,1)]*zref;
P0 = P1*R';

t = (D-center*[A;B;1])./((P0-center)*[A;B;1]);
PP = t.*(P0-center)+center;

x = PP(:,1)./PP(:,3);
y = PP(:,2)./PP(:,3);

r2 = x.^2+y.^2;
xd = x.*(1 + k1*r2 + k2*r2.^2 + k3*r2.^3)+(p1*(2*x.*y) + p2*(r2+2*x.^2));
yd = y.*(1 + k1*r2 + k2*r2.^2 + k3*r2.^3)+(p1*(r2+2*y.^2) + p2*(2*x.*y));

u2 = fx*xd+cx;
v2 = fy*yd+cy;
u2 = reshape(u2,h,w);
v2 = reshape(v2,h,w);
img = interp2(u1,v1,img,u2,v2,'spline',0);
end