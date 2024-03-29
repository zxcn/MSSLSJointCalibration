function c = Zncc(f,g)
% Author: Zhaoxiang Jiang
% Mail: jzx345@163.com
f = f(:);
g = g(:);
N = numel(f);
fmean = sum(f)/N;
f = f-fmean;
gmean = sum(g)/N;
g = g-gmean;
c = f'*g/sqrt((f'*f)*(g'*g));