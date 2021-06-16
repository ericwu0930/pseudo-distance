clear;clc;
load('path.mat');
dd = diff(path);
d = 0;
n = size(path,1)-1;
p = 5;
m = n+p+1;
dim = 6;
for i = 1:size(dd,1)
    d = d+norm(dd(i,:));
end
% 给每个数据点分配一个参数
u_ = ones(1,size(path,1));
u_(1)=0;
u_(end) = 1;
for i = 2:length(u_)-1
    u_(i) = u_(i-1)+norm(dd(i,:))/d;
end
% construct knot vector
u = zeros(1,m+1);
u(m-p+1:end) = 1;
for i = p+2:m-p
    u(i) = 1/p * sum(u_(i-p:i-1));
end

P = getControlPoints(path,u_,u,p);

P = P';
path = path';
% nrbs = nrbs6d(P,u);
nrbs = nrbs6dim(path,u);
% nrbs_1 =  nrbmak(path(1:3,:),u);
% nrbs_2 = nrbmak(path(4:end,:),u);