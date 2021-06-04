clc;clear;
load('path.mat');
dd = path(2:end,:)-path(1:end-1,:);
d = 0;
n = size(path,1)-1;
p = 5;
m = n+p+1;
for i = 1:size(dd,1)
    d = d+norm(dd(i,:));
end
% 给每个数据点分配一个参数
u_ = ones(1,size(path,1));
u_(1)=0;
u_(end) = 1;
for i = 2:length(u_-1)
    u_(i) = u_(i-1)+norm(path(i,:)-path(i-1,:))/d;
end
% construct knot vector
u = zeros(1,m+1);
u(m-p+1:end) = 1;
for i = p+2:m-p
    u(i) = 1/p * sum(u_(i-p:i-1));
end
nrbs6d.form = 'B-NURBS';
nrbs6d.dim = size(path,2);
nrbs6d.number = n+1;
nrbs6d.coefs = path';
nrbs6d.order = 6;
nrbs6d.knot = u;

n=3000;%离散点数目
h=1/(n-1);%参数间距
ii=linspace(0,1,n);%参数点



