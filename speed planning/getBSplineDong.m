function t2 = getBSplineDong(np)
%% 构造B样条曲线
load('path.mat');
dd = path(2:end,:)-path(1:end-1,:);
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
for i = 2:length(u_-1)
    u_(i) = u_(i-1)+norm(path(i,:)-path(i-1,:))/d;
end
% construct knot vector
u = zeros(1,m+1);
u(m-p+1:end) = 1;
for i = p+2:m-p
    u(i) = 1/p * sum(u_(i-p:i-1));
end
path = path';
nrbs_1 =  nrbmak(path(1:3,:),u);
nrbs_2 = nrbmak(path(4:end,:),u);

tic;
vmax=0.5;%设置最大速度
alpha=0.5;%设置三个方向最大加速度
J=1;%设置三个方向最大跃度
Ts=0.02;%时间步长
eps=1;%允许弓高误差

n=np;%离散点数目
h=1/(n-1);%参数间距
ii=linspace(0,1,n);%参数点

[g_1,~]=nrbeval(nrbs_1,ii);%g是每个参数点的曲线值
nrbs1_1=nrbderiv(nrbs_1);%求一阶导后的B样条曲线
[g1_1,~]=nrbeval(nrbs1_1,ii);
nrbs2_1=nrbderiv(nrbs1_1);%B样条曲线的二阶导
[g2_1,~]=nrbeval(nrbs2_1,ii);
nrbs3_1=nrbderiv(nrbs2_1);%B样条曲线的三阶导
[g3_1,~]=nrbeval(nrbs3_1,ii);

[g_2,~]=nrbeval(nrbs_2,ii);%g是每个参数点的曲线值
nrbs1_2=nrbderiv(nrbs_2);%求一阶导后的B样条曲线
[g1_2,~]=nrbeval(nrbs1_2,ii);
nrbs2_2=nrbderiv(nrbs1_2);%B样条曲线的二阶导
[g2_2,~]=nrbeval(nrbs2_2,ii);
nrbs3_2=nrbderiv(nrbs2_2);%B样条曲线的三阶导
[g3_2,~]=nrbeval(nrbs3_2,ii);

g1 = [g1_1;g1_2];
g2 = [g2_1;g2_2];
g3 = [g3_1;g3_2];

%% 首先不考虑跃度约束
% 下面的u为优化参数，物理含义为曲线参数对时间求导后平方
% 规划du/dt
%ro=((sum(g1.^2)).^(3/2))./sqrt(sum((cross(g1,g2)).^2));

u=(vmax)^2./sum(g1.^2);%u的约束
%uu=(8*ro*eps/Ts/Ts)./sum(g1.^2);%弓高误差
%u=min(u,uu);
%clear uu;
u(1)=0;
u(n)=0;

f = -ones(1,n);
lb = zeros(1,n);
ub = u';
Aeq = [1,zeros(1,n-1);zeros(1,n-1),1];
beq = [0;0];
b = alpha*ones(2*dim*(n-2),1);
% 根据加速度约束构造A矩阵
A = zeros((n-2)*2*dim,n);
for i = 1:n-2
    v1 = -g1(:,i+1)/4/h; % du/dt_{i-1}的系数
    v2 = g2(:,i+1);      % du/dt_{i}的系数   
    v3 = -v1;            % du/dt_{i+1}的系数
    tmp = [v1,v2,v3;
        -v1,-v2,-v3];
    tmp = [zeros(2*dim,i-1),tmp];
    tmp = [tmp,zeros(2*dim,n-size(tmp,2))];
    A(2*dim*i-2*dim+1:2*dim*i,:) = tmp;
end
u = linprog(f,A,b,Aeq,beq,lb,ub);
u = u';

su=sqrt(u);%sqrt of u
u1=u;
t2=toc
