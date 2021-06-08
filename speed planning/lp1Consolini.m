function t2 = lp1Consolini(np)
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
nrbs = nrbs6d(path,u);
% nrbs_1 =  nrbmak(path(1:3,:),u);
% nrbs_2 = nrbmak(path(4:end,:),u);

tic;
vmax=0.5;%设置最大速度
alpha=0.5;%设置三个方向最大加速度
J=1;%设置三个方向最大跃度
Ts=0.02;%时间步长
%eps=1;%允许弓高误差

n=np;%离散点数目
h=1/(n-1);%参数间距
ii=linspace(0,1,n);%参数点
%
% [g_1,~]=nrbeval(nrbs_1,ii);%g是每个参数点的曲线值
% nrbs1_1=nrbderiv(nrbs_1);%求一阶导后的B样条曲线
% [g1_1,~]=nrbeval(nrbs1_1,ii);
% nrbs2_1=nrbderiv(nrbs1_1);%B样条曲线的二阶导
% [g2_1,~]=nrbeval(nrbs2_1,ii);
% nrbs3_1=nrbderiv(nrbs2_1);%B样条曲线的三阶导
% [g3_1,~]=nrbeval(nrbs3_1,ii);
%
% [g_2,~]=nrbeval(nrbs_2,ii);%g是每个参数点的曲线值
% nrbs1_2=nrbderiv(nrbs_2);%求一阶导后的B样条曲线
% [g1_2,~]=nrbeval(nrbs1_2,ii);
% nrbs2_2=nrbderiv(nrbs1_2);%B样条曲线的二阶导
% [g2_2,~]=nrbeval(nrbs2_2,ii);
% nrbs3_2=nrbderiv(nrbs2_2);%B样条曲线的三阶导
% [g3_2,~]=nrbeval(nrbs3_2,ii);

g=my_nrbeval(nrbs,ii);%g是每个参数点的曲线值
nrbs1=my_nrbderiv(nrbs);%求一阶导后的B样条曲线
g1=my_nrbeval(nrbs1,ii);
nrbs2=my_nrbderiv(nrbs1);%B样条曲线的二阶导
g2=my_nrbeval(nrbs2,ii);
nrbs3=my_nrbderiv(nrbs2);%B样条曲线的三阶导
g3=my_nrbeval(nrbs3,ii);

% g1 = [g1_1;g1_2];
% g2 = [g2_1;g2_2];
% g3 = [g3_1;g3_2];

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

%引入变量
yita=sign(g1.*g2)*0.5+0.5;
k1=-g1+(1-yita).*g2*2*h;
k2=g1+g2.*yita*2*h;
kb=-k1./k2;
kf=1./kb;
bb=abs((alpha*2*h)./k2);
bf=abs((alpha*2*h)./k1);
%前向扫描
x=ones(100,1);
for i=1:n-1
    x(1)=u(i);
    y=-inf;
    z=inf;
    j=1;
    while z-y>0.00001
        [y,kba]=min(kb(:,i)*x(j)+bb(:,i)); % 求算B
        if u(i+1)<y
            y=u(i+1);
            kba=4; % flag
        end
        [z,jba]=max((x(j)-bf(:,i))./kf(:,i)); % 求算F^{-1}
        if kba<4
            x(j+1)=(kf(jba,i)*bb(kba,i)+bf(jba,i))/(1-kf(jba,i)*kb(kba,i));
        else
            x(j+1)=kf(jba,i)*u(i+1)+bf(jba,i);
        end
        if x(j+1)<x(j) % 严格递减
        else
            x(j+1)=x(j);
        end
        j=j+1;
    end
    u(i)=x(j);
    u(i+1)=y;
end
%反向扫描
i=n-1;
while i>0
    if min(kf(:,i)*u(i+1)+bf(:,i))<u(i)
        u(i)=min(kf(:,i)*u(i+1)+bf(:,i));
    end
    i=i-1;
end
su=sqrt(u);%du/dt
u1=u;
t2=toc