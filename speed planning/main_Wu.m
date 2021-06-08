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

%% 下面进行考虑跃度约束的速度规划
tic
k1=su.*(g1/2/h^2-3*g2/4/h);
kk1=k1;
kk2=su.*(g3-g1/h^2);
k2=kk2+(J/2)./u;
k3=su.*(g1/2/h^2+3*g2/4/h);
kk3=k3;
a1=diag(k2(1,2:n-1))+diag(k1(1,3:n-1),-1)+diag(k3(1,2:n-2),1);
a2=diag(k2(2,2:n-1))+diag(k1(2,3:n-1),-1)+diag(k3(2,2:n-2),1);
a3=diag(k2(3,2:n-1))+diag(k1(3,3:n-1),-1)+diag(k3(3,2:n-2),1);
a4=diag(k2(4,2:n-1))+diag(k1(4,3:n-1),-1)+diag(k3(4,2:n-2),1);
a5=diag(k2(5,2:n-1))+diag(k1(5,3:n-1),-1)+diag(k3(5,2:n-2),1);
a6=diag(k2(6,2:n-1))+diag(k1(6,3:n-1),-1)+diag(k3(6,2:n-2),1);
A=zeros(dim*2*n,n-2);%线性规划不等式约束Ax<=b
A(1:n-2,:)=a1;
A(n-1:2*n-4,:)=a2;
A(2*n-3:3*n-6,:)=a3;
A(3*n-5:4*n-8,:)=a4;
A(4*n-7:5*n-10,:)=a5;
A(5*n-9:6*n-12,:)=a6;
A(6*n-11:6*n-6,1)=J/2/u(2);
for i=1:6
    if g1(i,3)==0 && g1(i,2)==0
        A(6*n-11+i-1,2)=0;
    else
        A(6*n-11+i-1,2)=(g1(i,3).^2)./g1(i,2)/8*su(2)*(n-1)^2;
    end
end
A(6*n-5:6*n,n-2)=J/2/u(n-1);
for i=1:6
    if g1(i,n-2)==0 && g1(i,n-1)==0
        A(6*n-5+i-1,n-3)=0;
    else
        A(6*n-5+i-1,n-3)=(g1(i,n-2).^2)./g1(i,n-1)/8*su(n-1)*(n-1)^2;
    end
end
k1=-k1;
k2=-kk2+(J/2)./u;
k3=-k3;
a1=diag(k2(1,2:n-1))+diag(k1(1,3:n-1),-1)+diag(k3(1,2:n-2),1);
a2=diag(k2(2,2:n-1))+diag(k1(2,3:n-1),-1)+diag(k3(2,2:n-2),1);
a3=diag(k2(3,2:n-1))+diag(k1(3,3:n-1),-1)+diag(k3(3,2:n-2),1);
a4=diag(k2(4,2:n-1))+diag(k1(4,3:n-1),-1)+diag(k3(4,2:n-2),1);
a5=diag(k2(5,2:n-1))+diag(k1(5,3:n-1),-1)+diag(k3(5,2:n-2),1);
a6=diag(k2(6,2:n-1))+diag(k1(6,3:n-1),-1)+diag(k3(6,2:n-2),1);
A(6*n+1:7*n-2,:)=a1;
A(7*n-1:8*n-4,:)=a2;
A(8*n-3:9*n-6,:)=a3;
A(9*n-5:10*n-8,:)=a4;
A(10*n-7:11*n-10,:)=a5;
A(11*n-9:12*n-12,:)=a6;
A(12*n-11:12*n-6,1)=J/2/u(2);
for i=1:6
    if g1(i,3)==0 && g1(i,2)==0
        A(12*n-11+i-1,2)=0;
    else
        A(12*n-11+i-1,2)=-1*(g1(i,3).^2)./g1(i,2)/8*su(2)*(n-1)^2;
    end
end
A(12*n-5:12*n,n-2)=J/2/u(n-1);
for i=1:6
    if g1(i,n-2)==0 && g1(i,n-1)==0
        A(12*n-5+i-1,n-3)=0;
    else
        A(12*n-5+i-1,n-3)=-1*(g1(i,n-2).^2)./g1(i,n-1)/8*su(n-1)*(n-1)^2;
    end
end
k1=-g1+2*h*g2;
a1=diag(k1(1,2:n-2))+diag(g1(1,2:n-3),1);
ls=zeros(n-3,1);
ls(n-3)=g1(1,n-2);
a1=[a1 ls];
a2=diag(k1(2,2:n-2))+diag(g1(2,2:n-3),1);
ls(n-3)=g1(2,n-2);
a2=[a2 ls];
a3=diag(k1(3,2:n-2))+diag(g1(3,2:n-3),1);
ls(n-3)=g1(3,n-2);
a3=[a3 ls];
a4=diag(k1(4,2:n-2))+diag(g1(4,2:n-3),1);
ls(n-3)=g1(4,n-2);
a4=[a4 ls];
a5=diag(k1(5,2:n-2))+diag(g1(5,2:n-3),1);
ls(n-3)=g1(5,n-2);
a5=[a5 ls];
a6=diag(k1(6,2:n-2))+diag(g1(6,2:n-3),1);
ls(n-3)=g1(6,n-2);
a6=[a6 ls];
A=[A;a1;a2;a3;a4;a5;a6;-a1;-a2;-a3;-a4;-a5;-a6];
A=sparse(A);
b=J*3/2*ones(12*n,1);
b=[b;2*h*alpha*ones(12*n-36,1)];

f=(sum(g1.^2))';
f=-1*f(2:n-1);%目标函数min f^(T)*x

lb=zeros(n-2,1);%x>=lb

un=linprog(f,A,b,[],[],lb,u(2:n-1));
un=un';
u(2:n-1)=un;
su=sqrt(u);
t3=toc
%% 插值
% 对规划的速度进行插值
v=linspace(0,1,n);%运动路径参数点
pp=csape(v,u);%s三次样条插值，(u,(du/dt)^2)
% plot(v,u,'o');
% hold on
% plot(v,fnval(pp,v),'g');
pp1=fnder(pp,1);%求一阶导数
pp2=fnder(pp,2); %求二阶导数
%% 下面是绘制速度，加速度，跃度图像的代码，横轴为参数u[0,1]

% uu1=fnval(pp1,v);              %uu1为db/du
% uu2=fnval(pp2,v);             %uu2为d^2b/du^2
uu1=(u(3:n)-u(1:n-2))/2/h;
uu2=u(3:n)+u(1:n-2)-2*u(2:n-1);
uu2=uu2/h/h;
uu1=[uu1(1) uu1 uu1(end)];
uu2=[uu2(1) uu2 uu2(end)];   %%使用差分法得到db/du，d^2b/du^2

% uuu1=(u1(3:n)-u1(1:n-2))/2/h;
% uuu2=u1(3:n)+u1(1:n-2)-2*u1(2:n-1);
% uuu2=uuu2/h/h;
% uuu1=[uuu1(1) uuu1 uuu1(end)];
% uuu2=[uuu2(1) uuu2 uuu2(end)];   %%不考虑跃度约束时

qv=g1.*su; % dC(u)/du·du/dt = dC(u)/dt = V(u)
% 六个关节的速度
figure
subplot(dim,1,1);
plot(v,qv(1,:),'b--');
title("V_1");
hold;
subplot(dim,1,2);
plot(v,qv(2,:),'b--');
title("V_2");
hold;
subplot(dim,1,3);
plot(v,qv(3,:),'b--');
title("V_3");

subplot(dim,1,4);
plot(v,qv(4,:),'b--');
title("V_4");
hold;
subplot(dim,1,5);
plot(v,qv(5,:),'b--');
title("V_5");
hold;
subplot(dim,1,6);
plot(v,qv(6,:),'b--');
title("V_6");
xlabel("u");
% 总速度
% figure
% qv=sqrt(sum(qv.^2));
% plot(v,qv);
% xlabel("u");
% title("V");


% uu1=diff(u)/h;
% uu1(n)=uu1(n-1);
qa=g1.*uu1/2+g2.*u; % eq.26
% 六个关节的加速度
figure
subplot(dim,1,1);
plot(v,qa(1,:),'b--');
title("A_1");
hold;
subplot(dim,1,2);
plot(v,qa(2,:),'b--');
title("A_2");
hold;
subplot(dim,1,3);
plot(v,qa(3,:),'b--');
xlabel("u");
title("A_3");
subplot(dim,1,4);
plot(v,qa(4,:),'b--');
title("A_4");
hold;
subplot(dim,1,5);
plot(v,qa(5,:),'b--');
title("A_5");
hold;
subplot(dim,1,6);
plot(v,qa(6,:),'b--');
xlabel("u");
title("A_6");
% 总加速度
% figure
% qa=sqrt(sum(qa.^2));
% plot(v,qa);
% xlabel("u");
% title("A");

% uu2=diff(uu1)/h;
% uu2(n)=uu2(n-1);

% 跃度
qqj=(g3.*u+g2.*uu1*3/2+g1.*uu2/2).*su;
figure
plot(v,qqj(1,:),v,qqj(2,:),v,qqj(3,:),v,qqj(4,:),v,qqj(5,:),v,qqj(6,:));
legend('q1','q2','q3','q4','q5','q6');
xlabel("u");
title("J");

%% 插补
t=0;
for i=1:n-1
    t=t+1/(su(i)+su(i+1));
end
t=2*h*t;%总运动时间 ？没看懂

tn=ceil(t/Ts);
ut=zeros(1,tn);%插补得到的参数点

C=zeros(dim,tn);%位置坐标
C1=zeros(dim,tn);%速度
C2=zeros(dim,tn);%加速度
C3=zeros(dim,tn);%跃度

i=1;
while abs(ut(i)-1)>0.00001
    
    vu1=fnval(pp,ut(i));
    vu=sqrt(vu1); % 得到 du/dt
    au1=fnval(pp1,ut(i));
    au=au1/2;     % 得到 d2u/dt2
    ju1=fnval(pp2,ut(i));
    ju=ju1/2*vu;  % 得到 d3u/dt3
    
    %     C(1:3,i)=nrbeval(nrbs_1,ut(i));
    %     [c1_1,~]=nrbeval(nrbs1_1,ut(i));
    %     C1(1:3,i)=c1_1*vu;
    %     [c2_1,~]=nrbeval(nrbs2_1,ut(i));
    %     C2(1:3,i)=c1_1*au+c2_1*vu1;
    %     [c3_1,~]=nrbeval(nrbs3_1,ut(i));
    %     C3(1:3,i)=(c1_1*ju1/2+c2_1*3/2*au1+c3_1*vu1)*vu;
    %
    %     C(4:6,i)=nrbeval(nrbs_2,ut(i));
    %     [c1_2,~]=nrbeval(nrbs1_2,ut(i));
    %     C1(4:6,i)=c1_2*vu;
    %     [c2_2,~]=nrbeval(nrbs2_2,ut(i));
    %     C2(4:6,i)=c1_2*au+c2_2*vu1;
    %     [c3_2,~]=nrbeval(nrbs3_2,ut(i));
    %     C3(4:6,i)=(c1_2*ju1/2+c2_2*3/2*au1+c3_2*vu1)*vu;
    
    C(:,i)=my_nrbeval(nrbs,ut(i));
    c1=my_nrbeval(nrbs1,ut(i));
    C1(:,i)=c1*vu;
    c2=my_nrbeval(nrbs2,ut(i));
    C2(:,i)=c1*au+c2*vu1;
    c3=my_nrbeval(nrbs3,ut(i));
    C3(:,i)=(c1*ju1/2+c2*3/2*au1+c3*vu1)*vu;
    
    % for what?
%     for j=1:6
%         if C3(j,i)>1
%             C3(j,i)=1+(C3(j,i)-1)/10;
%         end
%         if C3(j,i)<-1
%             C3(j,i)=-1-(-1-C3(j,i))/10;
%         end
%     end
    
    ut(i+1)=ut(i)+vu*Ts+au*Ts^2/2+ju*Ts^3/6; %使用泰勒公式进行插补
    i=i+1;
end
ut(i:end)=[];%每隔Ts的参数点
C(:,i:end)=[];
C1(:,i:end)=[];
C2(:,i:end)=[];
C3(:,i:end)=[];

%% 下面是绘制位置，速度，加速度图像的代码，横轴为时间t
tt=0:Ts:Ts*(i-2);

% 三个方向位置
figure
subplot(dim,1,1);
plot(tt,C(1,:),'b--');
title("q1");
hold;
subplot(dim,1,2);
plot(tt,C(2,:),'b--');
title("q2");
hold;
subplot(dim,1,3);
plot(tt,C(3,:),'b--');
title("q3");
subplot(dim,1,4);
plot(tt,C(4,:),'b--');
title("q4");
hold;
subplot(dim,1,5);
plot(tt,C(5,:),'b--');
title("q5");
hold;
subplot(dim,1,6);
plot(tt,C(6,:),'b--');
title("q6");
xlabel("t");

% 三个方向速度
figure
subplot(dim,1,1);
plot(tt,C1(1,:),'b--');
title("V_1");
hold;
subplot(dim,1,2);
plot(tt,C1(2,:),'b--');
title("V_2");
hold;
subplot(dim,1,3);
plot(tt,C1(3,:),'b--');
title("V_3");
subplot(dim,1,4);
plot(tt,C1(4,:),'b--');
title("V_4");
hold;
subplot(dim,1,5);
plot(tt,C1(5,:),'b--');
title("V_5");
hold;
subplot(dim,1,6);
plot(tt,C1(6,:),'b--');
title("V_6");
xlabel("t");

% vC1=sqrt(sum(C1.^2));
% figure
% plot(tt,vC1,'b--');
% xlabel("t");
% title("V");

% 三个方向加速度
figure
subplot(dim,1,1);
plot(tt,C2(1,:),'b--');
title("A_1");
hold;
subplot(dim,1,2);
plot(tt,C2(2,:),'b--');
title("A_2");
hold;
subplot(dim,1,3);
plot(tt,C2(3,:),'b--');
title("A_3");
subplot(dim,1,4);
plot(tt,C2(4,:),'b--');
title("A_4");
hold;
subplot(dim,1,5);
plot(tt,C2(5,:),'b--');
title("A_5");
hold;
subplot(dim,1,6);
plot(tt,C2(6,:),'b--');
title("A_6");
xlabel("t");

% 六个方向跃度
figure
plot(tt,C3(1,:),tt,C3(2,:),tt,C3(3,:),tt,C3(4,:),tt,C3(5,:),tt,C3(6,:));
xlabel("t");
title("J");


