%% 速度规划 Dong
%离散化
clear;clc;
load('nrbs.mat');
tic;
vmax=0.5;%设置最大速度
alpha=0.5;%设置三个方向最大加速度
J=1;%设置三个方向最大跃度
Ts=0.02;%时间步长
eps=1;%允许弓高误差

n=3000;%离散点数目
h=1/(n-1);%参数间距
ii=linspace(0,1,n);%参数点

% g=zeros(3,n);%g是每个参数点的曲线值
% g1=zeros(3,n);%每个参数点处对u求导的值
% g2=zeros(3,n);%每个参数点处对u求两次导的值
% g3=zeros(3,n);%每个参数点处对u求三次导的值

[g,~]=nrbeval(nrbs,ii);%g是每个参数点的曲线值
nrbs1=nrbderiv(nrbs);%求一阶导后的B样条曲线
[g1,~]=nrbeval(nrbs1,ii);
nrbs2=nrbderiv(nrbs1);%B样条曲线的二阶导
[g2,~]=nrbeval(nrbs2,ii);
nrbs3=nrbderiv(nrbs2);%B样条曲线的三阶导
[g3,~]=nrbeval(nrbs3,ii);
%% 首先不考虑跃度约束
% 下面的u为优化参数，物理含义为曲线参数对时间求导后平方
% 规划du/dt
ro=((sum(g1.^2)).^(3/2))./sqrt(sum((cross(g1,g2)).^2));

u=(vmax)^2./sum(g1.^2);%u的约束
uu=(8*ro*eps/Ts/Ts)./sum(g1.^2);%弓高误差
u=min(u,uu);
clear uu;
u(1)=0;
u(n)=0;

f = -ones(1,n);
lb = zeros(1,n);
ub = u';
Aeq = [1,zeros(1,n-1);zeros(1,n-1),1];
beq = [0;0];
b = alpha*ones(6*(n-2),1);
% 根据加速度约束构造A矩阵
A = zeros((n-2)*6,n);
for i = 1:n-2
    v1 = -g1(:,i+1)/4/h; % du/dt_{i-1}的系数
    v2 = g2(:,i+1);      % du/dt_{i}的系数   
    v3 = -v1;            % du/dt_{i+1}的系数
    tmp = [v1,v2,v3;
        -v1,-v2,-v3];
    tmp = [zeros(6,i-1),tmp];
    tmp = [tmp,zeros(6,n-size(tmp,2))];
    A(6*i-5:6*i,:) = tmp;
end
u = linprog(f,A,b,Aeq,beq,lb,ub);
u = u';

su=sqrt(u);%sqrt of u
u1=u;
t2=toc

% 下面进行考虑跃度约束的速度规划
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
A=zeros(6*n,n-2);%线性规划不等式约束Ax<=b
A(1:n-2,:)=a1;
A(n-1:2*n-4,:)=a2;
A(2*n-3:3*n-6,:)=a3;
A(3*n-5:3*n-3,1)=J/2/u(2);
for i=1:3
    if g1(i,3)==0 && g1(i,2)==0
        A(3*n-5+i-1,2)=0;
    else
        A(3*n-5+i-1,2)=(g1(i,3).^2)./g1(i,2)/8*su(2)*(n-1)^2;
    end
end
A(3*n-2:3*n,n-2)=J/2/u(n-1);
for i=1:3
    if g1(i,n-2)==0 && g1(i,n-1)==0
        A(3*n-2+i-1,n-3)=0;
    else
        A(3*n-2+i-1,n-3)=(g1(i,n-2).^2)./g1(i,n-1)/8*su(n-1)*(n-1)^2;
    end
end
k1=-k1;
k2=-kk2+(J/2)./u;
k3=-k3;
a1=diag(k2(1,2:n-1))+diag(k1(1,3:n-1),-1)+diag(k3(1,2:n-2),1);
a2=diag(k2(2,2:n-1))+diag(k1(2,3:n-1),-1)+diag(k3(2,2:n-2),1);
a3=diag(k2(3,2:n-1))+diag(k1(3,3:n-1),-1)+diag(k3(3,2:n-2),1);
A(3*n+1:4*n-2,:)=a1;
A(4*n-1:5*n-4,:)=a2;
A(5*n-3:6*n-6,:)=a3;
A(6*n-5:6*n-3,1)=J/2/u(2);
for i=1:3
    if g1(i,3)==0 && g1(i,2)==0
        A(6*n-5+i-1,2)=0;
    else
        A(6*n-5+i-1,2)=-1*(g1(i,3).^2)./g1(i,2)/8*su(2)*(n-1)^2;
    end
end
A(6*n-2:6*n,n-2)=J/2/u(n-1);
for i=1:3
    if g1(i,n-2)==0 && g1(i,n-1)==0
        A(6*n-2+i-1,n-3)=0;
    else
        A(6*n-2+i-1,n-3)=-1*(g1(i,n-2).^2)./g1(i,n-1)/8*su(n-1)*(n-1)^2;
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
A=[A;a1;a2;a3;-a1;-a2;-a3];
A=sparse(A);
b=J*3/2*ones(6*n,1);
b=[b;2*h*alpha*ones(6*n-18,1)];

f=(sum(g1.^2))';
f=-1*f(2:n-1);%目标函数min f^(T)*x

lb=zeros(n-2,1);%x>=lb

un=linprog(f,A,b,[],[],lb,u(2:n-1));
un=un';
u(2:n-1)=un;
su=sqrt(u); % du/dt
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
% 三个方向速度
figure
subplot(3,1,1);
plot(v,qv(1,:),'b--');
title("V_x");
hold;
subplot(3,1,2);
plot(v,qv(2,:),'b--');
title("V_y");
hold;
subplot(3,1,3);
plot(v,qv(3,:),'b--');
xlabel("u");
title("V_z");
% 总速度
figure
qv=sqrt(sum(qv.^2));
plot(v,qv);
xlabel("u");
title("V");


% uu1=diff(u)/h;
% uu1(n)=uu1(n-1);
qa=g1.*uu1/2+g2.*u; % eq.26
% 三个方向加速度
figure
subplot(3,1,1);
plot(v,qa(1,:),'b--');
title("A_x");
hold;
subplot(3,1,2);
plot(v,qa(2,:),'b--');
title("A_y");
hold;
subplot(3,1,3);
plot(v,qa(3,:),'b--');
xlabel("u");
title("A_z");
% 总加速度
figure
qa=sqrt(sum(qa.^2));
plot(v,qa);
xlabel("u");
title("A");

% uu2=diff(uu1)/h;
% uu2(n)=uu2(n-1);

% 跃度
qqj=(g3.*u+g2.*uu1*3/2+g1.*uu2/2).*su;
figure
plot(v,qqj(1,:),v,qqj(2,:),v,qqj(3,:));
legend('x','y','z');
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

C=zeros(3,tn);%位置坐标
C1=zeros(3,tn);%速度
C2=zeros(3,tn);%加速度
C3=zeros(3,tn);%加速度

i=1; 
while abs(ut(i)-1)>0.00001
    
    vu1=fnval(pp,ut(i));
    vu=sqrt(vu1); % 得到 du/dt
    au1=fnval(pp1,ut(i));
    au=au1/2;     % 得到 d2u/dt2
    ju1=fnval(pp2,ut(i));
    ju=ju1/2*vu;  % 得到 d3u/dt3
    
    C(:,i)=nrbeval(nrbs,ut(i));
    [c1,~]=nrbeval(nrbs1,ut(i));
    C1(:,i)=c1*vu;
    [c2,~]=nrbeval(nrbs2,ut(i));
    C2(:,i)=c1*au+c2*vu1;
    [c3,~]=nrbeval(nrbs3,ut(i));
    C3(:,i)=(c1*ju1/2+c2*3/2*au1+c3*vu1)*vu;
  
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
subplot(3,1,1);
plot(tt,C(1,:),'b--');
title("x");
hold;
subplot(3,1,2);
plot(tt,C(2,:),'b--');
title("y");
hold;
subplot(3,1,3);
plot(tt,C(3,:),'b--');
xlabel("t");
title("z");

% 三个方向速度
figure
subplot(3,1,1);
plot(tt,C1(1,:),'b--');
title("V_x");
hold;
subplot(3,1,2);
plot(tt,C1(2,:),'b--');
title("V_y");
hold;
subplot(3,1,3);
plot(tt,C1(3,:),'b--');
xlabel("t");
title("V_z");

vC1=sqrt(sum(C1.^2));
figure
plot(tt,vC1,'b--');
xlabel("t");
title("V");

% 三个方向加速度
figure
subplot(3,1,1);
plot(tt,C2(1,:),'b--');
title("A_x");
hold;
subplot(3,1,2);
plot(tt,C2(2,:),'b--');
title("A_y");
hold;
subplot(3,1,3);
plot(tt,C2(3,:),'b--');
xlabel("t");
title("A_z");

% 三个方向跃度
figure
plot(tt,C3(1,:),tt,C3(2,:),tt,C3(3,:));
xlabel("t");
title("J");

        
    






