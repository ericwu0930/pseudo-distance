clear;
clc;
close all;
hold on;
axis equal;
%% Zhu's problem environment
rec1 = [9 0;10 0;10 8.5;9 8.5];
rec1 = [rec1;rec1(1,:)];
for i = 1:4
    plot(rec1(i:i+1,1),rec1(i:i+1,2),'r-');
end
rec2 = [9 9.5;10 9.5;10 18;9 18];
rec2 = [rec2;rec2(1,:)];
for i = 1:4
    plot(rec2(i:i+1,1),rec2(i:i+1,2),'r-');
end
rec3 = [6 11;7 11;7 12;6 12];
rec3 = [rec3;rec3(1,:)];
for i = 1:4
    plot(rec3(i:i+1,1),rec3(i:i+1,2),'r-');
end
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec4 = [rec4;rec4(1,:)];
for i = 1:4
    plot(rec4(i:i+1,1),rec4(i:i+1,2),'r-');
end
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
obstacles(:,:,4) = rec4;
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
%% initial and end position of manipulator
base = [7 7.5];
l = 2;
thetas = [45 0 -60 -65];
xi = fk(thetas,base,l);
for i = 1:4
    plot(xi(i:i+1,1),xi(i:i+1,2),'r-')
end
thetae = [200 130 110 75];
xe = fk(thetae,base,l);
for i = 1:4
    plot(xe(i:i+1,1),xe(i:i+1,2),'r-')
end

%% solve constrained nonlinear optimization
D=100; % number of variables
% initialization of de parameters
N=20;     % same generation population size
itmax=30; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a(1:N,1)=-1.9; b(1:N,1)=1.9; % bounds on variable x1
a(1:N,2)=-1.1; b(1:N,2)=1.1; % bounds on variable x2
d=(b-a);
basemat=repmat(int16(linspace(1,N,N)),N,1); % used later
basej=repmat(int16(linspace(1,D,D)),N,1); % used later
% random initialization of postions
x=a+d.*rand(N,D);
% evaluate objective for all particles
fx=objf(x(:,1),x(:,2));
% find bset
[fxbest,ixbest] = min(fx);
xbest = x(ixbest,1:D);
% iterate
for it=1:itmax
    permat=bsxfun(@(x,y) x(randperm(y(1))),basemat',N(ones(N,1)))';
    % generate donors by mutation
    v(1:N,1:D)=repmat(xbest,N,1)+F*(x(permat(1:N,1),1:D)-x(permat(1:N,2),1:D));
    % perform recobination
    r=repmat(randi([1 D],N,1),1,D);
    muv=((rand(N,D)<CR)+(basej==r))~=0;
    mux=1-muv;
    u(1:N,1:D)=x(1:N,1:D).*mux(1:N,1:D)+v(1:N,1:D).*muv(1:N,1:D);
    % greedy selection
    fu=objf(u(:,1),u(:,2));
    idx=fu<fx;
    fx(idx)=fu(idx);
    x(idx,1:D)=u(idx,1:D);
    % find best
    [fxbest,ixbest]=min(fx);
    xbest=x(ixbest,1:D);
end
[xbest,fxbest]

%% objective function with penalty function
function fval=objf(theta,thetas,thetae)
% theta     variables to be solved Dx4xN
% thetas    initial configuration  1x4xN
% thetae    end configuration      1x4xN
%% Body
% n  the number of variables
% dc dimension of configuration space
% N  same generation population size
[D,dc,N] = size(theta);
retheta = [thetas;theta;thetae];
% retheta   N x (dc*n)
retheta = reshape(permute(retheta,[2 1 3]),[],N)';
tmp = retheta(:,dc+1:end)-retheta(:,1:end-dc);
fval = sum(tmp.*tmp,2);
% todo: add penalty funciton
p = zeros(N,1);
for i = 1:N
    p(i) = penalty(theta(:,:,i));
end
fval = fval+p;
end

%% penalty function
function p = penalty(theta,rk)
% theta     variables to be solved  D x dc
%% Body
global obstacles;
[D dc] = size(theta);
p = 0;
for i = 1:D % 遍历每一个位置
    x = fk(theta(i,:));
    [r c] = size(x);
    for j = 1:r-1 % 遍历每一个连杆位置
        [~,~,on] = size(obstacles);
        for k = 1:on % 遍历每一个障碍物
            [~,fval,~] = QDistance(obstacles(:,:,k),x(i:i+1,:),Q');
            p = p+fval^2*rk;
        end
    end
end
end

%% forward kinematics of manipulator
function [x] = fk(theta)
% theta  Nx1 configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
global a0;
global l;
theta = theta/180*pi;
% 2-d environment
x = zeros(5,2);
x(1,:) = a0(:)';
for i = 2:5
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end