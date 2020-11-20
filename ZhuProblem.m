clear all;
clc;
close all;
hold on;
axis equal;
%% Zhu's problem environment
%% global variables declaration;
global a0;     % base of manipulators;
global l;      % length of link;
global points; % points in c-space of path;
global dc;     % dimension of configuration;
global rk;     % factor of penalty function;
global Q;      % Q 
global obstacles; % environments 

rec1 = [9 0;10 0;10 8.5;9 8.5];
rec1p = [rec1;rec1(1,:)];
for i = 1:4
    plot(rec1p(i:i+1,1),rec1p(i:i+1,2),'r-');
end
rec2 = [9 9.5;10 9.5;10 18;9 18];
rec2p = [rec2;rec2(1,:)];
for i = 1:4
    plot(rec2p(i:i+1,1),rec2p(i:i+1,2),'r-');
end
rec3 = [6 11;7 11;7 12;6 12];
rec3p = [rec3;rec3(1,:)];
for i = 1:4
    plot(rec3p(i:i+1,1),rec3p(i:i+1,2),'r-');
end
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec4p = [rec4;rec4(1,:)];
for i = 1:4
    plot(rec4p(i:i+1,1),rec4p(i:i+1,2),'r-');
end
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
obstacles(:,:,4) = rec4;
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
plot(a0(1),a0(2),'ko');
thetas = [45 0 -60 -65];
xi = fk(thetas);
for i = 1:4
    plot(xi(i:i+1,1),xi(i:i+1,2),'r-')
end
thetae = [200 130 110 75];
xe = fk(thetae);
for i = 1:4
    plot(xe(i:i+1,1),xe(i:i+1,2),'r-')
end

%% solve constrained nonlinear optimization
% function to be minimized
points = 10;
dc = 4;
D=points*dc; % number of variables
%% initialization of de parameters
rk = 0.5;
c = 1.5;
N=20;     % same generation population size
itmax=30; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a=0;b=360;
d=(b-a);
basemat=repmat(int16(linspace(1,N,N)),N,1); % used later
basej=repmat(int16(linspace(1,D,D)),N,1); % used later
while 1
    % random initialization of postions
    x=a+d.*rand(N,D);
    % evaluate objective for all particles
    fx=objf(x,thetas,thetae);
    % find bset
    [fxbest,ixbest] = min(fx);
    xbest = x(ixbest,1:D);
    %% iterate
    for it=1:itmax
        permat=bsxfun(@(x,y) x(randperm(y(1))),basemat',N(ones(N,1)))';
        % generate donors by mutation
        v(1:N,1:D)=repmat(xbest,N,1)+F*(x(permat(1:N,1),1:D)-x(permat(1:N,2),1:D));
        % perform recobination
        r=repmat(randi([1 D],N,1),1,D);
        muv=((rand(N,D)<CR)+(basej==r))~=0;
        mux=1-muv;
        theta(1:N,1:D)=x(1:N,1:D).*mux(1:N,1:D)+v(1:N,1:D).*muv(1:N,1:D);
        % greedy selection
        fu=objf(theta,thetas,thetae);
        idx=fu<fx;
        fx(idx)=fu(idx);
        x(idx,1:D)=theta(idx,1:D);
        % find best
        [fxbest,ixbest]=min(fx);
        xbest=x(ixbest,1:D);
    end
    if penalty(reshape(xbest,4,[])')<=1e-3
        break;
    end
    rk = c*rk;
end
%% result display
xbest = reshape(xbest,4,[])';
for i = 1:size(xbest,1)
    resultp = fk(xbest(i,:));
    for j = 1:size(resultp,1)-1
        plot(resultp(j:j+1,1),resultp(j:j+1,2),'r-');
    end
end
[xbest,fxbest]



%% objective function with penalty function
function fval=objf(theta,thetas,thetae)
% theta     variables to be solved Nx(dc*points)
% thetas    initial configuration  1x4
% thetae    end configuration      1x4
%% Body
% n  the number of variables
% dc dimension of configuration space
% N  same generation population size
global dc;
[N,~]=size(theta);
thetas = repmat(thetas,N,1);
thetae = repmat(thetae,N,1);
tmp = [thetas,theta,thetae];
ksai = tmp(:,dc+1:end)-tmp(:,1:end-dc);
fval = sum(ksai.*ksai,2);
p = zeros(N,1);
for i = 1:N
    p(i) = penalty(reshape(theta(i,:),4,[])');
end
fval = fval+p;
end

%% penalty function
function p = penalty(theta)
% theta     variables to be solved  points x dc
%% Body
global obstacles;
global rk;
global Q;
[points ~] = size(theta);
p = 0;
for i = 1:points % 遍历每一个位置
    x = fk(theta(i,:));
    [r,~] = size(x);
    for j = 1:r-1 % 遍历每一个连杆位置
        [~,~,on] = size(obstacles);
        for k = 1:on % 遍历每一个障碍物
            [~,fval,~] = QDistance(obstacles(:,:,k),x(j:j+1,:),Q');
            p = p+fval^2*rk;
        end
    end
end
end

%% forward kinematics of manipulator
function [x] = fk(theta)
% theta  1xdc configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
global l;
global a0;
dc = length(theta);
theta = theta/180*pi;
% 2-d environment
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:5
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end