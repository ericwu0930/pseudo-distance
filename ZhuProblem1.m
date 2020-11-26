clear all;
clc;
close all;
%% global variables declaration;
global a0;     % base of manipulators;
global l;      % length of link;
global dc;     % dimension of configuration;
global Q;      % Q
global obstacles; % environments
%% Zhu's problem environment
rec1 = [9 0;10 0;10 8.5;9 8.5];
rec1p = [rec1;rec1(1,:)];
rec2 = [9 9.5;10 9.5;10 18;9 18];
rec2p = [rec2;rec2(1,:)];
rec3 = [6 11;7 11;7 12;6 12];
rec3p = [rec3;rec3(1,:)];
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec4p = [rec4;rec4(1,:)];
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
dc = 3;
thetas = [45 0 -60]*pi/180;
thetae = [200 130 110]*pi/180;


%% path planning
path = detAndGen(thetas,thetae);

%% plot environment
function [] = plotEnv
global a0;
global obstacles;
figure;
hold on;
axis equal;
plot(a0(1),a0(2),'ko');
for i = 1:size(obstacles,3)
    rec = obstacles(:,:,i);
    for j = 1:size(rec,1)-1
        plot(rec(j:j+1,1),rec(j:j+1,2),'r-');
    end
    plot([rec(end,1),rec(1,1)],[rec(end,2),rec(1,2)],'r-');
end
end

function handle = plotLink(theta)
plotEnv;
x=fk(theta);
for i = 1:size(x,1)-1
    handle(i)=plot(x(i:i+1,1),x(i:i+1,2),'r-');
end
end

%% detect collision and generate new middle point
function path = detAndGen(thetas,thetae)
thetas = thetas(:)';
thetae = thetae(:)';
if coldet(thetas,thetae) == 1
    [thetam,~] = de(thetas,thetae);
    path1=detAndGen(thetas,thetam);
    path2=detAndGen(thetam,thetae);
    path=[path1;path2(2:end,:)];
else
    path = [thetas;thetae];
end
end

%% collision detection
function isCol = coldet(thetas,thetae)
global obstacles;
global Q;
epsilon = 0;
steps = 100;
dtheta = (thetae-thetas)/steps;
cols = zeros(steps+1,size(thetas,2));
cols(1,:) = thetas;
% for test
% figure;
% hold on;
% axis equal;

for i = 2:steps+1
    cols(i,:)=cols(i-1,:)+dtheta;
    x = fk(cols(i,:));
    [r,~]=size(x);
    % for test
%     plotEnv;
%     handle = plotLink(x);
    
    for j=1:r-1
        [~,~,on]=size(obstacles);
        % for test
        for k =1:on
            [~,fval] = QDistanceNew(x(j:j+1,:),obstacles(:,:,k),Q);
            if fval < epsilon % 如果有碰撞
                isCol = true;
                return;
            end
        end
        for k = j+2:r-1  % avoid self-conllision
            [~,fval] = QDistanceNew(x(k:k+1,:),x(j:j+1,:),Q);
            if fval < epsilon
                isCol = true;
                return;
            end
        end
    end
    % for test
%     delete(handle);
    
end
end

%% DE Algorithm
function [thetam,fval] = de(thetas,thetae)
global rk;
global c;
rk = 0.5; c = 1.5;
%% function to be minimized
D=3; % number of variables
%% initialization of de parameters
N=20;     % same generation population size
itmax=30; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a=0;
b=2*pi;
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
        u(1:N,1:D)=x(1:N,1:D).*mux(1:N,1:D)+v(1:N,1:D).*muv(1:N,1:D);
        % greedy selection
        fu=objf(u,thetas,thetae);
        idx=fu<fx;
        fx(idx)=fu(idx);
        x(idx,1:D)=u(idx,1:D);
        % find best
        [fxbest,ixbest]=min(fx);
        xbest=x(ixbest,1:D);
    end
    if penalty(xbest,thetas,thetae) <= 1e-5
        break;
    end
    rk=c*rk;
end
thetam = xbest;
fval = fxbest;
end

function fval = objf(thetam,thetas,thetae)
fval = sum((thetam-thetas).^2,2).^(1/2)+sum((thetam-thetae).^2,2).^(1/2);
fval = fval+penalty(thetam,thetas,thetae);
end

function p = penalty(thetam,thetas,thetae)
global rk;
global obstacles;
global Q;
epsilon = 0;
% thetam N*dc
[N,~]=size(thetam);
p = zeros(N,1);
% for test 
% figure;
% hold on;
% axis equal;

for i = 1:N
    x=fk(thetam(i,:));
    % for test
%     plotEnv;
%     handle = plotLink(x);
    
    [r,~]=size(x);
    for j=1:r-1
        [~,~,on]=size(obstacles);
        for k =1:on
            [xstar,fval] = QDistanceNew(x(j:j+1,:),obstacles(:,:,k),Q);
            if fval < epsilon % 如果有碰撞
                p(i,:) = p(i,:)+fval^2*rk;
            elseif fval == epsilon % active constraint
                a = thetam-thetas;
                b = thetam-thetae;
                z = a/norm(a)+b/norm(b);
                gradient = grad(x(j:j+1,:),obstacles(:,:,k),Q,xstar,thetam,j);
                p(i,:)=p(i,:)+max([-gradient'*z,0])^2*rk;
            end
        end
        for k = j+2:r-1  % avoid self-conllision
            [~,fval] = QDistanceNew(x(k:k+1,:),x(j:j+1,:),Q);
            fval = min([0,fval]);
            p(i,:) = p(i,:)+fval^2*rk;
        end
    end
    % for test
%     delete(handle);
    
end
end

%% gradient
function dd = grad(VA,VB,VQ,xstar,theta,j)
% VA vertices of A
% VB vertices of B
% VQ vertices of Q
% xstar best solution
% theta joint configuration of link
% j jth link
global l;
[rq,~]=size(VQ);
[ra,~]=size(VA);
[rb,~]=size(VB);
dPA1 = [0,-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1));
    0,l*cos(theta(1)),l*cos(theta(1)),l*cos(theta(1))];
dPA2 = [0,0,-l*sin(theta(2)),-l*sin(theta(2));
    0,0,l*cos(theta(2)),l*cos(theta(2))];
dPA3 = [0,0,0,-l*sin(theta(3));
    0,0,0,l*cos(theta(3))];
dPA1=dPA1(:,j);
dPA2=dPA2(:,j);
dPA3=dPA3(:,j);

anz=xstar(1:ra,:)~=0;
bnz=xstar(ra+1:ra+rb,:)~=0;
qnz=xstar(rb+ra+1:end,:)~=0;
PQ = VQ(qnz)';
tmp = VA(anz);
PA=tmp(2:end,:)-tmp(1,:);
PA=PA';
tmp = VB(bnz);
PB=tmp(2:end,:)-tmp(1,:);
PB=PB';
astar = xstar(anz);
dd(1,:)=eye(size(PQ,2))*inv([PQ,-PA,PB])*(dPA1(:,anz)*astar);
dd(2,:)=eye(size(PQ,2))*inv([PQ,-PA,PB])*(dPA2(:,anz)*astar);
dd(3,:)=eye(size(PQ,2))*inv([PQ,-PA,PB])*(dPA3(:,anz)*astar);
end

%% forward kinematics of manipulator
function [x] = fk(theta)
% theta  1xdc configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
global l;
global a0;
global dc;
% 2-d environment
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:dc+1
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end