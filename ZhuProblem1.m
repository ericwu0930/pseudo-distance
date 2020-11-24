clear all;
clc;
close all;
axis equal;
%% global variables declaration;
global a0;     % base of manipulators;
global l;      % length of link;
global points; % points in c-space of path;
global dc;     % dimension of configuration;
global rk;     % factor of penalty function;
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
plot(a0(1),a0(2),'ko');
thetas = [45 0 -60]*pi/180;
xi = fk(thetas);
thetae = [200 130 110 75]*pi/180;
xe = fk(thetae);

function fval = objf(thetam,thetas,thetae)
fval = norm(thetam-thetas)+norm(thetam-thetae);

end

function p = penalty(thetam,thetas,thetae)
global obstacles;
global epsilon; % safety margin
% thetam N*dc
[N,~]=size(thetam);
p = zeros(N,1);
for i = 1:N
    x=fk(thetam(i,:));
    [r,~]=size(x);
    for j=1:r-1
        [~,~,on]=size(x);
        for k =1:on
            [xstar,fval,~] = QDistance(x(j:j+1,:),obstacles(:,:,k),Q');
            if fval < epsilon % 如果有碰撞
                p = p+fval^2*rk;
            elseif fval == epsilon % active constraint
                a = thetam-thetas;
                b = thetam-thetae;
                z = a/norm(a)+b/norm(b);
                gradient = grand(x(j:j+1,:),obstacles(:,:,k),Q,xstar,theta,j);
                p = p+
            end
        end
        for k = j+1:r-1  % avoid self-conllision
            [~,fval,~] = QDistance(x(k:k+1,:),x(j:j+1,:),Q');
            p = p+fval^2*rk;
        end
    end
end
end

%% gradient
function dd = grad(VA,VB,VQ,xstar,theta,j)
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
dPA1=dPA1(:,j:j+1);
dPA2=dPA2(:,j:j+1);
dPA3=dPA3(:,j:j+1);

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