% rrt for Zhu Problem
clear all;
%% Zhu's problem environment
global obstacles;
global a0;
global dc;
global l;
global Q;
% rec1 = [9 0;10 0;10 6;9 6];
rec1 = [9 0;10 0;10 8.5;9 8.5];
rec1p = [rec1;rec1(1,:)]; 
% rec2 = [9 12;10 12;10 20;9 20];
rec2 = [9 10.5;10 10.5;10 18;9 18];
rec2p = [rec2;rec2(1,:)];
rec3 = [6 11;7 11;7 12;6 12];
rec3p = [rec3;rec3(1,:)];
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec4p = [rec4;rec4(1,:)];
rec5 = [10 9.5;10 10.5;11 10.5;11 9.5];
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
obstacles(:,:,4) = rec4;
obstacles(:,:,5) = rec5;
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
source = [45 0 -60]*pi/180;
goal = [200 130 110]*pi/180;
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 100000;
dc = size(source,2);

tic
RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
toGoal = false;
while failedAttempts<=maxFailedAttempts
    if rand < 0.3
        sample = rand(1,dc).* [pi*2 pi*2 pi*2]; 
        toGoal = false;
    else
        sample = goal;
        toGoal = true;
    end
    [A, I] = min(distanceCost(RRTree(:,1:dc),sample),[],1);
    closestNode = RRTree(I(1),1:dc);
    dir = (sample - closestNode )/norm(sample - closestNode);
    newPoint = closestNode + stepsize * dir;
    [feasible,colPoint,cstep] = checkPath(newPoint,closestNode);
    if feasible == 0
        % 是否加个判断在哪一点发生的碰撞
        if toGoal == true
           newPoint = getNewPoint(colPoint,cstep*3/4);
        end
    end
    if det(newPoint) == 1
        failedAttempts=failedAttempts+1;
        continue;
    end
    
    if distanceCost(newPoint,goal)<disTh
        pathFound = true;
        break;
    end
    
    RRTree = [RRTree;newPoint I];
    failedAttempts=0; 
end

path = [goal];
prev = I;
while prev>0
    path = [RRTree(prev,1:dc);path];
    prev = RRTree(prev,dc+1);
end
    
pathLength=0;
for i =1:length(path)-1
    pathLength = pathLength+distanceCost(path(i,1:dc),path(i+1,1:dc));
end

figure;
plotLink(a0,l,path,obstacles);
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength);


if ~pathFound
    error('no path found. maximum attempts reached'); 
end

toc

function [feasible,colPoint,step] = checkPath(node,parent)
global dc;
step = 0;
p1 = parent(1:dc);
p2 = node(1:dc);
colPoint = []; 
feasible = 1;
dir = (p2-p1)/norm(p2-p1);
for i=0:0.05:sqrt(sum((p2-p1).^2))
    feasible = ~det(p1+i*dir);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
end
end

function isCols = det(config)
global obstacles;
x = fk(config);
[r,~] = size(x);
isCols = 0;
for i = 1:r-1
    [~,~,on] = size(obstacles);
    for j = 1:on
        isCols = gjk2d(x(i:i+1,:),obstacles(:,:,j));
        if isCols == 1
            return;
        end
    end
    for j = i+2:r-1
        isCols = gjk2d(x(i:i+1,:),x(j:j+1,:));
        if isCols == 1
            return;
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
global dc;
% 2-d environment
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:dc+1
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end

function newPoint = getNewPoint(point,cstep)
%% 通过Q距离的微分得到新的点
global obstacles;
global Q;
x = fk(point);
dd = 0;
for i = 1:size(x,1)-1
    for j = 1:size(obstacles,3)
        [xstar,fval,qe]=qDistance2d(x(i:i+1,:),obstacles(:,:,j),Q);
        if fval<=0
            dd = dd + gradN(x(i:i+1,:),obstacles(:,:,j),Q,xstar,point,i,qe);
        end
    end
end
dd = dd(:)';
newPoint = point+cstep*dd/norm(dd);
end

function dd = gradN(VA,VB,VQ,xstar,theta,j,qe)
global l;
[~,cq]=size(VQ);
[ra,~]=size(VA);
[rb,~]=size(VB);
if sum(xstar~=0) ~= cq+2
    dd = 0;
    return;
end
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
PQ = VQ(qe,:)';
tmp = VA(anz,:);
PA=tmp(2:end,:)-tmp(1,:);
PA=PA';
tmp = VB(bnz,:);
PB=tmp(2:end,:)-tmp(1,:);
PB=PB';
astar = xstar(anz);
dd(1,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA1(:,anz)*astar);
dd(2,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA2(:,anz)*astar);
dd(3,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA3(:,anz)*astar);
end

function dd = gradP(VA,VB,VQ,xstar,theta,j)
% VA vertices of A
% VB vertices of B
% VQ vertices of Q
% xstar best solution
% theta joint configuration of link
% j jth link
global l;
[~,cq]=size(VQ);
[ra,~]=size(VA);
[rb,~]=size(VB);
if sum(xstar~=0) ~= cq+2
    dd = 0;
    return;
end
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
PQ = VQ(qnz,:)';
tmp = VA(anz,:);
PA=tmp(2:end,:)-tmp(1,:);
PA=PA';
tmp = VB(bnz,:);
PB=tmp(2:end,:)-tmp(1,:);
PB=PB';
astar = xstar(anz);
dd(1,:)=[ones(1,size(PQ,2)),zeros(1,cq-size(PQ,2))]*inv([PQ,-PA,PB])*(-dPA1(:,anz)*astar);
dd(2,:)=[ones(1,size(PQ,2)),zeros(1,cq-size(PQ,2))]*inv([PQ,-PA,PB])*(-dPA2(:,anz)*astar);
dd(3,:)=[ones(1,size(PQ,2)),zeros(1,cq-size(PQ,2))]*inv([PQ,-PA,PB])*(-dPA3(:,anz)*astar);
end

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end