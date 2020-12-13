% rrt* for Zhu Problem
clear all;
%% Zhu's problem environment
global obstacles;
global a0;
global dc;
global l;

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
maxFailedAttempts = 10000;
display = true;
radius = 0.4;
dc = size(source,2);

tic
RRTree = [source 0 -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    if rand < 0.3
        sample = rand(1,dc).* [pi*2 pi*2 pi*2];
    else
        sample = goal;
    end
    [A, I] = min(distanceCost(RRTree(:,1:dc),sample),[],1);
    closestPoint = RRTree(I(1),1:dc);
    dir = (sample - closestPoint )/norm(sample - closestPoint);
    newPoint = closestPoint + stepsize * dir;
    [feasible,colPoint,cstep] = checkPath(newPoint,closestPoint);
    if feasible == 0
        failedAttempts=failedAttempts+1;
        % todo fix here
        continue;
    end
    
    if distanceCost(newPoint,goal)<disTh
        pathFound = true;
        break;
    end
    
    minCost = distanceCost(newPoint,closestPoint)+RRTree(I,dc+1);
    minParentIdx = I;
    
    distCols = distanceCost(RRTree(:,1:dc),newPoint);
    nearIdx = find(distCols<=radius);
    sizeNear = size(nearIdx,1);
    if sizeNear > 1
        for i =1:sizeNear-1
            nowNode = RRTree(nearIdx(i),:);
            nowPoint = RRTree(nearIdx(i),1:dc);
            [feasible,colPoint,cstep] = checkPath(newPoint,nowPoint);
            if feasible == 1
                costNear = nowNode(dc+1)+distanceCost(nowPoint,newPoint);
                if costNear < minCost
                    minCost = costNear;
                    minParentIdx = nearIdx(i);
                end
            end
        end
    end
    newNode = [newPoint,minCost,minParentIdx];
    RRTree = [RRTree;newNode];
    newNodeIdx = size(RRTree,1);
    
    if sizeNear > 1
        reducedIdx = nearIdx;
        for i =1:sizeNear
            reducedNode = RRTree(reducedIdx(i),:);
            reducedPoint = reducedNode(1:dc);
            [feasible,colPoint,cstep]=checkPath(newPoint,reducedPoint);
            if feasible == 1
                nearCost = reducedNode(dc+1);
                newLine = distanceCost(reducedPoint,newPoint);
                if nearCost > minCost + newLine
                    RRTree(reducedIdx(i),dc+2) = newNodeIdx;
                    RRTree(reducedIdx(i),dc+1) = minCost+newLine;
                end
            end
        end
    end
    failedAttempts=0;
end
toc

path = [goal];
prev = I;
while prev>0
    path = [RRTree(prev,1:dc);path];
    prev = RRTree(prev,dc+2);
end

pathLength=RRTree(end,dc+1);
figure;
plotLink(a0,l,path,obstacles);
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 

if ~pathFound
    error('no path found. maximum attempts reached');
end

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

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end