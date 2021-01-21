%% QD-RRT for hyper-redundant robot
function [time,pathLength,path] = rrtmForHyperRedundant()
display = false;
path = [];
global obstacles;
global a0;
global dc;
global l;
global Q;
%% obstacles
rec1 = [11,-5;15,-5;15,-15;11,-15]/10;
rec2 = [35 -32;85,-32;85 -60;35 -60]/10;
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
%% link
a0=[9,-28]/10;
l=8/10;
source = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi];
goal = [pi/4,pi/3,pi/2,pi/2,pi*3/4,0];
dc = size(source,2);
%% Q
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];

%% QD-RRT parameters
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 2500;

%% start process
tic
RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
toGoal = false;
while failedAttempts<=maxFailedAttempts
    if rand < 0.5
        sample = rand(1,dc).* ones(1,dc)*pi*2;
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
        if toGoal == true
            adjustAttempts = 1;
            while adjustAttempts < 4
                newPoint = getNewPoint(colPoint,cstep);
                if ~det(newPoint)
                    break;
                end
                adjustAttempts =  adjustAttempts+1;
            end
            if adjustAttempts >= 4
                failedAttempts=failedAttempts+1;
                continue;
            end
        else
            failedAttempts=failedAttempts+1;
            continue;
        end
    end
    RRTree = [RRTree;newPoint I];
    
    if distanceCost(newPoint,goal)<disTh
        pathFound = true;
        break;
    end
    
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

if display == true
    figure;
    plotLink(a0,l,path,obstacles);
end

time = toc;
fprintf('processing time=%d \nPath Length=%d \n\n', time,pathLength);


if ~pathFound
    time = inf;
    pathLength = nan;
    fprintf('no path found. maximum attempts reached');
end
end

%% auxiliary function
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
global dc;
dd = zeros(dc,1);
[~,cq]=size(VQ);
[ra,~]=size(VA);
[rb,~]=size(VB);
if sum(xstar~=0) ~= cq+2
    dd = 0;
    return;
end
% origin
% dPA1 = [0,-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1)),-l*sin(theta(1));
%     0,l*cos(theta(1)),l*cos(theta(1)),l*cos(theta(1))];
% dPA2 = [0,0,-l*sin(theta(2)),-l*sin(theta(2));
%     0,0,l*cos(theta(2)),l*cos(theta(2))];
% dPA3 = [0,0,0,-l*sin(theta(3));
%     0,0,0,l*cos(theta(3))];
% dPA1=dPA1(:,j:j+1);
% dPA2=dPA2(:,j:j+1);
% dPA3=dPA3(:,j:j+1);

dPA = zeros(2,dc*2);
i = 1;
while i <= dc*2
    ti = floor(i/2)+1;
    if j>=ti+1
        dPA(:,i) = [-l*sin(theta(ti));l*cos(theta(ti))];
    end
    if j+1>=ti+1
        dPA(:,i+1) = [-l*sin(theta(ti));l*cos(theta(ti))];
    end
    i = i+2;
end

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
% dd(1,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA1(:,anz)*astar);
% dd(2,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA2(:,anz)*astar);
% dd(3,:)=[1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA3(:,anz)*astar);
for i = 1:dc
    tmp =  dPA(:,2*i-1:2*i);
    dd(i,:) = [1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-tmp(:,anz)*astar);
end
end

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end