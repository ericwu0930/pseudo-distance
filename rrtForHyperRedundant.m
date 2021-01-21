%% QD-RRT for hyper-redundant robot
function [time,pathLength] = rrtForHyperRedundant()
display = true;
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
source = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi];
goal = [pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi];
dc = size(source,2);
%% Q
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];

%% QD-RRT parameters
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 500000;

%% start process
tic
RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
toGoal = false;
while failedAttempts<=maxFailedAttempts
    if rand < 0.9
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
        failedAttempts=failedAttempts+1;
        continue;
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
    error('no path found. maximum attempts reached');
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

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end