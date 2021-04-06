% rrt for ur10(3d)
function [time,pathLength,path] = qdrrtForUr()
%% define global variables
global obstacles of;
global ur10;
global dc;
global Q;
global bb bf;
global gexp; % gradient expression
%% define UR10 and bounding box
mdl_ur10;
% Bounding box size
bb1 = [0.16/2 -0.13 0.16/2;
    0.16/2  -0.13 -0.16/2;
    -0.16/2 -0.13 -0.16/2;
    -0.16/2 -0.13 0.16/2;
    0.16/2 0.05 0.16/2;
    0.16/2 0.05 -0.16/2;
    -0.16/2 0.05 -0.16/2;
    -0.16/2 0.05 0.16/2];
% bounding box face
bf(:,:,1)=[1,2,6,5;3,4,8,7;1,2,3,4;5,6,7,8;1,4,8,5;2,3,7,6];
% for i = 1:6
% h = patch(bb1(bf1(i,:),1),bb1(bf1(i,:),2),bb1(bf1(i,:),3),'r')
% set(h,'facealpha',0.2);
% end
bb2 = [-0.04 0.05 0.08+0.18;
    -0.04 -0.13 0.08+0.18;
    0.7 0.05 0.08+0.18;
    0.7 -0.13 0.08+0.18;
    -0.04 0.05 0.08;
    -0.04 -0.13 0.08;
    0.7 0.05 0.08;
    0.7 -0.13 0.08];
bf(:,:,2)=[1,2,4,3;5,6,8,7;1,3,7,5;2,4,8,6;1,2,6,5;3,4,8,7];
% for i = 1:6
% h = patch(bb2(bf2(i,:),1),bb2(bf2(i,:),2),bb2(bf2(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb3 = [-0.04 0.05 0.09;
    -0.04 -0.13 0.09;
    0.7 0.05 0.09;
    0.7 -0.13 0.09;
    -0.04 0.05 -0.08;
    -0.04 -0.13 -0.08;
    0.7 0.05 -0.08;
    0.7 -0.13 -0.08];
bf(:,:,3)=[1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6;1,2,4,3;5,6,8,7];
% for i = 1:6
% h = patch(bb3(bf3(i,:),1),bb3(bf3(i,:),2),bb3(bf3(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb4 = [0.05 0.05 -0.06;
    0.05 0.05 0.18;
    0.05 -0.05 -0.06;
    0.05 -0.05 0.18;
    -0.05 0.05 -0.06;
    -0.05 0.05 0.18;
    -0.05 -0.05 -0.06;
    -0.05 -0.05 0.18];
bf(:,:,4)=[1,2,4,3;5,6,8,7;1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6];
% for i = 1:6
% h = patch(bb4(bf4(i,:),1),bb4(bf4(i,:),2),bb4(bf4(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb(:,:,1) = bb1;
bb(:,:,2) = bb2;
bb(:,:,3) = bb3;
bb(:,:,4) = bb4;
%% calculate gradient matrix in advance
syms q1 q2 q3 q4;
q = [q1,q2,q3,q4];
% gradient expression: cell(i,j) -> theta(i) link(j)
% each cell -> 3x8 matrix
gexp = cell(4,4);
for i = 1:4
    theta1 = sym(zeros(3,8));
    theta2 = sym(zeros(3,8));
    theta3 = sym(zeros(3,8));
    theta4 = sym(zeros(3,8));
    for j = 1:8
        tmp = ur10.A(1:i,q)*SE3(bb(j,:,i));
        theta1(:,j) = diff(tmp.t,q1);
        theta2(:,j) = diff(tmp.t,q2);
        theta3(:,j) = diff(tmp.t,q3);
        theta4(:,j) = diff(tmp.t,q4);
    end
    gexp(1,i) = mat2cell(theta1,[3]);
    gexp(2,i) = mat2cell(theta2,[3]);
    gexp(3,i) = mat2cell(theta3,[3]);
    gexp(4,i) = mat2cell(theta4,[3]);
end
%% define obstacles
rec1 = [-0.1,-0.5,0.5;
    -0.1,-0.5,0;
    -0.6,-0.5,0.5;
    -0.6,-0.5,0;
    -0.6,-0.8,0.5;
    -0.6,-0.8,0;
    -0.1,-0.8,0.5;
    -0.1,-0.8,0];
% obstacles face
of(:,:,1) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
figure
hold on
axis equal
grid on
for i = 1:6
    h = patch(rec1(of(i,:,1),1),rec1(of(i,:,1),2),rec1(of(i,:,1),3),'g')
    set(h,'facealpha',0.2);
end
rec2 = [0.8250,0.2000,0.7500;
    0.8250,0.2000,0;
    0.6250,0.2000,0.7500;
    0.6250,0.2000,0;
    0.6250,-0.5000,0.7500;
    0.6250,-0.5000,0;
    0.8250,-0.5000,0.7500;
    0.8250,-0.5000,0];
of(:,:,2) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
    h = patch(rec2(of(i,:,2),1),rec2(of(i,:,2),2),rec2(of(i,:,2),3),'g');
    set(h,'facealpha',0.2);
end
rec3 = [0.4356,0.0499,1.6592;
    0.4356,0.0499,1.1592;
    0.3156,0.0499,1.6592;
    0.3156,0.0499,1.1592;
    0.3156,-0.9501,1.6592;
    0.3156,-0.9501,1.1592;
    0.4356,-0.9501,1.6592;
    0.4356,-0.9501,1.1592];
of(:,:,3) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
    h = patch(rec3(of(i,:,3),1),rec3(of(i,:,3),2),rec3(of(i,:,3),3),'g')
    set(h,'facealpha',0.2);
end
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
%% define Q
Q1 = [sqrt(6)/3 -sqrt(2)/3 -1/3];
Q2 = [-sqrt(6)/3 -sqrt(2)/3 -1/3];
Q3 = [0 2*sqrt(2)/3 -1/3];
Q4 = [0 0 1];
Q = [Q1;Q2;Q3;Q4];
%% initial and end position of manipulator
point0 = [20,-45,60,-50,0,0]*pi/180;
point1 = [40,-70+360,60,-50+360,0,0]*pi/180;
point2 = [106,-45+360,60,-50+360,0,0]*pi/180;
point3 = [160,-68,30,-30,0,0]*pi/180;
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 10000;
source = point2;
goal = point3;
dc = size(source,2);
%% main proccession
tic
RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    if rand<=0.3
        sample = rand(1,dc).*[pi*2 pi*2 pi*2 pi*2 pi*2 pi*2];
        toGoal = false;
    else
        sample = goal;
        toGoal = true;
    end
    [A,I] = min(distanceCost(RRTree(:,1:dc),sample),[],1);
    closestNode = RRTree(I(1),1:dc);
    newPoint = getNewPoint(closestNode,sample,stepsize);
    [feasible,colPoint,cstep] = checkPath(newPoint,closestNode);
    if feasible == 0
        if toGoal == true
            adjustAttempts = 1;
            while adjustAttempts < 4
                if cstep == 0
                    cstep = 0.2;
                end
                newPoint = avoidObstacles(colPoint,cstep);
                [A,I] = min(distanceCost(RRTree(:,1:dc),newPoint),[],1);
                closestNode = RRTree(I(1),1:dc);
                [feasible,colPoint,cstep] = checkPath(newPoint,closestNode);
                if feasible == 1
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
    
    failedAttempts = 0;
end

if ~pathFound
    time = inf;
    pathLength = NaN
end

time = toc;
path = [goal];
prev = I;
while prev>0
    path = [RRTree(prev,1:dc);path];
    prev = RRTree(prev,dc+1);
end
pathLength=0;
for i=1:length(path)-1
    pathLength=pathLength+distanceCost(path(i,1:dc),path(i+1,1:dc));
end
end

function newPoint = avoidObstacles(point,cstep)
% avoid obstacles strategy
global obstacles;
global Q;
global ur10;
global bb;
dd = 0;
for i=1:4
    % bounding box translation
    bbt(:,:,i) = transBox(ur10.A(1:i,point),bb(:,:,i));
end
for i = 1:4
    for j = 1:size(obstacles,3)
        % traverse ith bounding box and jth obstacles
        [xstar,fval,qe]=qDistance3d(bbt(:,:,i),obstacles(:,:,j),Q);
        if fval<=0
            dd = dd + gradN(bbt(:,:,i),obstacles(:,:,j),Q,xstar,point,i,qe);
        end
    end
end
dd = dd(:)';
dd = [dd,0,0];
newPoint = point+cstep*dd/norm(dd);
end

function dd = gradN(VA,VB,VQ,xstar,theta,j,qe)
% reference to "A Pseudodistance Function and its Applications"
global gexp;
[~,cq]=size(VQ);
[ra,~]=size(VA);
[rb,~]=size(VB);
if sum(xstar~=0)~=cq+2
    dd=0
    return;
end
anz = xstar(1:ra,:)~=0;
bnz = xstar(ra+1:ra+rb,:)~=0;
PQ = VQ(qe,:)';
tmp = VA(anz,:);
PA = tmp(2:end,:)-tmp(1,:);
PA = PA';
tmp = VB(bnz,:);
PB = tmp(2:end,:)-tmp(1,:);
PB = PB';
astar = xstar(anz);
q1 = theta(1);
q2 = theta(2);
q3 = theta(3);
q4 = theta(4);
dPA1 = eval(gexp{1,j});
dPA2 = eval(gexp{2,j});
dPA3 = eval(gexp{3,j});
dPA4 = eval(gexp{4,j});
dd(1,:) = [1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA1(:,anz)*astar);
dd(2,:) = [1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA2(:,anz)*astar);
dd(3,:) = [1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA3(:,anz)*astar);
dd(4,:) = [1,zeros(1,cq-1)]*inv([PQ,-PA,PB])*(-dPA4(:,anz)*astar);
end

function newPoint = getNewPoint(p1,p2,stepsize)
% p1 -> p2
tmp = getMinDistVec(p1,p2);
dir = tmp/norm(tmp);
newPoint = p1+stepsize*dir;
end

function minDistVec = getMinDistVec(p1,p2)
p1 = mod(p1,2*pi);
p2 = mod(p2,2*pi);
minDistVec = p2-p1;
revCol = minDistVec>pi;
minDistVec(revCol) = minDistVec(revCol) - 2*pi;
revCol = minDistVec<-pi;
minDistVec(revCol) = minDistVec(revCol) + 2*pi;
end

function [feasible,colPoint,step] = checkPath(node,parent)
% check path feasible between parent and node
global dc;
step = 0;
p1 = parent(1:dc);
p2 = node(1:dc);
diffVec = getMinDistVec(p1,p2);
colPoint = [];
feasible = 1;
if norm(diffVec)<1e-3
    return;
end
dir = (diffVec)/norm(diffVec);
for i=0:0.05:norm(diffVec)
    feasible = ~det(p1+i*dir);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
end
end

function isCols = det(config)
% config -- configuration
global obstacles;
global bb;
global ur10;
isCols = 0;
% total 4 bounding boxes for ur10
for i=1:4
    % bounding box translation
    bbt(:,:,i) = transBox(ur10.A(1:i,config),bb(:,:,i));
end
% collision detection
% plot for test
detPlot;
pause(0.01)
for i=1:4
    [~,~,on] = size(obstacles);
    % traverse jth obstacles for ith bounding box
    for j = 1:on
        obj1 = wrapObj(bbt(:,:,i));
        obj2 = wrapObj(obstacles(:,:,j));
        isCols = GJK(obj1,obj2,6);
        if isCols == 1
            delete(h)
            return;
        end
    end
    for j = i+2:4
        obj1 = wrapObj(bbt(:,:,i));
        obj2 = wrapObj(bbt(:,:,j));
        isCols = GJK(obj1,obj2,6);
        if isCols == 1
            delete(h)
            return;
        end
    end
end
delete(h)
end

function obj = wrapObj(vertices)
obj.XData = vertices(:,1);
obj.YData = vertices(:,2);
obj.ZData = vertices(:,3);
end

function box = transBox(T,box)
% translate box under T
for i=1:8
    tmp = T*SE3(box(i,:));
    box(i,:) = tmp.t;
end
end

function h = distanceCost(a,b)
tmp = getMinDistVec(a,b);
h = sum(tmp.^2,2).^(1/2);
end