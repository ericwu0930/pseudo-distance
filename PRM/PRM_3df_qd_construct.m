% Probabilistic Roadmap Method (PRM)

% clear
clc;
clear;
close all;

%% Environments
global obstacles;
global a0;
global dc;
global l;
global Q;
rec1 = [9 0;10 0;10 8.5;9 8.5];
% rec2 = [9 12;10 12;10 20;9 20];
rec2 = [9 10.5;10 10.5;10 18;9 18];
rec3 = [6 11;7 11;7 12;6 12];
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec5 = [10 9.5;10 10.5;11 10.5;11 9.5];
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
obstacles(:,:,4) = rec4;
obstacles(:,:,5) = rec5;
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
node = 500;
nghb_cnt = 5;
adj_step = 0.2; % 调整步长
% figure;
% hold on;
% grid on;
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
start = [45 0 300]*pi/180;
target = [200 130 110]*pi/180;
dc = size(start,2);
%% Random Sampling

t = cputime; % run time
temp = [start;target];
% get random nodes until total nodes is reach
while length(temp)<node+2
    x = rand(1,dc).* [pi*2 pi*2 pi*2]; % random value
    
    % if okay random point, put into array (temp)
    if det(x) == false% 碰撞检测
        temp = [temp;x];
    else
        % 使用伪距离生成新的节点
        adjustAttempts = 1;
        while adjustAttempts < 4
            newPoint = getNewPoint(x,adj_step);
            if det(newPoint) == false
                temp = [temp;newPoint];
                break;
            end
            adjustAttempts = adjustAttempts+1;
        end
    end
end

%% create node paths (potential paths)

adjacency = cell(node+2,1); % adjacency list
for i=1:node+2
    distances = distanceCost(temp(i,:),temp);
    [P,I] = sort(distances);
    k = min(numel(P),nghb_cnt+1);
    nghb_nodes = temp(I(2:k),:);
    for j=1:length(nghb_nodes)
        [feasible,~,~] = checkPath(temp(i,:),nghb_nodes(j,:));
        if feasible
            adjacency{i} = [adjacency{i};I(j+1)];adjacency{I(j+1)}=[adjacency{I(j+1)};i];
%             p4 = plot3([temp(i,1);nghb_nodes(j,1)],[temp(i,2);nghb_nodes(j,2)],[temp(i,3);nghb_nodes(j,3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
            %pause(0.01);
        end
    end
end

e = cputime-t;
fprintf("Construct Time: %.2f sec\n", e);
save('roadmap.mat','a0','adjacency','dc','l','obstacles','start','target','temp');


%% collision detection function
function [feasible,colPoint,step] = checkPath(node,parent)
global dc;
global obstacles;
global a0;
global l;
step = 0;
p2 = parent(1:dc);
p1 = node(1:dc);
colPoint = [];
feasible = 1;
if norm(p2-p1)<1e-3
    return;
end
delta = getMinDistVec(node,parent);
dir = delta/norm(delta);
for i=0:0.05:sqrt(sum((delta).^2))
    feasible = ~det(p1+i*dir);
%     plotLink(a0,l,p1+i*dir,obstacles);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
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

function isCols = det(config)
global obstacles;
x = fk(config);
[r,~] = size(x);
isCols = 0;
for i = 1:r-1
    [~,~,on] = size(obstacles);
    for j = 1:on
        shape1.XData = x(i:i+1,1);
        shape1.YData = x(i:i+1,2);
        shape2.XData = obstacles(:,1,j);
        shape2.YData = obstacles(:,2,j);
        isCols = gjk2d(shape1,shape2,6);
        if isCols == 1
            return;
        end
    end
    for j = i+2:r-1
        shape1.XData = x(i:i+1,1);
        shape1.YData = x(i:i+1,2);
        shape2.XData = x(j:j+1,1);
        shape2.YData = x(j:j+1,2);
        isCols = gjk2d(shape1,shape2,6);
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

% auxiliary function
function minDistVec = getMinDistVec(p1,p2)
p1 = mod(p1,2*pi);
p2 = mod(p2,2*pi);
minDistVec = p2-p1;
revCol = minDistVec>pi;
minDistVec(revCol) = minDistVec(revCol) - 2*pi;
revCol = minDistVec<-pi;
minDistVec(revCol) = minDistVec(revCol) + 2*pi;
end

function h = distanceCost(a,b)
tmp = getMinDistVec(a,b);
h = sum(tmp.^2,2).^(1/2);
end