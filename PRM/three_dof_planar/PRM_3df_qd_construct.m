% Probabilistic Roadmap Method (PRM)

% clear
clc;
clear;
close all;

%% Environments
global Q;
global obstacles;
global l;
rec1 = [8.8 0;10.4 0;10.4 8.8;8.8 8.8];
% rec2 = [9 12;10 12;10 20;9 20];
rec2 = [9 9.5;10 9.5;10 18;9 18];
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
node = 400;
nghb_cnt = 5;
adj_step = 0.2; % 调整步长
% figure;
% hold on;
% grid on;
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
dc = 3;
three_dof.a0 = a0;
three_dof.l = l;
three_dof.dc = dc;
%% Random Sampling

t = cputime; % run time
sample_nodes = [];
% get random nodes until total nodes is reach
while length(sample_nodes)<node
    x = rand(1,dc).* [pi*2 pi*2 pi*2]; % random value
    
    % if okay random point, put into array (temp)
    if checkPoint(x,obstacles,three_dof) == false% 碰撞检测
        sample_nodes = [sample_nodes;[x,0]];
    else
        % 使用伪距离生成新的节点
        adjustAttempts = 1;
        while adjustAttempts < 4
            newPoint = getNewPoint(x,adj_step,three_dof);
            if checkPoint(newPoint(1:dc),obstacles,three_dof) == false
                sample_nodes = [sample_nodes;[newPoint,1]];
                break;
            end
            adjustAttempts = adjustAttempts+1;
        end
    end
end

%% create node paths (potential paths)

adjacency = cell(node,1); % adjacency list
for i=1:node
    distances = distanceCost(sample_nodes(i,1:dc),sample_nodes(:,1:dc));
    [P,I] = sort(distances);
    k = min(numel(P),nghb_cnt+1);
    nghb_nodes = sample_nodes(I(2:k),:);
    for j=1:length(nghb_nodes)
        [feasible,~,~] = checkPath(sample_nodes(i,1:dc),nghb_nodes(j,1:dc),obstacles,three_dof);
        if feasible
            adjacency{i} = [adjacency{i};I(j+1)];adjacency{I(j+1)}=[adjacency{I(j+1)};i];
%             p4 = plot3([temp(i,1);nghb_nodes(j,1)],[temp(i,2);nghb_nodes(j,2)],[temp(i,3);nghb_nodes(j,3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
        end
    end
end

e = cputime-t;
fprintf("Construct Time: %.2f sec\n", e);
save('roadmap.mat','adjacency','three_dof','obstacles','sample_nodes','e');

%% 通过Q距离的微分得到新的点
function newPoint = getNewPoint(point,cstep,robot)
global obstacles;
global Q;
x = fk(point,robot);
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