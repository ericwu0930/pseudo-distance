% test dijkstra function
clear all;
tic;
path = linspace(0,1,20);
path = path';
path = [path ones(20,1)*(2*sqrt(2)+2) ones(20,1)*pi/2];
global a0;
global l;
global n;
l = 2;
a0 = [0 0];
n = 3; % cnt of joints is 3
cPath = track_end_effector_path([pi/4,pi/2,-pi/4],path);
toc;

function [cPath] = track_end_effector_path(q,pPath)
cnt = size(pPath,1);
configures = [q];
pre_idx = [1,1];
adj = cell(1,1);
end_idx = [];
% construct adj table
for i = 2:cnt
    theta = ikine3r(pPath(i,:));
    % to fix : check collision
    cur_cnt = size(theta,1);
    new_idx = [size(configures,1)+1,cur_cnt]; %[start_idx,此位姿位形数]
    configures = [configures;theta];
    for j = 1:cur_cnt
        adj = [adj;cell(1,1)];
        for k = 1:pre_idx(2)
            adj{end} = [adj{end};pre_idx(1)+k-1];
            adj{pre_idx(1)+k-1} = [adj{pre_idx(1)+k-1};length(adj)];
        end
    end
    pre_idx = new_idx;
end
for i = 1:new_idx(2)
    end_idx = [end_idx;new_idx(1)+i-1];
end
cPath = dijkstra(configures,adj,end_idx);
end

function cPath = dijkstra(configures,adj,end_idx)
X = [1,0,0]; % [idx,parent_idx,cost]
p_idx = [];
path_F = false;
while size(X,1)>0
    [A,I] = min(X,[],1);
    n = X(I(3),:);
    X = [X(1:I(3)-1,:);X(I(3)+1:end,:)];
    
    if ismember(n(1),end_idx)
        path_F = true;
        break;
    end
    
    for mv = 1:length(adj{n(1),1})
        temp1 = adj{n(1)}(mv);
        if length(p_idx)==0 || length(find(p_idx(:,1)==temp1))==0
            cost = n(3)+ distanceCost(configures(n(1),:),configures(temp1,:));
            
            add = true;
            if length(find(X(:,1)==temp1))>=1 % 如果该近邻还在待搜索列表中
                I = find(X(:,1)==temp1);
                if X(I,3)<cost
                    add = false;
                else
                    X=[X(1:I-1,:);X(I+1:end,:);]; % 删除该节点
                    add = true;
                end
            end
            if add 
                X = [X;temp1,size(p_idx,1)+1, cost];
            end
        end
    end
    p_idx = [p_idx;n]; % update list
end
cPath = configures(n(1),:);
prev = n(2);
while prev > 0
    cPath = [configures(p_idx(prev,1),:);cPath];
    prev = p_idx(prev,2);
end
end

function theta = ikine3r(p)
global l;
x_g = p(1);
y_g = p(2);
gamma = p(3);
x = ones(1,3);
y = ones(1,3);
theta = zeros(2,3);
x(3) = x_g-l*cos(gamma);
y(3) = y_g-l*sin(gamma);
% solve a 2r planar inverse kinematics
D = (x(3)^2+y(3)^2-2*l^2)/(2*l^2);
if D>1
    theta = nan;
    return;
end
theta(1,2) = atan2(sqrt(1-D^2),D);
theta(2,2) = atan2(-sqrt(1-D^2),D);
theta(1,1) = atan2(y(3),x(3))-atan2(l*sin(theta(1,2)),(l+l*cos(theta(1,2))));
theta(2,1) = atan2(y(3),x(3))-atan2(l*sin(theta(2,2)),(l+l*cos(theta(2,2))));
theta(1,3) = gamma - theta(1,2)-theta(1,1);
theta(2,3) = gamma - theta(2,2)-theta(2,1);
end

function p = to_end_pose(q)
% input configuration and fkine function to get end-effector pose
x = fkine(q);
p  = x(end,:);
p = [p,sum(q)];
end

function x = fkine(q)
global a0 l n;
x = zeros(n+1,2);
x(1,:) = a0(:)';
curQ = 0;
for i = 2:n+1
    curQ = curQ+q(i-1);
    x(i,:) = x(i-1,:)+[l*cos(curQ) l*sin(curQ)];
end
end