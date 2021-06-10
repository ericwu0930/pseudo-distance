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
node = 2500;
nghb_cnt = 5;
figure;
grid on;
hold on;
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
start = [45 0 300]*pi/180;
target = [200 130 110]*pi/180;
dc = size(start,2);
%% Random Sampling

t = cputime; % run time
temp = [start;target];
p1 = plot(temp(1,1), temp(1,2),'kh','MarkerFaceColor','g'); %start
p2 = plot(temp(2,1), temp(2,2),'mh','MarkerFaceColor','m'); %target
% get random nodes until total nodes is reach
while length(temp)<node+2
    x = rand(1,dc).* [pi*2 pi*2 pi*2]; % random value
    
    % if okay random point, put into array (temp)
    if det(x) == false% 碰撞检测
        temp = [temp;x];
        p3 = plot3(x(1),x(2),x(3),'b.'); % plot nodes
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
            p4 = plot3([temp(i,1);nghb_nodes(j,1)],[temp(i,2);nghb_nodes(j,2)],[temp(i,3);nghb_nodes(j,3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
            pause(0.01);
        end
    end
end

%% Determine the best path

% node = [idx, historic cost, heuristic cost, total cost, parent idx]
X = [1 0 heu(temp(1,:),target) 0+heu(temp(1,:),target) -1]; % the process through A* algorihtm
p_index = []; % parent index
path_F = false; % path found
while size(X,1)>0
    [A, I] = min(X,[],1);
    n = X(I(4),:); % check smallest cost element
    X = [X(1:I(4)-1,:);X(I(4)+1:end,:)]; % delete element (currently)
    
    if n(1)==2 % check
        path_F = true;
        break;
    end
    
    % iterate through all adjacency from the node
    for mv=1:length(adjacency{n(1),1})
        temp1 = adjacency{n(1),1}(mv);
        
        % if not already in p_index
        if length(p_index)==0 || length(find(p_index(:,1)==temp1))==0
            historic_c = n(2)+hist(temp(n(1),:),temp(temp1,:)); % historic cost
            heuristic_c = heu(temp(temp1,:),target); % heuristic cost
            total_c = historic_c+heuristic_c; % total cost
            
            add = true; % add if better cost
            if length(find(X(:,1)==temp1))>=1 % 如果该近邻还在待搜索列表中
                I = find(X(:,1)==temp1);
                if X(I,4)<total_c
                    add = false;
                else
                    X=[X(1:I-1,:);X(I+1:end,:);]; % 删除该节点
                    add = true;
                end
            end
            if add % 更新待搜索节点列表
                X = [X;temp1 historic_c heuristic_c total_c size(p_index,1)+1]; % add new nodes
            end
        end
    end
    p_index = [p_index;n]; % update list
end

% if path is not found
if ~path_F
    error('No Path!')
end

%retrieve path from parent index
path = temp(n(1),:);
prev = n(5);
while prev>0
    path = [temp(p_index(prev,1),:);path];
    prev = p_index(prev,5);
end

% visualize the path
p5 = plot3(path(:,1),path(:,2),path(:,3),'color','c','LineWidth',2);
%legend([p1 p2 p3(1) p4 p5], {'Start','Target', 'Nodes', 'Potental Paths', 'Path'}, 'Location', 'southeast')
legend([p1 p2 p3(1) p5], {'Start','Target', 'Nodes', 'Path'}, 'Location', 'bestoutside')
% title(['PRM in Environment ', num2str(Environ)]) % title
hold off
figure;
plotLink(a0,l,path,obstacles);

%% Time and cost function (recalculated just in case)

% time
e = cputime-t;

% cost function
d = diff([path(:,1), path(:,2) path(:,3)]);
total_length = sum(sqrt(sum(d.*d,2)));
fprintf("Processing Time: %.2f sec \t Path Length: %.2f bits \n", e, total_length);
%saveas(p5,'image1.png');



%% auxiliary function
%% Cost function calculation

% historic
function h=hist(a,b)
h = sqrt(sum((a-b).^2));
end

% heuristic
function h=heu(c,d)
% h = sqrt(sum((c-d).^2));
h=0;
end

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