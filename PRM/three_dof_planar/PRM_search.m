%% Determine the best path
clear;clc;
load('roadmap.mat');

%% 待查询节点
start = [45 0 300]*pi/180;
target = [200 130 110]*pi/180; % 把target放到最尾部
temp = [temp;start;target];
nghb_cnt = 10;
for i = -1:0
    distances = distanceCost(temp(end+i,:),temp);
    [P,I] = sort(distances);
    k = min(numel(P),nghb_cnt+1);
    nghb_nodes = temp(I(2:k),:);
    for j=1:length(nghb_nodes)
        [feasible,~,~] = checkPath(temp(end+i,:),nghb_nodes(j,:),obstacles,three_dof);
        if feasible
            adjacency{end+i} = [adjacency{end+i};I(j+1)];
            adjacency{I(j+1)}=[adjacency{I(j+1)};size(temp,1)+i];
%             p4 = plot3([temp(i,1);nghb_nodes(j,1)],[temp(i,2);nghb_nodes(j,2)],[temp(i,3);nghb_nodes(j,3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
            %pause(0.01);
        end
    end
end

%% plot
figure;
grid on;
hold on;
p1 = plot3(temp(end-1,1), temp(end-1,2),temp(end-1,3),'kh','MarkerFaceColor','g'); %start
p2 = plot3(temp(end,1), temp(end,2),temp(end,3),'mh','MarkerFaceColor','m'); %target
p3 = plot3(temp(1:end-2,1),temp(1:end-2,2),temp(1:end-2,3),'b.');
for i = 1:length(adjacency)
    for j = 1:length(adjacency{i})
        %p4 = plot3([temp(i,1);temp(adjacency{i}(j),1)],[temp(i,2);temp(adjacency{i}(j),2)],[temp(i,3);temp(adjacency{i}(j),3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
%         pause(0.01);
    end
end
title("Roadmap");

t = cputime;
% node = [idx, historic cost, heuristic cost, total cost, parent idx]
X = [size(temp,1)-1 0 heu(temp(end-2,:),target) 0+heu(temp(end-2,:),target) -1]; % the process through A* algorihtm
p_index = []; % parent index
path_F = false; % path found

if isempty(adjacency{end-1}) || isempty(adjacency{end})
    error('No Path!')
end

while size(X,1)>0
    [A, I] = min(X,[],1);
    n = X(I(4),:); % check smallest cost element
    X = [X(1:I(4)-1,:);X(I(4)+1:end,:)]; % delete element (currently)
    
    if n(1)==size(temp,1) % check
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
%title(['PRM in Environment ', num2str(Environ)]) % title
hold off
figure;
plotLink(three_dof,path,obstacles);
%% Time and cost function (recalculated just in case)

% time
e = cputime-t;

% cost function
total_length = 0;
for i = 1: size(path,1)-1
    total_length = total_length + distanceCost(path(i,:),path(i+1,:));
end
fprintf("Search Time: %.2f sec \t Path Length: %.2f bits \n", e, total_length);
%saveas(p5,'image1.png');

%% Cost function calculation

% historic
function h=hist(a,b)
h = sqrt(sum((a-b).^2));
end

% heuristic
function h=heu(c,d)
% h = sqrt(sum((c-d).^2));
h = 0;
end