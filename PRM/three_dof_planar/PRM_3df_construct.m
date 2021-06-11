% Probabilistic Roadmap Method (PRM)

% clear
clc;
clear;
close all;

%% Environments
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
node = 4000;
nghb_cnt = 5;
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
temp = [];
% get random nodes until total nodes is reach
while length(temp)<node
    x = rand(1,dc).* [pi*2 pi*2 pi*2]; % random value
    
    % if okay random point, put into array (temp)
    if checkPoint(x,obstacles,three_dof) == false% 碰撞检测
        temp = [temp;x];
%         p3 = plot3(x(1),x(2),x(3),'b.'); % plot nodes
    end
end

%% create node paths (potential paths)

adjacency = cell(node+2,1); % adjacency list
for i=1:node
    distances = distanceCost(temp(i,:),temp);
    [P,I] = sort(distances);
    k = min(numel(P),nghb_cnt+1);
    nghb_nodes = temp(I(2:k),:);
    for j=1:length(nghb_nodes)
        [feasible,~,~] = checkPath(temp(i,:),nghb_nodes(j,:),obstacles,three_dof);
        if feasible
            adjacency{i} = [adjacency{i};I(j+1)];adjacency{I(j+1)}=[adjacency{I(j+1)};i];
%             p4 = plot3([temp(i,1);nghb_nodes(j,1)],[temp(i,2);nghb_nodes(j,2)],[temp(i,3);nghb_nodes(j,3)], 'r-', 'LineWidth', 0.1); % plot potentials lines
            %pause(0.01);
        end
    end
end

e = cputime-t;
fprintf("Construct Time: %.2f sec\n", e);
save('roadmap.mat','adjacency','three_dof','obstacles','temp');

