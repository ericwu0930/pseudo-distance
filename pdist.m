function [dist]=pdist(link,R,vertexes,faces,n)
% pdist 计算机器人到障碍平面的伪距离
% link 连杆的中心点 nx3
% R 包络每个连杆的半径
% vertexes 障碍平面顶点集合 nx3
% faces 障碍平面 nx1的胞元数组
%% Body
if ~iscell(faces)
    error("输入的faces不是胞元");
end
if size(link,1)-1 ~= length(R)
    error("输入的连杆半径个数有误");
end
dist = inf;
% 立方体面个数
faceCnt = size(faces,1);
% 遍历每一个障碍面
for i = 1:faceCnt
    % 遍历每一个连杆
    for j = 1:size(link,1)-1
        % 提取面上的各个顶点
        plane = faces{i,1};
        % 记录顶点个数
        vCnt = length(plane);
        cpps = zeros(vCnt,3);
        for k = 1:vCnt
           cpps(k,:) = vertexes(plane(k));
        end
        dist = min([dist,lineToPlaneDist(link(i,:),link(i+1,:),...
            cpps,n,R(j))]);
    end
end
end