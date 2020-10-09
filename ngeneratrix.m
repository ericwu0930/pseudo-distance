function [j1g j2g] = ngeneratrix(j1,j2,vertexes,n)
%% 注释
% 得到距离障碍平面最近的母线，如果投影全部在障碍平面之外则返回一定数量的母线，否则返回一条
% j1,j2 连杆两端点
% vertexes 障碍平面顶点
% n 返回母线的数量
[a b c d] = gplane(vertexes(1,:),vertexes(2,:),vertexes(3,:));

end