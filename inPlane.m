function [inPlane] = inPlane(o,vertexes)
%% Comments
% 判断向量是否在障碍平面内
% o 向量
% vertexes 顶点，按顺时针组成一个平面
%% Body
o=o(:);
[a b c d] = gplane(vertexes(1),vertexes(2),vertexes(3));
if [a b c d]*[o;1]==0
    inPlane = true;
else
    inPlane = false;
end
end