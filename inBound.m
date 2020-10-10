function [inBound] = inBound(o,vertexes)
%% Comments
% 判断给定的向量点是否在障碍物边界内(射线法，参考https://blog.csdn.net/lynon/article/details/82015834)
% o 向量点
% vertexes 障碍平面顶点，按顺时针进行排列
%% Body
proj = 1;
o = o(:)';
[a b c d] = gplane(vertexes(1),vertexes(2),vertexes(3));
if a~=0
    proj = 1;
elseif a==0&&b~=0
    proj = 2;
elseif a==0&&b==0&&c~=0
    proj = 3;
end
% 投影，将某一列删除
vertexes(:,proj)=[];
o(:,proj) = [];
% 通过射线法判断是否在平面内
inBound = false;
for i = 1:size(vertexes,2)
    if i == size(vertexes,2)
        if (vertexes(i,2)>o(2))~=(vertexes(1,2)>o(2))&&(o(1)<(vertexes(i,1)-vertexes(1,1))*(o(2)-vertexes(1,2))/(vertexes(i,2)-vertexes(1,2))+vertexes(1,1))
            inBound = ~inBound;
        end
    end
end
end