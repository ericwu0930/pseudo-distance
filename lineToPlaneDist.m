function [dist] = lineToPlaneDist(j1,j2,vertexes,n,R)
%% Comments
% 计算得到直线到平面的距离
% j1,j2 连杆的中心线
% vertexes 障碍顶点
% n 最近母线的生成数
% R 包络连杆圆柱的半径
%% Body
vertexes = [vertexes;vertexes(1,:)];
[j1g,j2g]=ngeneratrix(j1,j2,vertexes,n,R);
for i = 1:size(j1g,1)
    % 首先判断j1g和j2g是否有在障碍平面内的,如果在直接返回0
    if (inPlane(j1g(i,:),vertexes) && inBound(j1g(i,:),vertexes))||(inPlane(j2g(i,:),vertexes) && inBound(j2g(i,:),vertexes))
        dist = 0;
        return
    % 判断j1g和j2g是否在障碍平面上，但在边界外，且与障碍多边形有交点
    elseif inPlane(j1g(i,:),vertexes) && inPlane(j2g(i,:),vertexes)
        for j = 1:size(vertexes,1)-1
            num = lineIntersect(j1g(i,:),j2g(i,:),vertexes(i,:),vertexes(i+1,:));
            if num == inf || num == 1
                dist = 0;
                return
            end
        end
    end
    
    
end

end