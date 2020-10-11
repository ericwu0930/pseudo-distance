function [dist] = lineToPlaneDist(j1,j2,vertexes,n,R)
%% Comments
% 计算得到直线到平面的距离
% j1,j2 连杆的中心线
% vertexes 障碍顶点
% n 最近母线的生成数
% R 包络连杆圆柱的半径
%% Body
% 将障碍平面顶点首尾相连
[j1g,j2g]=ngeneratrix(j1,j2,vertexes,n,R);
vertexes = [vertexes;vertexes(1,:)];
for i = 1:size(j1g,1)
    % 将母线两点向障碍平面投影
    j1g_=projToPlane(j1g(i,:),a,b,c,d);
    j2g_=projToPlane(j2g(i,:),a,b,c,d);
    % 计算母线在障碍平面的投影与障碍多边形的交点个数
    for j = 1:size(vertexes,1)-1
        if Nnum == inf
            break
        end
        Nnum = Nnum + lineIntersect(j1g_,j2g_,vertexes(j,:),vertexes(j+1,:));
    end
    % 首先判断j1g和j2g是否有在障碍平面内的,如果在直接返回0
    if (inPlane(j1g(i,:),vertexes) && inBound(j1g(i,:),vertexes))||(inPlane(j2g(i,:),vertexes) && inBound(j2g(i,:),vertexes))
        dist = 0;
        return
    % 判断j1g和j2g是否在障碍平面上，但在边界外，且与障碍多边形有交点
    elseif inPlane(j1g(i,:),vertexes) && inPlane(j2g(i,:),vertexes)
        if Nnum > 0
            dist = 0;
            return
        end
    else
        
    end
    
    
end

end