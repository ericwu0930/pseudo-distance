function [dist] = lineToPlaneDist(j1, j2, vertexes, n, R)
%% Comments
% 计算得到直线到平面的距离
% j1,j2 连杆的中心线
% vertexes 障碍顶点
% n 最近母线的生成数
% R 包络连杆圆柱的半径
%% Body
% 将障碍平面顶点首尾相连
[j1g, j2g] = ggeneratrix(j1, j2, vertexes, n, R);
vertexes = [vertexes; vertexes(1, :)];
% 计算得到平面方程
[a, b, c, d] = gplane(vertexes);
% 将母线两点向障碍平面投影
j1gp = projToPlane(j1g, a, b, c, d);
j2gp = projToPlane(j2g, a, b, c, d);
% 计算母线在障碍平面的投影与障碍多边形的交点个数
intersection = [];
intersectEdgeS = [];
intersectEdgeE = [];
for i = 1:size(vertexes, 1) - 1
    if Nnum == inf
        break
    end
    [num,inters] = lineIntersect(j1gp, j2gp, vertexes(i, :), ...
        vertexes(i + 1, :));
    if num > 0
        Nnum = Nnum + num;
        intersection = [intersection;inters];
        intersectEdgeS = [intersectEdgeS;vertexes(i,:)];
        intersectEdgeE = [intersectEdgeE;vertexes(i+1,:)];
    end
end

% 首先判断j1g和j2g是否有在障碍平面内的,如果在直接返回0
if (inPlane(j1g, vertexes) && inBound(j1g, vertexes)) || ...
        (inPlane(j2g, vertexes) && inBound(j2g, vertexes))
    dist = 0;
    return
    % 判断j1g和j2g是否在障碍平面上，但在边界外，且与障碍多边形有交点
elseif inPlane(j1g, vertexes) && inPlane(j2g, vertexes)
    if Nnum > 0
        dist = 0;
        return
    end
else
    % 求j1'j2'与障碍平面的交点
    So = getIntersection(j1g, j2g, a, b, c, d);
    % j1'和j2'在平面的两侧,且交点在障碍多边形内
    if (j1gp - j1g) * (j2gp - j2g) < 0 && inBound(So, vertexes)
        dist = 0;
        return
    else
        if Nnum == 0
            % 母线在障碍平面内的投影完全在多边形内
            if inBound(j1gp, vertexes) && inBound(j2gp, vertexes)
                dist = ltoDist1(j1g,j1gp,j2g,j2gp);
            else
                [j1g, j2g] = ggeneratrixex(j1, j2, vertexes, n, R);
                for j = 1:size(j1g, 1)
                    dist = min([dist, ...
                        ltoDist2(j1g(i, :), j2g(i, :),vertexes)]);
                end
            end
        elseif Nnum == inf
            oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE);
            if inBound(j1gp, vertexes)&&~inBound(j2gp, vertexes)
                dist = min([ltoDist1(j1g,j1gp,oj,intersection),...
                    ltoDist2(j2g,oj,vertexes)]);
            elseif ~inBound(j1gp, vertexes)&&inBound(j2gp, vertexes)
                dist = min([ltoDist1(j2g,j2gp,oj,intersection),...
                    ltoDist2(j1g,oj,vertexes)]);
            else
                if norm(j1gp-intersection(1,:)) <= norm(j2gp-intersection(1,:))
                    dist = min([ltoDist1(intersection(1,:),...
                    intersection(2,:)),ltoDist2(j1g,oj(1,:),vertexes),...
                    ltoDist2(j2g,oj(2,:),vertexes)]);
                else
                    dist = min([ltoDist1(intersection(1,:),...
                    intersection(2,:)),ltoDist2(j1g,oj(2,:),vertexes),...
                    ltoDist2(j2g,oj(1,:),vertexes)]);
                end
            end
        elseif Nnum == 1
            if size(intersection,1)~=1 && size(intersectEdgeE,1)~=1 && ...
                    size(intersectEdgeS,1)~=1
                error("母线投影与障碍多边形的交点个数有误，请检查");
            end
            oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE);
            if inBound(j1gp,vertexes) && ~inBound(j2gp,vertexes)
                dist = min([ltoDist1(j1g,j1gp,oj,intersection),ltoDist2(oj,j2g,vertexes)]);
            else
                dist = min([ltoDist1(j2g,j2gp,oj,intersection),ltoDist2(oj,j1g,vertexes)]);
            end
        else
            if size(intersection,1)~=2 && size(intersectEdgeE,1)~=2 && ...
                    size(intersectEdgeS,1)~=2
                error("母线投影与障碍多边形的交点个数有误，请检查");
            end
            oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE);
            if norm(oj(1,:)-j1g)<=norm(oj(2,:)-j1g)
                dist = min([ltoDist1(oj(1,:),intersection(1,:),oj(2,:),...
                    intersection(2,:)),ltoDist2(oj(1,:),j1g,vertexes),...
                    ltoDist2(oj(2,:),j2g,vertexes)]);
            else
                dist = min([ltoDist1(oj(1,:),intersection(1,:),oj(2,:),...
                    intersection(2,:)),ltoDist2(oj(2,:),j1g,vertexes),...
                    ltoDist2(oj(1,:),j2g,vertexes)]);
            end
        end
    end
end
end

function [So] = getIntersection(j1g, j2g, a, b, c, d)
%% Comments
% 当j1'j2'分布在障碍平面两侧时，求得交点
%% Body
x = fsolve(@(x) myfunc(x, j1g, j2g, a, b, c, d), 1);
So = (j2g - j1g) * x + j1g;
end

function f = myfunc(x, j1g, j2g, a, b, c, d)
f = [a b c d] * ((j2g - j1g) * x + j1g) + d;
end

function [dist] = ltlDist(o1, o2, o3, o4)
%% Comments
% 两条线段之间的最短距离
%% Body
dist = 0;
end

function [dist] = ltoDist1(o1,o1p,o2,o2p)
%% Comments
% 线段与障碍平面之间的最短距离 算法1
%% Body
dist = min([norm(o1p-o1),norm(o2p-o2)]);
end

function [dist] = ltoDist2(o1, o2, vertexes)
%% Comments
% 线段与障碍平面之间的最短距离 算法2
% 注意 vertexes已经首尾相连
%% Body
dist = 0;
end

function [oj] = goj(j1g,j2g,o1,o2,intersection)
%% Comments
% 将o1,或者o2，或者both投影到j1gj2g线段上
% 小tip，临时向量不能索引
% https://stackoverflow.com/questions/3627107/how-can-i-index-a-matlab-array-returned-by-a-function-without-first-assigning-it
%% Body
if size(intersection,1)>2
    error("交点个数不符合要求");
    return
end
oj = zeros(size(intersection,1),3);
for i = 1:size(intersection,1)
    if abs(intersection(i,:)-o1)<1e-4
        tmp = @(x)(x*(j2g(:)'-j1g(:)')+j1g(:)'-o1(:)')*(o2(:)-o1(:));
    else
        tmp = @(x)(x*(j2g(:)'-j1g(:)')+j1g(:)'-o2(:)')*(o2(:)-o1(:));
    end
    x = fsolve(tmp,1);
    oj(i,:) = x(j2g-j1g)+j1g;
end
end
