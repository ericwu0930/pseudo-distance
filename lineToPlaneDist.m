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
    for i = 1:size(vertexes, 1) - 1

        if Nnum == inf
            break
        end

        Nnum = Nnum + lineIntersect(j1gp, j2gp, vertexes(i, :), vertexes(i + 1, :));
    end

    % 首先判断j1g和j2g是否有在障碍平面内的,如果在直接返回0
    if (inPlane(j1g, vertexes) && inBound(j1g, vertexes)) || (inPlane(j2g, vertexes) && inBound(j2g, vertexes))
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
                    dist = min([norm(j1g, j1gp), norm(j2g, j2gp)]);
                else
                    [j1g, j2g] = ggeneratrixex(j1, j2, vertexes, n, R);

                    for j = 1:size(j1g, 1)
                        dist = min([dist, ltoDist(j1g(i, :), j2g(i, :), vertexes)]);
                    end

                end

            elseif Nnum == inf
                o =

                if inBound(j1gp, vertexes) &&~inBound(j2gp, vertexes)
                elseif ~inBound(j1gp, vertexes) && inBound(j2gp, vertexes)
                else

                    if norm(j1gp -)
                    end

                elseif Nnum == 1
                else
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

function [dist] = ltoDist(o1, o2, vertexes)
    %% Comments
    % 线段与障碍平面之间的最短距离
    % 注意 vertexes已经首尾相连
    %% Body
    dist = 0;
end

function [in] = pinL(o1, o2, o)
    %% Comments
    % 判断点o是否在线段o1o2上
    %% Body
    if abs(cross(o2 - o1, o - o1)) < 1e-4
        in = true;
    else
        in = false;
    end

end

function [o] = findPoint()
    %% Comments
    % 找到
end
