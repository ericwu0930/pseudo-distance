function [dist] = lineToPlaneDist(j1, j2, vertexes, n, R)
% lineToPlaneDist 计算得到直线到平面的距离
% j1,j2 连杆的中心线
% vertexes 障碍顶点
% n 最近母线的生成数
% R 包络连杆圆柱的半径
%% Body

% just for test
% patch(vertexes([1,2,3],1),vertexes([1,2,3],2),vertexes([1,2,3],3),'y');
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis on
% grid on
% axis equal
% hold on
% plot3([j1(1),j2(1)],[j1(2),j2(2)],[j1(3),j2(3)],'g');
% text(j1(1),j1(2),j1(3),'j1');
% text(j2(1),j2(2),j2(3),'j2');

if size(vertexes,1)<3
    error("给出的障碍物顶点无法构成平面");
end
[j1g, j2g] = ggeneratrix(j1, j2, vertexes, R);

% just for test
% pjg=plot3([j1g(1),j2g(1)],[j1g(2),j2g(2)],[j1g(3),j2g(3)],'r--');
% text(j1g(1),j1g(2),j1g(3),"j_{1}'");
% text(j2g(1),j2g(2),j2g(3),"j_{2}'");

% 将障碍平面顶点首尾相连
vertexes = [vertexes; vertexes(1, :)];
% 计算得到平面方程
[a, b, c, d] = gplane(vertexes(1,:),vertexes(2,:),vertexes(3,:));
% 将母线两点向障碍平面投影
j1gp = projToPlane(j1g, a, b, c, d);
j2gp = projToPlane(j2g, a, b, c, d);

% just for test
% pjgp=plot3([j1gp(1),j2gp(1)],[j1gp(2),j2gp(2)],[j1gp(3),j2gp(3)],'ro-');
% text(j1gp(1),j1gp(2),j1gp(3),"j_{1p}'");
% text(j2gp(1),j2gp(2),j2gp(3),"j_{2p}'");
% plot3([j1gp(1),j1g(1)],[j1gp(2),j1g(2)],[j1gp(3),j1g(3)],'r--');
% plot3([j2gp(1),j2g(1)],[j2gp(2),j2g(2)],[j2gp(3),j2g(3)],'r--');

dist = inf;
% 计算母线在障碍平面的投影与障碍多边形的交点个数
intersection = [];
intersectEdgeS = [];
intersectEdgeE = [];
Nnum = 0;
for i = 1:size(vertexes, 1) - 1
    [num,inters] = lineIntersect(j1gp, j2gp, vertexes(i, :), ...
        vertexes(i + 1, :));
    if num == inf
        Nnum = inf;
        intersection = [intersection;inters];
        intersectEdgeS = [intersectEdgeS;vertexes(i,:)];
        intersectEdgeE = [intersectEdgeE;vertexes(i+1,:)];
        break
    end
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
    % j1'和j2'在平面的两侧,且交点在障碍多边形内
    if (j1gp(:)' - j1g(:)') * (j2gp(:) - j2g(:)) < 0
        % 求j1'j2'与障碍平面的交点
        So = getIntersection(j1g, j2g, a, b, c, d);
        % just for test
%         plot3(So(1),So(2),So(3),'ro');
%         text(So(1),So(2),So(3)+0.5,'S');
        
        if inBound(So, vertexes)
            dist = 0;
            return;
        end
    end
    if Nnum == 0
        % 母线在障碍平面内的投影完全在多边形内
        if inBound(j1gp, vertexes) && inBound(j2gp, vertexes)
            dist = ltoDist1(j1g,j1gp,j2g,j2gp);
        else
            [j1g, j2g] = ggeneratrixes(j1, j2, vertexes, n, R);
            
            % just for test
%             pjgps = ones(50,1);
%             for i=1:length(j1g)
%                 pjgps(i)=plot3([j1g(i,1),j2g(i,1)],[j1g(i,2),j2g(i,2)],[j1g(i,3),j2g(i,3)]);
%             end
            
            for j = 1:size(j1g, 1)
                newDist = ltoDist2(j1g(j, :), j2g(j, :),vertexes);
                if dist>newDist
                    dist = newDist;
                    % just for test
%                     delete(pjg);
%                     pjg = plot3([j1g(j,1),j2g(j,1)],[j1g(j,2),j2g(j,2)],[j1g(j,3),j2g(j,3)],'r--');

                end
                dist = min([dist,ltoDist2(j1g(j, :), j2g(j, :),vertexes)]);
            end
        end
    elseif Nnum == inf
        oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE,intersection);
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
        oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE,intersection);
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
        oj = goj(j1g,j2g,intersectEdgeS,intersectEdgeE,intersection);
        if norm(oj(1,:)-j1g)<=norm(oj(2,:)-j1g)
            dist = min([ltoDist1(oj(1,:),intersection(1,:),oj(2,:),...
                intersection(2,:)),ltoDist2(oj(1,:),j1g,vertexes),...
                ltoDist2(oj(2,:),j2g,vertexes)]);
        else
            % just for test
%             plot3([oj(1,1),intersection(1,1)],[oj(1,2),intersection(1,2)],[oj(1,3),intersection(1,3)]);
%             plot3([oj(2,1),intersection(2,1)],[oj(2,2),intersection(2,2)],[oj(2,3),intersection(2,3)]);
%             text(oj(1,1),oj(1,2),oj(1,3)+0.4,"S_{1}'");
%             text(oj(2,1),oj(2,2),oj(2,3)+0.4,"S_{2}'");
%             text(intersection(1,1),intersection(1,2),intersection(1,3)+0.4,"S_{1}");
%             text(intersection(2,1),intersection(2,2),intersection(2,3)+0.4,"S_{2}");
            
            dist = min([ltoDist1(oj(1,:),intersection(1,:),oj(2,:),...
                intersection(2,:)),ltoDist2(oj(2,:),j1g,vertexes),...
                ltoDist2(oj(1,:),j2g,vertexes)]);
        end
    end
end
% for test
% delete(pjg);
% delete(pjgp);

end 

function [So] = getIntersection(j1g, j2g, a, b, c, d)
% getIntersectoin 当j1'j2'分布在障碍平面两侧时，求得交点
%% Body
x = fsolve(@(x) myfunc(x, j1g, j2g, a, b, c, d), 1);
So = (j2g - j1g) * x + j1g;
end

function f = myfunc(x, j1g, j2g, a, b, c, d)
f = [a b c] * ((j2g(:) - j1g(:)) * x + j1g(:)) + d;
end

function [dist] = ltlDist(j1g, j2g, o1, o2)
% ltlDist 线段j1j2与线段o1o2之间的最短距离
%% Body
k1 = 2*(norm(o2-o1))^2;
p1 = -2*(o2(:)'-o1(:)')*(j2g(:)-j1g(:));
k2 = p1;
k3 = -2*(o2(:)'-o1(:)')*(j1g(:)-o1(:));
p2 = 2*(norm(j2g-j1g))^2;
p3 = 2*(j2g(:)'-j1g(:)')*(j1g(:)-o1(:));
e1 = (p3*k2-k3*p2)/(k1*p2-p1*k2);
e2 = -(p3+p1*e1)/p2;
% 两线段之间距离函数
f = @(n1,n2) norm((j2g-j1g)*n2-(o2-o1)*n1+j1g-o1);
lambda1 = f(0,0);
lambda2 = f(1,0);
lambda3 = f(0,1);
lambda4 = f(1,1);
dist = min([lambda1,lambda2,lambda3,lambda4]);
if e1>=0 && e1<=1 && e2>=0 && e2<=1
    lambda0 = f(e1,e2);
    dist = min([dist,lambda0]);
end
end

function [dist] = ltoDist1(o1,o1p,o2,o2p)
% ltoDist1 线段与障碍平面之间的最短距离 算法1
%% Body
dist = min([norm(o1p-o1),norm(o2p-o2)]);
end

function [dist] = ltoDist2(j1g, j2g, vertexes)
% ltoDist2 线段与障碍平面之间的最短距离 算法2
% 注意 vertexes已经首尾相连
%% Body
dist = inf;
for i = 1:size(vertexes,1)-1
    dist = min([dist,ltlDist(j1g,j2g,vertexes(i,:),vertexes(i+1,:))]);
end
end

function [oj] = goj(j1g,j2g,o1,o2,intersection)
% goj 将o1,或者o2，或者both投影到j1gj2g线段上
% 小tip，临时向量不能索引
% https://stackoverflow.com/questions/3627107/how-can-i-index-a-matlab-array-returned-by-a-function-without-first-assigning-it
% 能不能改成数组编程的方式？
%% Body
if size(intersection,1)>2
    error("交点个数不符合要求");
    return
end
oj = zeros(size(intersection,1),3);
for i = 1:size(intersection,1)
    point1 = o1(i,:);
    point2 = o2(i,:);
    tmp = @(x)(x*(j2g(:)'-j1g(:)')+j1g(:)'-intersection(i,:))*(point2(:)-point1(:));
    x = fsolve(tmp,1);
    oj(i,:) = x*(j2g-j1g)+j1g;
end
end
