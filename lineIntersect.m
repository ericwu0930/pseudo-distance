function [intersectNum,intersection] = lineIntersect(j1,j2,o1,o2)
%% Comments
% 通过叉积判断j1j2,o1o2两条线段是否相交，并返回交点 
% 如果有无数交点，表明o1o2, o3o4有重叠部分，返回o3||o4（取决于是否在o1o2上）
% 如果有一个交点，则返回该交点
% https://www.cnblogs.com/kane1990/p/5742830.html
%% Body
t1 = cross(j1,j2,o1);
t2 = cross(j1,j2,o2);
t3 = cross(o1,o2,j1);
t4 = cross(o1,o2,j2);
intersection=[];
if abs(t1)<1e-4 && abs(t2)<1e-4
    if rectsIntersect(j1,j2,o1,o2)
        intersectNum = inf;
        if min([j1(1),j2(1)]) < o1(1) && o1(1) < max([j1(1),j2(1)])
            intersection = [intersection;o1];
        elseif min([j1(1),j2(1)]) < o2(1) && o2(1) < max([j1(1),j2(1)])
            intersection = [intersection;o2];
        end
    else
        intersectNum = 0;
    end
elseif t1*t2>0 || t3*t4>0
    intersectNum = 0;
else
    intersectNum = 1;
    intersection = gIntegersection(j1,j2,o1,o2);
end
end

function [intersection] = rectsIntersect(o1,o2,o3,o4)
%% Comments
% 快速排斥实验 https://www.cnblogs.com/TangMoon/archive/2017/09/29/7611115.html
%% Body
if min(o1(1),o2(1)) <= max(o3(1),o4(1)) && min(o3(1),o4(1)) <=max(o1(1),o2(1)) ...
    && min(o1(2),o2(2)) <= max(o3(2),o4(2)) && min(o3(2),o4(2)) <= max(o1(2),o2(2))
    intersection = true;
else
    intersection = false;
end
end

function [intersection] = gIntegersection(o1,o2,o3,o4)
%% Comments
% 使用叉积计算得到o1o2,o3o4的交点
% https://blog.csdn.net/xdedzl/article/details/86009147
%% Body
line1 = o2-o1;
line2 = o4-o3;
line3 = o3-o1;
space1 = norm(cross(line1,line2));
space2 = norm(cross(line2,line3));
ratio = space2/space1;
intersection = o1 + (o2-o1)*ratio;
end
