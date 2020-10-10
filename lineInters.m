function [intersection] = lineIntersect(o1,o2,o3,o4)
%% Comments
% 判断o1o2,o3o4两条线段是否相交https://www.cnblogs.com/kane1990/p/5742830.html
%% Body
t1 = cross(o1,o2,o3);
t2 = cross(o1,o2,o4);
t3 = cross(o3,o4,o1);
t4 = cross(o3,o4,o2);
if t1*t2>0 || t3*t4 >0
    intersection = false;
elseif t1==0 && t2==0
    intersection = rectsIntersect(o1,o2,o3,o4);
else
    intersection = true;
end
end

function [intersection] = rectsIntersect(o1,o2,o3,o4)
%% Comments
% 快速排斥实验 https://www.cnblogs.com/TangMoon/archive/2017/09/29/7611115.html
% Body
if min(o1(1),o2(1)) <= max(o3(1),o3(2)) && min(o3(1),o3(2)) <=max(o1(1),o2(1)) ...
    && min(o1(2),o2(2)) <= max(o3(2),o4(2)) && min(o3(2),o4(2)) <= max(o1(2),o2(2))
    intersection = true;
else
    intersection = false;
end
end
