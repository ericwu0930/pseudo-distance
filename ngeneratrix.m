function [j1g j2g] = ngeneratrix(j1,j2,vertexes,n,R)
%% Comment
% 得到距离障碍平面最近的母线，如果投影全部在障碍平面之外则返回一定数量的母线，否则返回一条
% j1,j2 连杆两端点
% vertexes 障碍平面顶点,需要按照顺时针或者逆时针给出
% n 返回母线的数量
% R 连杆的半径
%% Body
[a b c d] = gplane(vertexes(1,:),vertexes(2,:),vertexes(3,:));
d = gd(j1,j2,[a b c]);
% 如果j1,j2的投影有一部分在障碍平面内
if inPlane(j1,j2,vertexes) == 1
    j1g = j1+R*d;
    j2g = j2+R*d;
else
    j1g=zeros(n,3);
    j2g=zeros(n,3);
    phi = linspace(-pi/2,pi/2,n); % 在角度范围内平均取n个值
    c=cos(phi);
    s=sin(phi);
    h=1-c;
    r=(j2-j1)/norm(j2-j1);
    rx=r(1);
    ry=r(2);
    rz=r(3);
    for i = 1:n
        rot = [c+h*rx^2 h*rx*ry-rz*s h*rx*rz+ry*s;
            h*rx*ry+rz*s c+h*ry^2 h*ry*rz-rx*s;
            h*rx*rz-ry*s h*ry*rz+rx*s c+h*rz^2];
        j1g(i,:) = j1+R*rot*d;
        j2g(i,:) = j2+R*rot*d;
    end
end
end

function [d]=gd(j1,j2,n)
%% Comments
% 得到中心线向障碍平面投影方向的向量d
%% Body
j1=j1(:);
j2=j2(:);
n=n(:);
j1j2 = j2-j1;
A = [j1j2(1) j1j2(2) j1j2(3);
    n(1) n(2) n(3);
    0 0 1];
b = [0;0;1];
nx = A\b;
A = [j1j2(1) j1j2(2) j1j2(3);
    nx(1) nx(2) nx(3);
    0 0 1];
d = A\b;
d = d/norm(d);
end

function [case1] = dcase(j1,j2,vertexes)
%% Comments
% 给定j1,j2 判断向障碍平面的投影是否有部分在障碍平面内
%% Body
case1 = true;
end