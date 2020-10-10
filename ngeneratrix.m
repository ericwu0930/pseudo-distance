function [j1g j2g] = ngeneratrix(j1,j2,vertexes,n,R)
%% Comment
% 得到距离障碍平面最近的母线，如果投影全部在障碍平面之外则返回一定数量的母线，否则返回一条
% j1,j2 连杆两端点
% vertexes 障碍平面顶点,需要按照顺时针或者逆时针给出
% n 返回母线的数量
% R 连杆的半径
%% Body
[a b c d] = gplane(vertexes(1,:),vertexes(2,:),vertexes(3,:));
D = gD(j1,j2,[a b c]);
% 如果j1,j2的投影有一部分在障碍平面内
if inPlane(j1,j2,vertexes) == 1
    j1g = j1+R*D;
    j2g = j2+R*D;
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
        j1g(i,:) = j1+R*rot*D;
        j2g(i,:) = j2+R*rot*D;
    end
end
end

function [D]=gD(j1,j2,n)
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
D = A\b;
D = D/norm(D);
end

function [case1] = dcase(j1,j2,vertexes)
%% Comments
% determine case
% 给定j1,j2 判断向障碍平面的投影是否有部分在障碍平面内 
%% Body
% 先将线段往障碍物平面投影，然后判断与障碍物平面的边界是否有交线
case1 = true;
end

function [o] = projToPlane(o,a,b,c,d)
%% Comments
% 将一点投影到一平面上
% n表示该平面的法线
%% Body
x=fsolve(@(x) myfunc(x,o,a,b,c,d),1);
o = x*[a;b;c];
end

function f=myfunc(x,o,a,b,c,d)
f(1)= x*[a;b;c]+o(:)-y;
f(2)= [a b c]*y+d;
end