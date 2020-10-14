function [a b c d] = gplane(v0,v1,v2)
% gplane 根据三个顶点得到平面方程
%% Body
% 全部变成列向量
v0=v0(:);
v1=v1(:);
v2=v2(:);
v0v1=v1-v0;
v0v2=v2-v0;
n=cross(v0v1,v0v2);
a=n(1);
b=n(2);
c=n(3);
d=a*(-v0(1))+b*(-v0(2))+c*(-v0(3));
end