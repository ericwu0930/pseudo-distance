function [j1g,j2g] = ggeneratrixes(j1,j2,vertexes,n,R)
% 根据母线数的限制得到一组母线
% n 在[-pi/2,pi/2]区间内取角度的数量
% R 连杆的半径
%% Body
[j1g,j2g,D] = ggeneratrix(j1,j2,vertexes,R);
j1g=zeros(n,3);
j2g=zeros(n,3);
phi = linspace(-pi/2,pi/2,n);
for i = 1:length(phi)
    c=cos(phi(i));
    s=sin(phi(i));
    h=1-c;
    r=(j2-j1)/norm(j2-j1);
    rx=r(1);
    ry=r(2);
    rz=r(3);
    rot = [c+h*rx^2 h*rx*ry-rz*s h*rx*rz+ry*s;
        h*rx*ry+rz*s c+h*ry^2 h*ry*rz-rx*s;
        h*rx*rz-ry*s h*ry*rz+rx*s c+h*rz^2];
    j1g(i,:) = j1(:)+R*rot*D;
    j2g(i,:) = j2(:)+R*rot*D;
end
%  测试用
% for i = 1:length(j1g)
%     plot3([j1g(i,1),j2g(i,1)],[j1g(i,2),j2g(i,2)],[j1g(i,3),j2g(i,3)]);
% end
end