clc;
clear;
close all;
% 测试辅助函数

%% test ggneratrix
plane=@(x,y,z) x+2*y+3*z+4;
[X,Y]=meshgrid(-20:0.25:20);
[m,n]=size(X);
init = zeros(m,n);
Z = (-4-X-2*Y)/3;
% mesh(X,Y,Z)
j1 = [0 0 10];
j2 = [10 10 0];
R = 5;
vertexes = [1 2 -3;2 3 -4;7 2 -5];
patch(vertexes([1,2,3],1),vertexes([1,2,3],2),vertexes([1,2,3],3),'y');
xlabel('X')
ylabel('Y')
zlabel('Z')
axis on
grid on
axis equal
hold on
plot3([j1(1),j2(1)],[j1(2),j2(2)],[j1(3),j2(3)],'g');
[j1g,j2g] = ggeneratrix(j1,j2,vertexes,R);
plot3([j1g(1),j2g(1)],[j1g(2),j2g(2)],[j1g(3),j2g(3)],'r--');
%% test projToPlane
a=1;
b=2;
c=3;
d=4;
j1gp = projToPlane(j1g, a, b, c, d);
j2gp = projToPlane(j2g, a, b, c, d);

%% test inPlane
for i = 0:0.2:1
    if ~inPlane(j1gp+(j2gp-j1gp)*i,vertexes)
        error("算法有误，请仔细检查")
    end
end
%% test lineIntersect
% 与障碍平面有两个交点的情况
vertexes = [vertexes; vertexes(1, :)];
intersection = [];
intersectEdgeS = [];
intersectEdgeE = [];
Nnum = 0;

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
disp(['交点个数：',size(intersection,1)]);
plot3(intersection(:,1),intersection(:,2),intersection(:,3),'mo')
plot3([intersectEdgeS(1,1),intersectEdgeE(1,1)],[intersectEdgeS(1,2),intersectEdgeE(1,2)],...
    [intersectEdgeS(1,3),intersectEdgeE(1,3)],'-ro')
plot3([intersectEdgeS(2,1),intersectEdgeE(2,1)],[intersectEdgeS(2,2),intersectEdgeE(2,2)],...
    [intersectEdgeS(2,3),intersectEdgeE(2,3)],'-ro')

% 与障碍平面有无数交点的情况（与牟边重合）
j1gp = [0 1 -2];
j2gp = [3 4 -5];
plot3([j1gp(1),j2gp(1)],[j1gp(2),j2gp(2)],[j1gp(3),j2gp(3)],'r--');

intersection = [];
intersectEdgeS = [];
intersectEdgeE = [];
Nnum = 0;

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
disp(['交点个数：',size(intersection,1)]);
plot3(intersection(:,1),intersection(:,2),intersection(:,3),'mo')
plot3([intersectEdgeS(1,1),intersectEdgeE(1,1)],[intersectEdgeS(1,2),intersectEdgeE(1,2)],...
    [intersectEdgeS(1,3),intersectEdgeE(1,3)],'-ro')

