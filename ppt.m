clc;
clear;
close all;
vertexes = [60 50 0;
100 100 0;
60 150 0;
20 120 0;
20 80 0;];
patch(vertexes(:,1),vertexes(:,2),vertexes(:,3),'y');
hold on
grid on
xlabel('x')
ylabel('y')
zlabel('z')
p1 = [0 90 30];
p2 = [100 90 50];
plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'r');
x = 0:0.1:100;
[X Z] = meshgrid(x);
Y = 90*ones(size(X));
mesh(X,Y,Z,'y');
