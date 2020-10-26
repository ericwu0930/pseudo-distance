% rcim 与 qdistance效果对比
% rcim算法
clear;
clc;
close all;
dbclear all;

%% 定义连杆的两端点
p1 = [0 0 0];
p2 = [0 0 154];
link = [p1;p2];
%% groov-type obstacle
o1=[70 90 170];
o2=[70 290 170];
o3=[210 290 170];
o4=[210 90 170];
o5=[70 90 50];
o6=[70 290 50];
o7=[210 290 50];
o8=[210 90 50];
o9=[90 120 170];
o10=[90 260 170];
o11=[190 260 170];
o12=[190 120 170];
o13=[90 120 70];
o14=[90 260 70];
o15=[190 260 70];
o16=[190 120 70];
% 顶点
vertexes = [o1;o2;o3;o4;o5;o6;o7;o8;o9;o10;o11;o12;o13;o14;o15;o16];
% 面
faces = {[1 4 12 9]
    [1 2 10 9]
    [2 3 11 10]
    [4 3 11 12]
    [4 8 7 3]
    [3 7 6 2]
    [2 6 5 1]
    [1 5 8 4]
    [12 9 13 16]
    [9 13 14 10]
    [10 14 15 11]
    [11 15 16 12]
    [8 7 6 5]
    [13 14 15 16]};
%% 图形展示
plot3([p1(:,1),p2(:,1)],[p1(:,2),p2(:,2)],[p1(:,3) p2(:,3)],'r-')
hold on
grid on
axis equal
for i = 1 : 14
h = patch(vertexes(faces{i,:},1),vertexes(faces{i,:},2),vertexes(faces{i,:},3),'g');
set(h,'facealpha',0.2);
end

tic
dist = pdist(link,80,vertexes,faces,50);
toc