clc;
clear;
close all;
% test lineToPlaneDist
vertexes = [1 2 -3;2 3 -4;7 2 -5];

%% 投影交点有两个
j1 = [0 0 5];
j2 = [10 10 0];

tic
dist = lineToPlaneDist(j1,j2,vertexes,50,5)
toc
% goj函数中使用
% plot3(oj(1,1),oj(1,2),oj(1,3),'ko');
% plot3(oj(2,1),oj(2,2),oj(2,3),'ko');
% plot3(intersection(2,1),intersection(2,2),intersection(2,3),'ko');
% plot3(intersection(1,1),intersection(1,2),intersection(1,3),'ko');

%% 投影在多边形之外
j1 = [-10 -5 10];
j2 = [-5 5 7];
tic
dist = lineToPlaneDist(j1,j2,vertexes,50,5)
toc

%% 一个点在多边形内
plane=@(x,y,z) x+2*y+3*z+4;
z = fsolve(@(z)plane(4,2.5,z),0);
j1 = [4,2.5,z];
j2 = [-5 5 7];

tic
dist = lineToPlaneDist(j1,j2,vertexes,50,5)
toc