% 效果对比
close all;
clear all;
%% RRT
rrt = zeros(2,20);
for i = 1:20
    [time,pathLength]=rrtForZhu();
    rrt(:,i)=[time;pathLength];
end

%% QD-RRT
qdrrt = zeros(2,20);
for i = 1:20
    [time,pathLength]=rrtForZhuM();
    qdrrt(:,i)=[time;pathLength];
end

%% RRT*
rrt_ = zeros(2,20);
for i = 1:20
    [time,pathLength]=rrtStarForZhu();
    rrt_(:,i)=[time;pathLength];
end

%% QD-RRT*
qdrrt_ = zeros(2,20);
for i = 1:20
    [time,pathLength]=rrtStarForZhuM();
    qdrrt_(:,i)=[time;pathLength];
end