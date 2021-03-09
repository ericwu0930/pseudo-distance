% rrt qd-rrt for ur 
clear all;
close all;
load('qdrrt_for_ur.mat');
load('rrt_for_ur.mat');

%% time cmp
figure
hold on
grid on
plot(qdrrt(1,:),'^-');
plot(rrt(1,:),'o-');
legend('QD-RRT','P-RRT')
xlabel('仿真次数')
ylabel('耗时/s')

%% path length cmp
figure
hold on
grid on
plot(qdrrt(2,:),'^-');
plot(rrt(2,:),'o-');
legend('QD-RRT','P-RRT')
xlabel('仿真次数')
ylabel('路径长度')