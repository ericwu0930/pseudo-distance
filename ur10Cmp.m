% rrt qd-rrt for ur 
clear all;
close all;
load('qdrrt_for_ur.mat');
load('rrt_for_ur.mat');

%% time cmp
figure
hold on
grid on
x = [1,2,3,4,5,6,7,9,11,14,16,18,19,20];
plot(qdrrt(1,:),'^-');
plot(x,rrt(1,x),'o-');
legend('QD-RRT','P-RRT')
xlabel('仿真次数')
ylabel('耗时/s')

%% path length cmp
figure
hold on
grid on
plot(qdrrt(2,:),'^-');
plot(x,rrt(2,x),'o-');
legend('QD-RRT','P-RRT')
xlabel('仿真次数')
ylabel('路径长度')