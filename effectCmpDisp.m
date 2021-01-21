%% effect compare for rrt, qd-rrt,rrt*,qd-rrt*
clear all;
close all;
load('qdrrt_.mat')
load('rrt_bak.mat')
load('rrt.mat')
load('qdrrt.mat')

%% time cmp
figure
hold on
grid on
plot(rrt_(1,:),'^-')
plot(rrt(1,:),'o-')
plot(qdrrt(1,:),'*-')
plot(qdrrt_(1,:),'+-')
legend('RRT*','RRT','QD-RRT','QD-RRT*')
xlabel('仿真次数')
ylabel('耗时/s')

%% path length cmp
figure
hold on
grid on
plot(rrt(2,:),'^-')
plot(rrt_(2,:),'o-')
plot(qdrrt(2,:),'*-')
plot(qdrrt_(2,:),'+-')
legend('RRT*','RRT','QD-RRT','QD-RRT*')
xlabel('仿真次数')
ylabel('路径长度')