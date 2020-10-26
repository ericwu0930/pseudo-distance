clear;
clc;
close all;
pseudo_distance

link = zeros(7,3);
link(1,:)=[0 0 0];
robot.plot(q1)
for i = 1:6
    tmp = robot.A(1:i,q1);
    link(i+1,:) = tmp.t;
end
tic
dist = pdist(link,R,vertexes,faces,50);
toc
 
