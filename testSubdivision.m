% 测试subdivision函数
clear;
clc;
close all;
Q1 = [sqrt(6)/3 -sqrt(2)/3 -1/3];
Q2 = [-sqrt(6)/3 -sqrt(2)/3 -1/3];
Q3 = [0 2*sqrt(2)/3 -1/3];
Q4 = [0 0 1]
Q = [Q1;Q2;Q3;Q4];
face=[1 2 3;
    1 2 4;
    1 3 4;
    2 3 4];
% for i = 1 : size(face,1)
%     h = patch(Q(face(i,:),1),Q(face(i,:),2),Q(face(i,:),3),'g');
%     set(h,'facealpha',0.2);
% end
axis equal
grid on
hold on
[Q,face] = subdivision(Q,face);
for i = 1 : size(face,1)
    h = patch(Q(face(i,:),1),Q(face(i,:),2),Q(face(i,:),3),'y');
    set(h,'facealpha',0.2);
end