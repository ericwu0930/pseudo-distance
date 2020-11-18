% rcim 与 qdistance效果对比
% qdistance算法 Q是正八面体
clear all;
clc;
close all;

%% 定义连杆的两端点
p1 = [0 0 0];
p2 = [0 0 154];
co = sqrt(2)/2;
l1 = [-80*co -80*co 0];
l2 = [-80*co 80*co 0];
l3 = [80*co 80*co 0];
l4 = [80*co -80*co 0];
l5 = [-80*co -80*co 154];
l6 = [-80*co 80*co 154];
l7 = [80*co 80*co 154];
l8 = [80*co -80*co 154];
G = [l1;l2;l3;l4;l5;l6;l7;l8];
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
% 体
cubes = {[1,9,12,4,5,13,16,8]
    [1 9 10 2 6 5 13 14]
    [2 3 11 10 15 14 6 7]
    [12 11 3 4 7 8 15 16]
    [7 8 6 5 16 15 14 13]};
%% Q的选择
Q1 = [1/2 1/2 co];
Q2 = [1/2 -1/2 co];
Q3 = [-1/2 1/2 co];
Q4 = [-1/2 -1/2 co];
Q5 = [1/2 1/2 -co];
Q6 = [1/2 -1/2 -co];
Q7 = [-1/2 1/2 -co];
Q8 = [-1/2 -1/2 -co];
Q = [Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
tic
dist = inf;
for i = 1 : size(cubes,1)
    H = ones(size(cubes{i,:},2),3);
    cube = cubes{i,:};
    for j = 1:size(H,1)
        H(j,:) = vertexes(cube(j),:);
    end
    dist = min([dist,qPlusDistance(G,H,Q)]);
end
toc

function dist = qPlusDistance(G,H,Q)
row1 = ones(size(G,2),size(G,1)+size(H,1)+size(Q,1));
for i = 1:size(G,1)
    row1(:,i) = G(i,:)';
end
for i = 1:size(H,1)
    row1(:,size(G,1)+i) = -H(i,:)';
end
for i = 1:size(Q,1)
    row1(:,size(G,1)+size(Q,1)+i) = -Q(i,:)';
end
row2 = [ones(1,size(G,1)),zeros(1,size(row1,2)-size(G,1))];
row3 = [zeros(1,size(G,1)),ones(1,size(H,1)),zeros(1,size(row1,2)-size(H,1)-size(G,1))];
Aeq = [row1;
    row2;
    row3];
beq = [zeros(size(G,2),1);1;1];
f = [zeros(1,size(G,1)+size(H,1)),ones(1,size(Q,1))];
lb = zeros(size(f));
options = optimoptions('linprog','Algorithm','interior-point');
[x,fval] = linprog(f,[],[],Aeq,beq,lb,[],options);
dist = fval;
end