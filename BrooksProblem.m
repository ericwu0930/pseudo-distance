%% Brooks's problem environment
clear;
clc;
close all;
tri = [0.4 5;4.4 5;2.4 5+4*sqrt(3)/2];
tri = [tri;tri(1,:)];
hold on;
for i = 1:3
    plot(tri(i:i+1,1),tri(i:i+1,2),'r-');
end
rec1 = [2 1;2 3;5 3;5 1];
rec1 = [rec1;rec1(1,:)];
for i = 1:4
    plot(rec1(i:i+1,1),rec1(i:i+1,2),'r-');
end
rec2 = [3 2;3 4;6 4;6 2];
rec2 = [rec2;rec2(1,:)];
for i = 1:4
    plot(rec2(i:i+1,1),rec2(i:i+1,2),'r-');
end
rec3 = [6.2 5;6.2+6*sqrt(2)/2 5+6*sqrt(2)/2;6.2+6*sqrt(2)/2-sqrt(2)/2 5+6*sqrt(2)/2+sqrt(2)/2;6.2-sqrt(2)/2 5+sqrt(2)/2];
rec3 = [rec3;rec3(1,:)];
for i = 1:4
    plot(rec3(i:i+1,1),rec3(i:i+1,2),'r-');
end
