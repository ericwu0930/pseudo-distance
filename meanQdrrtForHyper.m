clear all;
close all;
qdrrtH = ones(2,20);
for i = 1:20
    [time,pathLength,~] = rrtmForHyperRedundant();
     qdrrtH(:,i) = [time;pathLength];
end
