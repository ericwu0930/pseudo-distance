% comparison between Dong and Chen in 6-d nurbs
clear;clc;
%% calculation

dong = zeros(8,6);
chen = zeros(8,6);

np = 1000;
for i = 1:6
    for j = 1:8
        dong(j,i) = getBSplineDong(np);
        chen(j,i) = getBSpline(np);
    end
    np = np+500;
end

%% plot
%load('effect_cmp.mat');
mean_d = mean(dong);
mean_c = mean(chen);
x = 1000:500:3500;
plot(x,mean_d,'ko-');
hold on 
plot(x,mean_c,'r*-');
legend("The LP method proposed in Dong's paper","The method proposed in Consolini's paper");
axis([500,3500,-inf,inf]);
