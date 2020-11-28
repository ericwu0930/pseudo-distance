%% plot environment
function handle = plotLink(a0,l,theta,obstacles)
plotEnv(a0,obstacles);
[r,~] = size(theta);
for i = 1:r
    x=fk(theta(i,:),a0,l);
    for j = 1:size(x,1)-1
        handle(i)=plot(x(j:j+1,1),x(j:j+1,2),'r-');
    end
end
end

function [] = plotEnv(a0,obstacles)
hold on;
axis equal;
plot(a0(1),a0(2),'ko');
for i = 1:size(obstacles,3)
    rec = obstacles(:,:,i);
    for j = 1:size(rec,1)-1
        plot(rec(j:j+1,1),rec(j:j+1,2),'r-');
    end
    plot([rec(end,1),rec(1,1)],[rec(end,2),rec(1,2)],'r-');
end
end

%% forward kinematics of manipulator
function [x] = fk(theta,a0,l)
% theta  1xdc configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
dc = size(theta,2);
% 2-d environment
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:dc+1
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end