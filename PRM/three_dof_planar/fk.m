%% forward kinematics of manipulator
function [x] = fk(theta,robot)
% theta  1xdc configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
% 2-d environment
a0 = robot.a0;
l = robot.l;
dc = robot.dc;
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:dc+1
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end