function [time,pathLength] = rrtForUr()
%% load ur10 model
loadUr10
%% define global variables
global obstacles;
global ur10;
global dc;
global Q;
%% define obstacles
rec1 = [-0.1,-0.5,0.5;
    -0.1,-0.5,0;
    -0.6,-0.5,0.5;
    -0.6,-0.5,0;
    -0.6,-0.8,0.5;
    -0.6,-0.8,0;
    -0.1,-0.8,0.5;
    -0.1,-0.8,0];
faces1 = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
figure
hold on
axis equal
grid on
for i = 1:6
    h = patch(rec1(faces1(i,:),1),rec1(faces1(i,:),2),rec1(faces1(i,:),3),'g')
    set(h,'facealpha',0.2);
end
rec2 = [0.8250,0.2000,0.7500;
    0.8250,0.2000,0;
    0.6250,0.2000,0.7500;
    0.6250,0.2000,0;
    0.6250,-0.5000,0.7500;
    0.6250,-0.5000,0;
    0.8250,-0.5000,0.7500;
    0.8250,-0.5000,0];
faces2 = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
    h = patch(rec2(faces2(i,:),1),rec2(faces2(i,:),2),rec2(faces2(i,:),3),'g');
    set(h,'facealpha',0.2);
end
rec3 = [0.4356,0.0499,1.6592;
    0.4356,0.0499,1.1592;
    0.3156,0.0499,1.6592;
    0.3156,0.0499,1.1592;
    0.3156,-0.9501,1.6592;
    0.3156,-0.9501,1.1592;
    0.4356,-0.9501,1.6592;
    0.4356,-0.9501,1.1592];
faces3 = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
    h = patch(rec3(faces3(i,:),1),rec3(faces3(i,:),2),rec3(faces3(i,:),3),'g')
    set(h,'facealpha',0.2);
end
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
%% define Q
Q1 = [sqrt(6)/3 -sqrt(2)/3 -1/3];
Q2 = [-sqrt(6)/3 -sqrt(2)/3 -1/3];
Q3 = [0 2*sqrt(2)/3 -1/3];
Q4 = [0 0 1];
Q = [Q1;Q2;Q3;Q4];
%% initial and end position of manipulator
source = [20,-45,60,-50,0,0]*pi/180;
midpoint = [106,-45,60,-50,0,0]*pi/180;
endpoint = [160,-68,30,-30,0,0]*pi/180;
stepsize = 0.2;
disTh = 0.2
maxFailedAttempts = 10000;
dc = size(source,2);

RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    if rand<=1
        sample = rand(1,dc).*[pi*2 pi*2 pi*2 pi*2 pi*2 pi*2];
    else
        sample = goal;
    end
    [A,I] = min(distanceCost(RRTree(:,1:dc),sample),[],1);
    
    
end

function h = distanceCost(a,b)
    h = sum((a-b).^2,2).^(1/2);
end