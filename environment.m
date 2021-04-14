% test evironment preparation for Oriolo's paper
% define 4-link planar forward kinematics and inverse kinematics

%% parameters definition
global l a0 n m obstacles;
obstacles = [];
l = 2;
a0 = [0 0];
n = 4; % cnt of joints is 4
m = 3; % x,y,theta
inv_kine = @ikine;
for_kine = @fkine;
check_path = @checkPath;

%% inverse kinematics
function [q_b,succ] = ikine(p,qr,q_bias,d)
global l m;
x_g = p(1);
y_g = p(2);
gamma = p(3);
x = x_g-l*cos(gamma);
y = y_g-l*sin(gamma);
new_p = [x,y,gamma-qr];
theta = ikine3r(new_p);
if isnan(theta)
    succ = 0;
    q_b = nan;
    return;
end
if isnan(q_bias)
    succ = 1;
    q_b = theta(1,:);
    return;
end
% flag = max(abs(getMinDistVec(theta,q_bias(1:3))),[],2)<d;
% if any(flag) == 0
%     succ=0;
%     q_b = nan;
% else
%     succ=1;
%     q_b = theta(flag,:);
%     q_b = q_b(1,:);
% end
succ = 1;
gap = distanceCost(q_bias(1:m),theta(1,:));
q_b = theta(1,:);
for i = 2:size(theta,1)
    tmp = distanceCost(q_bias(1:m),theta(i,:));
    if gap>tmp
        gap = tmp;
        q_b = theta(i,:);
    end
end
end

function theta = ikine3r(p)
global l;
x_g = p(1);
y_g = p(2);
gamma = p(3);
x = ones(1,3);
y = ones(1,3);
theta = zeros(2,3);
x(3) = x_g-l*cos(gamma);
y(3) = y_g-l*sin(gamma);
% solve a 2r planar inverse kinematics
D = (x(3)^2+y(3)^2-2*l^2)/(2*l^2);
if D>1
    theta = nan;
    return;
end
theta(1,2) = atan2(sqrt(1-D^2),D);
theta(2,2) = atan2(-sqrt(1-D^2),D);
theta(1,1) = atan2(y(3),x(3))-atan2(l*sin(theta(1,2)),(l+l*cos(theta(1,2))));
theta(2,1) = atan2(y(3),x(3))-atan2(l*sin(theta(2,2)),(l+l*cos(theta(2,2))));
theta(1,3) = gamma - theta(1,2)-theta(1,1);
theta(2,3) = gamma - theta(2,2)-theta(2,1);
end

%% forward kinematics
function x = fkine(q)
global a0 l n;
x = zeros(n+1,2);
x(1,:) = a0(:)';
curQ = 0;
for i = 2:n+1
    curQ = curQ+q(i-1);
    x(i,:) = x(i-1,:)+[l*cos(curQ) l*sin(curQ)];
end
end

%% collision avoidance
function [feasible,colPoint,step] = checkPath(node,parent)
global n;
step = 0;
p1 = parent(1:n);
p2 = node(1:n);
colPoint = [];
feasible = 1;
if norm(node-parent)<1e-3
    return;
end
dir = (p2-p1)/norm(p2-p1);
for i=0:0.05:sqrt(sum((p2-p1).^2))
    feasible = ~det(p1+i*dir);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
end
end

function isCols = det(config)
global obstacles;
x = fkine(config);
[r,~] = size(x);
isCols = 0;
for i = 1:r-1
%     [~,~,on] = size(obstacles);
%     for j = 1:on
%         isCols = gjk2d(x(i:i+1,:),obstacles(:,:,j));
%         if isCols == 1
%             return;
%         end
%     end
    for j = i+2:r-1
        isCols = gjk2d(x(i:i+1,:),x(j:j+1,:));
        if isCols == 1
            return;
        end
    end
end
end

