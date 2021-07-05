% test closest_ik function
path = linspace(0,1,20);
path = path';
path = [path ones(20,1)*(2*sqrt(2)+2) ones(20,1)*pi/2];
global a0;
global l;
global n;
l = 2;
a0 = [0 0];
n = 3; % cnt of joints is 3
cPath = track_end_effector_path([pi/4,pi/2,-pi/4],path);

function [cPath] = track_end_effector_path(q,pPath)
cnt = size(pPath,1);
cPath = ones(cnt,3);
cPath(1,:) = q;
for i = 2:cnt
    cPath(i,:) = closest_ik(q,pPath(i,:));
    q = cPath(i,:);
end
end

function q_new = closest_ik(q_old,p_new)
q_new = q_old;
MAX_ATTEMPT = 20;
iter = 0;
while iter<MAX_ATTEMPT
    p_temp = to_end_pose(q_new);
    dd = p_new-p_temp;
   if dd<1e-2
       return;
   else
       j = get_jacob(q_new);
       j_inv = j'*inv(j*j');
       dq = j_inv*dd(:);
       q_new = q_new+dq(:)';
   end
end
end

function j = get_jacob(q)
global l;
j = [-l*sin(q(1))-l*sin(q(1)+q(2))-l*sin(sum(q)),-l*sin(q(1)+q(2))-l*sin(sum(q)),-l*sin(sum(q));
    l*cos(q(1))+l*cos(q(1)+q(2))+l*cos(sum(q)),l*cos(q(1)+q(2))+l*cos(sum(q)),l*cos(sum(q));
    1,1,1];
end

function p = to_end_pose(q)
% input configuration and fkine function to get end-effector pose
x = fkine(q);
p  = x(end,:);
p = [p,sum(q)];
end

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