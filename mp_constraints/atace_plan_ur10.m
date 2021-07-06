function [time,len,path] = atace_plan_ur10(qs,pg)
%% parameter definition
% syms x y;
% constraint = -0.0421*x.^2+7.5; % constraint function
% grad = gradient(f,[x,y,z]); % symbolic expression of tangent plane of constraint function
environment;
global Tree;
ps = to_end_pose(qs,for_kine);
Ns.p = ps;
Ns.q = qs;
Ns.parent = nan;
Tree = [Ns];
maxFailedAttempts = 10000;
failedAttempts = 0
pathFound = false;
while failedAttempts<=maxFailedAttempts
    qd = rand(1,n).*ones(1,n)*pi*2;
    pd = to_end_pose(qd,for_kine);
    Nc = nearest_node(pd);
    [s,Nk] = extend_with_constraint(Nc,pd,false);
    if s == -1 % trapped
        state = connect_to_goal(Nk);
        if state == 1
            pathFound = true;
            break;
        end
    end
end
% todo : result display
end

% state -1 trapped 0 advanced 1 reached
function [state,Nk]=extend_with_constraint(Nc,pd,greedy)
% choosing feasible velocities toward a direction pd at every step
% along the path. When parameter greedy=FALSE, the sub-path is
% extracted for no more than M steps.
p = Nc.p;
pPath = p;
i = 1;
state = 0;
while (greedy || i<=10) && state~=-1 && state~=1
    [v,w,pd_proj] = compute_valid_velocity(p,pd);
    [state,p_next] = compute_next_pose(p,v,w,pd_proj);
    if ~check_path(p,p_next)
        pPath = [pPath;p_next];
        p = p_next;
        i = i+1;
    else
        state = -1;
    end
end
if state~=-1
    qc = Nc.q;
    cPath = track_end_effector_path(qc,pPath);
    if ~isempty(cPath)
        Nk.p = pPath(end,:);
        Nk.q = cPath(end,:);
        Nk.pPath = pPath;
        Nk.cPath = cPath;
        Nk.parent = Nc;
        Tree = [Tree;Nc];
    else
        state = -1;
    end
end
end

function [v,w,pd_proj] = compute_valid_velocity(p,pd)
% transforms end-effector constraints into end-effector velocity
% constraints
%         [x,y,z] = p(1:3);
%         [xd,yd,zd] = pd(1:3);
%         n = eval(grad);
%         a=n(1);
%         b=n(2);
%         c=n(3);
%         A = [a b c;-b a 0;-c 0 a];
%         b = [[a,b,c]*[x,y,z]';a*yd-b*xd;a*zd-c*xd];
%         pdp = linsolve(A,b);
%         v = (pdp-p(1:3))/(norm(pdp-p(1:3)));
x = p(1);
y = p(2);
xd = pd(1);
yd = pd(2);
k = -0.0421*2*x;
b = y-k*x;
pd_proj(1) = k*(yd-b)+xd;
pd_proj(2) = k*pd_proj(1)+b;
v = (pd_proj - p(1:2))/norm(pd_proj-p(1:2));
w = 0;
end

function [state,p_next] = compute_next_pose(p,v,w,pd_proj)
state = 0;
time_interval = 0.1;
p_next = [p(1:2)+time_interval*v,p(3)];
if distanceCost(p(1:2),pd_proj(1:2))<0.2
    state = 1;
end
end

function isCols = check_path(ps,pe)
isCols = false;
end

function [cPath] = track_end_effector_path(q,pPath)
% dijkstra refered to lu's paper or 2002 Orthey's paper
[~,~,cPath] = rrt_like(q,pPath);
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
       dq = j_inv*dd;
       q_new = q_new+dq;
   end
end
end

function j = get_jacob(q)
global l;
j = [-l*sin(q(1))-l*sin(q(1)+q(2))-l*sin(sum(q)),-l*sin(q(1)+q(2))-l*sin(sum(q)),-l*sin(sum(q));
    l*cos(q(1))+l*cos(q(1)+q(2))+l*cos(sum(q)),l*cos(q(1)+q(2))+l*cos(sum(q)),l*cos(sum(q));
    1,1,1];
end



function Nc = nearest_node(pd)
% todo: how to measure se3 to se3? reference to Jin?
global Tree;
Nc = nan;
min_value = inf;
cnt = size(Tree);
for i=1:cnt
    curNode = Tree(i);
    val = distanceCost(curNode.p(1:2),pd(1:2));
    if val<min_value
        min_value = val;
        Nc = curNode;
    end
end
end

function state = connect_to_goal(Nk)
state = extend_with_constraint(Nk,pg,true);
end

function p = to_end_pose(q,fkine)
% input configuration and fkine function to get end-effector pose
x = fkine(q);
p  = x(end,:);
p = [p,sum(q)];
end