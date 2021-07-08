%% ur10 motion planning with constraints
clear all;
%% define UR10 and bounding box
mdl_ur10;
global ur10;
% Bounding box size
bb1 = [0.16/2 -0.13 0.16/2;
    0.16/2  -0.13 -0.16/2;
    -0.16/2 -0.13 -0.16/2;
    -0.16/2 -0.13 0.16/2;
    0.16/2 0.05 0.16/2;
    0.16/2 0.05 -0.16/2;
    -0.16/2 0.05 -0.16/2;
    -0.16/2 0.05 0.16/2];
% bounding box face
bf(:,:,1)=[1,2,6,5;3,4,8,7;1,2,3,4;5,6,7,8;1,4,8,5;2,3,7,6];
% for i = 1:6
% h = patch(bb1(bf1(i,:),1),bb1(bf1(i,:),2),bb1(bf1(i,:),3),'r')
% set(h,'facealpha',0.2);
% end
bb2 = [-0.04 0.05 0.08+0.18;
    -0.04 -0.13 0.08+0.18;
    0.7 0.05 0.08+0.18;
    0.7 -0.13 0.08+0.18;
    -0.04 0.05 0.08;
    -0.04 -0.13 0.08;
    0.7 0.05 0.08;
    0.7 -0.13 0.08];
bf(:,:,2)=[1,2,4,3;5,6,8,7;1,3,7,5;2,4,8,6;1,2,6,5;3,4,8,7];
% for i = 1:6
% h = patch(bb2(bf2(i,:),1),bb2(bf2(i,:),2),bb2(bf2(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb3 = [-0.04 0.05 0.09;
    -0.04 -0.13 0.09;
    0.7 0.05 0.09;
    0.7 -0.13 0.09;
    -0.04 0.05 -0.08;
    -0.04 -0.13 -0.08;
    0.7 0.05 -0.08;
    0.7 -0.13 -0.08];
bf(:,:,3)=[1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6;1,2,4,3;5,6,8,7];
% for i = 1:6
% h = patch(bb3(bf3(i,:),1),bb3(bf3(i,:),2),bb3(bf3(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb4 = [0.05 0.05 -0.06;
    0.05 0.05 0.18;
    0.05 -0.05 -0.06;
    0.05 -0.05 0.18;
    -0.05 0.05 -0.06;
    -0.05 0.05 0.18;
    -0.05 -0.05 -0.06;
    -0.05 -0.05 0.18];
bf(:,:,4)=[1,2,4,3;5,6,8,7;1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6];
% for i = 1:6
% h = patch(bb4(bf4(i,:),1),bb4(bf4(i,:),2),bb4(bf4(i,:),3),'r')
% set(h,'facealpha',0.2);
% end

bb(:,:,1) = bb1;
bb(:,:,2) = bb2;
bb(:,:,3) = bb3;
bb(:,:,4) = bb4;

%% define obstacles
rec1 = [1.0750,0.075,1;
    1.0750,0.075,0;
    0.6250,0.075,1;
    0.6250,0.075,0;
    0.6250,-0.075,1;
    0.6250,-0.075,0;
    1.0750,-0.075,1;
    1.0750,-0.075,0];

% obstacles face
of(:,:,1) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
figure
hold on
axis equal
grid on
for i = 1:6
    h = patch(rec1(of(i,:,1),1),rec1(of(i,:,1),2),rec1(of(i,:,1),3),'y');
%     set(h,'facealpha',0.2);
end
rec2 = [0.7750,-0.5500,1;
    0.7750,-0.5500,0;
    0.6250,-0.5500,1;
    0.6250,-0.5500,0;
    0.6250,-1.1500,1;
    0.6250,-1.1500,0;
    0.7750,-1.1500,1;
    0.7750,-1.1500,0];
of(:,:,2) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
%     h = patch(rec2(of(i,:,2),1),rec2(of(i,:,2),2),rec2(of(i,:,2),3),'y');
%     set(h,'facealpha',0.2);
end
rec3 = [0.7750,0.5500,1;
    0.7750,0.5500,0;
    0.6250,0.5500,1;
    0.6250,0.5500,0;
    0.6250,1.1500,1;
    0.6250,1.1500,0;
    0.7750,1.1500,1;
    0.7750,1.1500,0];
of(:,:,3) = [1,2,8,7;3,4,6,5;1,2,4,3;5,6,8,7;1,3,5,7;2,4,6,8];
for i = 1:6
%     h = patch(rec3(of(i,:,3),1),rec3(of(i,:,3),2),rec3(of(i,:,3),3),'y')
%     set(h,'facealpha',0.2);
end
global obstacles;
obstacles(:,:,1) = rec1;
% obstacles(:,:,2) = rec2;
% obstacles(:,:,3) = rec3;

global qs;
T = SE3(transl(0.65,0.5,0.5));
newT = SE3(transl(0.65,-0.5,0.5));
r = SE3.rpy([0 0 0]);
ps = T*r;
qs = [-2.6872   -0.8965    1.2490    1.2183   -1.5708    1.4654];
global pg;
pg = newT*r;
qg = [ 2.2846   -0.8965    1.2490    1.2183   -1.5708    2.4278];

global dc;dc = 3;
%% atace_plan
global Tree;
global dc;dc=6;
ps = to_end_pose(qs);
Ns.p = ps;
Ns.q = qs;
Ns.parent = nan;
Tree = [Ns];
maxFailedAttempts = 10000;
failedAttempts = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    qd = rand(1,dc).*ones(1,dc)*pi*2;
    pd = to_end_pose(qd);
    Nc = nearest_node(pd);
    [s,Nk] = extend_with_constraint(Nc,pd,false);
    if s ~= -1 % trapped
        state = connect_to_goal(Nk);
        if state == 1
            pathFound = true;
            break;
        end
    end
end
%% display


%% auxiliary funciton
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
    [state,cPath] = track_end_effector_path(qc,pPath);
    if ~isempty(cPath) && state == 1
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
pd_proj(1) = pd(1);
pd_proj(2) = pd(2);
pd_proj(3) = 0.5;
pd_proj = pd_proj(:)';
v = (pd_proj - p(1:3))/norm(pd_proj-p(1:3));
%rpy = pd(4:6);
k = 1;
if rand < 0.5
    k=-1;
end
w = k*[0 0 1];
end

function [state,p_next] = compute_next_pose(p,v,w,pd_proj)
state = 0;
time_interval = 0.1;
t = p(1:3)+time_interval*v;
r = SO3.angvec(time_interval,w);
o_next = r*SO3.rpy(p(4:6));
rpy = o_next.torpy;
rpy = rpy(:)';
p_next = [t(:)',rpy];
p_next = p_next(:)';
end

function isCols = check_path(ps,pe)
global obstacles;
diff_vec = pe(1:3)-ps(1:3);
dir = diff_vec/norm(diff_vec);
for i = 0:0.05:norm(diff_vec)
    pn = ps(1:3)+i*dir;
    obj1 = wrapObj(pn);
    obj2 = wrapObj(obstacles(:,:,1));
    isCols = GJK(obj1,obj2,6);
    if isCols
        return;
    end
end
isCols = false;
end

function obj = wrapObj(vertices)
vertices = vertices(:)';
obj.XData = vertices(:,1);
obj.YData = vertices(:,2);
obj.ZData = vertices(:,3);
end

function [state,cPath] = track_end_effector_path(q,pPath)
cnt = size(pPath,1);
cPath = ones(cnt,6);
cPath(1,:) = q;
state = 1;
for i = 2:cnt
    cPath(i,:) = closest_ik(q,pPath(i,:));
    [feasible,~,~]=checkPath(cPath(i-1,:),cPath(i,:));
    if ~feasible
        cPath = [];
        state = -1;
        return;
    end
    q = cPath(i,:);
end
end

function [feasible,colPoint,step] = checkPath(node,parent)
% check path feasible between parent and node
global dc;
step = 0;
p1 = parent(1:dc);
p2 = node(1:dc);
diffVec = getMinDistVec(p1,p2);
colPoint = [];
feasible = 1;
if norm(diffVec)<1e-3
    return;
end
dir = (diffVec)/norm(diffVec);
for i=0:0.05:norm(diffVec)
    feasible = ~det(p1+i*dir);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
end
end

function q_new = closest_ik(q_old,p_new)
q_new = q_old;
MAX_ATTEMPT = 20;
iter = 0;
while iter<MAX_ATTEMPT
    p_temp = to_end_pose(q_new);
    dd = p_new-p_temp;
   if abs(dd)<1e-2
       return;
   else
       j = get_jacob(q_new);
       j_inv = inv(j);
       dq = j_inv*dd(:);
       dq = dq(:)';
       q_new = q_new+dq;
   end
end
end

function isCols = det(config)
% config -- configuration
global obstacles;
global bb;
global ur10;
isCols = 0;
% total 4 bounding boxes for ur10
for i=1:4
    % bounding box translation
    bbt(:,:,i) = transBox(ur10.A(1:i,config),bb(:,:,i));
end
% collision detection
% plot for test
detPlot;
pause(0.01)
for i=1:4
    [~,~,on] = size(obstacles);
    % traverse jth obstacles for ith bounding box
    for j = 1:on
        obj1 = wrapObj(bbt(:,:,i));
        obj2 = wrapObj(obstacles(:,:,j));
        isCols = GJK(obj1,obj2,6);
        if isCols == 1
            delete(h)
            return;
        end
    end
    for j = i+2:4
        obj1 = wrapObj(bbt(:,:,i));
        obj2 = wrapObj(bbt(:,:,j));
        isCols = GJK(obj1,obj2,6);
        if isCols == 1
            delete(h)
            return;
        end
    end
end
delete(h)
end


function j = get_jacob(q)
global ur10;
j = ur10.jacob0(q,'rpy');
end



function Nc = nearest_node(pd)
% just 
global Tree;
Nc = nan;
min_value = inf;
cnt = size(Tree);
for i=1:cnt
    curNode = Tree(i);
    val = sqrt((sum(curNode.p(1:3)-pd(1:3))).^2);
    if val<min_value
        min_value = val;
        Nc = curNode;
    end
end
end

function state = connect_to_goal(Nk)
state = extend_with_constraint(Nk,pg,true);
end

function p = to_end_pose(q)
global ur10;
% input configuration and fkine function to get end-effector pose
se3 = ur10.fkine(q);
p = [se3.t];
rpy = se3.torpy;
rpy = rpy';
p = [p;rpy];
p = p(:)'
end


