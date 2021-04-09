%% reference to Giuseppe Oriolo's paper Probabilistic Motion Planning for Redundant Robots along Given End-Effector Paths
% TODD function:
% [q_b,succ]=inv_kine(p,qr,q_bias,d)
% [feasible]=check_path(qi,qj)
function [time,len,path] = rrt_like(pPath)
environment;
global d Tree;
d = 0.1;
j = 0;
maxTreeCnt = 100;
p_new = nan;
len = nan; % todo:cal len
pathFound = false;
tic;
while ~pathFound && j<maxTreeCnt
    q0 = rand_conf(pPath(1,:),inv_kine);
    Tree = [q0,-1,0]; % [q,parent,associated pose_idx]
    i = 0;
    maxAttemptTimes = 1000;
    while ~pathFound && i<maxAttemptTimes
        [p_new,p_idx,q_new] = extend_like();
        if norm(p_new,pPath(end,1:n))
            pathFound = true;
            break;
        end
        subPath = step(pPath(p_idx,:),q_new,check_path); % greedy strategy
        if len(subPath)>0
            path = subPath;
            pathFound = true;
            break;
        end
        i = i+1;
    end
    j = j+1;
end
time = toc;
if p_new ~= ps
    path = [];
    return;
end
path = [Tree(end,1:n);path];
prev = Tree(end,end-1);
while prev>0
    path = [Tree(prev,1:n);path];
    prev = Tree(prev,end-1);
end
end

function [p_new,p_idx,q_new] = extend_like(pPath)
global n m Tree;
q_rand = rand(1,n).*ones(1,n)*2*pi;
q_rand_r = q_rand(end-(n-m)+1:end);
[~,idx] = min(distanceCost(Tree(:,m+1:end-1),q_rand(m+1,end-1)),[],1);
k = Tree(idx,end);
q_near = Tree(idx,1:end-1);
q_near_r = q_near(end-(n-m)+1:end);
dir = (q_rand_r-q_near_r)/norm(q_rand_r-q_near_r);
q_new_r = q_near_r+dir*d*sqrt(n);
[q_new_b,succ] = inv_kine(pPath(k+1,:),q_new_r,q_near);
q_new = [q_new_b,q_new_r];
if succ == 1 && check_path(q_near,q_new)
    Tree = [Tree;q_new,idx,k+1];
    p_new = pPath(k+1,:);
    p_idx = k+1;
else
    p_new = nan;
    p_idx = -1;
    q_new = nan;
end
end

function q = rand_conf(p,inv_kine,q_bias)
global n m d;
if nargin == 2
    q_bias = nan;
end
qr = rand_red(q_bias);
[qb,succ] = inv_kine(p,qr,q_bias,d);
if succ == 0
    q = nan;
else
    q = [qb,qr];
end
end

function qr = rand_red(q_bias)
global d n m;
if isnan(q_bias)
    % no bias
    qr = rand(1,n-m).*ones(1,n-m)*pi*2;
else
    % have bias;
    d = 0.1;
    q_bias_r = q_bias(end-n+m+1:end);
    qr = -d + (2*d)*rand(1,n-m)+q_bias_r;
end
end

function cPath = step(pPath,q_init,check_path)
% given two generic pose pi,pk belonging to the end-effector sequence
% and a configuration q_i on the self-motion corresponding to p_i tries to
% build a subsequence of configurations connecting p_i to p_k
k = size(pPath,2);
cPath = q_init;
q = q_init;
maxAttemptTimes = 100; % the number of calls to RAND_CONF for each end-effector pose
for i = 2:k
    l = 0;
    succ = 0;
    while l<maxAttemptTimes && succ == 0
        q_ = rand_conf(pPath(i,:),q);
        if ~isnan(q_) && check_path(q,q_)
            succ = 1;
            q = q_;
            cPath = [cPath;q];
        end
    end
    if l>=maxAttemptTimes
        cPath = [];
        return
    end
end
end

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end