%% reference to Giuseppe Oriolo's paper Probabilistic Motion Planning for Redundant Robots along Given End-Effector Paths
% [q_b,succ]=inv_kine(p,qr,q_bias)
% [feasible]=checkPath(qi,qj)
function q = rand_conf(p,inv_kine,q_bias)
if nargin == 1
    q_bias = nan;
end
m = size(p);
n = size(q_bias);
qr = rand_red(n-m,q_bias);
[state,q] = inv_kine(p,qr);
if state == -1
    q = nan;
end
end

function qr = rand_red(dim,q_bias)
if isnan(q_bias)
    % no bias
    qr = rand(1,dim).*ones(1,dim)*pi*2;
else
    % have bias;
    d = 0.1;
    q_bias_r = q_bias(end-dim+1:end);
    qr = -d + (2*d)*rand(1,dim)+q_bias_r;
end
end

function cPath = step(pPath,q_init,checkPath)
k = size(pPath,2);
cPath = q_init;
q = q_init;
maxAttemptTimes = 100; % the number of calls to RAND_CONF for each end-effector pose
for i = 1:k
    l = 0;
    succ = 0;
    while l<maxAttemptTimes && succ == 0
        q_ = rand_conf(pPath(i,:),q);
        if ~isnan(q_) && checkPath(q,q_)
            succ = 1;
            q = q_;
            cPath = [cPath;q];
        end
    end
    if l==maxAttemptTimes
        cPath = [];
        return
    end
end
end

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end

function [tiem,len,path] = rrt_like(pPath,inv_kine,checkPath)
    function p_new = extend_like()
        n = size(q0,2);
        m = size(pPath(1,1),2);
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
        if succ == 1 && checkPath(q_near,q_new)
            Tree = [Tree;q_new,idx,k+1];
            p_new = pPath(k+1,:);
        else
            p_new = nan;
        end
    end
d = 0.1;
j = 0;
maxTreeCnt = 100;
p_new = nan;
pathFound = false;
while p_new~=pPath(end,:) && j<maxTreeCnt
    q0 = rand_conf(pPath(0,:),inv_kine);
    Tree = [q0,-1,0]; % [q,parent,associated pose_idx]
    i = 0;
    maxAttemptTimes = 1000;
    while p_new~=pPath(end,:) || i<maxAttemptTimes
        p_new = extend_like();
        i = i+1;
    end
    j = j+1;
end
if p_new ~= ps
    path = [];
    return;
end
% todo: find path
path = Tree(end,1:n);
prev = Tree(end,end-1);
while prev>0
    path = [Tree(prev,1:n);path];
    prev = Tree(prev,end-1);
end
end
