%% reference to Giuseppe Oriolo's paper Probabilistic Motion Planning for Redundant Robots along Given End-Effector Paths
function q = rand_conf(p,q_bias,inv_kine)
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

function [tiem,len,path] = rrt_like(pPath,inv_kine)
    function extend_like()
        n = size(q0,2);
        m = size(pPath(1,1),2);
        q_rand = rand(1,n).*ones(1,n)*2*pi;
        
    end
j = 0;
maxTreeCnt = 100;
p_new = nan;
pathFound = false;
while p_new~=pPath(end,:) && j<maxTreeCnt
    q0 = rand_conf(pPath(0,:),inv_kine);
    Tree = [q0,-1];
    i = 0;
    maxAttemptTimes = 1000;
    while p_new~=pPath(end,:) || i<maxAttemptTimes
        p_new = extend_like();
        i = i+1;
    end
    j = j+1;
end
if p_new == ps
    pathFound = true;
else
    pathFound = false;
    path = [];
    return;
end
if pathFound
    % todo: find path
end
end
