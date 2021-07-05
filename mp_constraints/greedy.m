%% reference to Giuseppe Oriolo's paper Probabilistic Motion Planning for Redundant Robots along Given End-Effector Paths
% TODD function:
% [q_b,succ]=inv_kine(p,qr,q_bias,d)
% [feasible]=check_path(qi,qj)
function [time,len,path] = greedy(pPath)
environment;
global d;
d = 0.1;
j = 0;
maxAttemptsCnt = 1000;
len = nan; % todo:cal len
pathFound = false;
tic;
while j<maxAttemptsCnt && pathFound == false
    while true
        q0 = rand_conf(pPath(1,:),inv_kine);
        if ~isnan(q0)
            break;
        end
    end
    cPath = step(pPath,q0,inv_kine,check_path);
    if isempty(cPath)~=0
        pathFound = true;
    end
    j = j+1;
end
time = toc;
if pathFound == false
    path = [];
    return;
end
path = cPath;
end

function q = rand_conf(p,inv_kine,q_bias)
global d;
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
    q_bias_r = q_bias(end-n+m+1:end);
    qr = -d + (2*d)*rand(1,n-m)+q_bias_r;
end
end

function cPath = step(pPath,q_init,inv_kine,check_path)
% given two generic pose pi,pk belonging to the end-effector sequence
% and a configuration q_i on the self-motion corresponding to p_i tries to
% build a subsequence of configurations connecting p_i to p_k
k = size(pPath,1);
cPath = q_init;
q = q_init;
maxAttemptTimes = 100; % the number of calls to RAND_CONF for each end-effector pose
for i = 2:k
    l = 0;
    succ = 0;
    while l<maxAttemptTimes && succ == 0
        q_ = rand_conf(pPath(i,:),inv_kine,q);
        if ~any(isnan(q_)) && check_path(q,q_)
            succ = 1;
            q = q_;
            cPath = [cPath;q];
        end
        l = l+1;
    end
    if l>=maxAttemptTimes
        cPath = [];
        return
    end
end
end