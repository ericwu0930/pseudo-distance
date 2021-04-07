function [time,len,path] = atace_plan()
%% function definition
% state -1 trapped 0 advanced 1 reached
    function [state,Nk]=extend_with_constraint(Nc,pd,greedy)
        pc = Nc(1:3);
        pPath = pc;
        i = 1;
        p = pc;
        state = 0;
        while (greedy || i<=M) && state~=-1 && state~=1
            [v,w,pd_proj] = compute_valid_velocity(p,pd);
            [state,p_next]=compute_next_pose(p,v,w,pd_proj);
            if ~check_path(p,p_next)
                pPath = [pPath;p_next];
                p = p_next;
            else
                state = -1;
            end
        end
        if state~=-1
            [ss,cPath] = track_end_effector_path(qc,pPath);
            if ss == 1
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
        a = [x;-0.0839*2*x];
        pd_proj = (a*a')*pd/(a'*a);
        v = (pd_proj - p(1:2))/norm(pd_proj-p(1:2));
        w = 0;
    end

    function [state,p_next] = compute_next_pose(p,v,w,pd_proj)
        state = 0;
        time_interval = 0.1;
        p_next = [p(1:2)+time_interval*v,p(3)];
        if norm(p(1:2),pd_proj(1:2))<disTh
            state = 1;
        end
    end

    function isCols = check_path(ps,pe)
        isCols = false;
    end

    function [state,cPath] = track_end_effector_path(q,pPath)
        % dijkstra refered to lu's paper or 2002 Orthey's paper
    end

    function x = fkine(q)
        x = zeros(dc+1,2);
        x(1,:) = a0(:)';
        curQ = 0;
        for i = 2:dim+1
            curQ = curQ+q(i-1);
            x(i,:) = x(i-1,:)+[l*cos(curQ) l*sin(curQ)];
        end
    end

    function Nc = nearest_node(pd)
        % todo: how to measure se3 to se3? reference to Jin?
        Nc = nan;
        min_value = inf;
        cnt = size(Tree);
        for i=1:cnt
            curNode = Tree(i);
            val = sqrt((curNode.p(1)-pd(1))^2+(curNode.p(2)-pd(2))^2+(curNode.p(3)-pd(3))^2);
            if val<min_value
                min_value = val;
                Nc = curNode;
            end
        end
    end

    function state = connect_to_goal(Nk)
        state = extend_with_constraint(Nk,pg,true);
    end
%% parameter definition
syms x y;
constraint = -0.0839*x^2+5.5-y; % constraint function
grad = gradient(f,[x,y,z]); % symbolic expression of tangent plane of constraint function
disTh = 0.2;
a0 = [0 0];
l = 2;
dim = size(qs,2);
qs = [135,0,-45]*pi/180;
pg = [1 2*sqrt(2)+2 pi/2];
ps = to_end_pose(qs);
Ns.p = ps;
Ns.q = qs;
Ns.parent = nan;
Tree = [Ns];
maxFailedAttempts = 10000;
failedAttempts = 0
pathFound = false;
while failedAttempts<=maxFailedAttempts
    qd = rand(q,dim).*ones(1,dim)*pi*2;
    pd = to_end_pose(qd);
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

function p = to_end_pose(q)
x = fkine(q);
p  = x(end,:);
p = [p,sum(q)];
end