function [time,len,path] = rrtrgd()
%% reference to Yao's paper planning with general end-effector constrains
%% nested function definition
    function q = generate_feasible_vertex(q)
        i = 0;
        j = 0;
        while i<maxSteps && j<maxFailSteps && constraintFunc(q)>epsilon
            i=i+1;
            j=j+1;
            q_ = random_ngbr(q);
            if constraintFunc(q_)<constraintFunc(q):
                j=0;
                q = q_;
            end
        end
        if constraintFunc(q)>epsilon
            q=nan;
            return;
        end
    end

    %search a certain scope of q and find collision free point
    function q = random_nhbr(q)
    end

    function x = fkine(q)
        x = zeros(dc+1,2);
        x(1,:) = a0(:)';
        curQ = 0
        for i = 2:dim+1
            curQ = curQ+q(i-1);
            x(i,:) = x(i-1,:)+[l*cos(curQ) l*sin(curQ)];
        end
    end

    function isCols()
    end
%% variable definition
maxSteps = 10000;
maxFailSteps = 100;
constraintFunc = nan % constraint function
epsilon = 1e-2 % constraint error
dim = 3; % 4-link plannar and 3 joints
a0 = [7 7.5];
l = 2;
nhbrRadius = 0.1 % random neighborhood scope
rec1 = [9 0;10 0;10 8.5;9 8.5];
obstacles(:,:,1)=rec1

source = [45 0 -60]*pi/180;
goal = [200 130 110]*pi/180;
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 10000;

tic
RRTree = [source -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    if rand <=0.7
        sample = rand(1,dim)*ones(1,dim)*pi*2;
    else
        sample = goal;
    end
    [A, I] = min(distanceCost(RRTree(:,1:dc),sample),[],1);
    closestNode = RRTree(I(1),1:dc);
    dir = (sample - closestNode )/norm(sample - closestNode);
    newPoint = closestNode + stepsize * dir;
    [feasible,~,~]=checkPath(newPoint,closestNode);
    if feasible == 0
        failedAttempts=failedAttempts+1;
        continue;
    else
        % generate point in C_satisfy
        generate_feasible_vertex(newPoint)
    end
    if distanceCost(newPoint,goal)<disTh
        pathFound = true;
        break;
    end
    
    RRTree = [RRTree;newPoint I];
    failedAttempts=0;
end

path = [goal];
prev = I;
while prev>0
    path = [RRTree(prev,1:dc);path];
    prev = RRTree(prev,dc+1);
end

pathLength=0;
for i =1:length(path)-1
    pathLength = pathLength + distanceCost(path(i,1:dc),path(i+1,1:dc));
end

if display == true
    figure;
    plotLink(a0,l,path,obstacles);
end

time = toc;
fprintf('processing time=%d \nPath Length=%d \n\n',time,pathLength);


if ~pathFound
    time = inf;
    pathLength = NaN;
    error('no path found. maximum attempts reached');
end
end





