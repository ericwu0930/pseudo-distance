% rrt* for Zhu Problem
close all;
clear all;
%% Zhu's problem environment
global obstacles;
global a0;
global dc;
global l;

rec1 = [9 0;10 0;10 6;9 6];
% rec1 = [9 0;10 0;10 8.5;9 8.5];
rec1p = [rec1;rec1(1,:)];
rec2 = [9 12;10 12;10 20;9 20];
% rec2 = [9 9.5;10 9.5;10 18;9 18];
rec2p = [rec2;rec2(1,:)];
rec3 = [6 11;7 11;7 12;6 12];
rec3p = [rec3;rec3(1,:)];
rec4 = [12 7.5;13.8 7.5;13.8 9.3;12 9.3];
rec4p = [rec4;rec4(1,:)];
obstacles(:,:,1) = rec1;
obstacles(:,:,2) = rec2;
obstacles(:,:,3) = rec3;
obstacles(:,:,4) = rec4;
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
%% initial and end position of manipulator
a0 = [7 7.5];
l = 2;
source = [45 0 -60]*pi/180;
goal = [200 130 110]*pi/180;
stepsize = 0.2;
disTh = 0.2;
maxFailedAttempts = 10000;
display = true;
radius = 0.4;
dc = size(start,2);

tic
RRTree = [source 0 -1];
failedAttempts = 0;
counter = 0;
pathFound = false;
while failedAttempts<=maxFailedAttempts
    if rand < 0.3
        sample = rand(1,dc).* size(map);
    else
        sample = goal;
    end
    [A, I] = min(dist(RRTree(:,1:dc),sample),[],1);
    closestNode = RRTree(I(1),1:dc);
    dir = (sample - closestNode )/norm(sample - closestNode);
    newPoint = double(int32(closestNode + stepsize * dir));
    if ~checkPath(closestNode(1:dc), newPoint, map)
        failedAttempts=failedAttempts+1;
        % todo fix here
        continue;
    end
    
    if dist(newPoint,goal)<disTh
        pathFound = true;
        break;
    end
    [A,I2] = min(dist(RRTree(:,1:dc),newPoint),[],1);
    if dist(newPoint,RRTree(I2(1),1:dc))<disTh
        failedAttempts=failedAttempts+1;
        continue;
    end
    minCost = A+RRTree(I,dc+1);
    minParentIdx = I;
    
    distCols = dist(RRTree(:,1:dc),newPoint);
    nearIdx = find(distCols<=radius);
    sizeNear = size(nearIdx,1);
    if sizeNear > 1
        for i =1:sizeNear-1
            nowNode = RRTree(nearIdx(i),:);
            nowPoint = RRTree(nearIdx(i),1:dc);
            if checkPath(newPoint,nowPoint,map)
                costNear = nowNode(dc+1)+dist(nowPoint,newPoint);
                if costNear < minCost
                    minCost = costNear;
                    minParentIdx = nearIdx(i);
                end
            end
        end
    end
    newNode = [newPoint,minCost,minParentIdx];
    RRTree = [RRTree;newNode];
    newNodeIdx = size(RRTree,1);
    
    if sizeNear > 1
        reducedIdx = nearIdx;
        for i =1:sizeNear
            reducedNode = RRTree(reducedIdx(i),:);
            reducedPoint = reducedNode(1:dc);
            nearCost = reducedNode(dc+1);
            newLine = dist(reducedPoint,newPoint);
            if nearCost > minCost + newLine && checkPath(reducedPoint,newPoint,map)
                RRTree(reducedIdx(i),dc+2) = newNodeIdx;
                RRTree(reducedIdx(i),dc+1) = minCost+newLine;
            end
        end
    end
    failedAttempts=0;
    if display
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);
        counter=counter+1;M(counter)=getframe;
    end
end
toc

function [new_tree,flag] = extendTree(tree,end_node,segmentLength,r,flag_chk)
global dc;
flag1 = 0;
while flag1==0
    random_point = rand(1,dc).* [pi*2 pi*2 pi*2];
    [min_dist,idx]=min(distanceCost(tree(:,1:dc),random_point),[],1);
    min_parent_idx = idx;
    
    dir = (random_point-tree(idx,1:dc));
    new_point = tree(idx,1:dc) + dir/norm(dir)*segmentLength;
    
    min_cost = distanceCost(tree(idx,1:dc),new_point)+tree(idx,dc+2);
    new_node =[new_point,0,min_cost,idx];
    
    if checkPath(new_node,tree(idx,:)) == 0
        dist = distanceCost(tree(:,1:dc),new_point);
        near_idx = find(dist<=r);
        
        if size(near_idx,1)>1
            size_near = size(near_idx,1);
            for i = 1:size_near
                now_node = tree(near_idx(i),:);
                now_point = now_node(1:dc);
                if checkPath(new_node,now_node) == 0
                    cost_near = now_node(dc+2)+distanceCost(now_point,new_point);
                    if cost_near < min_cost
                        min_cost = cost_near;
                        min_parent_idx = near_idx(i);
                    end
                end
            end
            new_node = [new_point ,0,min_cost,min_parent_idx];
            new_tree = [tree;new_node];
            new_node_idx = size(new_tree,1);
            
            if size(near_idx,1)>1
                reduced_idx = near_Idx;
                for i = 1:size(reduced_idx,1)
                    reduced_node = new_tree(reduced_idx(i),:);
                    reduced_point = reduced_node(1:dc);
                    near_cost = reduced_node(dc+2);
                    newLine = distanceCost(reduced_point,new_point);
                    if near_cost > min_cost + newLine && checkPath(reduced_node,new_node) ==0
                        new_tree(reduced_idx(i),dim+3) = new_node_idx;
                        new_tree(reduced_idx(i),dim+2) = min_cost+newLine;
                    end
                end
            end
        end
        flag1 = 1;
    end
end

if flag_chk ==0
    if norm(new_node(1:dim)-end_node(1:dim)<segmentLength) && checkPath(new_node,end_node) == 0
        flag = 1;
        new_tree(end,dim+1) = 1;
    else
        flag = 0;
    end
else
    flag = 1;
end
end

function isCols = checkPath(node,parent)
global dc;
p1 = parent(1:dc);
p2 = node(1:dc);

isCols = 0;
dir = (p2-p1)/norm(p2-p1);
for i=0:0.5:sqrt(sum((p2-p1).^2))
    isCols = det(p1+i*dir);
    if isCols == 1
        return
    end
end
end

function isCols = det(config)
global obstacles;
x = fk(config);
[r,~] = size(x);
isCols = 0;
for i = 1:r-1
    [~,~,on] = size(obstacles);
    for j = 1:on
        isCols = GJK(x(i:i+1,:),obstacles(:,:,j));
        if isCols == 1
            return;
        end
    end
    for j = i+2:r-1
        isCols = GJK(x(i:i+1,:),x(j:j+1,:));
        if isCols == 1
            return;
        end
    end
end
end

function h = distanceCost(a,b)
h = sum((a-b).^2,2).^(1/2);
end