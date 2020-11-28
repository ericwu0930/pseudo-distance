close all;
clear all;

map=im2bw(imread('map1.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=true;
dc = 2;
radius = 30;

tic
if display
    imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');
end
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

if display && pathFound
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
end
if display
    disp('click/press any key');
    waitforbuttonpress;
end

path = [goal];
prev = I;
while prev>0
    path = [RRTree(prev,1:dc);path];
    prev = RRTree(prev,dc+2);
end

pathLength=RRTree(end,dc+1);
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 

if ~pathFound
    error('no path found. maximum attempts reached');
end
toc

function h = dist(a,b)
h = sum((a-b).^2,2).^(1/2);
end