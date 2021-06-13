function newNode = obstacleMove(obstacles,robot,cur_nodes)
r1 = 1;
H1 = 1;
moveStep = r1/2;
dc = robot.dc;
obstaclelogi = cur_nodes(:,end)== -1;
idx = find(obstaclelogi);
obstaclePoints = cur_nodes(obstaclelogi,1:dc); % points in free space
newNode = [];

for i = 1:length(idx)
    distances = distanceCost(obstaclePoints(i,1:dc),obstaclePoints(:,1:dc));
    [P,I] = sort(distances);
    F = zeros(1,dc);
    for j = 2:length(distances)
        if P(i)>r1
            break;
        end
        ci = obstaclePoints(I(i),1:dc);
        f = H1/(1+P(i)^2)*((obstaclePoints(i,1:dc)-ci)/norm(obstaclePoints(i,1:dc)-ci));
        F = [F;f];
    end
    F = sum(F,1);
    while checkPoint(obstaclePoints(i,1:dc))
        obstaclePoints(i,1:dc) = obstaclePoints(i,1:dc)+F*moveStep;
    end
end
end