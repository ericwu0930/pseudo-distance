function newNode = freeMove(obstacles,robot,cur_nodes,point)
k2 = 2;
r2 = 0.5;
H2 = 1;
moveStep = r2/2;
dc = robot.dc;
freePointsIdx = find(~cur_nodes(:,dc+1));
freePoints = cur_nodes(freePointsIdx,1:dc); % points in free space
k = 0;
newNode = [point,0];
if isempty(freePoints)
    return
end

while k<k2
    distances = distanceCost(point(1:dc),freePoints(:,1:dc));
    [P,I] = sort(distances);
    F = [];
    if P(1)>r2
        newNode = [point,0];
        return;
    end
    for i = 1:length(distances)
        if P(i)>r2
            break;
        end
        ci = freePoints(I(i),1:dc);
        f = H2/(1+P(i)^2)*((point(1:dc)-ci)/norm(point(1:dc)-ci));
        F = [F;f];
    end
    % 计算合力
    F = sum(F,1);
    lastPoint = point(1:dc);
    if norm(F) < 1e-3
        F_rand = 1/(1+r2^2)*rand(1,dc);
        point(1:dc) = point(1:dc)+F_rand*moveStep;
        point(1:dc) = mod(point(1:dc)+2*pi,2*pi);
        k = k+1;
    else
        point(1:dc) = point(1:dc)+F*moveStep;
        point(1:dc) = mod(point(1:dc)+2*pi,2*pi);
    end
    if checkPoint(point,obstacles,robot)
        newNode = [lastPoint,1];
        return;
    end
end
end