function newNode = freeMove(obstacles,robot,points,point)
k2 = 2;
r2 = 1;
H2 = 1;
moveStep = 0.5;
dc = robot.dc;
freePointsIdx = find(~points(:,dc+1));
freePoints = points(freePointsIdx,1:dc); % points in free space
k = 0;


while k<k2
    distances = distanceCost(point(1:dc),freePoints(:,1:dc));
    [P,I] = sort(distances);
    F = [];
    for i = 1:length(distances)
        if P(i)>r2
            break;
        end
        ci = freePoints(I(i),1:dc);
        f = H2/(1+distanceCost(point(1:dc),ci)^2)*((point(1:dc)-ci)/norm(point(1:dc)-ci));
        F = [F;f];
    end
    % 计算合力
    F = sum(F,1);
    lastPoint = point(1:dc);
    point(1:dc) = point(1:dc)+F*moveStep;
    if checkPoint(point,obstacles,robot)
        newNode = [lastPoint,1];
        return;
    end
    if F < 1e-3
        distances = distanceCost(point(1:dc),freePoints(:,1:dc));
        [P,I] = sort(distances);
        if freePoints(I(1),1:dc)>r2
            newNode = [point,0];
        else
            F_rand = norm(F)*rand(1,dc);
            point(1:dc) = point(1:dc)+F_rand*moveStep;
            k = k+1;
        end
    end
end

end