function isCols = checkPoint(config,obstacles,robot)
x = fk(config,robot);
[r,~] = size(x);
isCols = 0;
for i = 1:r-1
    [~,~,on] = size(obstacles);
    for j = 1:on
        shape1.XData = x(i:i+1,1);
        shape1.YData = x(i:i+1,2);
        shape2.XData = obstacles(:,1,j);
        shape2.YData = obstacles(:,2,j);
        isCols = gjk2d(shape1,shape2,6);
        if isCols == 1
            return;
        end
    end
    for j = i+2:r-1
        shape1.XData = x(i:i+1,1);
        shape1.YData = x(i:i+1,2);
        shape2.XData = x(j:j+1,1);
        shape2.YData = x(j:j+1,2);
        isCols = gjk2d(shape1,shape2,6);
        if isCols == 1
            return;
        end
    end
end
end