%% collision detection function
function [feasible,colPoint,step] = checkPath(node,parent,obstacles,robot)
step = 0;
dc = robot.dc;
p2 = parent(1:dc);
p1 = node(1:dc);
colPoint = [];
feasible = 1;
if norm(p2-p1)<1e-3
    return;
end
delta = getMinDistVec(node,parent);
dir = delta/norm(delta);
for i=0:0.05:sqrt(sum((delta).^2))
    feasible = ~checkPoint(p1+i*dir,obstacles,robot);
%     plotLink(a0,l,p1+i*dir,obstacles);
    if feasible == 0
        colPoint = p1+i*dir;
        step = i;
        return
    end
end
end