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

if search(p1,p2,obstacles,robot)
    feasible = false;
else
    feasible = true;
end

% delta = getMinDistVec(node,parent);
% dir = delta/norm(delta);
% for i=0:0.05:sqrt(sum((delta).^2))
%     feasible = ~checkPoint(p1+i*dir,obstacles,robot);
% %     plotLink(a0,l,p1+i*dir,obstacles);
%     if feasible == 0
%         colPoint = p1+i*dir;
%         step = i;
%         return
%     end
% end
end

function isCols = search(s,e,obstacles,robot)
isCols = false;
if getMinDistVec(s,e)<0.05
    return;
end
midPoint = (s+e)/2;
if checkPoint(midPoint,obstacles,robot)
    isCols = true;
    return;
end
if search(s,midPoint,obstacles,robot)
    isCols = true;
    return;
end
if search(midPoint,e,obstacles,robot)
    isCols = true;
    return;
end
end