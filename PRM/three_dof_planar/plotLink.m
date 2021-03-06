%% plot environment
function handle = plotLink(three_dof,theta,obstacles)
a0 = three_dof.a0;
l = three_dof.l;
plotEnv(obstacles);
plot(a0(1),a0(2),'k^');
[r,~] = size(theta);
dc = three_dof.dc;
for i = 1:r
    x=fk(theta(i,1:dc),a0,l);
    for j = 1:size(x,1)-1
        handle(i)=plot(x(j:j+1,1),x(j:j+1,2),'r-');
    end
    for j = 2:size(x,1)-1
        plot(x(j,1),x(j,2),'ro')
    end
end
end

function [] = plotEnv(obstacles)
hold on;
axis equal;
for i = 1:size(obstacles,3)
    rec = obstacles(:,:,i);
    for j = 1:size(rec,1)-1
        plot(rec(j:j+1,1),rec(j:j+1,2),'k-');
    end
    plot([rec(end,1),rec(1,1)],[rec(end,2),rec(1,2)],'k-');
    patch(rec([1:size(rec,1),1],1),rec([1:size(rec,1),1],2),'k');
end
end

%% forward kinematics of manipulator
function x = fk(q,a0,l)
n = size(q,2);
x = zeros(n+1,2);
x(1,:) = a0(:)';
curQ = 0;
for i = 2:n+1
    x(i,:) = x(i-1,:)+[l*cos(q(i-1)) l*sin(q(i-1))];
end
end