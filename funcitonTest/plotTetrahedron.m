function plotTetrahedron(v)
figure;
axis equal;
grid on;
x = zeros(3,4);
y = x;
z = x;
for i = 1: 4
    tmp = v;
    tmp(i,:)=[];
    x(:,i) = tmp(:,1);
    y(:,i) = tmp(:,2);
    z(:,i) = tmp(:,3);
end
h = patch(x,y,z,'g');
set(h,'facealpha',0.2);
hold on;
plot3(0,0,0,'ro');
end