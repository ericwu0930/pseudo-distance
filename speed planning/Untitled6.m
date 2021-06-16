pos_d = ones(3,length(path));
pos = ones(3,length(g));
for i = 1:length(path)
    pos_d(:,i) = ur10.fkine(path(:,i)).t;
end
for i=1:length(g)
    pos(:,i) = ur10.fkine(g(:,i)).t;
end

figure;
plot3(pos_d(1,:),pos_d(2,:),pos_d(3,:),'ro');
grid on;
axis equal;
hold on;
plot3(pos(1,:),pos(2,:),pos(3,:),'b-');
legend('desire position','actual position');
xlabel('x');
ylabel('y');
zlabel('z');