%% 速度规划后 横轴为u
% 六个关节的速度
figure
subplot(dim,1,1);
plot(v,qv(1,:),'b--');
title("V_1");
hold;
subplot(dim,1,2);
plot(v,qv(2,:),'b--');
title("V_2");
hold;
subplot(dim,1,3);
plot(v,qv(3,:),'b--');
title("V_3");

subplot(dim,1,4);
plot(v,qv(4,:),'b--');
title("V_4");
hold;
subplot(dim,1,5);
plot(v,qv(5,:),'b--');
title("V_5");
hold;
subplot(dim,1,6);
plot(v,qv(6,:),'b--');
title("V_6");
xlabel("u");

qa=g1.*uu1/2+g2.*u; % eq.26
% 六个关节的加速度
figure
subplot(dim,1,1);
plot(v,qa(1,:),'b--');
title("A_1");
hold;
subplot(dim,1,2);
plot(v,qa(2,:),'b--');
title("A_2");
hold;
subplot(dim,1,3);
plot(v,qa(3,:),'b--');
xlabel("u");
title("A_3");
subplot(dim,1,4);
plot(v,qa(4,:),'b--');
title("A_4");
hold;
subplot(dim,1,5);
plot(v,qa(5,:),'b--');
title("A_5");
hold;
subplot(dim,1,6);
plot(v,qa(6,:),'b--');
xlabel("u");
title("A_6");
% 总加速度
% figure
% qa=sqrt(sum(qa.^2));
% plot(v,qa);
% xlabel("u");
% title("A");

% uu2=diff(uu1)/h;
% uu2(n)=uu2(n-1);

% 跃度
qqj=(g3.*u+g2.*uu1*3/2+g1.*uu2/2).*su;
figure
plot(v,qqj(1,:),v,qqj(2,:),v,qqj(3,:),v,qqj(4,:),v,qqj(5,:),v,qqj(6,:));
legend('q1','q2','q3','q4','q5','q6');
xlabel("u");
title("J");

%% 插补后 横轴为时间t
% 六关节位置
figure
subplot(dim,1,1);
plot(tt,C(1,:),'b--');
title("q1");
hold;
subplot(dim,1,2);
plot(tt,C(2,:),'b--');
title("q2");
hold;
subplot(dim,1,3);
plot(tt,C(3,:),'b--');
title("q3");
subplot(dim,1,4);
plot(tt,C(4,:),'b--');
title("q4");
hold;
subplot(dim,1,5);
plot(tt,C(5,:),'b--');
title("q5");
hold;
subplot(dim,1,6);
plot(tt,C(6,:),'b--');
title("q6");
xlabel("t");

% 离散路径点
figure
subplot(dim,1,1);
plot(path(1,:),'b--');
title("q1");
hold;
subplot(dim,1,2);
plot(path(2,:),'b--');
title("q2");
hold;
subplot(dim,1,3);
plot(path(3,:),'b--');
title("q3");
subplot(dim,1,4);
plot(path(4,:),'b--');
title("q4");
hold;
subplot(dim,1,5);
plot(path(5,:),'b--');
title("q5");
hold;
subplot(dim,1,6);
plot(path(6,:),'b--');
title("q6");
xlabel("t");

% 实际位置以及插补后位置

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

% 六关节速度
figure
subplot(dim,1,1);
plot(tt,C1(1,:),'b--');
title("V_1");
hold;
subplot(dim,1,2);
plot(tt,C1(2,:),'b--');
title("V_2");
hold;
subplot(dim,1,3);
plot(tt,C1(3,:),'b--');
title("V_3");
subplot(dim,1,4);
plot(tt,C1(4,:),'b--');
title("V_4");
hold;
subplot(dim,1,5);
plot(tt,C1(5,:),'b--');
title("V_5");
hold;
subplot(dim,1,6);
plot(tt,C1(6,:),'b--');
title("V_6");
xlabel("t");

% 总速度
figure
plot(tt,end_v,'k--','LineWidth',2);
xlabel("time/s",'FontSize',15);
ylabel("velocity/m \cdot s^{-1}",'FontSize',15);
title("End-effector speed curve",'FontSize',20);

% 六关节加速度
figure
subplot(dim,1,1);
plot(tt,C2(1,:),'b--');
title("A_1");
hold;
subplot(dim,1,2);
plot(tt,C2(2,:),'b--');
title("A_2");
hold;
subplot(dim,1,3);
plot(tt,C2(3,:),'b--');
title("A_3");
subplot(dim,1,4);
plot(tt,C2(4,:),'b--');
title("A_4");
hold;
subplot(dim,1,5);
plot(tt,C2(5,:),'b--');
title("A_5");
hold;
subplot(dim,1,6);
plot(tt,C2(6,:),'b--');
title("A_6");
xlabel("t");

% 六个方向跃度
figure
plot(tt,C3(1,:),tt,C3(2,:),tt,C3(3,:),tt,C3(4,:),tt,C3(5,:),tt,C3(6,:));
xlabel("t");
title("J");