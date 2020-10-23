%% 建立机械臂
% 设置初始属性
l1=145.7;
l2=50;
l3=270;
l4=70;
l5=147;
l6=152;
l7=78.5;
% 设置连杆 
L(1)=Link('d',l1,'alpha',pi/2,'a',l2,'offset',pi/2);
L(2)=Link('d',0,'alpha',0,'a',l3,'offset',pi/2);
L(3)=Link('d',0,'alpha',pi/2,'a',l4);
L(4)=Link('d',l5+l6,'alpha',-pi/2,'a',0);
L(5)=Link('d',0,'alpha',pi/2,'a',0);
L(6)=Link('d',l7,'alpha',0,'a',0);
robot = SerialLink(L,'name','robot');
q1=[0 0 0 0 0 0];
q2=[10 20 20 20 20 10];
q3=[-40 20 -70 10 -20 10];
q4=[-20 30 -70 10 -20 10];
q5=[30,-20,-60,40,-60,10];
robot.plot(q1);
%% groov-type obstacle
o1=[70 90 170];
o2=[70 290 170];
o3=[210 290 170];
o4=[210 90 170];
o5=[70 90 50];
o6=[70 290 50];
o7=[210 290 50];
o8=[210 90 50];
o9=[90 120 170];
o10=[90 260 170];
o11=[190 260 170];
o12=[190 120 170];
o13=[90 120 70];
o14=[90 260 70];
o15=[190 260 70];
o16=[190 120 70];
% 顶点
vertexes = [o1;o2;o3;o4;o5;o6;o7;o8;o9;o10;o11;o12;o13;o14;o15;o16];
% 面
faces = [1 4 12 9;
    1 2 10 9;
    2 3 11 10;
    4 3 11 12;
    4 8 7 3;
    3 7 6 2;
    2 6 5 1;
    1 5 8 4;
    12 9 13 16;
    9 13 14 10;
    10 14 15 11;
    11 15 16 12;
    8 7 6 5
    13 14 15 16];
for i = 1 : 14
    h = patch(vertexes(faces(i,:),1),vertexes(faces(i,:),2),vertexes(faces(i,:),3),'g');
    set(h,'facealpha',0.2);
end
axis equal
% for i = 1 : 16
%     text(vertices(i,1),vertices(i,2),vertices(i,3),num2str(i));
% end
view(3);