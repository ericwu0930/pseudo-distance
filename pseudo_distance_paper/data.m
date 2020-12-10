p1=[1 2.3;
    2 2;
    3 1.8;
    4 1.9;
    5 1.9;
    6 1.9;
    7 1.9;
    8 1.9;
    9 1.9];
figure(1);
plot(p1(:,1),p1(:,2),'ko-');

axis([1 9 -4 4]);

p2=[1 9;
    2 9;
    3 9;
    4 9;
    5 9;
    6 9;
    7 9;
    8 9;
    9 9];
figure(2);
plot(p2(:,1),p2(:,2),'ko-');
ylabel('Shortest distance(mm)');
xlabel('Factor \mu')
axis([1 9 4 10]);

p3 =[1 25.7;
    2 25.7;
    3 25.7;
    4 25.7;
    5 25.7;
    6 25.7;
    7 25.7;
    8 25.7;
    9 25.7];
figure(3);
plot(p3(:,1),p3(:,2),'ko-');
ylabel('Shortest distance(mm)');
xlabel('Factor \mu')
axis([1 9 20 26]);

p4 =[1 0;
    2 0;
    3 0;
    4 0;
    5 0;
    6 0;
    7 0;
    8 0;
    9 0];
figure(4);
plot(p4(:,1),p4(:,2),'ko-');
ylabel('Shortest distance(mm)');
xlabel('Factor \mu')

p5= [1 0;
    2 0;
    3 0;
    4 0;
    5 0;
    6 0;
    7 0;
    8 0;
    9 0];
figure(5);
plot(p5(:,1),p5(:,2),'ko-');
ylabel('Shortest distance(mm)');
xlabel('Factor \mu')