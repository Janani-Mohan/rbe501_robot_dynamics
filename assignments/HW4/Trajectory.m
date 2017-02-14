clc;
clear;
close all;
[qi1,qi2] = Inverse_Kinematics(300,450,300,300);
[qf1,qf2] = Inverse_Kinematics(-300,450,300,300);

vi1 = 0;
vi2 = 0;
vf1 = 0;
vf2 = 0;

d = [qi1,qf1,vi1,vf1,0,5];
[qd1,vd1,ad1] = Cubic_Generator(d(1),d(2),d(3),d(4),d(5),d(6));
d = [qi2,qf2,vi2,vf2,0,5];
[qd2,vd2,ad2] = Cubic_Generator(d(1),d(2),d(3),d(4),d(5),d(6));

t = linspace(0,5,100*5);
figure
subplot(3,1,1);
plot(t,qd1);
xlabel('time(ms)');
ylabel('Angle(deg)');
title('Position Trajectory of Link1 ');
subplot(3,1,2)
plot(t,vd1);
xlabel('time(sec)');
ylabel('Velocity(deg/sec)');
title('Velocity Trajectory of Link1');
subplot(3,1,3)
plot(t,ad1);
xlabel('time(sec)');
ylabel('Acceleration(deg/sec^2)');
title(' Acceleration Trajectory of Link1');
figure
subplot(3,1,1);
plot(t,qd2);
xlabel('time(ms)');
ylabel('Angle(deg)');
title('Position Trajectory of Link2 ');
subplot(3,1,2);
plot(t,vd2);
xlabel('time(sec)');
ylabel('Velocity(deg/sec)');
title('Velocity Trajectory of Link2');
subplot(3,1,3);
plot(t,ad2);
xlabel('time(sec)');
ylabel('Acceleration(deg/sec^2)');
title(' Acceleration Trajectory of Link2');

[x1,x2,y1,y2] =Forward_Kinematics(qd1,qd2,300,300);
LinkX=[0 x1 y1];
LinkY=[0 x2 y2];
figure
for i = 1:500
plot([0 x1(i) y1(i)],[0 x2(i) y2(i)],'-o','LineWidth',1,'MarkerSize',5,'MarkerFaceColor',[0.5 0.5 0.5]);
xlabel('x');
ylabel('y');
axis([-1000 1000 -1000 1000])
pause (0.01);
end







