%%
%  |$Q6)Determine the forward kinematics of the end effector if the joint configuration of the robot is given as follows: [?1,?2,?3,?4,?5,?6]=[0,75,30,135,?45,60] in degrees$|
clc;
clear ;
close all;
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;
d1 = 0.330;
a1 = 0.075;
a2 = 0.300;
a3 = 0.075;
d4 = 0.320;
d6 = 0.080;
     
 
A1 = [cosd(theta1),-(cosd(90)* sind(theta1)),(sind(theta1)* sind(90)),(a1*cosd(theta1));
    sind(theta1),(cosd(90)* cosd(theta1)),-(cosd(theta1)* sind(90)),(a1*sind(theta1));
    0,sind(90),cosd(90),d1 ;
    0,0,0,1]
A2 = [cos(theta2+90),-cosd(0)* sin(theta2+90),sin(theta2+90)* sind(0),a2*cos(theta2+90);
    sin(theta2+90),cosd(0)* cos(theta2+90),cos(theta2+90)* sind(0),a2*sin(theta2+90);
    0,sind(0),cosd(0),0 ;
    0,0,0,1]
A3 = [cosd(theta3),-(cosd(90)* sind(theta3)),(sind(theta3)* sind(90)),(a3*cosd(theta3));
    sind(theta3),(cosd(90)* cosd(theta3)),-(cosd(theta3)* sind(90)),(a3*sind(theta3));
    0,sind(90),cosd(90),0 ;
    0,0,0,1]
A4 = [cosd(theta4),-(cosd(-90)* sind(theta4)),(sind(theta4)* sind(-90)),(0*cosd(theta4));
    sind(theta4),(cosd(-90)* cosd(theta4)),-(cosd(theta4)* sind(90)),(0*sind(theta4));
    0,sind(-90),cosd(-90),d4 ;
    0,0,0,1]
A5 = [cosd(theta5),-(cosd(90)* sind(theta5)),(sind(theta5)* sind(90)),(0*cosd(theta5));
    sind(theta5),(cosd(90)* cosd(theta5)),-(cosd(theta5)* sind(90)),(0*sind(theta5));
    0,sind(90),cosd(90),0 ;
    0,0,0,1]
A6 = [cosd(theta6),-(cosd(0)* sind(theta6)),(sind(theta6)* sind(0)),(0*cosd(theta6));
    sind(theta6),(cosd(0)* cosd(theta6)),-(cosd(theta6)* sind(0)),(0*sind(theta6));
    0,sind(0),cosd(0),d6 ;
    0,0,0,1]


H = A1*A2*A3*A4*A5*A6


fprintf('The forward kinematics of end effector is: ');
F = H(1:3,4)

A11 = A1;
A22 = A1*A2;
A33 = A1*A2*A3;
A44 = A1*A2*A3*A4;
A55 = A1*A2*A3*A4*A5;
A66 = A1*A2*A3*A4*A5*A6;

LinkX=[0 A11(1,4) A22(1,4) A33(1,4) A44(1,4) A55(1,4) A66(1,4)];
LinkY=[0 A11(2,4) A22(2,4) A33(2,4) A44(2,4) A55(2,4) A66(2,4)];
LinkZ=[0 A11(3,4) A22(3,4) A33(3,4) A44(3,4) A55(3,4) A66(3,4)];

plot3(0,0,0,'-o','LineWidth',1,'MarkerSize',5,'MarkerFaceColor',[0.1 0.1 0.1]);
hold on;
plot3(LinkX,LinkY,LinkZ,'-o','LineWidth',1,'MarkerSize',5,'MarkerFaceColor',[0.5 0.5 0.5]);
hold on;

xlabel('x');
ylabel('y');
zlabel('z');
title('GIVEN POSITION')
grid on
