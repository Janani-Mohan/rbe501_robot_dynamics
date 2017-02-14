%% Compute a quintic polynomial reference trajectory 
% q0 = initial position 
% v0 = initial velocity 
% ac0 = initial acceleration
% q1 = final position
% v1 = final velocity 
% ac1 = final acceleration 
% t0 = initial time 
% tf = final time 
clear;
clc;
close all;
d = input('initial data = [q0,v0,ac0,q1,v1,ac1,t0,tf] = ') 
[qd,vd,ad] = Quintic_Generator(d(1),d(2),d(3),d(4),d(5),d(6),d(7),d(8));

d = input('initial data = [q0,v0,ac0,q1,v1,ac1,t0,tf] = ') 
[qd1,vd1,ad1] = Quintic_Generator(d(1),d(2),d(3),d(4),d(5),d(6),d(7),d(8));

d = input('initial data = [q0,v0,ac0,q1,v1,ac1,t0,tf] = ') 
[qd2,vd2,ad2] = Quintic_Generator(d(1),d(2),d(3),d(4),d(5),d(6),d(7),d(8));

t = linspace(0,3,100*3);
q(1,:) = qd;
q(1,101:200) = qd1;
q(1,201:300) = qd2;
figure
plot(t,q);
xlabel('time(ms)');
ylabel('Angle(deg)');
title('Trajectory with multiple Quintic Segments');
v(1,:) = vd;
v(1,101:200) = vd1;
v(1,201:300) = vd2;
figure
plot(t,v);
xlabel('time(sec)');
ylabel('Velocity(deg/sec)');
title(' Velocity Profile for Multiple Quintic Segments');
figure
a(1,1:100) = ad;
a(1,101:200) = ad1;
a(1,201:300) = ad2;
plot(t,a);
xlabel('time(sec)');
ylabel('Acceleration(deg/sec^2)');
title(' Acceleration Profile for Multiple Quintic Segments');
axis([0 3 -400 400])


