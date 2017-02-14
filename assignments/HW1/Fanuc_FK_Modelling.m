%%
% |Q5) Solve for the composite homogeneous transformation representing the forward kinematics|
% |where H = [ nx ox ax px ; ny oy ay py ; nz oz az pz ; 0 0 0 1]|
clc;
clear ;
close all;
syms  theta4 theta5 theta6
syms d1 d2 d3 d6
dh_param = [0 d1 0 -90; 90 d2 0 -90; 0 d3 0 0;
            theta4 0 0 90;theta5 0 0 -90;theta6 d6 0 0];      
 
A1 = [cosd(0),-cosd(-90)* sind(0),sind(0)* sind(-90),0*cos(0);
    sin(0),cosd(-90)* cos(0),-cos(0)* sind(-90),0*sin(0); 
    0,sind(-90),cosd(-90),d1 ; 0,0,0,1] 
A2 = [cosd(90),-cosd(-90)* sind(90),sind(90)* sind(-90),0*cosd(90);
    sind(90),cosd(-90)* cosd(90),-cosd(90)* sind(-90),0*sind(90);
    0,sind(-90),cosd(-90), d2 ; 0,0,0,1]
A3 = [cos(0),-cosd(0)* sin(0),sin(0)* sind(0),0*cos(0);
    sin(0),cosd(0)* cos(0),-cos(0)* sind(0),0*sin(0); 
    0,sind(0),cosd(0),d3 ; 0,0,0,1] 
A4 = [cos(theta4),-cosd(90)* sin(theta4),sin(theta4)* sind(90),0*cos(theta4); 
    sin(theta4),cosd(90)* cos(theta4),-cos(theta4)* sind(90),0*sin(theta4);
    0,sind(90),cosd(90),0 ; 0,0,0,1] 
A5 = [cos(theta5),-cosd(-90)* sin(theta5),sin(theta5)* sind(-90),0*cos(theta5);
    sin(theta5),cosd(-90)* cos(theta5),-cos(theta5)* sind(-90),0*sin(theta5); 
    0,sind(-90),cosd(-90),0 ; 0,0,0,1] 
A6 = [cos(theta6),-cosd(0)* sin(theta6),sin(theta6)* sind(0),0*cos(theta6); 
    sin(theta6),cosd(0)* cos(theta6),-cos(theta6)* sind(0),0*sin(theta6); 
    0,sind(0),cosd(0),d6 ; 0,0,0,1]

A = A1*A2*A3*A4*A5*A6
H = simplify(vpa(A))

nx = H(1,1)
ny = H(2,1)
nz = H(3,1)
ox = H(1,2)
oy = H(2,2)
oz = H(3,2)
ax = H(1,3)
ay = H(2,3)
az = H(3,3)
px = H(1,4)
py = H(2,4)
pz = H(3,4)
