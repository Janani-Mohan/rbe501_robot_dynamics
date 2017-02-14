%% Dynamic Model of a 3 Link Articulated Elbow manipulator
clc;
clear;
close all;

%Defining the position components of each link
syms l1 l2 l3 m1 m2 m3 theta1(t) theta2(t) theta3(t) a b c t g 
syms theta1(t) theta2(t) theta3(t) x_v1(t) x_v2(t) x_v3(t) 

%Link 1
x1 = 0;
y1 = 0;
z1 = l1;
x_v1(t) = [x1 ; y1; z1];

%Link 2
x2 = l2 * cos(b) * cos(a);
y2 = l2 * cos(b) * sin(a);
z2 = l1 + l2 * sin(b);
x_v2(t) = [x2 ; y2 ; z2];
x_v2(t) = subs(x_v2(t), [a,b], [theta1(t),theta2(t)]);
z2 = subs(z2,[a,b], [theta1(t),theta2(t)]); 

%Link 3
x3 =  cos(a) * (l2 * cos(b) + l3 * cos(b + c));
y3 =  sin(a) * (l2 * cos(b) + l3 * cos(b + c));
z3 = l1 + l2 * sin(b) + l3 * sin(b+c);
x_v3(t) = [x3 ; y3 ; z3];
x_v3(t) = subs(x_v3(t), [a,b,c],[theta1(t),theta2(t),theta3(t)]);
z3 = subs(z3,[a,b,c], [theta1(t),theta2(t),theta3(t)]); 

%Defining the velocity components of each link.
v1 = diff(x_v1(t),t); 
v2 = diff(x_v2(t),t);
v3 = diff(x_v3(t),t);

%Defining Kinetic energy of masses associated with each link
d1 = transpose(sym(v1))*(sym(v1));
K1 = 0.5 * m1 * d1;

d2 = transpose(sym(v2))*(sym(v2));
K2 = 0.5 * m2 * d2;

d3 = transpose(sym(v3))*(sym(v3));
K3 = 0.5 * m3 * d3;

%Defining Potential energy of masses associated with each link

P1 = m1 * g * z1;
P2 = m2 * g * z2;
P3 = m3 * g * z3;


% Defining Lagrange Equation

L = (K1+K2+K3)-(P1+P2+P3);

% Defining Euler-Lagrange Equation
syms theta1d theta2d theta3d t1 t2 t3 
L = subs(L,[diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),theta1(t), theta2(t), theta3(t)],[theta1d,theta2d,theta3d,t1,t2,t3]);
L1 = diff(L,theta1d);
L2 = diff(L,theta2d);
L3 = diff(L,theta3d);

L4 = diff(L,t1);
L5 = diff(L,t2);
L6 = diff(L,t3);

syms dtheta1(t) dtheta2(t) dtheta3(t) 
L1 = subs(L1,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);
L2 = subs(L2,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);
L3 = subs(L3,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);

ddtL1 = diff(L1,t);
ddtL2 = diff(L2,t);
ddtL3 = diff(L3,t);

L4 = subs(L4,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);
L5 = subs(L5,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);
L6 = subs(L6,[theta1d,theta2d,theta3d,t1,t2,t3],[dtheta1(t),dtheta2(t),dtheta3(t),theta1(t),theta2(t),theta3(t)]);

tau1 = ddtL1 - L4;
tau2 = ddtL2 - L5;
tau3 = ddtL3 - L6;

tau = [tau1;tau2;tau3];
syms ddtheta1 ddtheta2 ddtheta3 
tau = subs(tau,[diff(dtheta1(t), t),diff(dtheta2(t), t),diff(dtheta3(t), t),diff(theta1(t), t),diff(theta2(t), t),diff(theta3(t), t)],[ddtheta1,ddtheta2,ddtheta3,dtheta1(t),dtheta2(t),dtheta3(t)]);
tau = simplify(tau);

fprintf('The Torque is given by :\n ');
tau



