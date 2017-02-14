clc;
clear ;
close all;
syms  theta1 theta2 theta3
d1 = 475;
d2 = 0;
d3 = 0;
d4 = 805;
a1 = 150;
a2 = 600;
a3 = 120;
alpha1 = 90;
alpha2 = 0;
alpha3 = 0;
ax = sqrt(a3^2+d4^2);
%Forward Kinematics Solution
A1 = [cos(theta1),-cosd(alpha1)* sin(theta1),sin(theta1)* sind(alpha1),a1*cos(theta1);
sin(theta1),cosd(alpha1)* cos(theta1),-cos(theta1)*sind(alpha1),a1*sin(theta1);
0,sind(alpha1),cosd(alpha1),d1 ;
0,0,0,1];
A2 = [-sin(theta2),-cosd(alpha2)* cos(theta2),cos(theta2)* sind(alpha2),-a2*sin(theta2);
cos(theta2),cosd(alpha2)* -sin(theta2),sin(theta2)* sind(alpha2),a2*cos(theta2);
0,sind(alpha2),cosd(alpha2),d2 ;
0,0,0,1];
A3 = [cos(theta3),-cosd(alpha3)* sin(theta3),sin(theta3)* sind(alpha3),a3*cos(theta3);
sin(theta3),cosd(alpha3)* cos(theta3),-cos(theta3)* sind(90),a3*sin(theta3);
0,sind(alpha3),cosd(alpha3),d3 ;
0,0,0,1];
y_trans=[0;-d4;0;1];
A1 = simplify(vpa(A1));
A2 = simplify(vpa(A2));
A3 = simplify(vpa(A3));
A11 = A1;
A22 = A1*A2;
H = (A1*A2*A3)
fprintf('The position of end effector tip is: ');
F = H*y_trans
fprintf('Plotting Home Position');
theta1 = 0;
theta2 = 0;
theta3 = 0;
HomoMatrix( theta1,theta2,theta3,d1,d2,d3,d4,a1,a2,a3,alpha1,alpha2,alpha3);

%Velocity Kinematics solution
% J = sym(zeros(6,3));
% J(1:3,1) = (F,theta1);
% J(1:3,2) = (F,theta2);
% J(1:3,3) = (F,theta3);
% diff1 = diff(F(1),theta1);
% diff2 = diff(F(2),theta1);
% diff3 = diff(F(3),theta1);
% diff4 = diff(F(1),theta2);
% diff5 = diff(F(2),theta2);
% diff6 = diff(F(3),theta2);
% diff7 = diff(F(1),theta3);
% diff8 = diff(F(2),theta3);
% diff9 = diff(F(3),theta3);
% % Jv = [diff1,diff4,diff7;diff2,diff5,diff8;diff3,diff6,diff9];
% k = [0 0 1]';
% jw1 = k;
% jw2 = A11(1:3,1:3)*k;
% jw3 = A22(1:3,1:3)*k;
% Jw = [jw1;jw2;jw3];
% fprintf('The Jacobian of an ABB-IRB 1400 without spherical wrist is: ');
% J = [diff1,diff4,diff7;diff2,diff5,diff8;diff3,diff6,diff9;jw1,jw2,jw3]

%Plotting Workspace
theta1 = linspace(-170*(pi/180),170*(pi/180));
theta2 = linspace(-70*(pi/180),70*(pi/180));
theta3 = linspace(70*(pi/180),-65*(pi/180));

[THETA1,THETA2,THETA3]= meshgrid(theta1,theta2,theta3);
X = 150.*cos(THETA1) - 600.*cos(THETA1).*sin(THETA2) - 120.*cos(THETA1).*cos(THETA2).*sin(THETA3) - 120.*cos(THETA1).*cos(THETA3).*sin(THETA2) + 805;
Y = 150.*sin(THETA1) - 600.*sin(THETA1).*sin(THETA2) - 120.*cos(THETA2).*sin(THETA1).*sin(THETA3) - 120.*cos(THETA3).*sin(THETA1).*sin(THETA2);
Z = 600.*cos(THETA2) + 120.*cos(THETA2).*cos(THETA3) - 120.*sin(THETA2).*sin(THETA3) + 475;

subplot(2,2,2);
plot3(X(:),Y(:),Z(:), 'r.');
axis equal;
xlabel('X','fontsize',10)
ylabel('Y','fontsize',10)
zlabel('Z','fontsize',10)
title('X-Y-Z workspace generated for all theta1,theta2 and theta3 combinations using forward kinematics formula','fontsize',10)
grid on;
hold on;
subplot(2,2,3);plot(X(:),Y(:), 'r.');
axis equal;
grid on;
xlabel('X','fontsize',10)
ylabel('Y','fontsize',10)
title('X-Y workspace generated for all theta1,theta2 and theta3 combinations using forward kinematics formula','fontsize',10)
hold on;

%Inverse Kinematics solution
fprintf('Home Position\n');
H = [0,-1,0,955;0,0,1,0;1,0,0,1195;0,0,0,1];
Inv_Kinematics(H,a1,a2,a3,d1,d4);

fprintf('Given Position\n');
H = [1 0 0 500;0 1 0 100;0 0 1 1500;0 0 0 1];
Inv_Kinematics(H,a1,a2,a3,d1,d4);


%Inverse Velocity Kinematics
H = [1 0 0 500;0 1 0 100;0 0 1 1500;0 0 0 1];
theta1 = 11.3099;
theta2 = 28.3302;
theta3 = 0.8152;
J11 = -150.0.*sin(theta1) + 600.0.*sin(theta1).*sin(theta2) + 805.0.*sin(theta1).*sin(theta2).*sin(theta3) - 805.0.*sin(theta1).*cos(theta2).*cos(theta3) + 120.0.*sin(theta1).*cos(theta2).*sin(theta3) + 120.0*sin(theta1)*cos(theta3)*sin(theta2);
J12 = 150.0.*cos(theta1) - 600.0.*cos(theta1).*sin(theta2) - 805.0.*cos(theta1).*sin(theta2).*sin(theta3) + 805.0.*cos(theta1).*cos(theta2).*cos(theta3) - 120.0.*cos(theta1).*cos(theta2).*sin(theta3) - 120.0.*cos(theta1).*cos(theta3).*sin(theta2);
J13 = 150.0.*cos(theta1) - 600.0.*cos(theta1).*sin(theta2) - 805.0.*cos(theta1).*sin(theta2).*sin(theta3) + 805.0.*cos(theta1).*cos(theta2).*cos(theta3) - 120.0.*cos(theta1).*cos(theta2).*sin(theta3) - 120.0.*cos(theta1).*cos(theta3).*sin(theta2);
J21 = 150.0.*sin(theta1) - 600.0.*sin(theta1).*sin(theta2) - 120.0.*cos(theta2).*sin(theta1).*sin(theta3) - 120.0.*cos(theta3).*sin(theta1).*sin(theta2) - 805.0.*sin(theta1).*sin(theta2).*sin(theta3) + 805.0.*cos(theta2).*cos(theta3).*sin(theta1);
J22 = 150.0.*sin(theta1) - 600.0.*sin(theta1).*sin(theta2) - 120.0.*cos(theta2).*sin(theta1).*sin(theta3) - 120.0.*cos(theta3).*sin(theta1).*sin(theta2) - 805.0.*sin(theta1).*sin(theta2).*sin(theta3) + 805.0.*cos(theta2).*cos(theta3).*sin(theta1);
J23 = 150.0.*sin(theta1) - 600.0.*sin(theta1).*sin(theta2) - 120.0.*cos(theta2).*sin(theta1).*sin(theta3) - 120.0.*cos(theta3).*sin(theta1).*sin(theta2) - 805.0.*sin(theta1).*sin(theta2).*sin(theta3) + 805.0.*cos(theta2).*cos(theta3).*sin(theta1);
J31 = 600.0.*cos(theta2) + 120.0.*cos(theta2).*cos(theta3) + 805.0.*cos(theta2).*sin(theta3) + 805.0.*cos(theta3).*sin(theta2) - 120.0.*sin(theta2).*sin(theta3) + 475.0;
J32 = 600.0.*cos(theta2) + 120.0.*cos(theta2).*cos(theta3) + 805.0.*cos(theta2).*sin(theta3) + 805.0.*cos(theta3).*sin(theta2) - 120.0.*sin(theta2).*sin(theta3) + 475.0;
J33 =600.0.*cos(theta2) + 120.0.*cos(theta2).*cos(theta3) + 805.0.*cos(theta2).*sin(theta3) + 805.0.*cos(theta3).*sin(theta2) - 120.0.*sin(theta2).*sin(theta3) + 475.0;
J41 = 0;
J42 = sin(theta1);
J43 = sin(theta1);
J51 = 0;
J52 = -cos(theta1);
J53 = -cos(theta1);
J61 = 1;
J62 = 0;
J63 = 0;
J = [ J11 J12 J13;J21 J22 J23;J31 J32 J33;J41 J42 J43;J51 J52 J53;J61 J62 J63];
% J_actual = vpa(subs(J,{theta1,theta2,theta3},{}))
x_v = [5 5 10 0 0 0 ]';
J_inv = pinv(J);
theta_v = J_inv*x_v;
