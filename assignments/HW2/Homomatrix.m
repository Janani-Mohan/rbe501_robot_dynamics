function HomoMatrix( theta1,theta2,theta3,d1,d2,d3,d4,a1,a2,a3,alpha1,alpha2,alpha3)
A1 = [cosd(theta1),-cosd(alpha1)* sind(theta1),sind(theta1)* sind(alpha1),a1*cosd(theta1);
sind(theta1),cosd(alpha1)* cosd(theta1),-cosd(theta1)*sind(alpha1),a1*sind(theta1);
0,sind(alpha1),cosd(alpha1),d1 ;
0,0,0,1];
A2 = [cosd(90+theta2),-cosd(alpha2)* sind(90+theta2),sind(90+theta2)* sind(alpha2),a2*cosd(90+theta2);
sind(90+theta2),cosd(alpha2)* cosd(90+theta2),-cosd(90+theta2)* sind(alpha2),a2*sind(90+theta2);
0,sind(alpha2),cosd(alpha2),d2 ;
0,0,0,1];
A3 = [cosd(theta3),-cosd(alpha3)* sind(theta3),sind(theta3)* sind(alpha3),a3*cosd(theta3);
sind(theta3),cosd(alpha3)* cosd(theta3),-cosd(theta3)* sind(alpha3),a3*sind(theta3);
0,sind(alpha3),cosd(alpha3),d3 ;
0,0,0,1];
% A4 = [cosd(theta4),-cosd(alpha4)* sind(theta4),sind(theta4)* sind(alpha4),a4*cosd(theta4);
% sind(theta4),cosd(alpha4)* cosd(theta4),-cosd(theta4)* sind(alpha4),a4*sind(theta4);
% 0,sind(alpha4),cosd(alpha4),d4 ;
% 0,0,0,1]
% A5 = [cosd(theta5),-(cosd(alpha5)* sind(theta5)),(sind(theta5)* sind(alpha5)),(a5*cosd(theta5));
% sind(theta5),(cosd(alpha5)* cosd(theta5)),-(cosd(theta5)* sind(alpha5)),(a5*sind(theta5));
% 0,sind(alpha5),cosd(alpha5),d5 ;
% 0,0,0,1]
% A6 = [cosd(theta6),-cosd(alpha6)* sind(theta6),sind(theta6)* sind(alpha6),a6*cosd(theta6);
% sind(theta6),cosd(alpha6)* cosd(theta6),-cosd(theta6)* sind(alpha6),a6*sind(theta6);
% 0,sind(alpha6),cosd(alpha6),d6 ;
% 0,0,0,1]

y_trans=[0;-d4;0;1];
A11 = A1;
A22 = A1*A2;
A33 = A1*A2*A3*y_trans;
H = A1*A2*A3;
subplot(2,2,1);
LinkX=[0 A11(1,4) A22(1,4) A33(1)];
LinkY=[0 A11(2,4) A22(2,4) A33(2)];
LinkZ=[0 A11(3,4) A22(3,4) A33(3)];
plot3(0,0,0,'-o','LineWidth',1,'MarkerSize',5,'MarkerFaceColor',[0.1 0.1 0.1]);
hold on;
plot3(LinkX,LinkY,LinkZ,'-o','LineWidth',1,'MarkerSize',5,'MarkerFaceColor',[0.5 0.5 0.5]);
hold on
xlabel('x');
ylabel('y');
zlabel('z');
grid on

end
