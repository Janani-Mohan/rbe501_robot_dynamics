function Inv_Kinematics(H,a1,a2,a3,d1,d4)
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
a = (H(1,4))^2+(H(2,4))^2;
r= sqrt(a);
px1 = r- a1;
pz1 = H(3,4)- d1;
ax = sqrt(a3^2+d4^2);
s31 = (((px1)^2)+(pz1^2)-(a2^2)-(ax^2))/(2*a2*ax);
c31 = sqrt(1-(s31^2));
s2 = ((ax*c31*pz1)-((a2+(ax*s31))*px1))/(a2^2+ax^2+2*a2*ax*s31);
c2 = ((ax*c31*px1)+((a2+(ax*s31))*pz1))/(a2^2+ax^2+2*a2*ax*s31);
theta1 = atan2(H(2,4),H(1,4));
theta2 = atan2(s2,c2);
theta3 = atan2(s31,c31)-atan2(a3,d4);
theta33 = -atan2(s31,c31)-atan2(a3,d4);
s1 = sin(theta1);
c1 = cos(theta1);
c23 = cos(theta2+theta3);
s23 = sin(theta2+theta3);
r13 = -c1*s23*H(1,3)-s1*s23*H(2,3)+c23*H(3,3);
r23 = -c1*c23*H(1,3)-s1*c23*H(2,3)-s23*H(3,3);
r33 = s1*H(1,3)-c1*H(2,3);
r32 = s1*H(1,2)-c1*H(2,2);
r31 = s1*H(1,1)-c1*H(2,1);
theta4 = atan2(r23,r13);
theta44 = atan2(-r23,-r13);
theta5 = atan2((sqrt(r13^2+r23^2)),-r33);
theta55 = atan2(-(sqrt(r13^2+r23^2)),r33);
theta6 = atan2(-r32,r31);
theta66 = atan2(r32,-r31);
theta=[theta1 theta2 theta3 theta4 theta5 theta6];
fprintf('Solution 1:The values of 6 joint angles are:\n');
theta1 = theta1*(180/pi);
theta2 = theta2*(180/pi);
theta3 = theta3*(180/pi);
theta4 = theta4*(180/pi);
theta5 = theta5*(180/pi);
theta6 = theta6*(180/pi);
theta =[theta1 theta2 theta3 theta4 theta5 theta6]
figure
HomoMatrix( theta1,theta2,theta3,d1,d2,d3,d4,a1,a2,a3,alpha1,alpha2,alpha3);
fprintf('Solution 2:The values of 6 joint angles are:\n');
theta11 = atan2(-H(2,4),-H(1,4))*(180/pi);
theta22 = atan2(-s2,-c2);
theta33 = theta33*(180/pi);
theta44 = theta44*(180/pi);
theta55 = theta55*(180/pi);
theta66 = theta66*(180/pi);
theta=[theta11 theta22 theta33 theta44 theta55 theta66]
% figure
% HomoMatrix( theta11,theta22,theta33,d1,d2,d3,d4,a1,a2,a3,alpha1,alpha2,alpha3);

end
