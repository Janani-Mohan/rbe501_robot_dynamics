function [J]= Jacobian(A11,A22,H,theta1,theta2,theta3)
F = H(1:3,4);
diff1 = diff(F(1),theta1);
diff2 = diff(F(2),theta1);
diff3 = diff(F(3),theta1);
diff4 = diff(F(1),theta2);
diff5 = diff(F(2),theta2);
diff6 = diff(F(3),theta2);
diff7 = diff(F(1),theta3);
diff8 = diff(F(2),theta3);
diff9 = diff(F(3),theta3);
Jv = [diff1,diff4,diff7;diff2,diff5,diff8;diff3,diff6,diff9];
k = [0 0 1]';
jw1 = k;
jw2 = A11(1:3,1:3)*k;
jw3 = A22(1:3,1:3)*k;
Jw = [jw1;jw2;jw3];
fprintf('The Jacobian of an ABB-IRB 1400 without spherical wrist is: ');
J = [diff1,diff4,diff7;diff2,diff5,diff8;diff3,diff6,diff9;jw1,jw2,jw3]

