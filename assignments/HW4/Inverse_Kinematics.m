function [theta1,theta2] = Inverse_Kinematics(x2,y2,l1,l2)
D = (((x2^2+y2^2)-(l1^2+l2^2))/(2*l2*l1));
theta2 = atan2d(sind(acosd(D)), D);
theta1 = atan2d(y2,x2) - atan2d(l2*sind(acosd(D)),(l1+l2*(D)));
end
