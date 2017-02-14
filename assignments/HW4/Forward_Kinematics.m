function [x1,y1,x2,y2] = Forward_Kinematics(t1,t2,l1,l2)
x1 = l1*cosd(t1);
y1 = l1*sind(t1);
x2 = l1*cosd(t1) + l2*cosd(t1+t2);
y2 = l1*sind(t1) + l2*sind(t1+t2);
end
