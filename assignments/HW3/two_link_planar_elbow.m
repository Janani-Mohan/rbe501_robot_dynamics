%% Dynamic Model of a 2 Link Planar Elbow manipulator with centre of masses
function Question_2(th1,th2)

%Defining the position components of each link
syms l1 l2 a1 a2 m1 m2 theta1(t) theta2(t) a b g 
syms x_v1(t) x_v2(t) I1 I2 t
a1 = 2*l1;
a2 = 2*l2;

%Link 1
x1 = l1*cos(a);
y1 = l1*sin(a);
x_v1(t) = [x1 ; y1];
x_v1(t) = subs(x_v1(t), a, theta1(t));

%Link 2
x2 = a1*cos(a)+l2*cos(a+b);
y2 = a1*sin(a)+l2*sin(a+b);
x_v2(t) = [x2 ; y2 ];
x_v2(t) = subs(x_v2(t), [a,b], [theta1(t),theta2(t)]);

y1 = subs(y1, a, theta1(t));
y2 = subs(y2,[a,b], [theta1(t),theta2(t)]); 


%Defining the velocity components of each link.
v1 = diff(x_v1(t),t); 
v2 = diff(x_v2(t),t);

% I1 = m1 * l1^2;
% I2 = m2 * l2^2;
%Defining Kinetic energy of masses associated with each link
d1 = transpose(sym(v1))*(sym(v1));
K1 = 0.5 * m1 * d1 + 0.5 * I1 * (diff(theta1(t),t)^2) ;

d2 = transpose(sym(v2))*(sym(v2));
K2 = 0.5 * m2 * d2 + 0.5 * I2 * (diff(theta1(t)+ theta2(t) ,t)^2);

KE = K1 + K2;

%Defining Potential energy of masses associated with each link

P1 = m1 * g * y1;
P2 = m2 * g * y2;

PE = P1 + P2 ;
% Defining Lagrange Equation

L = KE - PE;

% Defining Euler-Lagrange Equation
syms theta1d theta2d t1 t2  
L = subs(L,[diff(theta1(t),t),diff(theta2(t),t),theta1(t), theta2(t)],[theta1d,theta2d,t1,t2]);
L1 = diff(L,theta1d);
L2 = diff(L,theta2d);

L4 = diff(L,t1);
L5 = diff(L,t2);

syms dtheta1(t) dtheta2(t) dtheta3(t) 
L1 = subs(L1,[theta1d,theta2d,t1,t2],[dtheta1(t),dtheta2(t),theta1(t),theta2(t)]);
L2 = subs(L2,[theta1d,theta2d,t1,t2],[dtheta1(t),dtheta2(t),theta1(t),theta2(t)]);

ddtL1 = diff(L1,t);
ddtL2 = diff(L2,t);

L4 = subs(L4,[theta1d,theta2d,t1,t2],[dtheta1(t),dtheta2(t),theta1(t),theta2(t)]);
L5 = subs(L5,[theta1d,theta2d,t1,t2],[dtheta1(t),dtheta2(t),theta1(t),theta2(t)]);


tau1 = ddtL1 - L4;
tau2 = ddtL2 - L5;

tau = [tau1;tau2];
syms ddtheta1 ddtheta2  
tau1 = subs(tau1,[diff(dtheta1(t), t),diff(dtheta2(t), t),diff(theta1(t), t),diff(theta2(t), t)],[ddtheta1,ddtheta2,dtheta1(t),dtheta2(t)]);
tau2 = subs(tau2,[diff(dtheta1(t), t),diff(dtheta2(t), t),diff(theta1(t), t),diff(theta2(t), t)],[ddtheta1,ddtheta2,dtheta1(t),dtheta2(t)]);
tau = subs(tau,[diff(dtheta1(t), t),diff(dtheta2(t), t),diff(theta1(t), t),diff(theta2(t), t)],[ddtheta1,ddtheta2,dtheta1(t),dtheta2(t)]);
tau = simplify(tau);

syms th11(t) th22(t)
th11(t) = th1;
th22(t) = th2;
theta1_d = diff(th11(t),t);
theta2_d = diff(th22(t),t);
theta1_dd = diff(theta1_d,t);
theta2_dd = diff(theta2_d,t);
tau = subs(tau,[ddtheta1,ddtheta2,dtheta1(t),dtheta2(t),theta1(t),theta2(t)],[theta1_dd,theta2_dd,theta1_d,theta2_d,th1,th2]);
fprintf('The Torque is given by :\n ');
tau

end


