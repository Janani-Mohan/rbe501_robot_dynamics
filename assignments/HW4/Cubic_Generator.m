function [qd,vd,ad] =Cubic_Generator(q_i, q_f, v_i,v_f,t_i,t_f)

t = linspace(t_i,t_f,100*(t_f-t_i));
c = ones(size(t)); 
A=[1,t_i, t_i^2,t_i^3;
    0,1,2*t_i,3*t_i^2;
    1,t_f, t_f^2,t_f^3;
    0,1,2*t_f,3*t_f^2];

B=[q_i;v_i;q_f;v_f];
a=A\B;

qd = a(1).*c + a(2).*t +a(3).*t.^2 + a(4).*t.^3 ;
vd = a(2).*c +2*a(3).*t +3*a(4).*t.^2 ; 
ad = 2*a(3).*c + 6*a(4).*t;
end
