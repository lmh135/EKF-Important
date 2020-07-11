function f=hx(xx)
q0=xx(7);q1=xx(8);q2=xx(9);q3=xx(10);x=xx(1);y=xx(2);z=xx(3);u=xx(4);v=xx(5);w=xx(6);
A1=atan(2*(q2*q3+q0*q1)/(1-2*(q1^2+q2^2)));
A2=-1*asin(2*(q1*q3-q0*q2));
A3=atan(2*(q2*q1+q0*q3)/(1-2*(q2^2+q3^2)));
f=[x;y;z;u;v;w;A1;A2;A3];
% f=[A1;A2;A3];