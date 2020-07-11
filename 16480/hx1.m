function f=hx1(xx)
q0=xx(1);q1=xx(2);q2=xx(3);q3=xx(4);
%¼ÆËã¦Õ
phi=atan2(2*(q2*q3+q0*q1),(1-2*(q1^2+q2^2)));       


%¼ÆËã¦È
theta= -1*asin(2*(q1*q3-q0*q2));
if (theta>pi)
    theta=theta-2*pi;
end
if (theta<(-1*pi))
    theta=theta+2*pi;
end

%¼ÆËã¦×
psi=atan2(2*(q1*q2+q0*q3),(1-2*(q2^2+q3^2)));


f=[phi;theta;psi];