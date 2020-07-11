function f=q42eu(q0,q1,q2,q3)
% roll=atan(2.*(q2.*q3+q0.*q1)./(1-2.*(q1.^2+q2.^2)));
% pitch=-1.*asin(2.*(q1.*q3-q0.*q2));
% yaw=atan(2.*(q2.*q1+q0.*q3)./(1-2.*(q2.^2+q3.^2)));

roll=atan2(2.*(q2.*q3+q0.*q1),(1-2.*(q1.^2+q2.^2)));
pitch=-1.*asin(2.*(q1.*q3-q0.*q2));
yaw=atan2(2.*(q2.*q1+q0.*q3),(1-2.*(q2.^2+q3.^2)));
f=[roll',pitch',yaw'];