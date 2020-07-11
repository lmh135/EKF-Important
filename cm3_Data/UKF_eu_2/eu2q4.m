function f=eu2q4(roll,pitch,yaw)
q0=cos(roll./2).*cos(pitch./2).*cos(yaw./2)+sin(roll./2).*sin(pitch./2).*sin(yaw./2);
q1=-1.*cos(roll./2).*sin(pitch./2).*sin(yaw./2)+sin(roll./2).*cos(pitch./2).*cos(yaw./2);
q2=cos(roll./2).*sin(pitch./2).*cos(yaw./2)+sin(roll./2).*cos(pitch./2).*sin(yaw./2);
q3=cos(roll./2).*cos(pitch./2).*sin(yaw./2)+sin(roll./2).*sin(pitch./2).*cos(yaw./2);
f=[q0,q1,q2,q3];