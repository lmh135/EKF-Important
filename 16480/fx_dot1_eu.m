function f=fx_dot1_eu(xx,p,q,r)
roll=xx(1);pitch=xx(2);yaw=xx(3);
f=[tan(pitch)*(q*cos(roll)+r*sin(roll)),(1/(1+cos(pitch)^2))*(q*sin(roll)+r*cos(roll)),0;
    -q*sin(roll)-r*cos(roll),0,0;
    q*cos(roll)*sec(pitch)-r*sin(roll)*sec(pitch),q*sin(roll)*tan(pitch)*sec(pitch)+r*cos(roll)*tan(pitch)*sec(pitch),0];