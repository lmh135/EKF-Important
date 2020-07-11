function f=fx1_eu(xx,p,q,r,p_e,q_e,r_e)
roll=xx(1);pitch=xx(2);yaw=xx(3);
A=[1,sin(roll)*tan(pitch),cos(roll)*tan(pitch);
    0,cos(roll),-sin(roll);
    0,sin(roll)*sec(pitch),cos(roll)*sec(pitch)];
f=A*[p-p_e;q-q_e;r-r_e];