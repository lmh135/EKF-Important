function f=fxv(vv,ax,ay,az,p,q,r,q0,q1,q2,q3,g)
u=vv(1);v=vv(2);w=vv(3);
A2=[2*(q1*q3-q0*q2);2*(q2*q3+q0*q1);1-2*(q1^2+q2^2)];
A3=[0,-r,q;r,0,-p;-q,p,0];
f=[ax;ay;az]-A2*g-A3*[u;v;w];
