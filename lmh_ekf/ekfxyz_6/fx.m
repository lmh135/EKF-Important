function f=fx(xx,q0,q1,q2,q3,ax,ay,az,p,q,r,g)
u=xx(4);v=xx(5);w=xx(6);
A1=[1-2*(q2^2+q3^2),2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3),1-2*(q1^2+q3^2),2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),1-2*(q1^2+q2^2);];
f1=A1*[u;v;w];
A2=[2*(q1*q3-q0*q2);2*(q2*q3+q0*q1);1-2*(q1^2+q2^2)];
A3=[0,-r,q;r,0,-p;-q,p,0];
f2=[ax;ay;az]-A2*g-A3*[u;v;w];
f=[f1;f2];