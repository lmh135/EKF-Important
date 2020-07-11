function f=fx(xx,ax,ay,az,p,q,r)
q0=xx(7);q1=xx(8);q2=xx(9);q3=xx(10);u=xx(4);v=xx(5);w=xx(6);g=xx(11);
A1=[1-2*(q2^2+q3^2),2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3),1-2*(q1^2+q3^2),2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),1-2*(q1^2+q2^2);];
f1=A1*[u;v;w];
A2=[2*(q1*q3-q0*q2);2*(q2*q3+q0*q1);1-2*(q1^2+q2^2)];
A3=[0,-r,q;r,0,-p;-q,p,0];
f2=[ax;ay;az]-A2*g-A3*[u;v;w];
A4=[0,-p,-q,-r;p,0,r,-q;q,-r,0,p;r,q,-p,0];
f3=0.5*A4*[q0;q1;q2;q3];
f4=zeros(1,1);
f=[f1;f2;f3;f4];