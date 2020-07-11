function f=fx(xx,p,q,r)
q0=xx(1);q1=xx(2);q2=xx(3);q3=xx(4);

A4=[0,-p,-q,-r;
    p,0,r,-q;
    q,-r,0,p;
    r,q,-p,0];
f=0.5*A4*[q0;q1;q2;q3];

