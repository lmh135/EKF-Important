function f=fx1(xx,p,q,r,p_e,q_e,r_e)
q0=xx(1);q1=xx(2);q2=xx(3);q3=xx(4);

A=[0,-1*(p-p_e),-1*(q-q_e),-1*(r-r_e);
   (p-p_e), 0,    (r-r_e), -1*(q-q_e);
   (q-q_e),-1*(r-r_e),  0,   (p-p_e);
   (r-r_e), (q-q_e),  -1*(p-p_e),  0;];
f=[1/2*A*[q0;q1;q2;q3];0];