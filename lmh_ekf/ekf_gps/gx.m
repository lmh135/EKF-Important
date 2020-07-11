function f=gx(axk1,axk,ayk1,ayk,azk1,azk,q0_imuk1,q0_imuk,q1_imuk1,q1_imuk,q2_imuk1,q2_imuk,q3_imuk1,q3_imuk)
ax=(axk+axk1)/2;ay=(ayk+ayk1)/2;az=(azk+azk1)/2;
q0=(q0_imuk1+q0_imuk)/2;
q1=(q1_imuk1+q1_imuk)/2;
q2=(q2_imuk1+q2_imuk)/2;
q3=(q3_imuk1+q3_imuk)/2;
g=sqrt(ax.^2+ay.^2+az.^2);

A2=[2*(q1*q3-q0*q2);2*(q2*q3+q0*q1);1-2*(q1^2+q2^2)];
f2=[ax;ay;az]-A2*g;
f=f2;
