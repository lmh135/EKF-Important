function f=fx_dot1(xx,p,q,r)

A=[0,-p,-q,-r,0;
   p, 0, r,-q,0;
   q,-r, 0, p,0;
   r, q,-p, 0,0;
   0, 0, 0, 0,0;
];
f=1/2*A;