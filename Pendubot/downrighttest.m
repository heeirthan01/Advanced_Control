clc
clear 
close all

%Params
real_params_pend; %load params

Q = diag([15,15,1,1]);
R = 50;
eqb = [-pi/2;0;0;0];
u_max = 10;
dt = 0.01;
%%INitialize variables for linearization
syms q1 q2 q1d q2d u1 real
q = [q1; q2; q1d; q2d];
x0 = [-pi/4;0;0;0];
f = dx_pb_lqrreal(0,q,u1,p);

A_sym = jacobian(f, q);
B_sym = jacobian(f, u1);
A_fun = matlabFunction(A_sym, 'Vars', {q1, q2, q1d, q2d, u1});
B_fun = matlabFunction(B_sym, 'Vars', {q1, q2, q1d, q2d, u1});
clear q1 q2 q1d q2d u1 
A = A_fun(eqb(1), eqb(2), eqb(3), eqb(4), 0);
B = B_fun(eqb(1), eqb(2), eqb(3), eqb(4), 0);

A = eye(4) + dt*A;
B = dt * B;

K = dlqr(A,B,Q,R)

pvar = 0;
pNoise = diag([pvar,pvar,pvar,pvar]);

mvar = 10;
mNoise = diag([mvar,mvar]);

initvar = 1e-6; %confidence in IC
