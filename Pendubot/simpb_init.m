params_pend;

q1ref = load('q1ref_PB.mat', 'q1');
q2ref = load('q2ref_PB.mat', 'q2');
q1dref = load('q1dref_PB.mat', 'q1d');
q2dref = load('q2dref_PB.mat', 'q2d');
uref = load('uref_PB.mat', 'u');
ulqr = load('utrajLQR_PB.mat', 'u_traj');
Ksim = load('Klqr.mat','Ksim');

q1ref = q1ref.q1;
q2ref = q2ref.q2;
q1dref = q1dref.q1d;
q2dref = q2dref.q2d;
uref = uref.u;
ulqr = ulqr.u_traj;
Ksim = Ksim.Ksim;
dt = 0.01;
totalsim = 4;
steps = totalsim /dt;

t_ref = linspace(0, totalsim, length(q1ref));
t_sim = linspace(0, totalsim, steps);
q1ref = interp1(t_ref, q1ref, t_sim, 'pchip');
q2ref = interp1(t_ref, q2ref, t_sim, 'pchip');
q1dref = interp1(t_ref, q1dref, t_sim, 'pchip');
q2dref = interp1(t_ref, q2dref, t_sim, 'pchip');
uref = interp1(t_ref, uref, t_sim, 'pchip');
size(uref)

Q = diag([15,15,1,1]);
R = 50*5;
eqb = [pi/2;0;0;0];
u_max = 20;

%%INitialize variables for linearization
syms q1 q2 q1d q2d u1 real
q = [q1; q2; q1d; q2d];

f = dx_pb_lqr(0,q,u1,p);

A_sym = jacobian(f, q);
B_sym = jacobian(f, u1);
A_fun = matlabFunction(A_sym,'File', 'Afunfile','Vars', {q1, q2, q1d, q2d, u1});
B_fun = matlabFunction(B_sym, 'File', 'Bfunfile', 'Vars', {q1, q2, q1d, q2d, u1});
clear q1 q2 q1d q2d u1 

pvar = 1e-6;
pNoise = diag([pvar,pvar,pvar,pvar]);
mvar = 1e-2;
mNoise = diag([mvar,mvar]);
initvar = 1e-20;