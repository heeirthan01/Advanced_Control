clear;
close all;

params_pend;

q1ref = load('q1ref_AB.mat', 'q1');
q2ref = load('q2ref_AB.mat', 'q2');
q1dref = load('q1dref_AB.mat', 'q1d');
q2dref = load('q2dref_AB.mat', 'q2d');
uref = load('uref_AB.mat', 'u');

q1ref = q1ref.q1;
q2ref = q2ref.q2;
q1dref = q1dref.q1d;
q2dref = q2dref.q2d;
uref = uref.u;

dt = 0.01;
totalsim = 4;
steps = totalsim /dt;
N = 20; %horizon length

t_ref = linspace(0, totalsim, length(q1ref));
t_sim = linspace(0, totalsim, steps);
q1ref = interp1(t_ref, q1ref, t_sim, 'pchip');
q2ref = interp1(t_ref, q2ref, t_sim, 'pchip');
q1dref = interp1(t_ref, q1dref, t_sim, 'pchip');
q2dref = interp1(t_ref, q2dref, t_sim, 'pchip');
uref = interp1(t_ref, uref, t_sim, 'pchip');

