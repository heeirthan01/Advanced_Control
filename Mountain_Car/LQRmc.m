clear;
close;

%https://underactuated.mit.edu/lqr.html for the linearization et.c.

poss = load('MC_posref.mat'); %pos ref from trajopt
vels = load('MC_velref.mat'); %vel ref from trajopt
us = load('MC_uref.mat');
posref = poss.Possim;
velref = vels.Velsim;
u_ref = us.usim;

dt = 0.1;
totalsim = 95;
steps = totalsim /dt;

t_ref = linspace(0, totalsim, length(posref));
t_sim = linspace(0, totalsim, steps);
posref = interp1(t_ref, posref, t_sim, 'pchip');
velref = interp1(t_ref, velref, t_sim, 'pchip');
uref = interp1(t_ref, u_ref, t_sim, 'pchip');

%Weigth matrices
Q = diag([5,1]);
R = 5;

p.gt = 0.002;
p.et = 0.001;

%%INitialize variables for linearization
syms x1 x2 u1 real
x = [x1; x2];

f = [x1 + dt*x2;
     x2 + dt*(p.et * u1) - (p.gt * cos(3*x1))];

A_sym = jacobian(f, x);
B_sym = jacobian(f, u1);

u0 = 0;
x0 = [-0.5;0.0];

x = x0;
u = u0;

x_traj = zeros(steps,2);
u_traj = zeros(steps,1);
x_traj(1,:) = x0;
u_traj(1) = u0;


for i = 2:steps
    
    %Select the current point of reference
    x_refk = [posref(i) ; velref(i)];
    urefk = uref(i);

    %Linearize about current ref traj
    A = double(subs(A_sym, {x1,x2,u1}, {x_refk(1),x_refk(2),urefk}));
    B = double(subs(B_sym, {x1,x2,u1}, {x_refk(1),x_refk(2),urefk}));

    K = dlqr(A, B, Q, R);
    %[p_refcurr, v_refcurr] = closeststate(x, posref, velref);
    x_err = x - x_refk;
    u = urefk - K*x_err; %Apply feedback law bascially u - urefk = ...
    u = max(min(u,1), -1); %Just in case but works without
    u_traj(i) = u;

    x = dx_lqr(x,u, p, dt); %propagate dynamics
    x_traj(i, :) = x;


end

t = (0:steps-1) * dt;

figure;
subplot(3,1,1)
plot(t,x_traj(:,1), 'LineWidth',2)
hold on;
plot(t, posref, 'r--', 'LineWidth',2)
title('Position cOmparison to Ref')

subplot(3,1,2)
plot(t, x_traj(:,2), 'LineWidth',2)
hold on
plot(t, velref, 'r--', 'LineWidth',2)
title('Velocity Comparison to Ref')

subplot(3,1,3)
plot(t, u_traj, 'LineWidth', 2)
hold on
plot(t, uref, 'r--', 'LineWidth',2)
title('Control COmparison to Ref')


%% ODE45 Closedloop

X0 = x0;
u_fun = @(tq) interp1(t, u_traj, tq, 'pchip', 'extrap');
tsPan = [t(1), t(end)];

S = odeset('RelTol',1e-6,'AbsTol',1e-6);  % low resolution
[tout,xout] = ode45(@(t,x) dx_mc(t,x,u_fun(t),p),tsPan,X0,S);

Possim = xout(:,1);
Velsim = xout(:, 2);
usim = arrayfun(@(ti) u_fun(ti), tout);

%save('MC_uref.mat', "usim")
possim = interp1(tout, Possim, t, 'pchip'); %to match reftraj
velsim = interp1(tout, Velsim, t, 'pchip');
figure; clf;

subplot(3,1,1)
plot(t, possim, 'LineWidth',2)
hold on
plot(t, posref, 'r--', 'Linewidth', 2)
title('Position Comparison ODE45 vs OL')
subplot(3,1,2)
plot(t, velsim, 'LineWidth',2)
hold on
plot(t, velref, 'r--', 'Linewidth', 2)
title('Velocity Comparison ODE45 vs OL')
subplot(3,1,3)
plot(t, u_traj, 'LineWidth',2)
hold on
plot(t, uref, 'r--', 'Linewidth', 2)
title('Control Comparison ODE45 vs OL')

disp(size(tout))
disp(size(t))