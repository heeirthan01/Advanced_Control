clear;
close all;

params_pend;

q1ref = load('q1ref_PB.mat', 'q1');
q2ref = load('q2ref_PB.mat', 'q2');
q1dref = load('q1dref_PB.mat', 'q1d');
q2dref = load('q2dref_PB.mat', 'q2d');
uref = load('uref_PB.mat', 'u');

q1ref = q1ref.q1;
q2ref = q2ref.q2;
q1dref = q1dref.q1d;
q2dref = q2dref.q2d;
uref = uref.u;

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

Q = diag([10,10,1,1]);
R = 5;

%%INitialize variables for linearization
syms q1 q2 q1d q2d u1 real
q = [q1; q2; q1d; q2d];

f = dx_pb_lqr(0,q,u1,p);

%Euler discretization didnt work
% q1 = q(1);
% q2 = q(2);
% q1dot = q(3);
% q2dot = q(4);
% 
% d11 = p.m1 * p.lc1^2 + p.m2 * (p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2.*cos(q2))...
%     + p.I1 + p.I2;
% d22 = p.m2 * p.lc2^2 + p.I2;
% d12 = p.m2 * (p.lc2^2 + p.l1.*p.lc2.*cos(q2)) + p.I2;
% d21 = d12;
% h1 = -p.m2 * p.l1*p.lc2.*sin(q2).*q2dot.^2 - (2 * p.m2 * p.l1 * p.lc2 .* sin(q2)...
%     .* q2dot .* q1dot);
% h2 = p.m2 * p.l1 * p.lc2 .* sin(q2) .* q1dot.^2;
% phi1 = ((p.m1 * p.lc1 + p.m2 * p.l1) * p.g .* cos(q1)) + (p.m2 * p.lc2 * p.g...
%     .* cos(q1 + q2));
% phi2 = p.m2 * p.lc2 * p.g .* cos(q1 + q2);
% 
% D = [d11 d12;
%     d21 d22];
% 
% qddot = D \ (  [u1 ; 0] - [h1 + phi1; h2 + phi2]);
% f = [q1 + dt *q1dot;
%     q2 + dt * q2dot;
%     q1dot + dt* qddot(1);
%     q2dot + dt * qddot(2)];

A_sym = jacobian(f, q);
B_sym = jacobian(f, u1);
A_fun = matlabFunction(A_sym, 'Vars', {q1, q2, q1d, q2d, u1});
B_fun = matlabFunction(B_sym, 'Vars', {q1, q2, q1d, q2d, u1});
clear q1 q2 q1d q2d u1 

x_traj = zeros(steps,4);
u_traj = zeros(steps,1);

eqb = [pi/2;0;0;0];

%Init states
x0 = [-pi/2;0;0;0];
u0 = 0;
x = x0;
u = u0;
x_traj(1,:) = x;
u_traj(1) = u;
u_max = 20;

%% Start loop

for i = 2:steps
    if (norm([x(1);x(2)]-[q1ref(end);q2ref(end)]) > 0.2)
        %Linearize about current ref traj
        A = A_fun(q1ref(i), q2ref(i), q1dref(i), q2dref(i), uref(i));
        B = B_fun(q1ref(i), q2ref(i), q1dref(i), q2dref(i), uref(i));
        
        x_err = [wrapToPi(x(1)-q1ref(i));
        wrapToPi(x(2)-q2ref(i));
        x(3) - q1dref(i);
        x(4)-q2dref(i)];
    else
        %A bit more stable when used, but not really needed!
        A = A_fun(eqb(1), eqb(2), eqb(3), eqb(4), 0);
        B = B_fun(eqb(1), eqb(2), eqb(3), eqb(4), 0);

        x_err = [wrapToPi(x(1)-eqb(1));
        wrapToPi(x(2)-eqb(2));
        x(3) - eqb(3);
        x(4)-eqb(4)];
    end
        
    %discretize here
    A = eye(4) + dt*A;
    B = dt * B;
    K = dlqr(A,B,Q,R);


    
    u = uref(i) - K * x_err;
    u = max(-u_max, min(u_max, u));

    x = rk4_step(@(xx, uu) dx_pb_lqr(0,xx, uu, p), x, u,dt);
    x(1) = wrapToPi(x(1));
    x(2) = wrapToPi(x(2));
    x_traj(i,:) = x';
    u_traj(i) = u;
   
    

end



%% Plotting

t = (0:steps-1) * dt;

figure;
subplot(2,1,1)
plot(t,x_traj(:,1), 'LineWidth',2)
hold on;
plot(t, q1ref, 'r--', 'LineWidth',2)
title('Q1 cOmparison to Ref')

subplot(2,1,2)
plot(t, x_traj(:,2), 'LineWidth',2)
hold on
plot(t, q2ref, 'r--', 'LineWidth',2)
title('q2 Comparison to Ref')


%% ODE45 Simulation

%% ODE45 Closedloop

X0 = x0;
u_fun = @(tq) u_traj( max(1, min(length(u_traj), floor(tq/dt) + 1)) ); %step by tq/dt + 1 which is like i+1 for RK4
tsPan = [t(1), t(end)];

S = odeset('RelTol',1e-8,'AbsTol',1e-8);  % low resolution
[tout,xout] = ode45(@(t,x) dx_pb_lqr(t,x,u_fun(t),p),tsPan,X0,S);

q1sim = xout(:,1);
q2sim = xout(:, 2);
usim = arrayfun(@(ti) u_fun(ti), tout);

q1sim = interp1(tout, q1sim, t, 'pchip'); %to match reftraj
q2sim = interp1(tout, q2sim, t, 'pchip');
figure; clf;

subplot(3,1,1)
plot(t, q1sim, 'LineWidth',2)
hold on
plot(t, q1ref, 'r--', 'Linewidth', 2)
title('q1 Comparison ODE45 vs OL')
subplot(3,1,2)
plot(t, q2sim, 'LineWidth',2)
hold on
plot(t, q2ref, 'r--', 'Linewidth', 2)
title('q2 Comparison ODE45 vs OL')
subplot(3,1,3)
plot(t, u_traj, 'LineWidth',2)
hold on
plot(t, uref, 'r--', 'Linewidth', 2)
title('Control Comparison ODE45 vs OL')

%% Animate Pendubot
L1 = p.l1;
L2 = p.l2;  
fps = 100;       
skip = round(length(tout)/(fps*5));  

figure; hold on;
axis equal;
axis([-L1-L2, L1+L2, -L1-L2, L1+L2]);
grid on;
title('Pendubot Swing-Up');
xlabel('X');
ylabel('Y');

for k = 1:skip:length(tout)
    q1 = xout(k,1);
    q2 = xout(k,2);
    
    %model x,y tips
    x1 = L1*cos(q1);
    y1 = L1*sin(q1);
    x2 = x1+L2*cos(q1+q2);
    y2 = y1+L2*sin(q1+q2);

    cla;
    plot([0 x1 x2],[0 y1 y2],'-o','LineWidth',2,'MarkerFaceColor','k');
    plot(0,0,'ks','MarkerSize',10,'MarkerFaceColor','k'); 
    title(sprintf('Time = %.2f s', tout(k)));
    drawnow;
end