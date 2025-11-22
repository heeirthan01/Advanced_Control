clc
clear 
close all

%Params
params_pend; %load params

q1_bnd = [-2*pi;2*pi];
q2_bnd = q1_bnd;
q1d_bnd = [-inf, inf];
q2d_bnd = q1d_bnd;
u_bnd = [-20, 20];  
q_ic = [-pi/2;0;0;0];
q_tf = [pi/2;0;0;0];

t0 = 0.0;
tF = 4.0;
%Initial guess
T1 = t0 + 0.25*(tF-t0);
T2 = t0 + 0.75 *(tF-t0);
Xstart = [-pi/2,0,0,0];
Xend = [pi/2,0,0,0];
X1 = [-pi/4,0,0,0];
X2 = [pi/4,0,0,0];

%dynamics roll and cost fcn
problem.func.dynamics = @(t, x, u)(dx_ab(t,x,u,p));
Qf = diag([50, 50, 2000, 2000]);
problem.func.pathObj = @(t,x,u) (0*u.^2 + 1); %minimize time
%problem.func.bndObj = [];
problem.func.bndObj = @(t0,x0,tF,xF) ((xF - q_tf).' * Qf * (xF - q_tf));
%problem.func.bndCst = @(t0,x0,tF,xF) bndCstPendubot(t0,x0,tF,xF,q_tf);
%Problem bounds
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low =tF;
problem.bounds.finalTime.upp = tF;

problem.bounds.state.low = [q1_bnd(1), q2_bnd(1), q1d_bnd(1), q2d_bnd(1)]';
problem.bounds.state.upp = [q1_bnd(2), q2_bnd(2), q1d_bnd(2), q2d_bnd(2)]';

problem.bounds.initialState.low = q_ic;
problem.bounds.initialState.upp = q_ic;

% problem.bounds.finalState.low = [-inf;-inf;-inf;-inf];
% problem.bounds.finalState.upp = [inf;inf;inf;inf];
% problem.bounds.finalState.low = q_tf;
% problem.bounds.finalState.upp = q_tf;
problem.bounds.finalState.low = q_tf;
problem.bounds.finalState.upp = q_tf;

problem.bounds.control.low = u_bnd(1);
problem.bounds.control.upp = u_bnd(2);


problem.guess.time = [t0, T1, T2, tF];
problem.guess.state = [Xstart;X1;X2;Xend]';
problem.guess.control = [0, 0, 0, 0];

%Transcription stuff

problem.option.method = 'trapezoid';

%problem.option.trapezoid.nGrid = 60;
%problem.option.defaultAccuracy = 'high';
problem.options.nlpOpt = optimset( ...
    'FunValCheck', 'on',...
    'display', 'iter', ...
    'MaxFunEval', 1e5, ...
    'tolFun', 1e-6, ...
    'TolCon', 1e-6, ...
    'TolX', 1e-6);


% Solve the problem
soln = optimTraj(problem)

t = soln.grid.time;
q1 = soln.grid.state(1,:);
q2 = soln.grid.state(2,:);
q1d = soln.grid.state(3,:);
q2d = soln.grid.state(4,:);
u = soln.grid.control;
% save('q1ref_AB.mat', 'q1')
% save('q2ref_AB.mat', 'q2')
% save('q1dref_AB.mat', 'q1d')
% save('q2dref_AB.mat', 'q2d')
% save('uref_AB.mat', 'u')


figure; clf;

subplot(2,1,1)
plot(t,q1)
ylabel('q1')
title('Acrobot movement q1');

subplot(2,1,2)
plot(t,q2)
ylabel('q2')
title('Acrobot movement q2');

figure; clf;

subplot(2,1,1)
plot(t,q1d)
ylabel('q1d')
title('Acrobot Velcoity q1')

subplot(2,1,2)
plot(t,q2d)
ylabel('q2d')
title('Acrobot Velcoity q2')

% 
% figure; clf;
% plot(t, u)
% ylabel('u')
% title('Input torque')

%% ODE45 Simulation

X0 = soln.interp.state(0);
tsPan = [soln.grid.time(1), soln.grid.time(end)];

S = odeset('RelTol',1e-6,'AbsTol',1e-6);  % low resolution
[tout,xout] = ode45(@(t,x) dx_ab(t,x,soln.interp.control(t),p),tsPan,X0,S);
xout(:,1:2) = unwrap(xout(:,1:2));
q1sim = xout(:,1);
q2sim = xout(:, 2);
usim = arrayfun(@(ti) soln.interp.control(ti), tout);


figure; clf;
plot(tout, q1sim, 'LineWidth',2)
hold on
plot(t, q1, 'k--' ,'LineWidth',2)
ylabel('Posq1')
title('Q1 ODE vs TO')
legend('ODE', 'TO')
figure;clf;
plot(tout, q2sim, 'LineWidth',2)
hold on
plot(t,q2, 'k--','LineWidth',2)
ylabel('Posq2')
title('Q2 ODE vs TO')
legend('ODE', 'TO')
figure;clf;
plot(tout, usim, 'LineWidth',2)
hold on;
plot(t, u, 'k--', 'LineWidth',2)
ylabel('Control')
title('u ODE vs TO')
legend('ODE', 'TO')
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