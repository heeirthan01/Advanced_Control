
clc
clear problem
close
% Params

p.gt = 0.002; %gravity term
p.et = 0.001; %engine strength

x_bnd = [-1.2, 0.6];
v_bnd = [-0.07, 0.07];
q_ic = [-0.5; 0.0];
q_tf = [0.5; 0.0];
u_bnd = [-1.0, 1.0];

%dynamics roll and cost fcn
problem.func.dynamics = @(t, x, u)(dx_mc(t,x,u,p));
%problem.func.dynamics = @(t,x,u)[x(2,:); p.et*u - p.gt*cos(3*x(1,:))];
problem.func.pathObj = @(t,x,u) (0*t + 1); %minimize tiem

%Problem bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 100;

problem.bounds.state.low = [x_bnd(1); v_bnd(1)];
problem.bounds.state.upp = [x_bnd(2); v_bnd(2)];

problem.bounds.initialState.low = q_ic;
problem.bounds.initialState.upp = q_ic;

problem.bounds.finalState.low = q_tf;
problem.bounds.finalState.upp = q_tf;

problem.bounds.control.low = u_bnd(1);
problem.bounds.control.upp = u_bnd(2);

%Initial guess

del = q_tf - q_ic;


problem.guess.time = [0, 5, 10];
problem.guess.state = [-0.5, -1.0, 0.5; 0.0, 0.0, 0.0];
problem.guess.control = [1.0, -1.0, 1.0];

%Transcription stuff
problem.options.method = 'trapezoid';
%problem.options.hermiteSimpson.nSegments = 100;
problem.options.verbose = 3;
problem.options.nlpOpt = optimset( ...
    'FunValCheck', 'on',...
    'display', 'iter', ...
    'MaxFunEval', 1e5, ...
    'tolFun', 1e-6, ...
    'TolCon', 1e-6, ...
    'TolX', 1e-6);


%Solve

soln = optimTraj(problem)

%plot

t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
x = z(1,:);
v = z(2,:);
u = soln.interp.control(t);
disp(soln.grid.time(end))
tGrid = soln.grid.time;
xGrid = soln.grid.state(1,:);
vGrid = soln.grid.state(2,:);
uGrid = soln.grid.control;
disp([soln.grid.time' soln.grid.state(1,:)' soln.grid.state(2,:)' soln.grid.control'])

figure(1);clf; 
xhill = linspace(-1.2, 0.6, 200);
yhill = sin(3*xhill);

plot(xhill, yhill);
hold on;
car = plot(xGrid(1),sin(3*xGrid(1)), 'ko','MarkerSize',5);
title('Car trajectory on hill')
for i = 1:length(xGrid)
    set(car, 'XData', xGrid(i), 'YData', sin(3*xGrid(i)));
    drawnow;
    pause(0.1)
end

% Plot the state and control:
figure(2); clf; 
plot(t,u, 'LineWidth',3)
title('Control Evolution')
figure(3); clf;
plot(x, sin(3*x));
hold on
plot(xhill, yhill, 'k--')
legend('Simulation','Hills')
title('State evolution')

figure(4); clf;

subplot(2,2,1); hold on;
plot(t,x);
plot(tGrid,xGrid,'ko','MarkerSize',5,'LineWidth',3);
ylabel('x');

subplot(2,2,3); hold on;
plot(t,v);
plot(tGrid,vGrid,'ko','MarkerSize',5,'LineWidth',3);
ylabel('v');

subplot(2,2,4); hold on;
plot(tGrid,uGrid,'ko','MarkerSize',5,'LineWidth',3);
plot(t,u);
ylabel('u');
disp(max(soln.info.error))
%% ODE45 SIMULATION
X0 = soln.interp.state(0);
tsPan = [soln.grid.time(1), soln.grid.time(end)];

S = odeset('RelTol',1e-2,'AbsTol',1e-2);  % low resolution
[tout,xout] = ode45(@(t,x) dx_mc(t,x,soln.interp.control(t),p),tsPan,X0,S);

Possim = xout(:,1);
Velsim = xout(:, 2);
usim = arrayfun(@(ti) soln.interp.control(ti), tout);

%save('MC_uref.mat', "usim")
figure; clf;

subplot(3,1,1)
plot(tout, Possim)
ylabel('Pos')
subplot(3,1,2)
plot(tout, Velsim)
ylabel('Vel')
subplot(3,1,3)
plot(tout, usim)
ylabel('Control')
