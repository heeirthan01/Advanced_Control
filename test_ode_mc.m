clc;
clear;
t = 0;
p.et = 0.001;
p.gt = 0.0025;

% Please set ode45 integration tolerances specifically!
%S = odeset('RelTol',1e-1,'AbsTol',1e-1);  % very low resolution
S = odeset('RelTol',1e-2,'AbsTol',1e-2);  % low resolution
%S = odeset('RelTol',1e-6,'AbsTol',1e-6); % high resolution
X0 = [-0.5;0.0]; % initial velocity and angle, of pendulum
Tsim = 20;
u = 1;

% Type help ode45 (or doc ode45) for more information:
[tout,xout] = ode45(@(t,x) dx_mc(t,x,u,p),[0 Tsim],X0,S);

figure(2); clf
plot(tout,xout)
legend('Position','Velocity')
