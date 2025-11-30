function [dX,u] = dX_cartPendulum_CL(t,X,K,Xeq)

% EOMs come from cartPendulum_eom.m
%
% K is a 1x4 matrix of control gains (e.g., from LQR)

cartPendulum_setparams;

xc = X(1);   % cart x position (meters)
thp = X(2);  % theta of pendulum (radians)
dxc = X(3);  % cart velocity, dxc/dt
dthp = X(4); % pendulum angular velocity (rad/s)

u = -K*(X-Xeq);
umax = 10;
u = max(min(u,umax),-umax);

if u>0, u=10; else, u=-10; end

M = [mc + mp, -L*mp*sin(thp)
    -L*mp*sin(thp),    mp*L^2 + Jp];

Tau =[L*mp*cos(thp)*dthp^2 + u
    - b*dthp - L*g*mp*cos(thp)];

d2X = M \ Tau; 
dX = [X(3:4); d2X]; % velocities, then accelerations, of the DOFs