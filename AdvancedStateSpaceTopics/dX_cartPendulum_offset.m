function [dX,u] = dX_cartPendulum_offset(t,X,u)

% EOMs come from cartPendulum_eom.m

cartPendulum_setparams;

xc = X(1);   % cart x position (meters)
thp = X(2);  % theta of pendulum (radians)
dxc = X(3);  % cart velocity, dxc/dt
dthp = X(4); % pendulum angular velocity (rad/s)
delta_ang = X(5); % integral of cart position (accumulated over time)

thp = thp; % no change!

M = [mc + mp, -L*mp*sin(thp)
    -L*mp*sin(thp),    mp*L^2 + Jp];

Tau =[L*mp*cos(thp)*dthp^2 + u
    - b*dthp - L*g*mp*cos(thp)];

d2X = M \ Tau; 
% Derivative of last state (integral of xc) is just xc:
dX = [X(3:4); d2X; 0]; % NO CHANGE in the offset...