function [dX,u] = dX_cartPendulum(t,X,u)

% EOMs come from cartPendulum_eom_pendpush.m
%
% Here, the "u" is a push at the midpoint of the pendulum

%keyboard
cartPendulum_setparams;

xc = X(1);   % cart x position (meters)
thp = X(2);  % theta of pendulum (radians)
dxc = X(3);  % cart velocity, dxc/dt
dthp = X(4); % pendulum angular velocity (rad/s)

%if ~exist('u','var'), u=0; end % passive, by default

%keyboard

%u = 10; % passive
% if xc<0
%     u = 1*F_aigym; % max force
% else
%     u = -1*F_aigym;
% end

M = [mc + mp, -L*mp*sin(thp)
    -L*mp*sin(thp),    mp*L^2 + Jp];

 Tau = [L*mp*cos(thp)*dthp^2 + u
L*g*mp*u*sin(thp) - (dxc^2*mp*u)/2 - (Jp*dthp^2*u)/2 - L*g*mp*cos(thp) - (L^2*dthp^2*mp*u)/2 - (dxc^2*mc*u)/2 + L*dthp*dxc*mp*u*sin(thp)];

% below is for original IP:
%Tau =[L*mp*cos(thp)*dthp^2 + u
%    - b*dthp - L*g*mp*cos(thp)];

d2X = M \ Tau; 
dX = [X(3:4); d2X]; % velocities, then accelerations, of the DOFs